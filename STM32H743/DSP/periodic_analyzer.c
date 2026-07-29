#include "periodic_analyzer.h"

#include "FFT.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define PA_PI                    3.14159265358979323846f
#define PA_TWO_PI                (2.0f * PA_PI)
#define PA_MAX_PEAKS             PERIODIC_ANALYZER_MAX_SPECTRUM_COMPONENTS
#define PA_PEAK_EXCLUSION_BINS   8U
#define PA_HARMONIC_TOL_BINS     2.25f
#define PA_BAND_EDGE_TOL_BINS    0.05f
#define PA_VPP_MIN_SEGMENTS      2048U
#define PA_VPP_SEGMENTS_PER_ORDER 32U
#define PA_VPP_BISECTION_STEPS   32U
#define PA_NUISANCE_MAX_COLUMNS  2U
#define PA_SOLVER_MAX_ORDER      \
    (1U + 2U * PERIODIC_ANALYZER_MAX_COMPONENTS + \
     PA_NUISANCE_MAX_COLUMNS)
#define PA_REFINE_STAGES         6U
#define PA_REFINE_MAX_SHIFTS     8U
#define PA_NUISANCE_MIN_COLUMN_MEAN_SQ 1e-10
#define PA_NUISANCE_NYQUIST_EDGE_BALANCE 1e-3
#define PA_NUISANCE_COLUMN_COSINE 0U
#define PA_NUISANCE_COLUMN_SINE   1U
#define PA_PI_D                  3.14159265358979323846264338327950288
#define PA_TWO_PI_D              (2.0 * PA_PI_D)

typedef struct {
    uint32_t bin;
    float frequency_hz;
    float amplitude_adc_v;
    float amplitude_corrected_v;
} PA_Peak;

typedef struct {
    uint8_t active;
    uint8_t column_count;
    uint8_t column_kind[PA_NUISANCE_MAX_COLUMNS];
    double frequency_hz;
    double column_scale[PA_NUISANCE_MAX_COLUMNS];
} PA_NuisanceModel;

static float s_centered[PERIODIC_ANALYZER_MAX_FFT_POINTS];

static float PA_AbsF(float x)
{
    return (x < 0.0f) ? -x : x;
}

static float PA_WrapPhase(float phase)
{
    while (phase > PA_PI) phase -= PA_TWO_PI;
    while (phase <= -PA_PI) phase += PA_TWO_PI;
    return phase;
}

static double PA_EvaluateAtRadians(const PeriodicAnalyzer_Result *result,
                                   double phase_rad)
{
    double value = 0.0;
    for (uint8_t c = 0U; c < result->component_count; ++c) {
        const PeriodicAnalyzer_Component *component = &result->component[c];
        value += (double)component->peak_amplitude_v *
                 sin((double)component->harmonic_order * phase_rad +
                     (double)component->phase_rad);
    }
    return value;
}

static double PA_DerivativeAtRadians(const PeriodicAnalyzer_Result *result,
                                     double phase_rad)
{
    double derivative = 0.0;
    for (uint8_t c = 0U; c < result->component_count; ++c) {
        const PeriodicAnalyzer_Component *component = &result->component[c];
        derivative +=
            (double)component->harmonic_order *
            (double)component->peak_amplitude_v *
            cos((double)component->harmonic_order * phase_rad +
                (double)component->phase_rad);
    }
    return derivative;
}

/*
 * Find every reconstructed-waveform extremum over one fundamental period.
 * A fixed phase grid alone can miss the peak of a high-order harmonic and
 * systematically under-report Vpp.  Bracketing derivative sign changes and
 * bisecting each root removes that grid-amplitude error.
 */
static float PA_ComputeVpp(const PeriodicAnalyzer_Result *result)
{
    uint16_t highest_order = 1U;
    uint32_t segments;
    double minimum = 1e300;
    double maximum = -1e300;
    double left_phase = 0.0;
    double left_derivative;

    for (uint8_t c = 0U; c < result->component_count; ++c) {
        if (result->component[c].harmonic_order > highest_order) {
            highest_order = result->component[c].harmonic_order;
        }
    }
    segments = (uint32_t)highest_order * PA_VPP_SEGMENTS_PER_ORDER;
    if (segments < PA_VPP_MIN_SEGMENTS) segments = PA_VPP_MIN_SEGMENTS;

    {
        const double value = PA_EvaluateAtRadians(result, 0.0);
        minimum = value;
        maximum = value;
    }
    left_derivative = PA_DerivativeAtRadians(result, left_phase);
    for (uint32_t i = 1U; i <= segments; ++i) {
        const double right_phase =
            PA_TWO_PI_D * (double)i / (double)segments;
        const double right_derivative =
            PA_DerivativeAtRadians(result, right_phase);

        if (fabs(left_derivative) <= 1e-18) {
            const double value =
                PA_EvaluateAtRadians(result, left_phase);
            if (value < minimum) minimum = value;
            if (value > maximum) maximum = value;
        } else if ((left_derivative < 0.0) !=
                   (right_derivative < 0.0)) {
            double lo = left_phase;
            double hi = right_phase;
            double dlo = left_derivative;
            for (uint32_t step = 0U;
                 step < PA_VPP_BISECTION_STEPS;
                 ++step) {
                const double mid = 0.5 * (lo + hi);
                const double dmid = PA_DerivativeAtRadians(result, mid);
                if ((dlo < 0.0) != (dmid < 0.0)) {
                    hi = mid;
                } else {
                    lo = mid;
                    dlo = dmid;
                }
            }
            {
                const double value =
                    PA_EvaluateAtRadians(result, 0.5 * (lo + hi));
                if (value < minimum) minimum = value;
                if (value > maximum) maximum = value;
            }
        }
        left_phase = right_phase;
        left_derivative = right_derivative;
    }
    return (float)(maximum - minimum);
}

static void PA_ClearResult(PeriodicAnalyzer_Result *result,
                           PeriodicAnalyzer_Status status)
{
    memset(result, 0, sizeof(*result));
    result->status = status;
}

/*
 * Compose two independent corrections:
 *   1. exact inverse of the declared deterministic digital response H(f);
 *   2. optional empirical calibration-table correction.
 *
 * The forward response is removed as 1/|H| and -arg(H).  The empirical
 * correction remains multiplicative/additive and therefore still stacks on
 * top when cal_count is non-zero.
 */
static uint8_t PA_GetCorrection(
    const PeriodicAnalyzer_Config *config,
    float frequency_hz,
    float *amplitude_correction,
    float *phase_correction_rad)
{
    float empirical_amp = 1.0f;
    float empirical_phase_deg = 0.0f;
    float digital_magnitude = 1.0f;
    float digital_phase_rad = 0.0f;
    float total_amp;
    float total_phase;

    if (config->cal_count != 0U) {
        uint32_t upper = 0U;
        if (frequency_hz <= config->cal[0].frequency_hz) {
            empirical_amp = config->cal[0].amplitude_correction;
            empirical_phase_deg = config->cal[0].phase_correction_deg;
        } else if (frequency_hz >=
                   config->cal[config->cal_count - 1U].frequency_hz) {
            empirical_amp =
                config->cal[config->cal_count - 1U].amplitude_correction;
            empirical_phase_deg =
                config->cal[config->cal_count - 1U].phase_correction_deg;
        } else {
            for (upper = 1U; upper < config->cal_count; ++upper) {
                if (frequency_hz <= config->cal[upper].frequency_hz) break;
            }
            if (upper < config->cal_count) {
                const PeriodicAnalyzer_CalPoint *lo = &config->cal[upper - 1U];
                const PeriodicAnalyzer_CalPoint *hi = &config->cal[upper];
                const float span = hi->frequency_hz - lo->frequency_hz;
                const float t = (span > 0.0f)
                    ? (frequency_hz - lo->frequency_hz) / span : 0.0f;
                empirical_amp = lo->amplitude_correction +
                    t * (hi->amplitude_correction -
                         lo->amplitude_correction);
                empirical_phase_deg = lo->phase_correction_deg +
                    t * (hi->phase_correction_deg -
                         lo->phase_correction_deg);
            }
        }
    }

    /* Preserve the legacy table behavior for an unfinished/disabled table. */
    if (!(empirical_amp > 0.0f) || !isfinite(empirical_amp)) {
        empirical_amp = 1.0f;
    }
    if (!isfinite(empirical_phase_deg)) empirical_phase_deg = 0.0f;

    if (config->digital_magnitude_response != NULL) {
        digital_magnitude =
            config->digital_magnitude_response(frequency_hz);
        if (!(digital_magnitude > 0.0f) ||
            !isfinite(digital_magnitude)) {
            return 0U;
        }
    }
    if (config->digital_phase_response_rad != NULL) {
        digital_phase_rad =
            config->digital_phase_response_rad(frequency_hz);
        if (!isfinite(digital_phase_rad)) return 0U;
    }

    total_amp = empirical_amp / digital_magnitude;
    total_phase =
        empirical_phase_deg * (PA_PI / 180.0f) - digital_phase_rad;
    if (!(total_amp > 0.0f) || !isfinite(total_amp) ||
        !isfinite(total_phase)) {
        return 0U;
    }

    if (amplitude_correction != NULL) {
        *amplitude_correction = total_amp;
    }
    if (phase_correction_rad != NULL) {
        *phase_correction_rad = total_phase;
    }
    return 1U;
}

static void PA_InterpPeak(const float *mag,
                          uint32_t n,
                          uint32_t bin,
                          float fs,
                          PA_Peak *peak)
{
    float delta = 0.0f;
    float amplitude = mag[bin];

    if ((bin > 0U) && (bin + 1U < n / 2U)) {
        const float left = mag[bin - 1U];
        const float center = mag[bin];
        const float right = mag[bin + 1U];
        const float denominator = left - 2.0f * center + right;
        if (PA_AbsF(denominator) > 1e-20f) {
            delta = 0.5f * (left - right) / denominator;
            if (delta > 0.5f) delta = 0.5f;
            if (delta < -0.5f) delta = -0.5f;
            amplitude = center - 0.25f * (left - right) * delta;
        }
    }

    peak->bin = bin;
    peak->frequency_hz = ((float)bin + delta) * fs / (float)n;
    peak->amplitude_adc_v = amplitude;
}

static uint8_t PA_CollectPeaks(const PeriodicAnalyzer_Config *config,
                               uint32_t n,
                               PA_Peak peaks[PA_MAX_PEAKS],
                               uint8_t *response_error)
{
    PA_Peak local[PA_MAX_PEAKS * 3U];
    uint8_t local_count = 0U;
    uint8_t selected_count = 0U;
    const float fs = config->sample_rate_hz;
    const float df = fs / (float)n;
    uint32_t min_bin = (uint32_t)floorf(config->min_frequency_hz / df);
    uint32_t max_bin = (uint32_t)ceilf(config->max_frequency_hz / df);
    float global_max_corrected_v = 0.0f;

    if (response_error != NULL) *response_error = 0U;
    if (min_bin < 2U) min_bin = 2U;
    if (max_bin >= n / 2U) max_bin = n / 2U - 1U;

    /*
     * 先保留幅值最大的若干局部峰。插入排序使 local 始终按幅值降序，
     * 可避免为 4096 点频谱额外分配大数组。
     */
    for (uint32_t k = min_bin; k <= max_bin; ++k) {
        const float v = FFT_normalized_output[k];
        PA_Peak interpolated;
        float amp_corr;
        uint8_t pos;
        if (!isfinite(v) ||
            (v < FFT_normalized_output[k - 1U]) ||
            (v <= FFT_normalized_output[k + 1U])) {
            continue;
        }

        PA_InterpPeak(FFT_normalized_output, n, k, fs, &interpolated);
        if ((interpolated.frequency_hz <
             config->min_frequency_hz -
             PA_BAND_EDGE_TOL_BINS * df) ||
            (interpolated.frequency_hz >
             config->max_frequency_hz +
             PA_BAND_EDGE_TOL_BINS * df)) {
            continue;
        }
        /*
         * A tone exactly on a configured band edge can interpolate a few Hz
         * outside the range. Keep that edge tone and report the legal edge.
         */
        if (interpolated.frequency_hz < config->min_frequency_hz) {
            interpolated.frequency_hz = config->min_frequency_hz;
        }
        if (interpolated.frequency_hz > config->max_frequency_hz) {
            interpolated.frequency_hz = config->max_frequency_hz;
        }
        if (PA_GetCorrection(config,
                             interpolated.frequency_hz,
                             &amp_corr,
                             NULL) == 0U) {
            if (response_error != NULL) *response_error = 1U;
            return 0U;
        }
        interpolated.amplitude_corrected_v =
            interpolated.amplitude_adc_v *
            config->input_volts_per_adc_volt *
            amp_corr;
        if (interpolated.amplitude_corrected_v >
            global_max_corrected_v) {
            global_max_corrected_v =
                interpolated.amplitude_corrected_v;
        }

        pos = local_count;
        if (pos > (uint8_t)(PA_MAX_PEAKS * 3U - 1U)) {
            pos = (uint8_t)(PA_MAX_PEAKS * 3U - 1U);
        }
        while ((pos > 0U) &&
               (local[pos - 1U].amplitude_corrected_v <
                interpolated.amplitude_corrected_v)) {
            if (pos < (uint8_t)(PA_MAX_PEAKS * 3U)) {
                local[pos] = local[pos - 1U];
            }
            --pos;
        }
        if (pos < (uint8_t)(PA_MAX_PEAKS * 3U)) {
            local[pos] = interpolated;
            if (local_count < (uint8_t)(PA_MAX_PEAKS * 3U)) local_count++;
        }
    }

    if (!(global_max_corrected_v > 0.0f)) return 0U;

    for (uint8_t i = 0U; i < local_count; ++i) {
        uint8_t separated = 1U;
        const PA_Peak *candidate = &local[i];

        if ((candidate->amplitude_corrected_v <
             global_max_corrected_v *
             config->relative_peak_threshold) ||
            (candidate->amplitude_corrected_v <
             config->min_component_peak_v)) {
            continue;
        }

        for (uint8_t j = 0U; j < selected_count; ++j) {
            const int32_t distance = (int32_t)candidate->bin -
                                     (int32_t)peaks[j].bin;
            if ((distance >= -(int32_t)PA_PEAK_EXCLUSION_BINS) &&
                (distance <= (int32_t)PA_PEAK_EXCLUSION_BINS)) {
                separated = 0U;
                break;
            }
        }
        if (separated != 0U) {
            peaks[selected_count++] = *candidate;
            if (selected_count >= PA_MAX_PEAKS) break;
        }
    }
    return selected_count;
}

static float PA_HarmonicMatchErrorBins(const PA_Peak *peak,
                                       float f0,
                                       float df,
                                       uint16_t *order)
{
    float ratio;
    uint16_t h;
    if (!(f0 > 0.0f)) return 1e30f;
    ratio = peak->frequency_hz / f0;
    h = (uint16_t)(ratio + 0.5f);
    if (h == 0U) h = 1U;
    if (order != NULL) *order = h;
    return PA_AbsF(peak->frequency_hz - (float)h * f0) / df;
}

static uint8_t PA_SelectHarmonicModel(const PA_Peak *peaks,
                                      uint8_t peak_count,
                                      float df,
                                      PA_Peak selected[
                                          PERIODIC_ANALYZER_MAX_COMPONENTS],
                                      uint16_t orders[
                                          PERIODIC_ANALYZER_MAX_COMPONENTS],
                                      float *f0_out)
{
    int best_candidate = -1;
    float best_score = -1.0f;
    uint8_t best_count = 0U;

    for (uint8_t c = 0U; c < peak_count; ++c) {
        float matched_amp[PERIODIC_ANALYZER_MAX_COMPONENTS] = {0.0f};
        uint8_t matched_count = 0U;
        float score;

        matched_amp[matched_count++] = peaks[c].amplitude_corrected_v;
        for (uint8_t i = 0U; i < peak_count; ++i) {
            uint16_t order;
            float error_bins;
            float amp;
            uint8_t pos;
            if (i == c) continue;
            error_bins = PA_HarmonicMatchErrorBins(&peaks[i],
                                                   peaks[c].frequency_hz,
                                                   df,
                                                   &order);
            if ((order < 2U) || (error_bins > PA_HARMONIC_TOL_BINS)) {
                continue;
            }

            amp = peaks[i].amplitude_corrected_v;
            pos = matched_count;
            if (pos >= PERIODIC_ANALYZER_MAX_COMPONENTS) {
                pos = PERIODIC_ANALYZER_MAX_COMPONENTS - 1U;
            }
            while ((pos > 1U) && (matched_amp[pos - 1U] < amp)) {
                if (pos < PERIODIC_ANALYZER_MAX_COMPONENTS) {
                    matched_amp[pos] = matched_amp[pos - 1U];
                }
                --pos;
            }
            if (pos < PERIODIC_ANALYZER_MAX_COMPONENTS) {
                matched_amp[pos] = amp;
                if (matched_count < PERIODIC_ANALYZER_MAX_COMPONENTS) {
                    matched_count++;
                }
            }
        }

        score = 0.0f;
        for (uint8_t i = 0U; i < matched_count; ++i) score += matched_amp[i];
        score *= 1.0f + 0.08f * (float)(matched_count - 1U);

        if ((score > best_score * 1.02f) ||
            ((best_candidate >= 0) &&
             (score >= best_score * 0.98f) &&
             (matched_count > best_count)) ||
            ((best_candidate >= 0) &&
             (score >= best_score * 0.98f) &&
             (matched_count == best_count) &&
             (peaks[c].frequency_hz <
              peaks[(uint8_t)best_candidate].frequency_hz))) {
            best_score = score;
            best_candidate = (int)c;
            best_count = matched_count;
        }
    }

    if (best_candidate < 0) return 0U;

    {
        const float candidate_f0 =
            peaks[(uint8_t)best_candidate].frequency_hz;
        uint8_t selected_count = 0U;

        for (uint8_t i = 0U; i < peak_count; ++i) {
            uint16_t order;
            const float error_bins = PA_HarmonicMatchErrorBins(
                &peaks[i], candidate_f0, df, &order);
            if ((error_bins > PA_HARMONIC_TOL_BINS) ||
                (order == 0U)) {
                continue;
            }

            if (selected_count < PERIODIC_ANALYZER_MAX_COMPONENTS) {
                selected[selected_count] = peaks[i];
                orders[selected_count] = order;
                selected_count++;
            } else {
                uint8_t weakest = 0U;
                for (uint8_t j = 1U; j < selected_count; ++j) {
                    if (selected[j].amplitude_corrected_v <
                        selected[weakest].amplitude_corrected_v) {
                        weakest = j;
                    }
                }
                if (peaks[i].amplitude_corrected_v >
                    selected[weakest].amplitude_corrected_v) {
                    selected[weakest] = peaks[i];
                    orders[weakest] = order;
                }
            }
        }

        /* 基波必须保留，即使它不是幅值最大的分量。 */
        {
            uint8_t has_fundamental = 0U;
            for (uint8_t i = 0U; i < selected_count; ++i) {
                if (orders[i] == 1U) has_fundamental = 1U;
            }
            if (has_fundamental == 0U) {
                uint8_t weakest = 0U;
                for (uint8_t i = 1U; i < selected_count; ++i) {
                    if (selected[i].amplitude_corrected_v <
                        selected[weakest].amplitude_corrected_v) {
                        weakest = i;
                    }
                }
                selected[weakest] = peaks[(uint8_t)best_candidate];
                orders[weakest] = 1U;
            }
        }

        /* 按谐波次数排序，便于显示和后续联合拟合。 */
        for (uint8_t i = 0U; i < selected_count; ++i) {
            for (uint8_t j = (uint8_t)(i + 1U);
                 j < selected_count; ++j) {
                if (orders[j] < orders[i]) {
                    const PA_Peak p = selected[i];
                    const uint16_t h = orders[i];
                    selected[i] = selected[j];
                    orders[i] = orders[j];
                    selected[j] = p;
                    orders[j] = h;
                }
            }
        }

        {
            double weighted_f0 = 0.0;
            double weight_sum = 0.0;
            for (uint8_t i = 0U; i < selected_count; ++i) {
                const double w =
                    (double)selected[i].amplitude_corrected_v *
                    (double)selected[i].amplitude_corrected_v;
                weighted_f0 += w *
                    ((double)selected[i].frequency_hz / (double)orders[i]);
                weight_sum += w;
            }
            *f0_out = (weight_sum > 0.0)
                ? (float)(weighted_f0 / weight_sum) : candidate_f0;
        }
        return selected_count;
    }
}

/*
 * Build a numerically stable, centered nuisance basis.  Centering makes the
 * sine and cosine columns orthogonal by symmetry.  Each retained column is
 * RMS-normalized to the same scale as an ordinary sinusoid, preventing the
 * near-Nyquist sine column from making the normal equations ill-conditioned.
 *
 * At exact Nyquist one centered quadrature has zero energy.  It is omitted,
 * leaving the only observable alternating sequence as a single basis column.
 */
static int PA_PrepareNuisanceModel(uint32_t n,
                                   float fs,
                                   double frequency_hz,
                                   PA_NuisanceModel *model)
{
    double oscillator_sin;
    double oscillator_cos;
    double step_sin;
    double step_cos;
    double sine_energy = 0.0;
    double cosine_energy = 0.0;
    double energy[PA_NUISANCE_MAX_COLUMNS];
    uint8_t kind[PA_NUISANCE_MAX_COLUMNS];
    const double omega =
        PA_TWO_PI_D * frequency_hz / (double)fs;
    const double centered_phase =
        -omega * 0.5 * (double)(n - 1U);

    if ((model == NULL) || (n == 0U) ||
        !(fs > 0.0f) || !(frequency_hz > 0.0) ||
        !(frequency_hz <= 0.5 * (double)fs) ||
        !isfinite(frequency_hz)) {
        return -1;
    }

    memset(model, 0, sizeof(*model));
    oscillator_sin = sin(centered_phase);
    oscillator_cos = cos(centered_phase);
    step_sin = sin(omega);
    step_cos = cos(omega);
    for (uint32_t sample = 0U; sample < n; ++sample) {
        sine_energy += oscillator_sin * oscillator_sin;
        cosine_energy += oscillator_cos * oscillator_cos;
        {
            const double next_sin =
                oscillator_sin * step_cos +
                oscillator_cos * step_sin;
            const double next_cos =
                oscillator_cos * step_cos -
                oscillator_sin * step_sin;
            oscillator_sin = next_sin;
            oscillator_cos = next_cos;
        }
    }

    if (cosine_energy >= sine_energy) {
        energy[0] = cosine_energy;
        kind[0] = PA_NUISANCE_COLUMN_COSINE;
        energy[1] = sine_energy;
        kind[1] = PA_NUISANCE_COLUMN_SINE;
    } else {
        energy[0] = sine_energy;
        kind[0] = PA_NUISANCE_COLUMN_SINE;
        energy[1] = cosine_energy;
        kind[1] = PA_NUISANCE_COLUMN_COSINE;
    }
    if (!(energy[0] > 0.0) || !isfinite(energy[0])) return -1;

    model->active = 1U;
    model->frequency_hz = frequency_hz;
    model->column_count = 1U;
    model->column_kind[0] = kind[0];
    model->column_scale[0] =
        sqrt(2.0 * energy[0] / (double)n);
    if ((energy[1] / (double)n) >=
        PA_NUISANCE_MIN_COLUMN_MEAN_SQ) {
        model->column_count = 2U;
        model->column_kind[1] = kind[1];
        model->column_scale[1] =
            sqrt(2.0 * energy[1] / (double)n);
    }
    return 0;
}

static int PA_Solve(double a[PA_SOLVER_MAX_ORDER]
                            [PA_SOLVER_MAX_ORDER + 1U],
                    uint32_t order,
                    double x[PA_SOLVER_MAX_ORDER])
{
    for (uint32_t col = 0U; col < order; ++col) {
        uint32_t pivot = col;
        double pivot_abs = fabs(a[col][col]);
        for (uint32_t row = col + 1U; row < order; ++row) {
            const double v = fabs(a[row][col]);
            if (v > pivot_abs) {
                pivot_abs = v;
                pivot = row;
            }
        }
        if (!(pivot_abs > 1e-12)) return -1;
        if (pivot != col) {
            for (uint32_t j = col; j <= order; ++j) {
                const double tmp = a[col][j];
                a[col][j] = a[pivot][j];
                a[pivot][j] = tmp;
            }
        }
        for (uint32_t row = col + 1U; row < order; ++row) {
            const double factor = a[row][col] / a[col][col];
            for (uint32_t j = col; j <= order; ++j) {
                a[row][j] -= factor * a[col][j];
            }
        }
    }

    for (int32_t row = (int32_t)order - 1; row >= 0; --row) {
        double rhs = a[(uint32_t)row][order];
        for (uint32_t j = (uint32_t)row + 1U; j < order; ++j) {
            rhs -= a[(uint32_t)row][j] * x[j];
        }
        x[(uint32_t)row] = rhs / a[(uint32_t)row][(uint32_t)row];
    }
    return 0;
}

static int PA_FitComponents(const float *data,
                            uint32_t n,
                            float fs,
                            const uint16_t *orders,
                            uint8_t component_count,
                            double f0,
                            const PA_NuisanceModel *nuisance,
                            double solution[PA_SOLVER_MAX_ORDER],
                            double *sse)
{
    double normal[PA_SOLVER_MAX_ORDER][PA_SOLVER_MAX_ORDER + 1U] = {{0.0}};
    double x_transpose_y[PA_SOLVER_MAX_ORDER] = {0.0};
    double oscillator_sin[PERIODIC_ANALYZER_MAX_COMPONENTS] = {0.0};
    double oscillator_cos[PERIODIC_ANALYZER_MAX_COMPONENTS] = {0.0};
    double step_sin[PERIODIC_ANALYZER_MAX_COMPONENTS] = {0.0};
    double step_cos[PERIODIC_ANALYZER_MAX_COMPONENTS] = {0.0};
    double nuisance_sin = 0.0;
    double nuisance_cos = 1.0;
    double nuisance_step_sin = 0.0;
    double nuisance_step_cos = 1.0;
    double row[PA_SOLVER_MAX_ORDER];
    double y_energy = 0.0;
    const uint8_t nuisance_columns =
        ((nuisance != NULL) && (nuisance->active != 0U))
        ? nuisance->column_count : 0U;
    const uint32_t nuisance_offset = 1U + 2U * component_count;
    const uint32_t model_order =
        nuisance_offset + (uint32_t)nuisance_columns;

    for (uint8_t c = 0U; c < component_count; ++c) {
        const double omega =
            PA_TWO_PI_D * (double)orders[c] * f0 / (double)fs;
        oscillator_sin[c] = 0.0;
        oscillator_cos[c] = 1.0;
        step_sin[c] = sin(omega);
        step_cos[c] = cos(omega);
    }
    if (nuisance_columns != 0U) {
        const double omega =
            PA_TWO_PI_D * nuisance->frequency_hz / (double)fs;
        const double centered_phase =
            -omega * 0.5 * (double)(n - 1U);
        nuisance_sin = sin(centered_phase);
        nuisance_cos = cos(centered_phase);
        nuisance_step_sin = sin(omega);
        nuisance_step_cos = cos(omega);
    }

    for (uint32_t sample = 0U; sample < n; ++sample) {
        const double y = (double)data[sample];
        row[0] = 1.0;
        for (uint8_t c = 0U; c < component_count; ++c) {
            row[1U + 2U * c] = oscillator_sin[c];
            row[2U + 2U * c] = oscillator_cos[c];
        }
        for (uint8_t c = 0U; c < nuisance_columns; ++c) {
            const double raw =
                (nuisance->column_kind[c] ==
                 PA_NUISANCE_COLUMN_COSINE)
                ? nuisance_cos : nuisance_sin;
            row[nuisance_offset + c] =
                raw / nuisance->column_scale[c];
        }
        for (uint32_t r = 0U; r < model_order; ++r) {
            for (uint32_t c = r; c < model_order; ++c) {
                normal[r][c] += row[r] * row[c];
            }
            x_transpose_y[r] += row[r] * y;
        }
        y_energy += y * y;
        for (uint8_t c = 0U; c < component_count; ++c) {
            const double next_sin =
                oscillator_sin[c] * step_cos[c] +
                oscillator_cos[c] * step_sin[c];
            const double next_cos =
                oscillator_cos[c] * step_cos[c] -
                oscillator_sin[c] * step_sin[c];
            oscillator_sin[c] = next_sin;
            oscillator_cos[c] = next_cos;
        }
        if (nuisance_columns != 0U) {
            const double next_sin =
                nuisance_sin * nuisance_step_cos +
                nuisance_cos * nuisance_step_sin;
            const double next_cos =
                nuisance_cos * nuisance_step_cos -
                nuisance_sin * nuisance_step_sin;
            nuisance_sin = next_sin;
            nuisance_cos = next_cos;
        }
    }

    for (uint32_t r = 0U; r < model_order; ++r) {
        for (uint32_t c = 0U; c < r; ++c) {
            normal[r][c] = normal[c][r];
        }
        normal[r][model_order] = x_transpose_y[r];
    }
    if (PA_Solve(normal, model_order, solution) != 0) return -1;

    if (sse != NULL) {
        double projected_energy = 0.0;
        double value;
        for (uint32_t r = 0U; r < model_order; ++r) {
            projected_energy += solution[r] * x_transpose_y[r];
        }
        value = y_energy - projected_energy;
        if ((value < 0.0) &&
            (value > -1e-12 * (1.0 + y_energy))) {
            value = 0.0;
        }
        if (!(value >= 0.0) || !isfinite(value)) return -1;
        *sse = value;
    }
    return 0;
}

static double PA_ComputeResidualSse(
    const float *data,
    uint32_t n,
    float fs,
    const uint16_t *orders,
    uint8_t component_count,
    double f0,
    const PA_NuisanceModel *nuisance,
    const double solution[PA_SOLVER_MAX_ORDER])
{
    double oscillator_sin[PERIODIC_ANALYZER_MAX_COMPONENTS] = {0.0};
    double oscillator_cos[PERIODIC_ANALYZER_MAX_COMPONENTS] = {0.0};
    double step_sin[PERIODIC_ANALYZER_MAX_COMPONENTS] = {0.0};
    double step_cos[PERIODIC_ANALYZER_MAX_COMPONENTS] = {0.0};
    double nuisance_sin = 0.0;
    double nuisance_cos = 1.0;
    double nuisance_step_sin = 0.0;
    double nuisance_step_cos = 1.0;
    double sum_sq = 0.0;
    const uint8_t nuisance_columns =
        ((nuisance != NULL) && (nuisance->active != 0U))
        ? nuisance->column_count : 0U;
    const uint32_t nuisance_offset = 1U + 2U * component_count;

    for (uint8_t c = 0U; c < component_count; ++c) {
        const double omega =
            PA_TWO_PI_D * (double)orders[c] * f0 / (double)fs;
        oscillator_cos[c] = 1.0;
        step_sin[c] = sin(omega);
        step_cos[c] = cos(omega);
    }
    if (nuisance_columns != 0U) {
        const double omega =
            PA_TWO_PI_D * nuisance->frequency_hz / (double)fs;
        const double centered_phase =
            -omega * 0.5 * (double)(n - 1U);
        nuisance_sin = sin(centered_phase);
        nuisance_cos = cos(centered_phase);
        nuisance_step_sin = sin(omega);
        nuisance_step_cos = cos(omega);
    }

    for (uint32_t sample = 0U; sample < n; ++sample) {
        double estimate = solution[0];
        for (uint8_t c = 0U; c < component_count; ++c) {
            estimate += solution[1U + 2U * c] * oscillator_sin[c] +
                        solution[2U + 2U * c] * oscillator_cos[c];
        }
        for (uint8_t c = 0U; c < nuisance_columns; ++c) {
            const double raw =
                (nuisance->column_kind[c] ==
                 PA_NUISANCE_COLUMN_COSINE)
                ? nuisance_cos : nuisance_sin;
            estimate += solution[nuisance_offset + c] *
                        raw / nuisance->column_scale[c];
        }
        {
            const double error = (double)data[sample] - estimate;
            sum_sq += error * error;
        }
        for (uint8_t c = 0U; c < component_count; ++c) {
            const double next_sin =
                oscillator_sin[c] * step_cos[c] +
                oscillator_cos[c] * step_sin[c];
            const double next_cos =
                oscillator_cos[c] * step_cos[c] -
                oscillator_sin[c] * step_sin[c];
            oscillator_sin[c] = next_sin;
            oscillator_cos[c] = next_cos;
        }
        if (nuisance_columns != 0U) {
            const double next_sin =
                nuisance_sin * nuisance_step_cos +
                nuisance_cos * nuisance_step_sin;
            const double next_cos =
                nuisance_cos * nuisance_step_cos -
                nuisance_sin * nuisance_step_sin;
            nuisance_sin = next_sin;
            nuisance_cos = next_cos;
        }
    }
    return sum_sq;
}

static int PA_FitCost(const float *data,
                      uint32_t n,
                      float fs,
                      const uint16_t *orders,
                      uint8_t component_count,
                      double f0,
                      const PA_NuisanceModel *nuisance,
                      double *cost)
{
    double trial_solution[PA_SOLVER_MAX_ORDER] = {0.0};
    return PA_FitComponents(data, n, fs, orders, component_count,
                            f0, nuisance, trial_solution, cost);
}

static double PA_RefineFundamentalStep(const float *data,
                                       uint32_t n,
                                       float fs,
                                       const uint16_t *orders,
                                       uint8_t component_count,
                                       const PA_NuisanceModel *nuisance,
                                       double center_hz,
                                       double step_hz,
                                       double min_hz,
                                       double max_hz)
{
    double left_hz;
    double right_hz;
    double left_cost = 0.0;
    double center_cost;
    double right_cost = 0.0;
    uint8_t left_valid;
    uint8_t right_valid;

    if (!(step_hz > 0.0) || !(max_hz >= min_hz)) return center_hz;
    if (center_hz < min_hz) center_hz = min_hz;
    if (center_hz > max_hz) center_hz = max_hz;
    if (PA_FitCost(data, n, fs, orders, component_count,
                   center_hz, nuisance, &center_cost) != 0) {
        return center_hz;
    }

    left_hz = center_hz - step_hz;
    if (left_hz < min_hz) left_hz = min_hz;
    right_hz = center_hz + step_hz;
    if (right_hz > max_hz) right_hz = max_hz;
    left_valid = (left_hz < center_hz) ? 1U : 0U;
    right_valid = (right_hz > center_hz) ? 1U : 0U;
    if ((left_valid != 0U) &&
        (PA_FitCost(data, n, fs, orders, component_count,
                    left_hz, nuisance, &left_cost) != 0)) {
        left_valid = 0U;
    }
    if ((right_valid != 0U) &&
        (PA_FitCost(data, n, fs, orders, component_count,
                    right_hz, nuisance, &right_cost) != 0)) {
        right_valid = 0U;
    }

    for (uint8_t shift = 0U;
         shift < PA_REFINE_MAX_SHIFTS;
         ++shift) {
        if ((left_valid != 0U) &&
            (left_cost < center_cost) &&
            ((right_valid == 0U) || (left_cost <= right_cost))) {
            right_hz = center_hz;
            right_cost = center_cost;
            right_valid = 1U;
            center_hz = left_hz;
            center_cost = left_cost;
            left_hz = center_hz - step_hz;
            if (left_hz < min_hz) left_hz = min_hz;
            left_valid = (left_hz < center_hz) ? 1U : 0U;
            if ((left_valid != 0U) &&
                (PA_FitCost(data, n, fs, orders, component_count,
                            left_hz, nuisance, &left_cost) != 0)) {
                left_valid = 0U;
            }
        } else if ((right_valid != 0U) &&
                   (right_cost < center_cost)) {
            left_hz = center_hz;
            left_cost = center_cost;
            left_valid = 1U;
            center_hz = right_hz;
            center_cost = right_cost;
            right_hz = center_hz + step_hz;
            if (right_hz > max_hz) right_hz = max_hz;
            right_valid = (right_hz > center_hz) ? 1U : 0U;
            if ((right_valid != 0U) &&
                (PA_FitCost(data, n, fs, orders, component_count,
                            right_hz, nuisance, &right_cost) != 0)) {
                right_valid = 0U;
            }
        } else {
            break;
        }
    }

    if ((left_valid != 0U) && (right_valid != 0U) &&
        (center_cost <= left_cost) &&
        (center_cost <= right_cost)) {
        const double slope_left =
            (center_cost - left_cost) / (center_hz - left_hz);
        const double slope_right =
            (right_cost - center_cost) / (right_hz - center_hz);
        const double curvature =
            (slope_right - slope_left) / (right_hz - left_hz);
        if (curvature > 0.0) {
            const double linear =
                slope_left - curvature * (left_hz + center_hz);
            const double vertex = -linear / (2.0 * curvature);
            if ((vertex >= left_hz) && (vertex <= right_hz) &&
                isfinite(vertex)) {
                return vertex;
            }
        }
    }
    return center_hz;
}

static float PA_RefineFundamental(const float *data,
                                  uint32_t n,
                                  const PeriodicAnalyzer_Config *config,
                                  const uint16_t *orders,
                                  uint8_t component_count,
                                  const PA_NuisanceModel *nuisance,
                                  float initial_hz)
{
    const double bin_width =
        (double)config->sample_rate_hz / (double)n;
    double min_hz = (double)config->min_frequency_hz;
    double max_hz = (double)config->max_frequency_hz;
    double refined = (double)initial_hz;
    double step_hz = bin_width / 8.0;

    for (uint8_t c = 0U; c < component_count; ++c) {
        const double band_limit =
            (double)config->max_frequency_hz / (double)orders[c];
        const double nyquist_limit =
            (0.5 * (double)config->sample_rate_hz -
             0.5 * bin_width) / (double)orders[c];
        if (band_limit < max_hz) max_hz = band_limit;
        if (nyquist_limit < max_hz) max_hz = nyquist_limit;
    }
    if (!(max_hz >= min_hz)) return initial_hz;
    if (refined < min_hz) refined = min_hz;
    if (refined > max_hz) refined = max_hz;

    for (uint8_t stage = 0U; stage < PA_REFINE_STAGES; ++stage) {
        refined = PA_RefineFundamentalStep(data,
                                           n,
                                           config->sample_rate_hz,
                                           orders,
                                           component_count,
                                           nuisance,
                                           refined,
                                           step_hz,
                                           min_hz,
                                           max_hz);
        step_hz *= 0.25;
    }
    return (float)refined;
}

static int PA_NuisanceFitAtFrequency(
    const float *data,
    uint32_t n,
    float fs,
    double frequency_hz,
    PA_NuisanceModel *model,
    double solution[PA_SOLVER_MAX_ORDER],
    double *sse)
{
    if (PA_PrepareNuisanceModel(n, fs, frequency_hz, model) != 0) {
        return -1;
    }
    return PA_FitComponents(data,
                            n,
                            fs,
                            NULL,
                            0U,
                            0.0,
                            model,
                            solution,
                            sse);
}

static int PA_NuisanceFitCost(const float *data,
                              uint32_t n,
                              float fs,
                              double frequency_hz,
                              double *cost)
{
    PA_NuisanceModel trial_model;
    double trial_solution[PA_SOLVER_MAX_ORDER] = {0.0};
    return PA_NuisanceFitAtFrequency(data,
                                     n,
                                     fs,
                                     frequency_hz,
                                     &trial_model,
                                     trial_solution,
                                     cost);
}

static double PA_RefineNuisanceStep(const float *data,
                                    uint32_t n,
                                    float fs,
                                    double center_hz,
                                    double step_hz,
                                    double min_hz,
                                    double max_hz)
{
    double left_hz;
    double right_hz;
    double left_cost = 0.0;
    double center_cost;
    double right_cost = 0.0;
    uint8_t left_valid;
    uint8_t right_valid;

    if (!(step_hz > 0.0) || !(max_hz >= min_hz)) return center_hz;
    if (center_hz < min_hz) center_hz = min_hz;
    if (center_hz > max_hz) center_hz = max_hz;
    if (PA_NuisanceFitCost(data, n, fs,
                           center_hz, &center_cost) != 0) {
        return center_hz;
    }

    left_hz = center_hz - step_hz;
    if (left_hz < min_hz) left_hz = min_hz;
    right_hz = center_hz + step_hz;
    if (right_hz > max_hz) right_hz = max_hz;
    left_valid = (left_hz < center_hz) ? 1U : 0U;
    right_valid = (right_hz > center_hz) ? 1U : 0U;
    if ((left_valid != 0U) &&
        (PA_NuisanceFitCost(data, n, fs,
                            left_hz, &left_cost) != 0)) {
        left_valid = 0U;
    }
    if ((right_valid != 0U) &&
        (PA_NuisanceFitCost(data, n, fs,
                            right_hz, &right_cost) != 0)) {
        right_valid = 0U;
    }

    for (uint8_t shift = 0U;
         shift < PA_REFINE_MAX_SHIFTS;
         ++shift) {
        if ((left_valid != 0U) &&
            (left_cost < center_cost) &&
            ((right_valid == 0U) || (left_cost <= right_cost))) {
            right_hz = center_hz;
            right_cost = center_cost;
            right_valid = 1U;
            center_hz = left_hz;
            center_cost = left_cost;
            left_hz = center_hz - step_hz;
            if (left_hz < min_hz) left_hz = min_hz;
            left_valid = (left_hz < center_hz) ? 1U : 0U;
            if ((left_valid != 0U) &&
                (PA_NuisanceFitCost(data, n, fs,
                                    left_hz, &left_cost) != 0)) {
                left_valid = 0U;
            }
        } else if ((right_valid != 0U) &&
                   (right_cost < center_cost)) {
            left_hz = center_hz;
            left_cost = center_cost;
            left_valid = 1U;
            center_hz = right_hz;
            center_cost = right_cost;
            right_hz = center_hz + step_hz;
            if (right_hz > max_hz) right_hz = max_hz;
            right_valid = (right_hz > center_hz) ? 1U : 0U;
            if ((right_valid != 0U) &&
                (PA_NuisanceFitCost(data, n, fs,
                                    right_hz, &right_cost) != 0)) {
                right_valid = 0U;
            }
        } else {
            break;
        }
    }

    if ((left_valid != 0U) && (right_valid != 0U) &&
        (center_cost <= left_cost) &&
        (center_cost <= right_cost)) {
        const double slope_left =
            (center_cost - left_cost) / (center_hz - left_hz);
        const double slope_right =
            (right_cost - center_cost) / (right_hz - center_hz);
        const double curvature =
            (slope_right - slope_left) / (right_hz - left_hz);
        if (curvature > 0.0) {
            const double linear =
                slope_left - curvature * (left_hz + center_hz);
            const double vertex = -linear / (2.0 * curvature);
            if ((vertex >= left_hz) && (vertex <= right_hz) &&
                isfinite(vertex)) {
                return vertex;
            }
        }
    }
    return center_hz;
}

static void PA_NuisanceDiagnostics(
    const PA_NuisanceModel *model,
    uint8_t component_count,
    const double solution[PA_SOLVER_MAX_ORDER],
    float *peak_adc_v,
    float *rms_adc_v)
{
    double cosine_coefficient = 0.0;
    double sine_coefficient = 0.0;
    double normalized_energy = 0.0;
    const uint32_t offset = 1U + 2U * component_count;

    if ((model == NULL) || (model->active == 0U)) {
        if (peak_adc_v != NULL) *peak_adc_v = 0.0f;
        if (rms_adc_v != NULL) *rms_adc_v = 0.0f;
        return;
    }
    for (uint8_t c = 0U; c < model->column_count; ++c) {
        const double normalized_coefficient = solution[offset + c];
        const double raw_coefficient =
            normalized_coefficient / model->column_scale[c];
        if (model->column_kind[c] ==
            PA_NUISANCE_COLUMN_COSINE) {
            cosine_coefficient = raw_coefficient;
        } else {
            sine_coefficient = raw_coefficient;
        }
        normalized_energy +=
            normalized_coefficient * normalized_coefficient;
    }
    if (peak_adc_v != NULL) {
        *peak_adc_v = (float)hypot(cosine_coefficient,
                                   sine_coefficient);
    }
    if (rms_adc_v != NULL) {
        /*
         * Every retained column is scaled to mean-square 1/2 and the
         * centered sine/cosine pair is orthogonal.
         */
        *rms_adc_v = (float)sqrt(0.5 * normalized_energy);
    }
}

static int PA_DetectInterference(
    const float *data,
    uint32_t n,
    const PeriodicAnalyzer_Config *config,
    PA_NuisanceModel *model,
    float *peak_adc_v,
    float *rms_adc_v)
{
    const double fs = (double)config->sample_rate_hz;
    const double nyquist_hz = 0.5 * fs;
    const double df = fs / (double)n;
    double min_hz =
        (double)config->interference_expected_frequency_hz -
        (double)config->interference_search_half_width_hz;
    double max_hz =
        (double)config->interference_expected_frequency_hz +
        (double)config->interference_search_half_width_hz;
    uint32_t min_bin;
    uint32_t max_bin;
    uint32_t best_bin;
    float best_magnitude;
    double refined_hz;
    double step_hz;
    double solution[PA_SOLVER_MAX_ORDER] = {0.0};
    double sse;
    uint8_t nyquist_edge_tone = 0U;

    memset(model, 0, sizeof(*model));
    if (peak_adc_v != NULL) *peak_adc_v = 0.0f;
    if (rms_adc_v != NULL) *rms_adc_v = 0.0f;
    if (config->interference_rejection_enable == 0U) return 0;

    if (min_hz < df) min_hz = df;
    if (max_hz > nyquist_hz) max_hz = nyquist_hz;
    if (!(max_hz >= min_hz)) return -1;
    min_bin = (uint32_t)ceil(min_hz / df);
    max_bin = (uint32_t)floor(max_hz / df);
    if (max_bin > n / 2U) max_bin = n / 2U;
    if ((min_bin > max_bin) || (max_bin == 0U)) return -1;

    best_bin = min_bin;
    best_magnitude = FFT_normalized_output[min_bin];
    for (uint32_t bin = min_bin + 1U; bin <= max_bin; ++bin) {
        if (FFT_normalized_output[bin] > best_magnitude) {
            best_magnitude = FFT_normalized_output[bin];
            best_bin = bin;
        }
    }
    if (!isfinite(best_magnitude)) return -1;
    if ((max_bin == n / 2U) && (n / 2U > 0U)) {
        const double edge =
            (double)FFT_normalized_output[n / 2U];
        const double adjacent =
            (double)FFT_normalized_output[n / 2U - 1U];
        const double scale =
            (edge > adjacent) ? edge : adjacent;
        if ((scale > 0.0) &&
            (fabs(edge - adjacent) / scale <=
             PA_NUISANCE_NYQUIST_EDGE_BALANCE)) {
            nyquist_edge_tone = 1U;
        }
    }

    refined_hz = (double)best_bin * df;
    if ((best_bin > min_bin) && (best_bin < max_bin) &&
        (best_bin < n / 2U)) {
        const double left =
            (double)FFT_normalized_output[best_bin - 1U];
        const double center =
            (double)FFT_normalized_output[best_bin];
        const double right =
            (double)FFT_normalized_output[best_bin + 1U];
        const double denominator = left - 2.0 * center + right;
        if (fabs(denominator) > 1e-30) {
            double delta = 0.5 * (left - right) / denominator;
            if (delta > 0.5) delta = 0.5;
            if (delta < -0.5) delta = -0.5;
            refined_hz = ((double)best_bin + delta) * df;
        }
    }
    if (refined_hz < min_hz) refined_hz = min_hz;
    if (refined_hz > max_hz) refined_hz = max_hz;

    step_hz = df / 8.0;
    for (uint8_t stage = 0U; stage < PA_REFINE_STAGES; ++stage) {
        refined_hz = PA_RefineNuisanceStep(data,
                                            n,
                                            config->sample_rate_hz,
                                            refined_hz,
                                            step_hz,
                                            min_hz,
                                            max_hz);
        step_hz *= 0.25;
    }
    /*
     * A Nyquist-bin tone has only one observable alternating column.  A
     * frequency infinitesimally below Nyquist has two columns and can lower
     * SSE slightly by fitting unrelated noise.  When the FFT peak is the
     * Nyquist bin, use a BIC comparison so that the extra quadrature is kept
     * only when the data materially supports a true offset frequency.
     */
    if (nyquist_edge_tone != 0U) {
        /*
         * A Hann-windowed exact Nyquist line has equal endpoint and adjacent
         * magnitudes (apart from the known finite-window rounding).  Use this
         * phase-independent signature to select the single alternating basis.
         */
        refined_hz = nyquist_hz;
    } else if ((nyquist_hz - refined_hz) < df / 1024.0) {
        /*
         * This offset produces less than 1/1024 bin of separation over the
         * record and is not observably distinct from the alternating column.
         */
        refined_hz = nyquist_hz;
    } else if ((best_bin == n / 2U) &&
               (refined_hz < nyquist_hz)) {
        PA_NuisanceModel boundary_model;
        PA_NuisanceModel refined_model;
        double boundary_solution[PA_SOLVER_MAX_ORDER] = {0.0};
        double refined_solution[PA_SOLVER_MAX_ORDER] = {0.0};
        double boundary_sse;
        double refined_sse;
        if ((PA_NuisanceFitAtFrequency(data,
                                       n,
                                       config->sample_rate_hz,
                                       nyquist_hz,
                                       &boundary_model,
                                       boundary_solution,
                                       &boundary_sse) == 0) &&
            (PA_NuisanceFitAtFrequency(data,
                                       n,
                                       config->sample_rate_hz,
                                       refined_hz,
                                       &refined_model,
                                       refined_solution,
                                       &refined_sse) == 0)) {
            const double boundary_bic =
                (double)n * log(boundary_sse / (double)n + 1e-30) +
                (double)boundary_model.column_count * log((double)n);
            const double refined_bic =
                (double)n * log(refined_sse / (double)n + 1e-30) +
                (double)refined_model.column_count * log((double)n);
            if (boundary_bic <= refined_bic) {
                refined_hz = nyquist_hz;
            }
        }
    }
    if (PA_NuisanceFitAtFrequency(data,
                                  n,
                                  config->sample_rate_hz,
                                  refined_hz,
                                  model,
                                  solution,
                                  &sse) != 0) {
        memset(model, 0, sizeof(*model));
        return -1;
    }
    PA_NuisanceDiagnostics(model,
                           0U,
                           solution,
                           peak_adc_v,
                           rms_adc_v);
    if ((rms_adc_v == NULL) ||
        !(*rms_adc_v >=
          config->interference_minimum_rms_adc_v)) {
        memset(model, 0, sizeof(*model));
        if (peak_adc_v != NULL) *peak_adc_v = 0.0f;
        if (rms_adc_v != NULL) *rms_adc_v = 0.0f;
    }
    return 0;
}

void PeriodicAnalyzer_GetDefaultConfig(PeriodicAnalyzer_Config *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    config->sample_rate_hz = 2000000.0f;
    config->min_frequency_hz = 10000.0f;
    config->max_frequency_hz = 500000.0f;
    config->input_volts_per_adc_volt = 0.125f;
    config->min_component_peak_v = 0.0005f;
    config->relative_peak_threshold = 0.01f;
}

PeriodicAnalyzer_Status PeriodicAnalyzer_Analyze(
    const float *adc_volts,
    uint32_t length,
    const PeriodicAnalyzer_Config *config,
    PeriodicAnalyzer_Result *result)
{
    PA_Peak peaks[PA_MAX_PEAKS];
    PA_Peak selected[PERIODIC_ANALYZER_MAX_COMPONENTS];
    PA_NuisanceModel nuisance = {0};
    const PA_NuisanceModel *nuisance_ptr = NULL;
    uint16_t orders[PERIODIC_ANALYZER_MAX_COMPONENTS] = {0U};
    double solution[PA_SOLVER_MAX_ORDER] = {0.0};
    uint8_t peak_count;
    uint8_t component_count;
    uint8_t response_error = 0U;
    double mean = 0.0;
    float f0 = 0.0f;
    float rms_sq = 0.0f;
    float interference_peak_adc_v = 0.0f;
    float interference_rms_adc_v = 0.0f;

    if ((adc_volts == NULL) || (config == NULL) || (result == NULL) ||
        (length < 64U) || (length > PERIODIC_ANALYZER_MAX_FFT_POINTS) ||
        ((length & (length - 1U)) != 0U) ||
        !(config->sample_rate_hz > 0.0f) ||
        !(config->input_volts_per_adc_volt > 0.0f) ||
        (config->cal_count > PERIODIC_ANALYZER_MAX_CAL_POINTS) ||
        (config->interference_rejection_enable > 1U) ||
        ((config->interference_rejection_enable != 0U) &&
         (!(config->interference_expected_frequency_hz > 0.0f) ||
          !(config->interference_expected_frequency_hz <=
            0.5f * config->sample_rate_hz) ||
          !isfinite(config->interference_expected_frequency_hz) ||
          !(config->interference_search_half_width_hz > 0.0f) ||
          !isfinite(config->interference_search_half_width_hz) ||
          !(config->interference_minimum_rms_adc_v >= 0.0f) ||
          !isfinite(config->interference_minimum_rms_adc_v)))) {
        if (result != NULL) {
            PA_ClearResult(result, PERIODIC_ANALYZER_E_ARGUMENT);
        }
        return PERIODIC_ANALYZER_E_ARGUMENT;
    }

    PA_ClearResult(result, PERIODIC_ANALYZER_E_NO_SIGNAL);
    for (uint32_t i = 0U; i < length; ++i) {
        if (!isfinite(adc_volts[i])) {
            return PERIODIC_ANALYZER_E_ARGUMENT;
        }
        mean += (double)adc_volts[i];
    }
    mean /= (double)length;
    for (uint32_t i = 0U; i < length; ++i) {
        s_centered[i] = adc_volts[i] - (float)mean;
    }

    if (!FFT_StartN(s_centered, length) ||
        !Normalize_FFT_To_Single_SideN(length)) {
        result->status = PERIODIC_ANALYZER_E_FFT;
        return result->status;
    }

    if (PA_DetectInterference(s_centered,
                              length,
                              config,
                              &nuisance,
                              &interference_peak_adc_v,
                              &interference_rms_adc_v) != 0) {
        result->status = PERIODIC_ANALYZER_E_MODEL;
        return result->status;
    }
    if (nuisance.active != 0U) {
        nuisance_ptr = &nuisance;
        result->interference_detected = 1U;
        result->interference_frequency_hz =
            (float)nuisance.frequency_hz;
        result->interference_post_dsp_peak_adc_v =
            interference_peak_adc_v;
        result->interference_post_dsp_rms_adc_v =
            interference_rms_adc_v;
    }

    peak_count = PA_CollectPeaks(config,
                                 length,
                                 peaks,
                                 &response_error);
    if (response_error != 0U) {
        result->status = PERIODIC_ANALYZER_E_MODEL;
        return result->status;
    }
    if (peak_count == 0U) return PERIODIC_ANALYZER_E_NO_SIGNAL;

    /*
     * Keep every significant spectral peak found in the configured band.
     * This list is independent of the smaller harmonic reconstruction model
     * used to calculate f0, RMS and Vpp.
     */
    result->spectrum_component_count = peak_count;
    for (uint8_t i = 0U; i < peak_count; ++i) {
        const uint16_t single_order = 1U;
        double single_solution[PA_SOLVER_MAX_ORDER] = {0.0};
        float refined_frequency;
        double amplitude_adc;
        float amp_corr;

        /*
         * The Hann FFT is used only to locate a line.  Its bin interpolation
         * still has scalloping error for non-coherent tones, so obtain every
         * reported spectrum amplitude from an unwindowed least-squares fit.
         */
        refined_frequency = PA_RefineFundamental(
            s_centered,
            length,
            config,
            &single_order,
            1U,
            nuisance_ptr,
            peaks[i].frequency_hz);
        if (PA_FitComponents(s_centered,
                             length,
                             config->sample_rate_hz,
                             &single_order,
                             1U,
                             (double)refined_frequency,
                             nuisance_ptr,
                             single_solution,
                             NULL) == 0) {
            amplitude_adc = hypot(single_solution[1],
                                  single_solution[2]);
        } else {
            refined_frequency = peaks[i].frequency_hz;
            amplitude_adc = (double)peaks[i].amplitude_adc_v;
        }
        if (PA_GetCorrection(config,
                             refined_frequency,
                             &amp_corr,
                             NULL) == 0U) {
            result->status = PERIODIC_ANALYZER_E_MODEL;
            return result->status;
        }
        result->spectrum[i].frequency_hz = refined_frequency;
        result->spectrum[i].peak_amplitude_v =
            (float)(amplitude_adc *
                    (double)config->input_volts_per_adc_volt *
                    (double)amp_corr);
    }

    component_count = PA_SelectHarmonicModel(
        peaks,
        peak_count,
        config->sample_rate_hz / (float)length,
        selected,
        orders,
        &f0);
    if ((component_count == 0U) ||
        !(f0 >= config->min_frequency_hz) ||
        !(f0 <= config->max_frequency_hz)) {
        return PERIODIC_ANALYZER_E_NO_SIGNAL;
    }

    f0 = PA_RefineFundamental(s_centered,
                              length,
                              config,
                              orders,
                              component_count,
                              nuisance_ptr,
                              f0);

    if (PA_FitComponents(s_centered,
                         length,
                         config->sample_rate_hz,
                         orders,
                         component_count,
                         (double)f0,
                         nuisance_ptr,
                         solution,
                         NULL) != 0) {
        result->status = PERIODIC_ANALYZER_E_MODEL;
        return result->status;
    }
    {
        const double residual_sse = PA_ComputeResidualSse(
            s_centered,
            length,
            config->sample_rate_hz,
            orders,
            component_count,
            (double)f0,
            nuisance_ptr,
            solution);
        if (!(residual_sse >= 0.0) || !isfinite(residual_sse)) {
            result->status = PERIODIC_ANALYZER_E_MODEL;
            return result->status;
        }
        result->residual_rms_v =
            (float)sqrt(residual_sse / (double)length) *
            config->input_volts_per_adc_volt;
    }
    if (nuisance_ptr != NULL) {
        PA_NuisanceDiagnostics(nuisance_ptr,
                               component_count,
                               solution,
                               &(result->
                                 interference_post_dsp_peak_adc_v),
                               &(result->
                                 interference_post_dsp_rms_adc_v));
    }

    result->component_count = component_count;
    result->fundamental_hz = f0;
    for (uint8_t c = 0U; c < component_count; ++c) {
        float amp_corr;
        float phase_corr;
        const double sin_coeff = solution[1U + 2U * c];
        const double cos_coeff = solution[2U + 2U * c];
        const float frequency = (float)orders[c] * f0;
        const double amplitude_adc = hypot(sin_coeff, cos_coeff);
        const float phase = (float)atan2(cos_coeff, sin_coeff);

        if (PA_GetCorrection(config,
                             frequency,
                             &amp_corr,
                             &phase_corr) == 0U) {
            result->status = PERIODIC_ANALYZER_E_MODEL;
            return result->status;
        }
        result->component[c].harmonic_order = orders[c];
        result->component[c].frequency_hz = frequency;
        result->component[c].peak_amplitude_v =
            (float)(amplitude_adc *
                    (double)config->input_volts_per_adc_volt *
                    (double)amp_corr);
        result->component[c].phase_rad = PA_WrapPhase(phase + phase_corr);
        rms_sq += 0.5f *
            result->component[c].peak_amplitude_v *
            result->component[c].peak_amplitude_v;
    }

    /*
     * Preserve every significant 10..500 kHz peak for spectrum output.
     * Where a peak belongs to the joint harmonic model, replace the FFT
     * estimate with the more accurate least-squares result.
     */
    for (uint8_t c = 0U; c < component_count; ++c) {
        uint8_t nearest = 0U;
        float nearest_error = 1e30f;
        for (uint8_t i = 0U;
             i < result->spectrum_component_count;
             ++i) {
            const float error = PA_AbsF(
                result->spectrum[i].frequency_hz -
                result->component[c].frequency_hz);
            if (error < nearest_error) {
                nearest_error = error;
                nearest = i;
            }
        }
        if ((result->spectrum_component_count != 0U) &&
            (nearest_error <=
             PA_HARMONIC_TOL_BINS *
             config->sample_rate_hz / (float)length)) {
            result->spectrum[nearest].harmonic_order =
                result->component[c].harmonic_order;
            result->spectrum[nearest].frequency_hz =
                result->component[c].frequency_hz;
            result->spectrum[nearest].peak_amplitude_v =
                result->component[c].peak_amplitude_v;
        }
    }
    for (uint8_t i = 0U;
         i < result->spectrum_component_count;
         ++i) {
        for (uint8_t j = (uint8_t)(i + 1U);
             j < result->spectrum_component_count;
             ++j) {
            if (result->spectrum[j].frequency_hz <
                result->spectrum[i].frequency_hz) {
                const PeriodicAnalyzer_SpectrumComponent temporary =
                    result->spectrum[i];
                result->spectrum[i] = result->spectrum[j];
                result->spectrum[j] = temporary;
            }
        }
    }
    result->true_rms_v = sqrtf(rms_sq);

    result->valid = 1U;
    result->vpp_v = PA_ComputeVpp(result);
    result->valid = 1U;
    result->status = PERIODIC_ANALYZER_OK;
    return PERIODIC_ANALYZER_OK;
}

float PeriodicAnalyzer_Evaluate(const PeriodicAnalyzer_Result *result,
                                float phase_turns)
{
    float output = 0.0f;
    if ((result == NULL) || (result->valid == 0U)) return 0.0f;
    for (uint8_t c = 0U; c < result->component_count; ++c) {
        const PeriodicAnalyzer_Component *component = &result->component[c];
        output += component->peak_amplitude_v *
                  sinf(PA_TWO_PI *
                       (float)component->harmonic_order *
                       phase_turns +
                       component->phase_rad);
    }
    return output;
}
