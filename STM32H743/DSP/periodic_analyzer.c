#include "periodic_analyzer.h"

#include "FFT.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define PA_PI                    3.14159265358979323846f
#define PA_TWO_PI                (2.0f * PA_PI)
#define PA_MAX_PEAKS             12U
#define PA_PEAK_EXCLUSION_BINS   8U
#define PA_HARMONIC_TOL_BINS     2.25f
#define PA_VPP_GRID_POINTS       2048U
#define PA_SOLVER_MAX_ORDER      (1U + 2U * PERIODIC_ANALYZER_MAX_COMPONENTS)

typedef struct {
    uint32_t bin;
    float frequency_hz;
    float amplitude_adc_v;
} PA_Peak;

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

static void PA_ClearResult(PeriodicAnalyzer_Result *result,
                           PeriodicAnalyzer_Status status)
{
    memset(result, 0, sizeof(*result));
    result->status = status;
}

static void PA_InterpolateCalibration(
    const PeriodicAnalyzer_Config *config,
    float frequency_hz,
    float *amplitude_correction,
    float *phase_correction_rad)
{
    float amp = 1.0f;
    float phase_deg = 0.0f;

    if (config->cal_count != 0U) {
        uint32_t upper = 0U;
        if (frequency_hz <= config->cal[0].frequency_hz) {
            amp = config->cal[0].amplitude_correction;
            phase_deg = config->cal[0].phase_correction_deg;
        } else if (frequency_hz >=
                   config->cal[config->cal_count - 1U].frequency_hz) {
            amp = config->cal[config->cal_count - 1U].amplitude_correction;
            phase_deg =
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
                amp = lo->amplitude_correction +
                      t * (hi->amplitude_correction -
                           lo->amplitude_correction);
                phase_deg = lo->phase_correction_deg +
                            t * (hi->phase_correction_deg -
                                 lo->phase_correction_deg);
            }
        }
    }

    if (!(amp > 0.0f) || !isfinite(amp)) amp = 1.0f;
    if (!isfinite(phase_deg)) phase_deg = 0.0f;
    if (amplitude_correction != NULL) *amplitude_correction = amp;
    if (phase_correction_rad != NULL) {
        *phase_correction_rad = phase_deg * (PA_PI / 180.0f);
    }
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
                               PA_Peak peaks[PA_MAX_PEAKS])
{
    PA_Peak local[PA_MAX_PEAKS * 3U];
    uint8_t local_count = 0U;
    uint8_t selected_count = 0U;
    const float fs = config->sample_rate_hz;
    const float df = fs / (float)n;
    uint32_t min_bin = (uint32_t)ceilf(config->min_frequency_hz / df);
    uint32_t max_bin = (uint32_t)floorf(config->max_frequency_hz / df);
    float global_max = 0.0f;

    if (min_bin < 2U) min_bin = 2U;
    if (max_bin >= n / 2U) max_bin = n / 2U - 1U;

    /*
     * 先保留幅值最大的若干局部峰。插入排序使 local 始终按幅值降序，
     * 可避免为 4096 点频谱额外分配大数组。
     */
    for (uint32_t k = min_bin; k <= max_bin; ++k) {
        const float v = FFT_normalized_output[k];
        uint8_t pos;
        if (!isfinite(v) ||
            (v < FFT_normalized_output[k - 1U]) ||
            (v <= FFT_normalized_output[k + 1U])) {
            continue;
        }
        if (v > global_max) global_max = v;

        pos = local_count;
        if (pos > (uint8_t)(PA_MAX_PEAKS * 3U - 1U)) {
            pos = (uint8_t)(PA_MAX_PEAKS * 3U - 1U);
        }
        while ((pos > 0U) && (local[pos - 1U].amplitude_adc_v < v)) {
            if (pos < (uint8_t)(PA_MAX_PEAKS * 3U)) {
                local[pos] = local[pos - 1U];
            }
            --pos;
        }
        if (pos < (uint8_t)(PA_MAX_PEAKS * 3U)) {
            PA_InterpPeak(FFT_normalized_output, n, k, fs, &local[pos]);
            if (local_count < (uint8_t)(PA_MAX_PEAKS * 3U)) local_count++;
        }
    }

    if (!(global_max > 0.0f)) return 0U;

    for (uint8_t i = 0U; i < local_count; ++i) {
        uint8_t separated = 1U;
        float amp_corr;
        const PA_Peak *candidate = &local[i];
        const float input_amp = candidate->amplitude_adc_v *
                                config->input_volts_per_adc_volt;

        PA_InterpolateCalibration(config,
                                  candidate->frequency_hz,
                                  &amp_corr,
                                  NULL);
        if ((candidate->amplitude_adc_v <
             global_max * config->relative_peak_threshold) ||
            (input_amp * amp_corr < config->min_component_peak_v)) {
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

        matched_amp[matched_count++] = peaks[c].amplitude_adc_v;
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

            amp = peaks[i].amplitude_adc_v;
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
                    if (selected[j].amplitude_adc_v <
                        selected[weakest].amplitude_adc_v) {
                        weakest = j;
                    }
                }
                if (peaks[i].amplitude_adc_v >
                    selected[weakest].amplitude_adc_v) {
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
                    if (selected[i].amplitude_adc_v <
                        selected[weakest].amplitude_adc_v) {
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
                    (double)selected[i].amplitude_adc_v *
                    (double)selected[i].amplitude_adc_v;
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
                            float f0,
                            double solution[PA_SOLVER_MAX_ORDER],
                            float *residual_rms)
{
    double normal[PA_SOLVER_MAX_ORDER][PA_SOLVER_MAX_ORDER + 1U] = {{0.0}};
    const uint32_t model_order = 1U + 2U * component_count;

    for (uint32_t sample = 0U; sample < n; ++sample) {
        double row[PA_SOLVER_MAX_ORDER] = {0.0};
        row[0] = 1.0;
        for (uint8_t c = 0U; c < component_count; ++c) {
            const double phase =
                2.0 * (double)PA_PI *
                (double)((float)orders[c] * f0) *
                (double)sample / (double)fs;
            row[1U + 2U * c] = sin(phase);
            row[2U + 2U * c] = cos(phase);
        }
        for (uint32_t r = 0U; r < model_order; ++r) {
            for (uint32_t c = 0U; c < model_order; ++c) {
                normal[r][c] += row[r] * row[c];
            }
            normal[r][model_order] += row[r] * (double)data[sample];
        }
    }

    if (PA_Solve(normal, model_order, solution) != 0) return -1;

    if (residual_rms != NULL) {
        double sum_sq = 0.0;
        for (uint32_t sample = 0U; sample < n; ++sample) {
            double estimate = solution[0];
            for (uint8_t c = 0U; c < component_count; ++c) {
                const double phase =
                    2.0 * (double)PA_PI *
                    (double)((float)orders[c] * f0) *
                    (double)sample / (double)fs;
                estimate += solution[1U + 2U * c] * sin(phase) +
                            solution[2U + 2U * c] * cos(phase);
            }
            {
                const double error = (double)data[sample] - estimate;
                sum_sq += error * error;
            }
        }
        *residual_rms = (float)sqrt(sum_sq / (double)n);
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
    uint16_t orders[PERIODIC_ANALYZER_MAX_COMPONENTS] = {0U};
    double solution[PA_SOLVER_MAX_ORDER] = {0.0};
    uint8_t peak_count;
    uint8_t component_count;
    double mean = 0.0;
    float f0 = 0.0f;
    float rms_sq = 0.0f;
    float vmin = 1e30f;
    float vmax = -1e30f;

    if ((adc_volts == NULL) || (config == NULL) || (result == NULL) ||
        (length < 64U) || (length > PERIODIC_ANALYZER_MAX_FFT_POINTS) ||
        ((length & (length - 1U)) != 0U) ||
        !(config->sample_rate_hz > 0.0f) ||
        !(config->input_volts_per_adc_volt > 0.0f) ||
        (config->cal_count > PERIODIC_ANALYZER_MAX_CAL_POINTS)) {
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

    peak_count = PA_CollectPeaks(config, length, peaks);
    if (peak_count == 0U) return PERIODIC_ANALYZER_E_NO_SIGNAL;

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

    if (PA_FitComponents(s_centered,
                         length,
                         config->sample_rate_hz,
                         orders,
                         component_count,
                         f0,
                         solution,
                         &result->residual_rms_v) != 0) {
        result->status = PERIODIC_ANALYZER_E_MODEL;
        return result->status;
    }
    result->residual_rms_v *= config->input_volts_per_adc_volt;

    result->component_count = component_count;
    result->fundamental_hz = f0;
    for (uint8_t c = 0U; c < component_count; ++c) {
        float amp_corr;
        float phase_corr;
        const float sin_coeff = (float)solution[1U + 2U * c];
        const float cos_coeff = (float)solution[2U + 2U * c];
        const float frequency = (float)orders[c] * f0;
        float amplitude_adc =
            sqrtf(sin_coeff * sin_coeff + cos_coeff * cos_coeff);
        float phase = atan2f(cos_coeff, sin_coeff);

        PA_InterpolateCalibration(config,
                                  frequency,
                                  &amp_corr,
                                  &phase_corr);
        result->component[c].harmonic_order = orders[c];
        result->component[c].frequency_hz = frequency;
        result->component[c].peak_amplitude_v =
            amplitude_adc * config->input_volts_per_adc_volt * amp_corr;
        result->component[c].phase_rad = PA_WrapPhase(phase + phase_corr);
        rms_sq += 0.5f *
            result->component[c].peak_amplitude_v *
            result->component[c].peak_amplitude_v;
    }
    result->true_rms_v = sqrtf(rms_sq);

    result->valid = 1U;
    for (uint32_t i = 0U; i < PA_VPP_GRID_POINTS; ++i) {
        const float phase_turns =
            (float)i / (float)PA_VPP_GRID_POINTS;
        const float v = PeriodicAnalyzer_Evaluate(result, phase_turns);
        if (v < vmin) vmin = v;
        if (v > vmax) vmax = v;
    }
    result->vpp_v = vmax - vmin;
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
