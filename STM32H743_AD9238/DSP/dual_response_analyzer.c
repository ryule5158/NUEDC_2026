#include "dual_response_analyzer.h"

#include "periodic_analyzer.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define DUAL_RESPONSE_PI       3.14159265358979323846
#define DUAL_RESPONSE_RAD2DEG  57.2957795130823208768
#define DUAL_RESPONSE_EPSILON  1.0e-12

typedef struct {
    double dc;
    double sin_coefficient;
    double cos_coefficient;
    double peak;
    double residual_ratio;
} DualResponse_Fit;

static int DualResponse_Solve3(double matrix[3][4], double solution[3])
{
    for (uint32_t column = 0U; column < 3U; ++column) {
        uint32_t pivot = column;
        double pivot_abs = fabs(matrix[pivot][column]);
        for (uint32_t row = column + 1U; row < 3U; ++row) {
            double candidate = fabs(matrix[row][column]);
            if (candidate > pivot_abs) {
                pivot = row;
                pivot_abs = candidate;
            }
        }
        if (!(pivot_abs > DUAL_RESPONSE_EPSILON)) {
            return 0;
        }
        if (pivot != column) {
            for (uint32_t k = column; k < 4U; ++k) {
                double temporary = matrix[column][k];
                matrix[column][k] = matrix[pivot][k];
                matrix[pivot][k] = temporary;
            }
        }
        for (uint32_t row = column + 1U; row < 3U; ++row) {
            double factor = matrix[row][column] / matrix[column][column];
            for (uint32_t k = column; k < 4U; ++k) {
                matrix[row][k] -= factor * matrix[column][k];
            }
        }
    }

    for (int32_t row = 2; row >= 0; --row) {
        double value = matrix[row][3];
        for (uint32_t column = (uint32_t)row + 1U; column < 3U; ++column) {
            value -= matrix[row][column] * solution[column];
        }
        if (!(fabs(matrix[row][row]) > DUAL_RESPONSE_EPSILON)) {
            return 0;
        }
        solution[row] = value / matrix[row][row];
    }
    return 1;
}

static int DualResponse_FitTone(const float *samples,
                                uint32_t length,
                                double omega,
                                DualResponse_Fit *fit)
{
    double normal[3][3] = {{0.0}};
    double rhs[3] = {0.0};
    double augmented[3][4];
    double solution[3] = {0.0};
    double residual_sum = 0.0;
    double signal_sum = 0.0;

    for (uint32_t i = 0U; i < length; ++i) {
        double phase = omega * (double)i;
        double basis[3] = {1.0, sin(phase), cos(phase)};
        double sample = (double)samples[i];
        if (!isfinite(sample)) {
            return 0;
        }
        for (uint32_t row = 0U; row < 3U; ++row) {
            rhs[row] += basis[row] * sample;
            for (uint32_t column = 0U; column < 3U; ++column) {
                normal[row][column] += basis[row] * basis[column];
            }
        }
    }

    for (uint32_t row = 0U; row < 3U; ++row) {
        for (uint32_t column = 0U; column < 3U; ++column) {
            augmented[row][column] = normal[row][column];
        }
        augmented[row][3] = rhs[row];
    }
    if (DualResponse_Solve3(augmented, solution) == 0) {
        return 0;
    }

    fit->dc = solution[0];
    fit->sin_coefficient = solution[1];
    fit->cos_coefficient = solution[2];
    fit->peak = hypot(solution[1], solution[2]);
    if (!(fit->peak > DUAL_RESPONSE_EPSILON)) {
        return 0;
    }

    for (uint32_t i = 0U; i < length; ++i) {
        double phase = omega * (double)i;
        double ac_sample = (double)samples[i] - fit->dc;
        double estimate = fit->sin_coefficient * sin(phase) +
                          fit->cos_coefficient * cos(phase);
        double error = ac_sample - estimate;
        residual_sum += error * error;
        signal_sum += ac_sample * ac_sample;
    }
    fit->residual_ratio = (signal_sum > DUAL_RESPONSE_EPSILON)
        ? sqrt(residual_sum / signal_sum) : 1.0;
    return 1;
}

static double DualResponse_WrapPi(double phase)
{
    while (phase >= DUAL_RESPONSE_PI) {
        phase -= 2.0 * DUAL_RESPONSE_PI;
    }
    while (phase < -DUAL_RESPONSE_PI) {
        phase += 2.0 * DUAL_RESPONSE_PI;
    }
    return phase;
}

DualResponse_Status DualResponse_Analyze(const float *ch1_v,
                                         const float *ch2_v,
                                         uint32_t length,
                                         float sample_rate_hz,
                                         float min_frequency_hz,
                                         float max_frequency_hz,
                                         DualResponse_Result *result)
{
    PeriodicAnalyzer_Config config;
    PeriodicAnalyzer_Result frequency_result;
    DualResponse_Fit ch1_fit;
    DualResponse_Fit ch2_fit;
    PeriodicAnalyzer_Status frequency_status;
    double frequency_hz;
    double omega;
    double phase;

    if (result != NULL) {
        memset(result, 0, sizeof(*result));
        result->status = DUAL_RESPONSE_E_ARGUMENT;
    }
    if ((ch1_v == NULL) || (ch2_v == NULL) || (result == NULL) ||
        (length < 64U) || (length > PERIODIC_ANALYZER_MAX_FFT_POINTS) ||
        ((length & (length - 1U)) != 0U) ||
        !(sample_rate_hz > 0.0f) || !(min_frequency_hz > 0.0f) ||
        !(max_frequency_hz > min_frequency_hz) ||
        !(max_frequency_hz < sample_rate_hz * 0.5f)) {
        return DUAL_RESPONSE_E_ARGUMENT;
    }

    PeriodicAnalyzer_GetDefaultConfig(&config);
    config.sample_rate_hz = sample_rate_hz;
    config.min_frequency_hz = min_frequency_hz;
    config.max_frequency_hz = max_frequency_hz;
    config.input_volts_per_adc_volt = 1.0f;
    config.min_component_peak_v = 0.002f;
    config.relative_peak_threshold = 0.02f;
    config.fir307_antialias_compensation_enabled = 0U;
    config.cic3_r25_compensation_enabled = 0U;
    config.cal_count = 0U;
    frequency_status = PeriodicAnalyzer_Analyze(
        ch1_v, length, &config, &frequency_result);
    if ((frequency_status != PERIODIC_ANALYZER_OK) ||
        (frequency_result.valid == 0U)) {
        result->status = DUAL_RESPONSE_E_NO_SIGNAL;
        return result->status;
    }

    frequency_hz = (double)frequency_result.fundamental_hz;
    omega = 2.0 * DUAL_RESPONSE_PI * frequency_hz /
            (double)sample_rate_hz;
    if ((DualResponse_FitTone(ch1_v, length, omega, &ch1_fit) == 0) ||
        (DualResponse_FitTone(ch2_v, length, omega, &ch2_fit) == 0)) {
        result->status = DUAL_RESPONSE_E_MODEL;
        return result->status;
    }
    if (!(ch1_fit.peak > 0.002)) {
        result->status = DUAL_RESPONSE_E_NO_SIGNAL;
        return result->status;
    }

    phase = DualResponse_WrapPi(
        atan2(ch2_fit.cos_coefficient, ch2_fit.sin_coefficient) -
        atan2(ch1_fit.cos_coefficient, ch1_fit.sin_coefficient));
    result->frequency_hz = (float)frequency_hz;
    result->ch1_peak_v = (float)ch1_fit.peak;
    result->ch2_peak_v = (float)ch2_fit.peak;
    result->gain_linear = (float)(ch2_fit.peak / ch1_fit.peak);
    result->gain_db = 20.0f * log10f(result->gain_linear);
    result->phase_rad = (float)phase;
    result->phase_deg = (float)(phase * DUAL_RESPONSE_RAD2DEG);
    result->ch1_dc_v = (float)ch1_fit.dc;
    result->ch2_dc_v = (float)ch2_fit.dc;
    result->ch1_residual_ratio = (float)ch1_fit.residual_ratio;
    result->ch2_residual_ratio = (float)ch2_fit.residual_ratio;
    result->status = DUAL_RESPONSE_OK;
    result->valid = 1U;
    return result->status;
}
