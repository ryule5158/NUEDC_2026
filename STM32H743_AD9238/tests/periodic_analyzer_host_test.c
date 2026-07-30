#include "periodic_analyzer.h"
#include <math.h>
#include <stdio.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_N 4096U
#define TEST_FS 2000000.0f
#define TEST_GAIN 8.0f
#define ADC_STEP_V (10.0f / 4096.0f)

typedef struct {
    const char *name;
    float f0;
    unsigned count;
    unsigned order[3];
    float amp[3];
    float phase[3];
} TestCase;

typedef struct {
    float frequency_hz;
    float amplitude_correction;
    float phase_correction_deg;
} CicExpectation;

static const CicExpectation cic_expectation[] = {
    {100000.0f, 1.0124037f,  27.36f},
    {200000.0f, 1.0506746f,  54.72f},
    {300000.0f, 1.1181651f,  82.08f},
    {400000.0f, 1.2210864f, 109.44f},
    {500000.0f, 1.3696207f, 136.80f}
};

static float samples[TEST_N];

static float absf_local(float x) { return (x < 0.0f) ? -x : x; }

static float wrap_phase_local(float phase)
{
    while (phase > TEST_PI) phase -= 2.0f * TEST_PI;
    while (phase <= -TEST_PI) phase += 2.0f * TEST_PI;
    return phase;
}

static float expected_vpp(const TestCase *tc)
{
    float lo = 1e9f;
    float hi = -1e9f;
    for (unsigned i = 0U; i < 200000U; ++i) {
        const float turn = (float)i / 200000.0f;
        float v = 0.0f;
        for (unsigned c = 0U; c < tc->count; ++c) {
            v += tc->amp[c] *
                sinf(2.0f * TEST_PI * (float)tc->order[c] * turn +
                     tc->phase[c]);
        }
        if (v < lo) lo = v;
        if (v > hi) hi = v;
    }
    return hi - lo;
}

static int run_case(const TestCase *tc)
{
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    float rms_sq = 0.0f;
    const float vpp = expected_vpp(tc);

    for (unsigned i = 0U; i < TEST_N; ++i) {
        const float t = (float)i / TEST_FS;
        float input_v = 0.0f;
        for (unsigned c = 0U; c < tc->count; ++c) {
            input_v += tc->amp[c] *
                sinf(2.0f * TEST_PI * tc->f0 * (float)tc->order[c] * t +
                     tc->phase[c]);
        }
        /* Emulate the nominal 8x front end and AD9238 12-bit, 10 V span. */
        samples[i] = roundf((input_v * TEST_GAIN) / ADC_STEP_V) * ADC_STEP_V;
    }

    PeriodicAnalyzer_GetDefaultConfig(&cfg);
    cfg.sample_rate_hz = TEST_FS;
    cfg.min_frequency_hz = 10000.0f;
    cfg.max_frequency_hz = 500000.0f;
    cfg.input_volts_per_adc_volt = 1.0f / TEST_GAIN;
    cfg.min_component_peak_v = 0.0005f;
    cfg.relative_peak_threshold = 0.01f;

    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK) {
        fprintf(stderr, "%s: analyzer returned %d\n", tc->name,
                (int)result.status);
        return 1;
    }
    for (unsigned c = 0U; c < tc->count; ++c) {
        rms_sq += 0.5f * tc->amp[c] * tc->amp[c];
    }
    if (result.component_count != tc->count ||
        absf_local(result.fundamental_hz - tc->f0) > 1000.0f ||
        absf_local(result.true_rms_v - sqrtf(rms_sq)) > 0.005f ||
        absf_local(result.vpp_v - vpp) > 0.005f) {
        fprintf(stderr,
                "%s: metric mismatch f=%f rms=%f vpp=%f count=%u\n",
                tc->name, result.fundamental_hz, result.true_rms_v,
                result.vpp_v, (unsigned)result.component_count);
        return 2;
    }
    for (unsigned c = 0U; c < tc->count; ++c) {
        if (result.component[c].harmonic_order != tc->order[c] ||
            absf_local(result.component[c].peak_amplitude_v - tc->amp[c]) >
            0.005f) {
            fprintf(stderr, "%s: line%u order=%u amp=%f\n", tc->name,
                    c, (unsigned)result.component[c].harmonic_order,
                    result.component[c].peak_amplitude_v);
            return 3;
        }
    }

    printf("PASS %-18s f0=%9.3fHz rms=%8.6fV vpp=%8.6fV\n",
           tc->name, result.fundamental_hz, result.true_rms_v,
           result.vpp_v);
    return 0;
}

static int run_cic_inverse_case(const CicExpectation *expectation,
                                int use_residual_calibration)
{
    const float input_peak_v = 0.100f;
    const float input_phase_rad = 0.37f;
    const float residual_gain =
        (use_residual_calibration != 0) ? 0.94f : 1.0f;
    const float residual_phase_deg =
        (use_residual_calibration != 0) ? 7.0f : 0.0f;
    const float residual_phase_rad =
        residual_phase_deg * TEST_PI / 180.0f;
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    float amplitude_correction;
    float phase_correction_rad;
    float phase_error;

    PeriodicAnalyzer_GetCic3R25Correction(
        expectation->frequency_hz,
        50000000.0f,
        &amplitude_correction,
        &phase_correction_rad);
    if (absf_local(amplitude_correction -
                   expectation->amplitude_correction) > 2e-5f ||
        absf_local(phase_correction_rad * 180.0f / TEST_PI -
                   expectation->phase_correction_deg) > 0.002f) {
        fprintf(stderr,
                "CIC table mismatch f=%.0f amp=%f phase=%fdeg\n",
                expectation->frequency_hz,
                amplitude_correction,
                phase_correction_rad * 180.0f / TEST_PI);
        return 10;
    }

    for (unsigned i = 0U; i < TEST_N; ++i) {
        const float t = (float)i / TEST_FS;
        const float cic_gain = 1.0f / amplitude_correction;
        const float adc_v = input_peak_v * TEST_GAIN *
            cic_gain * residual_gain *
            sinf(2.0f * TEST_PI * expectation->frequency_hz * t +
                 input_phase_rad - phase_correction_rad -
                 residual_phase_rad);
        samples[i] = roundf(adc_v / ADC_STEP_V) * ADC_STEP_V;
    }

    PeriodicAnalyzer_GetDefaultConfig(&cfg);
    cfg.sample_rate_hz = TEST_FS;
    cfg.min_frequency_hz = 10000.0f;
    cfg.max_frequency_hz = 500000.0f;
    cfg.input_volts_per_adc_volt = 1.0f / TEST_GAIN;
    cfg.min_component_peak_v = 0.0005f;
    cfg.relative_peak_threshold = 0.01f;
    PeriodicAnalyzer_EnableCic3R25Compensation(&cfg, 50000000.0f);
    if (use_residual_calibration != 0) {
        cfg.cal_count = 1U;
        cfg.cal[0].frequency_hz = expectation->frequency_hz;
        cfg.cal[0].amplitude_correction = 1.0f / residual_gain;
        cfg.cal[0].phase_correction_deg = residual_phase_deg;
    }

    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK) {
        fprintf(stderr, "CIC inverse f=%.0f analyzer returned %d\n",
                expectation->frequency_hz, (int)result.status);
        return 11;
    }
    phase_error = wrap_phase_local(
        result.component[0].phase_rad - input_phase_rad);
    if ((result.component_count != 1U) ||
        (absf_local(result.fundamental_hz -
                    expectation->frequency_hz) > 25.0f) ||
        (absf_local(result.component[0].peak_amplitude_v -
                    input_peak_v) > 0.0015f) ||
        (absf_local(phase_error) > 0.015f)) {
        fprintf(stderr,
                "CIC inverse mismatch f=%.0f measured_f=%f amp=%f phase=%f err=%f\n",
                expectation->frequency_hz,
                result.fundamental_hz,
                result.component[0].peak_amplitude_v,
                result.component[0].phase_rad,
                phase_error);
        return 12;
    }

    printf("PASS cic_inverse_%6.0fk amp_x=%1.7f phase=%7.3fdeg residual=%s\n",
           expectation->frequency_hz / 1000.0f,
           amplitude_correction,
           phase_correction_rad * 180.0f / TEST_PI,
           (use_residual_calibration != 0) ? "ON" : "OFF");
    return 0;
}

int main(void)
{
    static const TestCase cases[] = {
        {"mid_3lines", 123456.0f, 3U, {1U, 2U, 3U},
         {0.100f, 0.040f, 0.025f}, {0.31f, -0.77f, 1.13f}},
        {"low_edge_2lines", 10037.0f, 2U, {1U, 2U, 0U},
         {0.028f, 0.009f, 0.0f}, {-1.01f, 0.47f, 0.0f}},
        {"exact_10k_weak_f0", 10000.0f, 3U, {1U, 2U, 3U},
         {0.050f, 0.48235f, 0.28235f}, {0.20f, -0.50f, 1.00f}},
        {"high_edge_3lines", 166431.0f, 3U, {1U, 2U, 3U},
         {0.035f, 0.018f, 0.012f}, {0.20f, 1.70f, -0.50f}},
        {"weak_fundamental", 70031.0f, 3U, {1U, 2U, 3U},
         {0.010f, 0.080f, 0.025f}, {-0.30f, 0.80f, 2.00f}},
        {"near_500k_2lines", 249877.0f, 2U, {1U, 2U, 0U},
         {0.080f, 0.030f, 0.0f}, {0.90f, -1.20f, 0.0f}}
    };

    for (unsigned i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        const int status = run_case(&cases[i]);
        if (status != 0) return status;
    }
    for (unsigned i = 0U;
         i < sizeof(cic_expectation) / sizeof(cic_expectation[0]);
         ++i) {
        const int status = run_cic_inverse_case(&cic_expectation[i], 0);
        if (status != 0) return status;
    }
    if (run_cic_inverse_case(
            &cic_expectation[
                sizeof(cic_expectation) / sizeof(cic_expectation[0]) - 1U],
            1) != 0) {
        return 13;
    }
    puts("PERIODIC_ANALYZER_CIC3_R25_INVERSE_PASS");
    puts("PERIODIC_ANALYZER_HOST_MATRIX_PASS");
    return 0;
}
