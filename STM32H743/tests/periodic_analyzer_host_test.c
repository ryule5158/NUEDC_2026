#include "periodic_analyzer.h"
#include <math.h>
#include <stdio.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_N 4096U
#define TEST_FS 2000000.0f
#define TEST_GAIN 8.0f
#define ADC_STEP_V (10.0f / 256.0f)

typedef struct {
    const char *name;
    float f0;
    unsigned count;
    unsigned order[3];
    float amp[3];
    float phase[3];
} TestCase;

static float samples[TEST_N];

static float absf_local(float x) { return (x < 0.0f) ? -x : x; }

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
        /* Emulate the nominal 8x front end and the module's 8-bit, 10 V span. */
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

int main(void)
{
    static const TestCase cases[] = {
        {"mid_3lines", 123456.0f, 3U, {1U, 2U, 3U},
         {0.100f, 0.040f, 0.025f}, {0.31f, -0.77f, 1.13f}},
        {"low_edge_2lines", 10037.0f, 2U, {1U, 2U, 0U},
         {0.028f, 0.009f, 0.0f}, {-1.01f, 0.47f, 0.0f}},
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
    puts("PERIODIC_ANALYZER_HOST_MATRIX_PASS");
    return 0;
}