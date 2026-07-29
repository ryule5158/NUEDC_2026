#include "periodic_analyzer.h"
#include <math.h>
#include <stdio.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_PI_D 3.14159265358979323846264338327950288
#define TEST_N 4096U
#define TEST_FS 2000000.0f
#define TEST_GAIN 8.0f
#define ADC_STEP_V (10.0f / 256.0f)
#define CIC_INPUT_FS 32000000.0
#define CIC_DECIMATION 16.0
#define CIC_STAGES 3U
#define CIC_GROUP_DELAY_INPUT_SAMPLES 22.5

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

static float phase_error_rad(float actual, float expected)
{
    float error = actual - expected;
    while (error > TEST_PI) error -= 2.0f * TEST_PI;
    while (error <= -TEST_PI) error += 2.0f * TEST_PI;
    return absf_local(error);
}

/*
 * Exact forward response of the FPGA's normalized CIC3/R16 decimator:
 * [sin(pi*f*R/Fs)/(R*sin(pi*f/Fs))]^3.
 */
static float cic3_r16_magnitude(float frequency_hz)
{
    const double x =
        TEST_PI_D * (double)frequency_hz / CIC_INPUT_FS;
    double ratio;

    if (fabs(x) < 1e-15) return 1.0f;
    ratio = sin(CIC_DECIMATION * x) /
            (CIC_DECIMATION * sin(x));
    return (float)(ratio * ratio * ratio);
}

static float cic3_r16_phase_rad(float frequency_hz)
{
    return (float)(-2.0 * TEST_PI_D *
                   (double)frequency_hz *
                   CIC_GROUP_DELAY_INPUT_SAMPLES /
                   CIC_INPUT_FS);
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
        absf_local(result.fundamental_hz - tc->f0) > 1.0f ||
        absf_local(result.true_rms_v - sqrtf(rms_sq)) > 0.001f ||
        absf_local(result.vpp_v - vpp) > 0.001f) {
        fprintf(stderr,
                "%s: metric mismatch f=%f rms=%f vpp=%f count=%u\n",
                tc->name, result.fundamental_hz, result.true_rms_v,
                result.vpp_v, (unsigned)result.component_count);
        return 2;
    }
    for (unsigned c = 0U; c < tc->count; ++c) {
        if (result.component[c].harmonic_order != tc->order[c] ||
            absf_local(result.component[c].peak_amplitude_v - tc->amp[c]) >
            0.001f) {
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

static void configure_cic_response(PeriodicAnalyzer_Config *cfg)
{
    PeriodicAnalyzer_GetDefaultConfig(cfg);
    cfg->sample_rate_hz = TEST_FS;
    cfg->min_frequency_hz = 10000.0f;
    cfg->max_frequency_hz = 500000.0f;
    cfg->input_volts_per_adc_volt = 1.0f / TEST_GAIN;
    cfg->min_component_peak_v = 0.0005f;
    cfg->relative_peak_threshold = 0.01f;
    cfg->digital_magnitude_response = cic3_r16_magnitude;
    cfg->digital_phase_response_rad = cic3_r16_phase_rad;
}

static int run_cic_compensation_sweep(void)
{
    static const float frequencies_hz[] = {
        10000.0f, 50000.0f, 100000.0f, 200000.0f,
        300000.0f, 400000.0f, 500000.0f
    };
    const float input_peak_v = 0.173f;
    const float input_phase_rad = 0.37f;

    for (unsigned tone = 0U;
         tone < sizeof(frequencies_hz) / sizeof(frequencies_hz[0]);
         ++tone) {
        PeriodicAnalyzer_Config cfg;
        PeriodicAnalyzer_Result result;
        const float frequency_hz = frequencies_hz[tone];
        const float magnitude = cic3_r16_magnitude(frequency_hz);
        const float response_phase = cic3_r16_phase_rad(frequency_hz);

        for (unsigned i = 0U; i < TEST_N; ++i) {
            const double time_s = (double)i / (double)TEST_FS;
            samples[i] = (float)(
                (double)TEST_GAIN *
                (double)input_peak_v *
                (double)magnitude *
                sin(2.0 * TEST_PI_D *
                    (double)frequency_hz * time_s +
                    (double)input_phase_rad +
                    (double)response_phase));
        }

        configure_cic_response(&cfg);
        if (cfg.cal_count != 0U) {
            fputs("default empirical calibration is unexpectedly enabled\n",
                  stderr);
            return 10;
        }
        if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
            PERIODIC_ANALYZER_OK) {
            fprintf(stderr,
                    "CIC sweep %.0fHz: analyzer returned %d\n",
                    frequency_hz,
                    (int)result.status);
            return 11;
        }
        if ((result.component_count != 1U) ||
            (result.component[0].harmonic_order != 1U) ||
            (absf_local(result.fundamental_hz - frequency_hz) > 1.0f) ||
            (absf_local(result.component[0].peak_amplitude_v -
                        input_peak_v) > 0.0001f) ||
            (phase_error_rad(result.component[0].phase_rad,
                             input_phase_rad) > 0.003f) ||
            (absf_local(result.true_rms_v -
                        input_peak_v / sqrtf(2.0f)) > 0.0001f) ||
            (absf_local(result.vpp_v -
                        2.0f * input_peak_v) > 0.0002f)) {
            fprintf(stderr,
                    "CIC sweep %.0fHz: f=%f amp=%f phase=%f "
                    "rms=%f vpp=%f H=%f\n",
                    frequency_hz,
                    result.fundamental_hz,
                    result.component[0].peak_amplitude_v,
                    result.component[0].phase_rad,
                    result.true_rms_v,
                    result.vpp_v,
                    magnitude);
            return 12;
        }
        if ((result.spectrum_component_count != 1U) ||
            (absf_local(result.spectrum[0].peak_amplitude_v -
                        input_peak_v) > 0.0001f)) {
            fprintf(stderr,
                    "CIC sweep %.0fHz: spectrum count=%u amp=%f\n",
                    frequency_hz,
                    (unsigned)result.spectrum_component_count,
                    result.spectrum[0].peak_amplitude_v);
            return 13;
        }
        printf("PASS CIC3/R16 %6.0fHz H=%0.7f corrected=%0.6fV\n",
               frequency_hz,
               magnitude,
               result.component[0].peak_amplitude_v);
    }
    return 0;
}

static int run_cic_multiline_compensation(void)
{
    static const TestCase tc = {
        "cic_multiline",
        10000.0f,
        3U,
        {1U, 20U, 50U},
        {0.100f, 0.080f, 0.060f},
        {0.31f, -0.77f, 1.13f}
    };
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    float rms_sq = 0.0f;

    for (unsigned c = 0U; c < tc.count; ++c) {
        rms_sq += 0.5f * tc.amp[c] * tc.amp[c];
    }
    for (unsigned i = 0U; i < TEST_N; ++i) {
        const double time_s = (double)i / (double)TEST_FS;
        double measured_adc_v = 0.0;
        for (unsigned c = 0U; c < tc.count; ++c) {
            const float frequency_hz =
                tc.f0 * (float)tc.order[c];
            measured_adc_v +=
                (double)TEST_GAIN *
                (double)tc.amp[c] *
                (double)cic3_r16_magnitude(frequency_hz) *
                sin(2.0 * TEST_PI_D *
                    (double)frequency_hz * time_s +
                    (double)tc.phase[c] +
                    (double)cic3_r16_phase_rad(frequency_hz));
        }
        samples[i] = (float)measured_adc_v;
    }

    configure_cic_response(&cfg);
    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK) {
        fprintf(stderr,
                "CIC multiline: analyzer returned %d\n",
                (int)result.status);
        return 14;
    }
    if ((result.component_count != tc.count) ||
        (absf_local(result.fundamental_hz - tc.f0) > 1.0f) ||
        (absf_local(result.true_rms_v - sqrtf(rms_sq)) > 0.0001f) ||
        (absf_local(result.vpp_v - expected_vpp(&tc)) > 0.0002f)) {
        fprintf(stderr,
                "CIC multiline: count=%u f=%f rms=%f vpp=%f\n",
                (unsigned)result.component_count,
                result.fundamental_hz,
                result.true_rms_v,
                result.vpp_v);
        return 15;
    }
    for (unsigned c = 0U; c < tc.count; ++c) {
        if ((result.component[c].harmonic_order != tc.order[c]) ||
            (absf_local(result.component[c].peak_amplitude_v -
                        tc.amp[c]) > 0.0001f) ||
            (phase_error_rad(result.component[c].phase_rad,
                             tc.phase[c]) > 0.003f)) {
            fprintf(stderr,
                    "CIC multiline line%u: order=%u amp=%f phase=%f\n",
                    c,
                    (unsigned)result.component[c].harmonic_order,
                    result.component[c].peak_amplitude_v,
                    result.component[c].phase_rad);
            return 16;
        }
    }
    puts("PASS CIC3/R16 multiline 10/200/500kHz compensation");
    return 0;
}

static int run_cic_plus_empirical_calibration(void)
{
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    const float frequency_hz = 275000.0f;
    const float input_peak_v = 0.137f;
    const float input_phase_rad = -0.63f;
    const float empirical_amp_correction = 1.08f;
    const float empirical_phase_deg = 7.5f;
    const float empirical_phase_rad =
        empirical_phase_deg * TEST_PI / 180.0f;
    const float magnitude = cic3_r16_magnitude(frequency_hz);
    const float response_phase = cic3_r16_phase_rad(frequency_hz);

    /*
     * Emulate a remaining analog error that the optional empirical entry
     * corrects, in addition to the deterministic CIC response.
     */
    for (unsigned i = 0U; i < TEST_N; ++i) {
        const double time_s = (double)i / (double)TEST_FS;
        samples[i] = (float)(
            (double)TEST_GAIN *
            (double)input_peak_v *
            (double)magnitude /
            (double)empirical_amp_correction *
            sin(2.0 * TEST_PI_D *
                (double)frequency_hz * time_s +
                (double)input_phase_rad +
                (double)response_phase -
                (double)empirical_phase_rad));
    }

    configure_cic_response(&cfg);
    cfg.cal_count = 1U;
    cfg.cal[0].frequency_hz = frequency_hz;
    cfg.cal[0].amplitude_correction = empirical_amp_correction;
    cfg.cal[0].phase_correction_deg = empirical_phase_deg;

    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK) {
        fprintf(stderr,
                "CIC plus calibration: analyzer returned %d\n",
                (int)result.status);
        return 20;
    }
    if ((result.component_count != 1U) ||
        (absf_local(result.component[0].peak_amplitude_v -
                    input_peak_v) > 0.0001f) ||
        (phase_error_rad(result.component[0].phase_rad,
                         input_phase_rad) > 0.003f)) {
        fprintf(stderr,
                "CIC plus calibration: amp=%f phase=%f\n",
                result.component[0].peak_amplitude_v,
                result.component[0].phase_rad);
        return 21;
    }
    puts("PASS CIC3/R16 + empirical calibration composition");
    return 0;
}

static void enable_one_mhz_interference_rejection(
    PeriodicAnalyzer_Config *cfg)
{
    cfg->interference_rejection_enable = 1U;
    cfg->interference_expected_frequency_hz = 1000000.0f;
    cfg->interference_search_half_width_hz = 5000.0f;
    /* Analyzer-input/post-DSP ADC volts, not physical input volts. */
    cfg->interference_minimum_rms_adc_v = 0.001f;
}

static int compare_no_interference_toggle(void)
{
    static const TestCase tc = {
        "no_interference_toggle",
        10000.0f,
        3U,
        {1U, 20U, 50U},
        {0.100f, 0.040f, 0.025f},
        {0.31f, -0.77f, 1.13f}
    };
    PeriodicAnalyzer_Config disabled_cfg;
    PeriodicAnalyzer_Config enabled_cfg;
    PeriodicAnalyzer_Result disabled;
    PeriodicAnalyzer_Result enabled;

    for (unsigned i = 0U; i < TEST_N; ++i) {
        const double time_s = (double)i / (double)TEST_FS;
        double measured_adc_v = 0.0;
        for (unsigned c = 0U; c < tc.count; ++c) {
            const float frequency_hz =
                tc.f0 * (float)tc.order[c];
            measured_adc_v +=
                (double)TEST_GAIN *
                (double)tc.amp[c] *
                (double)cic3_r16_magnitude(frequency_hz) *
                sin(2.0 * TEST_PI_D *
                    (double)frequency_hz * time_s +
                    (double)tc.phase[c] +
                    (double)cic3_r16_phase_rad(frequency_hz));
        }
        samples[i] = (float)measured_adc_v;
    }

    configure_cic_response(&disabled_cfg);
    enabled_cfg = disabled_cfg;
    enable_one_mhz_interference_rejection(&enabled_cfg);

    if ((PeriodicAnalyzer_Analyze(samples,
                                  TEST_N,
                                  &disabled_cfg,
                                  &disabled) != PERIODIC_ANALYZER_OK) ||
        (PeriodicAnalyzer_Analyze(samples,
                                  TEST_N,
                                  &enabled_cfg,
                                  &enabled) != PERIODIC_ANALYZER_OK)) {
        fputs("no-interference toggle: analyzer failure\n", stderr);
        return 30;
    }
    if ((enabled.interference_detected != 0U) ||
        (disabled.component_count != enabled.component_count) ||
        (disabled.spectrum_component_count !=
         enabled.spectrum_component_count) ||
        (absf_local(disabled.fundamental_hz -
                    enabled.fundamental_hz) > 1e-7f) ||
        (absf_local(disabled.true_rms_v -
                    enabled.true_rms_v) > 1e-7f) ||
        (absf_local(disabled.vpp_v - enabled.vpp_v) > 1e-7f) ||
        (absf_local(disabled.residual_rms_v -
                    enabled.residual_rms_v) > 1e-7f)) {
        fprintf(stderr,
                "no-interference toggle mismatch: detected=%u "
                "df=%g drms=%g dvpp=%g dres=%g\n",
                (unsigned)enabled.interference_detected,
                (double)(disabled.fundamental_hz -
                         enabled.fundamental_hz),
                (double)(disabled.true_rms_v -
                         enabled.true_rms_v),
                (double)(disabled.vpp_v - enabled.vpp_v),
                (double)(disabled.residual_rms_v -
                         enabled.residual_rms_v));
        return 31;
    }
    for (unsigned c = 0U; c < disabled.component_count; ++c) {
        if ((disabled.component[c].harmonic_order !=
             enabled.component[c].harmonic_order) ||
            (absf_local(disabled.component[c].peak_amplitude_v -
                        enabled.component[c].peak_amplitude_v) >
             1e-7f) ||
            (phase_error_rad(disabled.component[c].phase_rad,
                             enabled.component[c].phase_rad) >
             1e-7f)) {
            fprintf(stderr,
                    "no-interference toggle line%u mismatch\n",
                    c);
            return 32;
        }
    }
    puts("PASS interference enabled/disabled no-signal invariance");
    return 0;
}

static int run_interference_rejection_case(
    const char *name,
    float wanted_frequency_hz,
    float wanted_peak_v,
    float wanted_phase_rad,
    float interference_frequency_hz,
    float interference_peak_adc_v,
    float interference_phase_rad)
{
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    double interference_sum_sq = 0.0;
    const float aliased_interference_frequency_hz =
        (interference_frequency_hz > 0.5f * TEST_FS)
        ? TEST_FS - interference_frequency_hz
        : interference_frequency_hz;
    const uint8_t exact_nyquist =
        (absf_local(aliased_interference_frequency_hz -
                    0.5f * TEST_FS) < 0.01f) ? 1U : 0U;
    const float expected_observable_peak_adc_v =
        (exact_nyquist != 0U)
        ? absf_local(interference_peak_adc_v *
                     cosf(interference_phase_rad))
        : interference_peak_adc_v;
    const float wanted_magnitude =
        cic3_r16_magnitude(wanted_frequency_hz);
    const float wanted_response_phase =
        cic3_r16_phase_rad(wanted_frequency_hz);

    for (unsigned i = 0U; i < TEST_N; ++i) {
        const double time_s = (double)i / (double)TEST_FS;
        const double interference =
            (double)interference_peak_adc_v *
            cos(2.0 * TEST_PI_D *
                (double)interference_frequency_hz * time_s +
                (double)interference_phase_rad);
        const double wanted =
            (double)TEST_GAIN *
            (double)wanted_peak_v *
            (double)wanted_magnitude *
            sin(2.0 * TEST_PI_D *
                (double)wanted_frequency_hz * time_s +
                (double)wanted_phase_rad +
                (double)wanted_response_phase);
        samples[i] = (float)(wanted + interference);
        interference_sum_sq += interference * interference;
    }

    configure_cic_response(&cfg);
    enable_one_mhz_interference_rejection(&cfg);
    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK) {
        fprintf(stderr,
                "%s: analyzer returned %d\n",
                name,
                (int)result.status);
        return 40;
    }
    if ((result.interference_detected == 0U) ||
        (absf_local(result.interference_frequency_hz -
                    aliased_interference_frequency_hz) >
         ((exact_nyquist != 0U) ? 0.01f : 1.0f)) ||
        (absf_local(result.interference_post_dsp_peak_adc_v -
                    expected_observable_peak_adc_v) > 0.001f) ||
        (absf_local(result.interference_post_dsp_rms_adc_v -
                    (float)sqrt(interference_sum_sq /
                                (double)TEST_N)) > 0.001f)) {
        fprintf(stderr,
                "%s: interference detected=%u f=%f peak=%f rms=%f\n",
                name,
                (unsigned)result.interference_detected,
                result.interference_frequency_hz,
                result.interference_post_dsp_peak_adc_v,
                result.interference_post_dsp_rms_adc_v);
        return 41;
    }
    if ((result.component_count != 1U) ||
        (result.spectrum_component_count != 1U) ||
        (result.component[0].harmonic_order != 1U) ||
        (absf_local(result.fundamental_hz -
                    wanted_frequency_hz) > 1.0f) ||
        (absf_local(result.component[0].peak_amplitude_v -
                    wanted_peak_v) > 0.0001f) ||
        (phase_error_rad(result.component[0].phase_rad,
                         wanted_phase_rad) > 0.003f) ||
        (absf_local(result.true_rms_v -
                    wanted_peak_v / sqrtf(2.0f)) > 0.0001f) ||
        (absf_local(result.vpp_v -
                    2.0f * wanted_peak_v) > 0.0002f) ||
        (result.residual_rms_v > 0.0001f)) {
        fprintf(stderr,
                "%s: wanted count=%u spectrum=%u f=%f amp=%f "
                "phase=%f rms=%f vpp=%f residual=%f\n",
                name,
                (unsigned)result.component_count,
                (unsigned)result.spectrum_component_count,
                result.fundamental_hz,
                result.component[0].peak_amplitude_v,
                result.component[0].phase_rad,
                result.true_rms_v,
                result.vpp_v,
                result.residual_rms_v);
        return 42;
    }
    printf("PASS %-22s wanted=%6.0fHz nuisance=%9.3fHz "
           "peak=%0.6fV rms=%0.6fV\n",
           name,
           wanted_frequency_hz,
           result.interference_frequency_hz,
           result.interference_post_dsp_peak_adc_v,
           result.interference_post_dsp_rms_adc_v);
    return 0;
}

static int run_multiline_interference_rejection(void)
{
    static const TestCase tc = {
        "nuisance_multiline",
        10000.0f,
        3U,
        {1U, 20U, 50U},
        {0.100f, 0.080f, 0.060f},
        {0.31f, -0.77f, 1.13f}
    };
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    const float interference_frequency_hz = 997300.25f;
    const float interference_peak_adc_v = 3.0f;
    const float interference_phase_rad = 0.57f;
    float wanted_rms_sq = 0.0f;

    for (unsigned c = 0U; c < tc.count; ++c) {
        wanted_rms_sq += 0.5f * tc.amp[c] * tc.amp[c];
    }
    for (unsigned i = 0U; i < TEST_N; ++i) {
        const double time_s = (double)i / (double)TEST_FS;
        double measured_adc_v =
            (double)interference_peak_adc_v *
            cos(2.0 * TEST_PI_D *
                (double)interference_frequency_hz * time_s +
                (double)interference_phase_rad);
        for (unsigned c = 0U; c < tc.count; ++c) {
            const float frequency_hz =
                tc.f0 * (float)tc.order[c];
            measured_adc_v +=
                (double)TEST_GAIN *
                (double)tc.amp[c] *
                (double)cic3_r16_magnitude(frequency_hz) *
                sin(2.0 * TEST_PI_D *
                    (double)frequency_hz * time_s +
                    (double)tc.phase[c] +
                    (double)cic3_r16_phase_rad(frequency_hz));
        }
        samples[i] = (float)measured_adc_v;
    }

    configure_cic_response(&cfg);
    enable_one_mhz_interference_rejection(&cfg);
    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK) {
        fprintf(stderr,
                "nuisance multiline: analyzer returned %d\n",
                (int)result.status);
        return 50;
    }
    if ((result.interference_detected == 0U) ||
        (absf_local(result.interference_frequency_hz -
                    interference_frequency_hz) > 1.0f) ||
        (absf_local(result.interference_post_dsp_peak_adc_v -
                    interference_peak_adc_v) > 0.001f) ||
        (result.component_count != tc.count) ||
        (result.spectrum_component_count != tc.count) ||
        (absf_local(result.fundamental_hz - tc.f0) > 1.0f) ||
        (absf_local(result.true_rms_v -
                    sqrtf(wanted_rms_sq)) > 0.0001f) ||
        (absf_local(result.vpp_v -
                    expected_vpp(&tc)) > 0.0002f) ||
        (result.residual_rms_v > 0.0001f)) {
        fprintf(stderr,
                "nuisance multiline: detected=%u if=%f iamp=%f "
                "count=%u spectrum=%u f=%f rms=%f vpp=%f res=%f\n",
                (unsigned)result.interference_detected,
                result.interference_frequency_hz,
                result.interference_post_dsp_peak_adc_v,
                (unsigned)result.component_count,
                (unsigned)result.spectrum_component_count,
                result.fundamental_hz,
                result.true_rms_v,
                result.vpp_v,
                result.residual_rms_v);
        return 51;
    }
    for (unsigned c = 0U; c < tc.count; ++c) {
        if ((result.component[c].harmonic_order != tc.order[c]) ||
            (absf_local(result.component[c].peak_amplitude_v -
                        tc.amp[c]) > 0.0001f) ||
            (phase_error_rad(result.component[c].phase_rad,
                             tc.phase[c]) > 0.003f)) {
            fprintf(stderr,
                    "nuisance multiline line%u: order=%u amp=%f "
                    "phase=%f\n",
                    c,
                    (unsigned)result.component[c].harmonic_order,
                    result.component[c].peak_amplitude_v,
                    result.component[c].phase_rad);
            return 52;
        }
    }
    puts("PASS nuisance joint fit 10/200/500kHz lines");
    return 0;
}

static int run_below_threshold_case(void)
{
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    const float wanted_frequency_hz = 100000.0f;
    const float wanted_peak_v = 0.173f;
    const float wanted_phase_rad = 0.37f;
    const float interference_peak_adc_v = 0.0005f;

    for (unsigned i = 0U; i < TEST_N; ++i) {
        const double time_s = (double)i / (double)TEST_FS;
        samples[i] = (float)(
            (double)TEST_GAIN *
            (double)wanted_peak_v *
            (double)cic3_r16_magnitude(wanted_frequency_hz) *
            sin(2.0 * TEST_PI_D *
                (double)wanted_frequency_hz * time_s +
                (double)wanted_phase_rad +
                (double)cic3_r16_phase_rad(wanted_frequency_hz)) +
            (double)interference_peak_adc_v *
            cos(2.0 * TEST_PI_D * 1000000.0 * time_s));
    }
    configure_cic_response(&cfg);
    enable_one_mhz_interference_rejection(&cfg);
    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK) {
        fputs("below-threshold case: analyzer failure\n", stderr);
        return 60;
    }
    if ((result.interference_detected != 0U) ||
        (result.component_count != 1U) ||
        (absf_local(result.fundamental_hz -
                    wanted_frequency_hz) > 1.0f) ||
        (absf_local(result.component[0].peak_amplitude_v -
                    wanted_peak_v) > 0.0001f)) {
        fprintf(stderr,
                "below-threshold case: detected=%u f=%f amp=%f\n",
                (unsigned)result.interference_detected,
                result.fundamental_hz,
                result.component[0].peak_amplitude_v);
        return 61;
    }
    puts("PASS sub-1mV-RMS nuisance remains below detection threshold");
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
         {0.080f, 0.030f, 0.0f}, {0.90f, -1.20f, 0.0f}},
        {"edge_500k_single", 500000.0f, 1U, {1U, 0U, 0U},
         {0.173f, 0.0f, 0.0f}, {0.37f, 0.0f, 0.0f}},
        {"high_order_vpp", 10000.0f, 3U, {1U, 25U, 50U},
         {0.100f, 0.040f, 0.025f}, {0.31f, -0.77f, 1.13f}}
    };

    for (unsigned i = 0U; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        const int status = run_case(&cases[i]);
        if (status != 0) return status;
    }
    {
        const int status = run_cic_compensation_sweep();
        if (status != 0) return status;
    }
    {
        const int status = run_cic_multiline_compensation();
        if (status != 0) return status;
    }
    {
        const int status = run_cic_plus_empirical_calibration();
        if (status != 0) return status;
    }
    {
        const int status = compare_no_interference_toggle();
        if (status != 0) return status;
    }
    {
        const int status = run_interference_rejection_case(
            "exact_nyquist_10k",
            10000.0f,
            0.173f,
            0.37f,
            1000000.0f,
            3.0f,
            0.0f);
        if (status != 0) return status;
    }
    {
        const int status = run_interference_rejection_case(
            "offset_above_nyquist",
            123456.0f,
            0.173f,
            -0.61f,
            1002349.75f,
            3.0f,
            0.71f);
        if (status != 0) return status;
    }
    {
        const int status = run_interference_rejection_case(
            "near_nyquist_500k",
            500000.0f,
            0.173f,
            0.37f,
            999350.0f,
            3.0f,
            -0.40f);
        if (status != 0) return status;
    }
    {
        const int status = run_interference_rejection_case(
            "search_lower_edge",
            50000.0f,
            0.173f,
            -0.44f,
            995000.0f,
            3.0f,
            0.26f);
        if (status != 0) return status;
    }
    {
        const int status = run_interference_rejection_case(
            "very_near_nyquist",
            300000.0f,
            0.173f,
            0.18f,
            999900.0f,
            3.0f,
            0.90f);
        if (status != 0) return status;
    }
    {
        const int status = run_interference_rejection_case(
            "exact_nyquist_phase",
            200000.0f,
            0.173f,
            -0.21f,
            1000000.0f,
            3.0f,
            0.73f);
        if (status != 0) return status;
    }
    {
        const int status = run_multiline_interference_rejection();
        if (status != 0) return status;
    }
    {
        const int status = run_below_threshold_case();
        if (status != 0) return status;
    }
    puts("PERIODIC_ANALYZER_HOST_MATRIX_PASS");
    return 0;
}
