#include "periodic_analyzer.h"
#include <math.h>
#include <stdio.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_N 4096U
#define TEST_FS 2000000.0f
#define TEST_GAIN 8.0f
#define ADC_PHYSICAL_STEP_V (10.0f / 4096.0f)
#define TRANSPORT_STEP_V    (10.0f / 65536.0f)

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
    {100000.0f, 1.0124036f,  27.36f},
    {200000.0f, 1.0506745f,  54.72f},
    {300000.0f, 1.1181650f,  82.08f},
    {400000.0f, 1.2210863f, 109.44f},
    {500000.0f, 1.3696206f, 136.80f}
};

typedef struct {
    float frequency_hz;
    /*
     * Independently calculated in double precision from the committed
     * symmetric Q16 coefficient set. Keeping fixed expectations here catches
     * coefficient, normalization, delay, or API changes instead of merely
     * recomputing the implementation's own result.
     */
    float fir_amplitude_correction;
    float fir_phase_correction_deg;
    float digital_amplitude_correction;
    float digital_phase_correction_deg;
} FirDigitalExpectation;

static const FirDigitalExpectation fir_digital_expectation[] = {
    { 10000.0f, 1.000023209f,  11.016f, 1.000146036f,  13.752f},
    {100000.0f, 1.001425848f, 110.160f, 1.013846896f, 137.520f},
    {200000.0f, 1.000564578f, 220.320f, 1.051267416f, 275.040f},
    {300000.0f, 0.999954892f, 330.480f, 1.118114296f, 412.560f},
    {400000.0f, 1.001372232f, 440.640f, 1.222761548f, 550.080f},
    {450000.0f, 0.999862807f, 495.720f, 1.288636957f, 618.840f},
    {500000.0f, 1.001567712f, 550.800f, 1.371767392f, 687.600f}
};

typedef struct {
    float frequency_hz;
    float reference_vpp_v;
    float before_empirical_vpp_v;
} MeasuredResponseCase;

static const PeriodicAnalyzer_CalPoint measured_chain_profile[] = {
    {100000.0f, 0.999944044f, 0.0f},
    {200000.0f, 0.945184332f, 0.0f},
    {400000.0f, 0.946924430f, 0.0f}
};

static const MeasuredResponseCase measured_response_cases[] = {
    {100000.0f, 0.0250f, 0.025002010f},
    {100000.0f, 0.0250f, 0.025000788f},
    {200000.0f, 0.0125f, 0.013232144f},
    {200000.0f, 0.0125f, 0.013217731f},
    {400000.0f, 0.1400f, 0.147838654f},
    {400000.0f, 0.1900f, 0.200661011f}
};

typedef struct {
    const char *name;
    float f0_hz;
    float before_empirical_vpp_v[3];
    float reference_vpp_v[3];
    float reference_phase_rad[3];
} MeasuredMultiToneCase;

static const MeasuredMultiToneCase measured_multitone_cases[] = {
    {"field_frame_140mV", 100000.26f,
     {0.025002010f, 0.013232144f, 0.147838654f},
     {0.025000000f, 0.012500000f, 0.140000000f},
     {0.37f, -0.91f, 1.73f}},
    {"field_frame_190mV", 100000.25f,
     {0.025000788f, 0.013217731f, 0.200661011f},
     {0.025000000f, 0.012500000f, 0.190000000f},
     {-0.63f, 1.29f, -2.11f}}
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
        samples[i] =
            roundf((input_v * TEST_GAIN) / ADC_PHYSICAL_STEP_V) *
            ADC_PHYSICAL_STEP_V;
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
        samples[i] = roundf(adc_v / TRANSPORT_STEP_V) * TRANSPORT_STEP_V;
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

static int run_fir_digital_response_case(
    const FirDigitalExpectation *expectation)
{
    float fir_amplitude_correction;
    float fir_phase_correction_rad;
    float digital_amplitude_correction;
    float digital_phase_correction_rad;

    PeriodicAnalyzer_GetFir307Correction(
        expectation->frequency_hz,
        50000000.0f,
        &fir_amplitude_correction,
        &fir_phase_correction_rad);
    PeriodicAnalyzer_GetAd9238DigitalCorrection(
        expectation->frequency_hz,
        50000000.0f,
        &digital_amplitude_correction,
        &digital_phase_correction_rad);

    if ((absf_local(fir_amplitude_correction -
                    expectation->fir_amplitude_correction) > 3e-6f) ||
        (absf_local(fir_phase_correction_rad * 180.0f / TEST_PI -
                    expectation->fir_phase_correction_deg) > 0.003f) ||
        (absf_local(digital_amplitude_correction -
                    expectation->digital_amplitude_correction) > 4e-6f) ||
        (absf_local(digital_phase_correction_rad * 180.0f / TEST_PI -
                    expectation->digital_phase_correction_deg) > 0.004f)) {
        fprintf(stderr,
                "FIR/digital table mismatch f=%.0f "
                "fir_x=%.9f fir_phase=%.6fdeg "
                "digital_x=%.9f digital_phase=%.6fdeg\n",
                expectation->frequency_hz,
                fir_amplitude_correction,
                fir_phase_correction_rad * 180.0f / TEST_PI,
                digital_amplitude_correction,
                digital_phase_correction_rad * 180.0f / TEST_PI);
        return 30;
    }

    printf("PASS fir_digital_%6.0fk fir_x=%1.7f total_x=%1.7f "
           "delay_phase=%7.3fdeg\n",
           expectation->frequency_hz / 1000.0f,
           fir_amplitude_correction,
           digital_amplitude_correction,
           digital_phase_correction_rad * 180.0f / TEST_PI);
    return 0;
}

static int run_ad9238_digital_inverse_case(
    const FirDigitalExpectation *expectation)
{
    const float input_peak_v = 0.100f;
    const float input_phase_rad = 0.37f;
    const float reference_vpp_v = 2.0f * input_peak_v;
    const float reference_rms_v = input_peak_v / sqrtf(2.0f);
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    float amplitude_correction;
    float phase_correction_rad;
    float phase_error;

    PeriodicAnalyzer_GetAd9238DigitalCorrection(
        expectation->frequency_hz,
        50000000.0f,
        &amplitude_correction,
        &phase_correction_rad);
    for (unsigned i = 0U; i < TEST_N; ++i) {
        const float t = (float)i / TEST_FS;
        const float transport_v = input_peak_v * TEST_GAIN /
            amplitude_correction *
            sinf(2.0f * TEST_PI * expectation->frequency_hz * t +
                 input_phase_rad - phase_correction_rad);
        samples[i] = roundf(transport_v / TRANSPORT_STEP_V) *
                     TRANSPORT_STEP_V;
    }

    PeriodicAnalyzer_GetDefaultConfig(&cfg);
    cfg.sample_rate_hz = TEST_FS;
    cfg.min_frequency_hz = 10000.0f;
    cfg.max_frequency_hz = 500000.0f;
    cfg.input_volts_per_adc_volt = 1.0f / TEST_GAIN;
    cfg.min_component_peak_v = 0.0005f;
    cfg.relative_peak_threshold = 0.01f;
    PeriodicAnalyzer_EnableAd9238DigitalCompensation(&cfg, 50000000.0f);

    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK) {
        fprintf(stderr,
                "AD9238 digital inverse f=%.0f analyzer returned %d\n",
                expectation->frequency_hz,
                (int)result.status);
        return 31;
    }
    phase_error = wrap_phase_local(
        result.component[0].phase_rad - input_phase_rad);
    if ((result.component_count != 1U) ||
        (absf_local(result.fundamental_hz -
                    expectation->frequency_hz) > 25.0f) ||
        (absf_local(result.component[0].peak_amplitude_v -
                    input_peak_v) > 0.0015f) ||
        (absf_local(result.vpp_v - reference_vpp_v) > 0.005f) ||
        (absf_local(result.true_rms_v - reference_rms_v) > 0.005f) ||
        (absf_local(phase_error) > 0.015f)) {
        fprintf(stderr,
                "AD9238 digital inverse mismatch f=%.0f measured_f=%f "
                "amp=%f vpp=%f rms=%f phase=%f err=%f\n",
                expectation->frequency_hz,
                result.fundamental_hz,
                result.component[0].peak_amplitude_v,
                result.vpp_v,
                result.true_rms_v,
                result.component[0].phase_rad,
                phase_error);
        return 32;
    }

    printf("PASS ad9238_inverse_%6.0fk amp=%9.6fV vpp=%9.6fV "
           "rms=%9.6fV phase_err=%8.5frad\n",
           expectation->frequency_hz / 1000.0f,
           result.component[0].peak_amplitude_v,
           result.vpp_v,
           result.true_rms_v,
           phase_error);
    return 0;
}
static int run_measured_response_case(const MeasuredResponseCase *tc)
{
    const float input_phase_rad = 0.37f;
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    float cic_amplitude_correction;
    float cic_phase_correction_rad;
    float measured_vpp_v;

    PeriodicAnalyzer_GetCic3R25Correction(
        tc->frequency_hz,
        50000000.0f,
        &cic_amplitude_correction,
        &cic_phase_correction_rad);
    for (unsigned i = 0U; i < TEST_N; ++i) {
        const float t = (float)i / TEST_FS;
        const float transport_peak_v =
            0.5f * tc->before_empirical_vpp_v /
            cic_amplitude_correction;
        const float transport_v = transport_peak_v *
            sinf(2.0f * TEST_PI * tc->frequency_hz * t +
                 input_phase_rad - cic_phase_correction_rad);
        samples[i] = roundf(transport_v / TRANSPORT_STEP_V) *
                     TRANSPORT_STEP_V;
    }

    PeriodicAnalyzer_GetDefaultConfig(&cfg);
    cfg.sample_rate_hz = TEST_FS;
    cfg.min_frequency_hz = 10000.0f;
    cfg.max_frequency_hz = 500000.0f;
    cfg.input_volts_per_adc_volt = 1.0f;
    cfg.min_component_peak_v = 0.0005f;
    cfg.relative_peak_threshold = 0.01f;
    PeriodicAnalyzer_EnableCic3R25Compensation(&cfg, 50000000.0f);
    cfg.cal_count = (uint8_t)(sizeof(measured_chain_profile) /
                              sizeof(measured_chain_profile[0]));
    for (uint8_t i = 0U; i < cfg.cal_count; ++i) {
        cfg.cal[i] = measured_chain_profile[i];
    }

    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK || result.component_count != 1U) {
        fprintf(stderr,
                "measured profile f=%.0f analyzer status=%d lines=%u\n",
                tc->frequency_hz,
                (int)result.status,
                (unsigned)result.component_count);
        return 20;
    }
    measured_vpp_v = 2.0f * result.component[0].peak_amplitude_v;
    if (absf_local(measured_vpp_v - tc->reference_vpp_v) > 0.00020f) {
        fprintf(stderr,
                "measured profile mismatch f=%.0f expected_vpp=%f actual_vpp=%f\n",
                tc->frequency_hz,
                tc->reference_vpp_v,
                measured_vpp_v);
        return 21;
    }
    printf("PASS measured_profile_%6.0fk before=%9.6fmVpp after=%9.6fmVpp\n",
           tc->frequency_hz / 1000.0f,
           1000.0f * tc->before_empirical_vpp_v,
           1000.0f * measured_vpp_v);
    return 0;
}

static void load_measured_profile(PeriodicAnalyzer_Config *cfg)
{
    cfg->cal_count = (uint8_t)(sizeof(measured_chain_profile) /
                               sizeof(measured_chain_profile[0]));
    for (uint8_t i = 0U; i < cfg->cal_count; ++i) {
        cfg->cal[i] = measured_chain_profile[i];
    }
}

static int run_measured_multitone_case(const MeasuredMultiToneCase *tc)
{
    static const unsigned orders[3] = {1U, 2U, 4U};
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    TestCase reference = {0};
    float reference_rms_sq = 0.0f;
    float reference_vpp;

    reference.name = tc->name;
    reference.f0 = tc->f0_hz;
    reference.count = 3U;
    for (unsigned c = 0U; c < 3U; ++c) {
        reference.order[c] = orders[c];
        reference.amp[c] = 0.5f * tc->reference_vpp_v[c];
        reference.phase[c] = tc->reference_phase_rad[c];
        reference_rms_sq += 0.5f * reference.amp[c] * reference.amp[c];
    }
    reference_vpp = expected_vpp(&reference);

    for (unsigned i = 0U; i < TEST_N; ++i) {
        const float t = (float)i / TEST_FS;
        float transport_v = 0.0f;
        for (unsigned c = 0U; c < 3U; ++c) {
            const float frequency_hz = tc->f0_hz * (float)orders[c];
            float cic_amplitude_correction;
            float cic_phase_correction_rad;
            PeriodicAnalyzer_GetCic3R25Correction(
                frequency_hz,
                50000000.0f,
                &cic_amplitude_correction,
                &cic_phase_correction_rad);
            transport_v +=
                (0.5f * tc->before_empirical_vpp_v[c] /
                 cic_amplitude_correction) *
                sinf(2.0f * TEST_PI * frequency_hz * t +
                     tc->reference_phase_rad[c] -
                     cic_phase_correction_rad);
        }
        samples[i] = roundf(transport_v / TRANSPORT_STEP_V) *
                     TRANSPORT_STEP_V;
    }

    PeriodicAnalyzer_GetDefaultConfig(&cfg);
    cfg.sample_rate_hz = TEST_FS;
    cfg.min_frequency_hz = 10000.0f;
    cfg.max_frequency_hz = 500000.0f;
    cfg.input_volts_per_adc_volt = 1.0f;
    cfg.min_component_peak_v = 0.0005f;
    cfg.relative_peak_threshold = 0.01f;
    PeriodicAnalyzer_EnableCic3R25Compensation(&cfg, 50000000.0f);
    load_measured_profile(&cfg);

    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK || result.component_count != 3U ||
        absf_local(result.fundamental_hz - tc->f0_hz) > 2.0f) {
        fprintf(stderr,
                "%s analyzer status=%d lines=%u f0=%f\n",
                tc->name,
                (int)result.status,
                (unsigned)result.component_count,
                result.fundamental_hz);
        return 22;
    }

    for (unsigned c = 0U; c < 3U; ++c) {
        uint8_t spectrum_match = 0U;
        const float measured_vpp_v =
            2.0f * result.component[c].peak_amplitude_v;
        if ((result.component[c].harmonic_order != orders[c]) ||
            (absf_local(measured_vpp_v -
                        tc->reference_vpp_v[c]) > 0.00020f)) {
            fprintf(stderr,
                    "%s H%u expected=%fmVpp actual=%fmVpp\n",
                    tc->name,
                    orders[c],
                    1000.0f * tc->reference_vpp_v[c],
                    1000.0f * measured_vpp_v);
            return 23;
        }
        for (uint8_t i = 0U;
             i < result.spectrum_component_count;
             ++i) {
            if ((result.spectrum[i].harmonic_order == orders[c]) &&
                (absf_local(result.spectrum[i].peak_amplitude_v -
                            result.component[c].peak_amplitude_v) < 1e-7f)) {
                spectrum_match = 1U;
                break;
            }
        }
        if (spectrum_match == 0U) {
            fprintf(stderr, "%s H%u spectrum was not LS-replaced\n",
                    tc->name, orders[c]);
            return 24;
        }
    }

    if ((absf_local(result.true_rms_v - sqrtf(reference_rms_sq)) >
         0.00020f) ||
        (absf_local(result.vpp_v - reference_vpp) > 0.00050f) ||
        (result.residual_rms_v > 0.00025f)) {
        fprintf(stderr,
                "%s aggregate rms=%f expected=%f vpp=%f expected=%f residual=%f\n",
                tc->name,
                result.true_rms_v,
                sqrtf(reference_rms_sq),
                result.vpp_v,
                reference_vpp,
                result.residual_rms_v);
        return 25;
    }

    printf("PASS %-18s H1=%9.6f H2=%9.6f H4=%9.6fmVpp residual=%7.4fmV\n",
           tc->name,
           2000.0f * result.component[0].peak_amplitude_v,
           2000.0f * result.component[1].peak_amplitude_v,
           2000.0f * result.component[2].peak_amplitude_v,
           1000.0f * result.residual_rms_v);
    return 0;
}

static int run_ad9238_measured_multitone_case(
    const MeasuredMultiToneCase *tc)
{
    static const unsigned orders[3] = {1U, 2U, 4U};
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    TestCase reference = {0};
    float reference_rms_sq = 0.0f;
    float reference_vpp;

    reference.name = tc->name;
    reference.f0 = tc->f0_hz;
    reference.count = 3U;
    for (unsigned c = 0U; c < 3U; ++c) {
        reference.order[c] = orders[c];
        reference.amp[c] = 0.5f * tc->reference_vpp_v[c];
        reference.phase[c] = tc->reference_phase_rad[c];
        reference_rms_sq += 0.5f * reference.amp[c] * reference.amp[c];
    }
    reference_vpp = expected_vpp(&reference);

    for (unsigned i = 0U; i < TEST_N; ++i) {
        const float t = (float)i / TEST_FS;
        float transport_v = 0.0f;
        for (unsigned c = 0U; c < 3U; ++c) {
            const float frequency_hz = tc->f0_hz * (float)orders[c];
            float digital_amplitude_correction;
            float digital_phase_correction_rad;
            PeriodicAnalyzer_GetAd9238DigitalCorrection(
                frequency_hz,
                50000000.0f,
                &digital_amplitude_correction,
                &digital_phase_correction_rad);
            transport_v +=
                (0.5f * tc->before_empirical_vpp_v[c] /
                 digital_amplitude_correction) *
                sinf(2.0f * TEST_PI * frequency_hz * t +
                     tc->reference_phase_rad[c] -
                     digital_phase_correction_rad);
        }
        samples[i] = roundf(transport_v / TRANSPORT_STEP_V) *
                     TRANSPORT_STEP_V;
    }

    PeriodicAnalyzer_GetDefaultConfig(&cfg);
    cfg.sample_rate_hz = TEST_FS;
    cfg.min_frequency_hz = 10000.0f;
    cfg.max_frequency_hz = 500000.0f;
    cfg.input_volts_per_adc_volt = 1.0f;
    cfg.min_component_peak_v = 0.0005f;
    cfg.relative_peak_threshold = 0.01f;
    PeriodicAnalyzer_EnableAd9238DigitalCompensation(&cfg, 50000000.0f);
    load_measured_profile(&cfg);

    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK || result.component_count != 3U ||
        absf_local(result.fundamental_hz - tc->f0_hz) > 2.0f) {
        fprintf(stderr,
                "AD9238 %s analyzer status=%d lines=%u f0=%f\n",
                tc->name,
                (int)result.status,
                (unsigned)result.component_count,
                result.fundamental_hz);
        return 33;
    }

    for (unsigned c = 0U; c < 3U; ++c) {
        const float measured_vpp_v =
            2.0f * result.component[c].peak_amplitude_v;
        const float phase_error = wrap_phase_local(
            result.component[c].phase_rad - tc->reference_phase_rad[c]);
        if ((result.component[c].harmonic_order != orders[c]) ||
            (absf_local(measured_vpp_v -
                        tc->reference_vpp_v[c]) > 0.00025f) ||
            (absf_local(phase_error) > 0.020f)) {
            fprintf(stderr,
                    "AD9238 %s H%u expected=%fmVpp actual=%fmVpp "
                    "phase_err=%frad\n",
                    tc->name,
                    orders[c],
                    1000.0f * tc->reference_vpp_v[c],
                    1000.0f * measured_vpp_v,
                    phase_error);
            return 34;
        }
    }

    if ((absf_local(result.true_rms_v - sqrtf(reference_rms_sq)) >
         0.00025f) ||
        (absf_local(result.vpp_v - reference_vpp) > 0.00060f) ||
        (result.residual_rms_v > 0.00025f)) {
        fprintf(stderr,
                "AD9238 %s aggregate rms=%f expected=%f "
                "vpp=%f expected=%f residual=%f\n",
                tc->name,
                result.true_rms_v,
                sqrtf(reference_rms_sq),
                result.vpp_v,
                reference_vpp,
                result.residual_rms_v);
        return 35;
    }

    printf("PASS ad9238_%-10s H1=%9.6f H2=%9.6f H4=%9.6fmVpp "
           "vpp_err=%7.4fmV rms_err=%7.4fmV\n",
           tc->name,
           2000.0f * result.component[0].peak_amplitude_v,
           2000.0f * result.component[1].peak_amplitude_v,
           2000.0f * result.component[2].peak_amplitude_v,
           1000.0f * (result.vpp_v - reference_vpp),
           1000.0f * (result.true_rms_v - sqrtf(reference_rms_sq)));
    return 0;
}

static int run_ad9238_500k_waveform_case(void)
{
    static const TestCase reference = {
        "ad9238_500k_edge", 100000.0f, 3U, {1U, 2U, 5U},
        {0.0125f, 0.00625f, 0.0950f}, {0.37f, -0.91f, 1.73f}
    };
    PeriodicAnalyzer_Config cfg;
    PeriodicAnalyzer_Result result;
    float reference_rms_sq = 0.0f;
    const float reference_vpp = expected_vpp(&reference);

    for (unsigned i = 0U; i < TEST_N; ++i) {
        const float t = (float)i / TEST_FS;
        float transport_v = 0.0f;
        for (unsigned c = 0U; c < reference.count; ++c) {
            const float frequency_hz =
                reference.f0 * (float)reference.order[c];
            float digital_amplitude_correction;
            float digital_phase_correction_rad;
            PeriodicAnalyzer_GetAd9238DigitalCorrection(
                frequency_hz,
                50000000.0f,
                &digital_amplitude_correction,
                &digital_phase_correction_rad);
            transport_v +=
                reference.amp[c] * TEST_GAIN /
                digital_amplitude_correction *
                sinf(2.0f * TEST_PI * frequency_hz * t +
                     reference.phase[c] - digital_phase_correction_rad);
        }
        samples[i] = roundf(transport_v / TRANSPORT_STEP_V) *
                     TRANSPORT_STEP_V;
    }

    PeriodicAnalyzer_GetDefaultConfig(&cfg);
    cfg.sample_rate_hz = TEST_FS;
    cfg.min_frequency_hz = 10000.0f;
    cfg.max_frequency_hz = 500000.0f;
    cfg.input_volts_per_adc_volt = 1.0f / TEST_GAIN;
    cfg.min_component_peak_v = 0.0005f;
    cfg.relative_peak_threshold = 0.01f;
    PeriodicAnalyzer_EnableAd9238DigitalCompensation(&cfg, 50000000.0f);

    if (PeriodicAnalyzer_Analyze(samples, TEST_N, &cfg, &result) !=
        PERIODIC_ANALYZER_OK || result.component_count != reference.count ||
        absf_local(result.fundamental_hz - reference.f0) > 25.0f) {
        fprintf(stderr,
                "AD9238 500k waveform analyzer status=%d lines=%u f0=%f\n",
                (int)result.status,
                (unsigned)result.component_count,
                result.fundamental_hz);
        return 36;
    }

    for (unsigned c = 0U; c < reference.count; ++c) {
        const float phase_error = wrap_phase_local(
            result.component[c].phase_rad - reference.phase[c]);
        reference_rms_sq +=
            0.5f * reference.amp[c] * reference.amp[c];
        if ((result.component[c].harmonic_order != reference.order[c]) ||
            (absf_local(result.component[c].peak_amplitude_v -
                        reference.amp[c]) > 0.0015f) ||
            (absf_local(phase_error) > 0.020f)) {
            fprintf(stderr,
                    "AD9238 500k waveform H%u amp=%f expected=%f "
                    "phase_err=%f\n",
                    reference.order[c],
                    result.component[c].peak_amplitude_v,
                    reference.amp[c],
                    phase_error);
            return 37;
        }
    }

    if ((absf_local(result.vpp_v - reference_vpp) >= 0.005f) ||
        (absf_local(result.true_rms_v - sqrtf(reference_rms_sq)) >=
         0.005f)) {
        fprintf(stderr,
                "AD9238 500k waveform vpp=%f expected=%f "
                "rms=%f expected=%f\n",
                result.vpp_v,
                reference_vpp,
                result.true_rms_v,
                sqrtf(reference_rms_sq));
        return 38;
    }

    printf("PASS ad9238_500k_waveform vpp_err=%7.4fmV "
           "rms_err=%7.4fmV H5=%9.6fmVpp\n",
           1000.0f * (result.vpp_v - reference_vpp),
           1000.0f * (result.true_rms_v - sqrtf(reference_rms_sq)),
           2000.0f * result.component[2].peak_amplitude_v);
    return 0;
}
static int run_empirical_boundary_test(void)
{
    static const float frequencies_hz[] = {
        50000.0f, 100000.0f, 300000.0f, 400000.0f, 450000.0f, 500000.0f
    };
    PeriodicAnalyzer_Config cfg;

    PeriodicAnalyzer_GetDefaultConfig(&cfg);
    PeriodicAnalyzer_EnableCic3R25Compensation(&cfg, 50000000.0f);
    load_measured_profile(&cfg);
    cfg.empirical_hold_last_until_hz = 500000.0f;
    for (unsigned i = 0U;
         i < sizeof(frequencies_hz) / sizeof(frequencies_hz[0]);
         ++i) {
        float cic_amp;
        float total_amp;
        float empirical_amp;
        PeriodicAnalyzer_EmpiricalStatus empirical_status;
        const PeriodicAnalyzer_EmpiricalStatus expected_status =
            (frequencies_hz[i] < 100000.0f) ?
                PERIODIC_ANALYZER_EMPIRICAL_UNCALIBRATED :
            ((frequencies_hz[i] <= 400000.0f) ?
                PERIODIC_ANALYZER_EMPIRICAL_TABLE :
                PERIODIC_ANALYZER_EMPIRICAL_INFERRED_HOLD);

        PeriodicAnalyzer_GetCic3R25Correction(
            frequencies_hz[i], 50000000.0f, &cic_amp, NULL);
        PeriodicAnalyzer_GetAppliedCorrection(
            &cfg,
            frequencies_hz[i],
            &total_amp,
            NULL,
            &empirical_amp,
            &empirical_status);
        if ((empirical_status != expected_status) ||
            ((expected_status ==
              PERIODIC_ANALYZER_EMPIRICAL_UNCALIBRATED) &&
             ((absf_local(empirical_amp - 1.0f) > 1e-7f) ||
              (absf_local(total_amp - cic_amp) > 2e-6f))) ||
            ((expected_status ==
              PERIODIC_ANALYZER_EMPIRICAL_INFERRED_HOLD) &&
             (absf_local(empirical_amp - 0.946924430f) > 1e-7f))) {
            fprintf(stderr,
                    "empirical boundary f=%f status=%u chain=%f total=%f cic=%f\n",
                    frequencies_hz[i],
                    (unsigned)empirical_status,
                    empirical_amp,
                    total_amp,
                    cic_amp);
            return 26;
        }
    }
    puts("PERIODIC_ANALYZER_EMPIRICAL_BOUNDARY_STATUS_PASS");
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
    for (unsigned i = 0U;
         i < sizeof(fir_digital_expectation) /
             sizeof(fir_digital_expectation[0]);
         ++i) {
        int status = run_fir_digital_response_case(
            &fir_digital_expectation[i]);
        if (status != 0) return status;
        status = run_ad9238_digital_inverse_case(
            &fir_digital_expectation[i]);
        if (status != 0) return status;
    }
    for (unsigned i = 0U;
         i < sizeof(measured_response_cases) /
             sizeof(measured_response_cases[0]);
         ++i) {
        const int status = run_measured_response_case(
            &measured_response_cases[i]);
        if (status != 0) return status;
    }
    for (unsigned i = 0U;
         i < sizeof(measured_multitone_cases) /
             sizeof(measured_multitone_cases[0]);
         ++i) {
        const int status = run_measured_multitone_case(
            &measured_multitone_cases[i]);
        if (status != 0) return status;
        if (run_ad9238_measured_multitone_case(
                &measured_multitone_cases[i]) != 0) {
            return 39;
        }
    }
    if (run_ad9238_500k_waveform_case() != 0) return 40;
    if (run_empirical_boundary_test() != 0) return 27;
    puts("PERIODIC_ANALYZER_MEASURED_RESPONSE_ARITHMETIC_REPLAY_PASS");
    puts("PERIODIC_ANALYZER_FIELD_MULTITONE_REGRESSION_PASS");
    puts("PERIODIC_ANALYZER_CIC3_R25_INVERSE_PASS");
    puts("PERIODIC_ANALYZER_FIR307_Q16_RESPONSE_PASS");
    puts("PERIODIC_ANALYZER_AD9238_DIGITAL_INVERSE_PASS");
    puts("PERIODIC_ANALYZER_AD9238_500KHZ_WAVEFORM_PASS");
    puts("PERIODIC_ANALYZER_HOST_MATRIX_PASS");
    return 0;
}
