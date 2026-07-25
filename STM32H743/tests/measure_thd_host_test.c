#include "Measure.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define TEST_PI 3.14159265358979323846
#define TEST_N 1024u

static int s_failures;

#define CHECK(condition, message)                                                \
    do {                                                                         \
        if (!(condition)) {                                                       \
            printf("FAIL: %s (line %d)\n", message, __LINE__);                   \
            s_failures++;                                                         \
        }                                                                         \
    } while (0)

static float Test_Abs(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void Test_BuildSpectrum(float fs,
                               float f0,
                               const float ratio[5],
                               float extra_freq,
                               float extra_amplitude,
                               float *mag)
{
    static float samples[TEST_N];
    double window_sum = 0.0;

    for (uint32_t i = 0u; i < TEST_N; i++) {
        double t = (double)i / (double)fs;
        double value = 0.21;
        for (uint32_t h = 1u; h <= 5u; h++) {
            double phase = 0.17 * (double)h;
            value += (double)ratio[h - 1u] *
                     sin(2.0 * TEST_PI * (double)h * (double)f0 * t + phase);
        }
        if (extra_amplitude > 0.0f) {
            value += (double)extra_amplitude *
                     sin(2.0 * TEST_PI * (double)extra_freq * t + 0.31);
        }
        value += 0.0005 * sin(2.0 * TEST_PI * 7311.0 * t + 0.9);
        samples[i] = (float)value;
    }

    for (uint32_t i = 0u; i < TEST_N; i++) {
        window_sum += 0.5 - 0.5 *
            cos(2.0 * TEST_PI * (double)i / (double)(TEST_N - 1u));
    }

    for (uint32_t k = 0u; k <= TEST_N / 2u; k++) {
        double real = 0.0;
        double imag = 0.0;
        for (uint32_t i = 0u; i < TEST_N; i++) {
            double window = 0.5 - 0.5 *
                cos(2.0 * TEST_PI * (double)i / (double)(TEST_N - 1u));
            double angle = -2.0 * TEST_PI *
                (double)k * (double)i / (double)TEST_N;
            double sample = (double)samples[i] * window;
            real += sample * cos(angle);
            imag += sample * sin(angle);
        }
        double scale = (k == 0u || k == TEST_N / 2u) ? 1.0 : 2.0;
        mag[k] = (float)(scale * sqrt(real * real + imag * imag) / window_sum);
    }
}

static void Test_LegacyAmplitudeTHD(void)
{
    const float amplitude[] = {1.0f, 0.3f, 0.4f, 0.0f, 0.0f};
    float thd_db = 0.0f;
    float thd = Measure_THD(amplitude, 5u, &thd_db);
    CHECK(Test_Abs(thd - 0.5f) < 1e-6f, "legacy THD ratio");
    CHECK(Test_Abs(thd_db + 6.0206f) < 0.001f, "legacy THD dB");
}

static void Test_NonCoherentTHD(void)
{
    static float mag[TEST_N / 2u + 1u];
    const float ratio[5] = {1.0f, 0.30f, 0.20f, 0.10f, 0.05f};
    const float fs = 200000.0f;
    const float f0 = fs * 17.37f / (float)TEST_N;
    const float expected = sqrtf(0.30f * 0.30f +
                                 0.20f * 0.20f +
                                 0.10f * 0.10f +
                                 0.05f * 0.05f);
    Measure_THDConfig_t config;
    Measure_THDResult_t result;

    Test_BuildSpectrum(fs, f0, ratio, 0.0f, 0.0f, mag);
    Measure_THDConfigDefault(&config);
    config.fund_min_hz = f0 - 1000.0f;
    config.fund_max_hz = f0 + 1000.0f;
    CHECK(Measure_THDFromSpectrum(mag, TEST_N, fs, &config, &result) ==
          MEASURE_THD_OK, "non-coherent THD status");
    CHECK(result.valid == 1u, "non-coherent THD valid");
    CHECK(result.harmonics_found == 5u, "H1-H5 returned");
    CHECK(Test_Abs(result.fund_freq - f0) < fs / (float)TEST_N,
          "interpolated fundamental frequency");
    CHECK(Test_Abs(result.thd - expected) < 0.01f,
          "non-coherent THD accuracy");
    CHECK(Test_Abs(result.harmonic_ratio[2] - 0.20f) < 0.01f,
          "normalized third harmonic");
    CHECK(Test_Abs(result.thd_percent - 100.0f * result.thd) < 1e-5f,
          "THD percent representation");
}

static void Test_SearchRangeRejectsStrongerSpur(void)
{
    static float mag[TEST_N / 2u + 1u];
    const float ratio[5] = {1.0f, 0.20f, 0.10f, 0.05f, 0.02f};
    const float fs = 200000.0f;
    const float f0 = 5000.0f;
    Measure_THDConfig_t config;
    Measure_THDResult_t result;

    Test_BuildSpectrum(fs, f0, ratio, 60000.0f, 1.5f, mag);
    Measure_THDConfigDefault(&config);
    config.fund_min_hz = 4000.0f;
    config.fund_max_hz = 6000.0f;
    CHECK(Measure_THDFromSpectrum(mag, TEST_N, fs, &config, &result) ==
          MEASURE_THD_OK, "range-limited search status");
    CHECK(Test_Abs(result.fund_freq - f0) < fs / (float)TEST_N,
          "strong out-of-band spur rejected");
}

static void Test_BandwidthAndResolutionStatus(void)
{
    static float mag[TEST_N / 2u + 1u];
    const float ratio[5] = {1.0f, 0.20f, 0.10f, 0.05f, 0.02f};
    const float fs = 200000.0f;
    Measure_THDConfig_t config;
    Measure_THDResult_t result;

    Test_BuildSpectrum(fs, 30000.0f, ratio, 0.0f, 0.0f, mag);
    Measure_THDConfigDefault(&config);
    config.fund_min_hz = 29000.0f;
    config.fund_max_hz = 31000.0f;
    CHECK(Measure_THDFromSpectrum(mag, TEST_N, fs, &config, &result) ==
          MEASURE_THD_ERROR_BANDWIDTH, "fifth harmonic bandwidth error");
    CHECK(result.harmonics_found == 3u, "partial harmonics preserved");
    CHECK(result.valid == 0u, "partial THD is not marked valid");

    memset(mag, 0, sizeof(mag));
    mag[3] = 1.0f;
    Measure_THDConfigDefault(&config);
    config.fund_min_hz = 2.0f * fs / (float)TEST_N;
    config.fund_max_hz = 4.0f * fs / (float)TEST_N;
    CHECK(Measure_THDFromSpectrum(mag, TEST_N, fs, &config, &result) ==
          MEASURE_THD_ERROR_RESOLUTION, "overlapping harmonic resolution error");
}

static void Test_OnePeriodExtraction(void)
{
    float input[350];
    float output[128];
    const float fs = 100000.0f;
    const float f0 = 1000.0f;
    for (uint32_t i = 0u; i < 350u; i++) {
        input[i] = 0.4f +
            1.2f * sinf((float)(2.0 * TEST_PI) * f0 * (float)i / fs + 0.7f);
    }
    CHECK(Measure_ExtractPeriod(input, 350u, fs, f0, output, 128u) == 128u,
          "one-period extraction count");
    CHECK(Test_Abs(output[0] - 0.4f) < 0.03f,
          "period starts at upward mean crossing");
    CHECK(output[1] > output[0], "period starts on rising edge");
    CHECK(Test_Abs(output[0] - output[127]) < 0.08f,
          "one-period endpoints are visually continuous");
}

int main(void)
{
    Test_LegacyAmplitudeTHD();
    Test_NonCoherentTHD();
    Test_SearchRangeRejectsStrongerSpur();
    Test_BandwidthAndResolutionStatus();
    Test_OnePeriodExtraction();

    if (s_failures != 0) {
        printf("measure_thd_host_test: %d failure(s)\n", s_failures);
        return 1;
    }
    printf("measure_thd_host_test: PASS\n");
    return 0;
}
