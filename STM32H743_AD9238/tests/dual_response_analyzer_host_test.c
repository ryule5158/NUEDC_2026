#include "dual_response_analyzer.h"

#include <math.h>
#include <stdio.h>

#define TEST_N       4096U
#define TEST_FS      2000000.0f
#define TEST_PI      3.14159265358979323846f
#define TEST_Q8_8_V  (10.0f / 65536.0f)

static float ch1[TEST_N];
static float ch2[TEST_N];

static float wrap_deg(float phase_deg)
{
    while (phase_deg >= 180.0f) phase_deg -= 360.0f;
    while (phase_deg < -180.0f) phase_deg += 360.0f;
    return phase_deg;
}

static int run_case(float frequency_hz, float gain, float phase_deg)
{
    const float input_peak_v = 0.125f;
    const float input_phase_rad = 0.31f;
    const float phase_rad = phase_deg * TEST_PI / 180.0f;
    DualResponse_Result result;

    for (unsigned i = 0U; i < TEST_N; ++i) {
        const float t = (float)i / TEST_FS;
        const float input = input_peak_v *
            sinf(2.0f * TEST_PI * frequency_hz * t + input_phase_rad) +
            0.0005f * sinf(4.0f * TEST_PI * frequency_hz * t - 0.2f);
        const float output = input_peak_v * gain *
            sinf(2.0f * TEST_PI * frequency_hz * t +
                 input_phase_rad + phase_rad) +
            0.0004f * sinf(4.0f * TEST_PI * frequency_hz * t + 0.8f);
        ch1[i] = roundf(input / TEST_Q8_8_V) * TEST_Q8_8_V;
        ch2[i] = roundf(output / TEST_Q8_8_V) * TEST_Q8_8_V;
    }

    if (DualResponse_Analyze(ch1, ch2, TEST_N, TEST_FS,
                             8500.0f, 505000.0f, &result) !=
        DUAL_RESPONSE_OK) {
        fprintf(stderr, "analyze failed f=%f status=%d\n",
                frequency_hz, (int)result.status);
        return 1;
    }
    if ((fabsf(result.frequency_hz - frequency_hz) > 25.0f) ||
        (fabsf(result.gain_linear - gain) > 0.005f) ||
        (fabsf(wrap_deg(result.phase_deg - phase_deg)) > 0.5f) ||
        (result.ch1_residual_ratio > 0.02f) ||
        (result.ch2_residual_ratio > 0.02f)) {
        fprintf(stderr,
                "mismatch f=%f measured=%f gain=%f phase=%f "
                "r1=%f r2=%f\n",
                frequency_hz, result.frequency_hz, result.gain_linear,
                result.phase_deg, result.ch1_residual_ratio,
                result.ch2_residual_ratio);
        return 2;
    }
    printf("PASS f=%9.3fHz gain=%1.6f phase=%8.4fdeg residual=%6.3f%%\n",
           result.frequency_hz, result.gain_linear, result.phase_deg,
           100.0f * result.ch2_residual_ratio);
    return 0;
}

int main(void)
{
    if (run_case(9000.0f, 0.92f, -12.5f) != 0) return 1;
    if (run_case(123456.0f, 0.63f, -57.0f) != 0) return 2;
    if (run_case(501000.0f, 0.31f, 143.0f) != 0) return 3;
    puts("DUAL_RESPONSE_ANALYZER_HOST_TEST_PASS");
    return 0;
}
