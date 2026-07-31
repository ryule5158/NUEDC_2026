#include "g_tjc_plot_scale.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_PI 3.14159265358979323846

static uint32_t s_rng = 0x92382026U;

static void Check(int condition, const char *message)
{
    if (!condition) {
        fprintf(stderr, "FAIL: %s\n", message);
        exit(1);
    }
}

static float RandomUnit(void)
{
    s_rng = 1664525U * s_rng + 1013904223U;
    return (float)(s_rng >> 8) / 16777215.0f;
}

static float Evaluate(const float amplitudes[3],
                      const uint8_t orders[3],
                      const float phases[3],
                      uint8_t count,
                      float turns)
{
    float value = 0.0f;

    for (uint8_t i = 0U; i < count; ++i) {
        value += amplitudes[i] *
                 sinf((float)(2.0 * TEST_PI) *
                      (float)orders[i] * turns +
                      phases[i]);
    }
    return value;
}

static void CheckTopicWaveformMatrix(void)
{
    static const float target_vpp_mv[3] = {50.0f, 100.0f, 250.0f};
    static const uint8_t cycle_options[2] = {1U, 3U};

    for (uint32_t trial = 0U; trial < 1000U; ++trial) {
        float amplitudes[3] = {
            0.2f + RandomUnit(),
            0.05f + RandomUnit(),
            0.05f + RandomUnit()
        };
        uint8_t orders[3] = {
            1U,
            (uint8_t)(2U + (uint8_t)(RandomUnit() * 24.0f)),
            (uint8_t)(26U + (uint8_t)(RandomUnit() * 25.0f))
        };
        float phases[3] = {
            (float)(2.0 * TEST_PI) * RandomUnit(),
            (float)(2.0 * TEST_PI) * RandomUnit(),
            (float)(2.0 * TEST_PI) * RandomUnit()
        };
        const uint8_t count = (uint8_t)(1U + trial % 3U);

        for (uint32_t target = 0U; target < 3U; ++target) {
            for (uint32_t cycle_index = 0U;
                 cycle_index < 2U;
                 ++cycle_index) {
                float values[G_TJC_PLOT_POINTS];
                float centered_sum = 0.0f;
                float maximum_abs;
                const uint8_t cycles = cycle_options[cycle_index];
                float gain;
                uint16_t full_scale_mv;

                for (uint32_t i = 0U;
                     i < G_TJC_PLOT_POINTS;
                     ++i) {
                    const float turns =
                        (float)cycles * (float)i /
                        (float)(G_TJC_PLOT_POINTS - 1U);
                    values[i] = Evaluate(amplitudes,
                                         orders,
                                         phases,
                                         count,
                                         turns);
                }
                {
                    float minimum = 1.0e30f;
                    float maximum = -1.0e30f;
                    for (uint32_t i = 0U;
                         i < G_TJC_PLOT_POINTS;
                         ++i) {
                        if (values[i] < minimum) minimum = values[i];
                        if (values[i] > maximum) maximum = values[i];
                    }
                    Check(maximum > minimum,
                          "generated waveform has no range");
                    gain = target_vpp_mv[target] / (maximum - minimum);
                }
                for (uint32_t i = 0U;
                     i < G_TJC_PLOT_POINTS;
                     ++i) {
                    values[i] *= gain;
                }
                maximum_abs = G_TjcPlotScale_RemoveDisplayDc(
                    values,
                    G_TJC_PLOT_POINTS);
                for (uint32_t i = 0U;
                     i < G_TJC_PLOT_POINTS;
                     ++i) {
                    centered_sum += values[i];
                }
                Check(fabsf(centered_sum) <
                          0.001f * (float)G_TJC_PLOT_POINTS,
                      "display DC center is not zero");
                full_scale_mv =
                    G_TjcPlotScale_SelectAmplitudeAxis(
                        1.15f * maximum_abs,
                        30U,
                        5000U);

                for (uint32_t i = 0U;
                     i < G_TJC_PLOT_POINTS;
                     ++i) {
                    const uint8_t code = G_TjcPlotScale_MapBipolar(
                        values[i],
                        (float)full_scale_mv);
                    Check(code < G_TJC_PLOT_HEIGHT,
                          "topic waveform exceeds component height");
                    Check(code > G_TJC_PLOT_CODE_MIN,
                          "topic waveform touches lower safety margin");
                    Check(code < G_TJC_PLOT_CODE_MAX,
                          "topic waveform touches upper safety margin");
                }
            }
        }
    }
}

static void CheckSpectrumMatrix(void)
{
    for (uint32_t trial = 0U; trial < 10000U; ++trial) {
        const float amplitude_mv = 25.0f + 100.0f * RandomUnit();
        const uint16_t full_scale_mv =
            G_TjcPlotScale_SelectAmplitudeAxis(
                1.15f * amplitude_mv,
                10U,
                5000U);
        const uint8_t code = G_TjcPlotScale_MapUnipolar(
            amplitude_mv,
            (float)full_scale_mv);
        const float reconstructed_mv =
            (float)code * (float)full_scale_mv /
            (float)(G_TJC_PLOT_HEIGHT - 1U);
        const float one_code_mv =
            (float)full_scale_mv /
            (float)(G_TJC_PLOT_HEIGHT - 1U);

        Check(code < G_TJC_PLOT_HEIGHT,
              "spectrum component exceeds component height");
        Check(code < G_TJC_PLOT_CODE_MAX,
              "spectrum component touches upper safety margin");
        Check(fabsf(reconstructed_mv - amplitude_mv) <= one_code_mv,
              "spectrum code and Y-axis scale disagree");
    }
}

static void CheckAxisCoordinateAgreement(void)
{
    static const float time_values_mv[7] = {
        -86.0f, -50.0f, -10.0f, 0.0f, 10.0f, 50.0f, 86.0f
    };
    const float axis_full_scale_mv = 100.0f;
    const float center =
        0.5f * (float)(G_TJC_PLOT_HEIGHT - 1U);
    const float one_code_mv = axis_full_scale_mv / center;

    for (uint8_t i = 0U; i < 7U; ++i) {
        const uint8_t code = G_TjcPlotScale_MapBipolar(
            time_values_mv[i],
            axis_full_scale_mv);
        const float reconstructed_mv =
            ((float)code - center) * axis_full_scale_mv / center;

        Check(fabsf(reconstructed_mv - time_values_mv[i]) <= one_code_mv,
              "time waveform pixel and Y-axis value disagree");
    }
}

static void CheckPhotographedSpectrumRegression(void)
{
    static const float frequencies_hz[3] = {
        166000.0f, 332000.0f, 498000.0f
    };
    static const float amplitudes_mv[3] = {
        25.0f, 36.0f, 38.0f
    };
    uint32_t x[3];
    uint8_t height[3];
    const uint16_t full_scale_mv =
        G_TjcPlotScale_SelectAmplitudeAxis(
            1.15f * amplitudes_mv[2],
            10U,
            5000U);

    for (uint8_t i = 0U; i < 3U; ++i) {
        x[i] = G_TjcPlotScale_MapFrequencyX(
            frequencies_hz[i],
            500000.0f);
        height[i] = G_TjcPlotScale_MapUnipolar(
            amplitudes_mv[i],
            (float)full_scale_mv);
    }

    Check((x[0] < x[1]) && (x[1] < x[2]),
          "166/332/498 kHz spectrum order is reversed");
    Check((height[0] < height[1]) && (height[1] < height[2]),
          "25/36/38 mV spectrum height order is reversed");
    Check(height[2] < G_TJC_PLOT_CODE_MAX,
          "photographed largest component touches the plot top");
}

int main(void)
{
    float asymmetric_values[3] = {60.0f, -130.0f, 25.0f};
    const float centered_maximum_abs =
        G_TjcPlotScale_RemoveDisplayDc(asymmetric_values, 3U);

    Check(G_TJC_PLOT_POINTS == 760U, "HMI plot width changed");
    Check(G_TJC_PLOT_HEIGHT == 232U, "HMI plot height changed");
    Check(G_TJC_PLOT_CODE_MAX < G_TJC_PLOT_HEIGHT,
          "maximum plot code is outside component");
    Check(G_TjcPlotScale_MapBipolar(0.0f, 100.0f) ==
              G_TJC_PLOT_HEIGHT / 2U,
          "zero is not centered");
    Check(G_TjcPlotScale_MapBipolar(-100.0f, 100.0f) ==
              G_TJC_PLOT_CODE_MIN,
          "negative full scale mapping");
    Check(G_TjcPlotScale_MapBipolar(100.0f, 100.0f) ==
              G_TJC_PLOT_CODE_MAX,
          "positive full scale mapping");
    Check(G_TjcPlotScale_MapBipolar(-1000.0f, 100.0f) ==
              G_TJC_PLOT_CODE_MIN,
          "negative saturation");
    Check(G_TjcPlotScale_MapBipolar(1000.0f, 100.0f) ==
              G_TJC_PLOT_CODE_MAX,
          "positive saturation");
    Check(G_TjcPlotScale_MapUnipolar(0.0f, 100.0f) == 0U,
          "spectrum baseline mapping");
    Check(G_TjcPlotScale_MapUnipolar(100.0f, 100.0f) ==
              G_TJC_PLOT_CODE_MAX,
          "spectrum full scale mapping");
    Check(G_TjcPlotScale_MapFrequencyX(0.0f, 500000.0f) ==
              G_TJC_PLOT_X_MIN,
          "frequency origin mapping");
    Check(G_TjcPlotScale_MapFrequencyX(250000.0f, 500000.0f) ==
              G_TJC_PLOT_POINTS / 2U,
          "frequency half-scale is not at the visual center");
    Check(G_TjcPlotScale_MapFrequencyX(500000.0f, 500000.0f) ==
              G_TJC_PLOT_X_MAX,
          "frequency full-scale mapping");
    Check(G_TjcPlotScale_SelectAmplitudeAxis(0.0f, 30U, 5000U) ==
              30U,
          "empty waveform amplitude axis");
    Check(G_TjcPlotScale_SelectAmplitudeAxis(61.1f, 30U, 5000U) ==
              70U,
          "amplitude axis is not rounded upward");
    Check(G_TjcPlotScale_SelectAmplitudeAxis(6000.0f, 30U, 5000U) ==
              5000U,
          "amplitude axis maximum limit");
    Check(G_TjcPlotScale_SelectFrequencyAxis(50000.0f) == 50000U,
          "50 kHz frequency axis");
    Check(G_TjcPlotScale_SelectFrequencyAxis(50001.0f) == 100000U,
          "100 kHz frequency axis");
    Check(G_TjcPlotScale_SelectFrequencyAxis(100001.0f) == 200000U,
          "200 kHz frequency axis");
    Check(G_TjcPlotScale_SelectFrequencyAxis(200001.0f) == 500000U,
          "500 kHz frequency axis");
    Check(fabsf(asymmetric_values[0] - 75.0f) < 0.001f,
          "positive peak DC centering");
    Check(fabsf(asymmetric_values[1] + 115.0f) < 0.001f,
          "negative peak DC centering");
    Check(fabsf(asymmetric_values[2] - 40.0f) < 0.001f,
          "third sample DC centering");
    Check(fabsf(centered_maximum_abs - 115.0f) < 0.001f,
          "asymmetric centered scale");
    Check(G_TjcPlotScale_RemoveDisplayDc(NULL, 3U) == 0.0f,
          "null waveform DC centering");

    CheckTopicWaveformMatrix();
    CheckSpectrumMatrix();
    CheckAxisCoordinateAgreement();
    CheckPhotographedSpectrumRegression();
    puts("G_TJC_PLOT_SCALE_HOST_PASS");
    return 0;
}
