#include "g_tjc_plot_scale.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define TEST_PI 3.14159265358979323846

static uint32_t s_rng = 0x7432026U;

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

static uint16_t RoundScale10(float required,
                             uint16_t minimum,
                             uint16_t maximum)
{
    uint16_t units;
    uint16_t scale;

    if (!(required > 0.0f)) return minimum;
    units = (uint16_t)(required / 10.0f);
    if ((float)(10U * units) < required) units++;
    scale = (uint16_t)(10U * units);
    if (scale < minimum) scale = minimum;
    if (scale > maximum) scale = maximum;
    return scale;
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
                Check(fabsf(centered_sum) < 0.001f *
                          (float)G_TJC_PLOT_POINTS,
                      "display DC center is not zero");
                full_scale_mv = RoundScale10(1.15f * maximum_abs,
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
            RoundScale10(1.15f * amplitude_mv, 10U, 5000U);
        const uint8_t code = G_TjcPlotScale_MapUnipolar(
            amplitude_mv,
            (float)full_scale_mv);

        Check(code < G_TJC_PLOT_HEIGHT,
              "spectrum component exceeds component height");
        Check(code < G_TJC_PLOT_CODE_MAX,
              "spectrum component touches upper safety margin");
    }
}

int main(void)
{
    float asymmetric_values[3] = {60.0f, -130.0f, 25.0f};
    const float centered_maximum_abs =
        G_TjcPlotScale_RemoveDisplayDc(asymmetric_values, 3U);

    Check(G_TJC_PLOT_POINTS == 270U, "HMI plot width changed");
    Check(G_TJC_PLOT_HEIGHT == 140U, "HMI plot height changed");
    Check(G_TJC_PLOT_CODE_MAX < G_TJC_PLOT_HEIGHT,
          "maximum plot code is outside component");
    Check(G_TjcPlotScale_MapBipolar(0.0f, 100.0f) == 70U,
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
    puts("G_TJC_PLOT_SCALE_HOST_PASS");
    return 0;
}
