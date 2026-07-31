#ifndef G_TJC_PLOT_SCALE_H
#define G_TJC_PLOT_SCALE_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/*
 * The delivered 480 x 272 HMI uses a 270 x 140 pixel waveform component.
 * TJC add/addt values are vertical pixel offsets; they are not normalized to
 * the component height.  Keep these dimensions synchronized with waveform
 * component ID 1 if the HMI layout is ever resized.
 */
#define G_TJC_PLOT_POINTS      270U
#define G_TJC_PLOT_HEIGHT      140U
#define G_TJC_PLOT_MARGIN        8U
#define G_TJC_PLOT_CODE_MIN    G_TJC_PLOT_MARGIN
#define G_TJC_PLOT_CODE_MAX    \
    (G_TJC_PLOT_HEIGHT - 1U - G_TJC_PLOT_MARGIN)

#if (2U * G_TJC_PLOT_MARGIN + 1U) >= G_TJC_PLOT_HEIGHT
#error "G TJC plot margin leaves no drawable vertical range"
#endif

/*
 * Display-only DC baseline normalization.
 *
 * Remove the arithmetic mean of the theoretical reconstructed display
 * points.  This places the DC/average center on the 0 grid line while
 * deliberately preserving any asymmetry between the positive and negative
 * peaks caused by harmonics.  It does not modify captured samples or any
 * reported measurement.  The return value is the centered peak magnitude
 * used to choose the display scale.
 */
static inline float G_TjcPlotScale_RemoveDisplayDc(float *values,
                                                   uint32_t count)
{
    float mean = 0.0f;
    float maximum_abs = 0.0f;
    uint32_t finite_count = 0U;

    if ((values == NULL) || (count == 0U)) return 0.0f;

    for (uint32_t i = 0U; i < count; ++i) {
        if (!isfinite(values[i])) continue;
        mean += values[i];
        finite_count++;
    }
    if (finite_count == 0U) return 0.0f;

    mean /= (float)finite_count;
    for (uint32_t i = 0U; i < count; ++i) {
        float absolute;

        if (!isfinite(values[i])) {
            values[i] = 0.0f;
            continue;
        }
        values[i] -= mean;
        absolute = fabsf(values[i]);
        if (absolute > maximum_abs) maximum_abs = absolute;
    }
    return maximum_abs;
}

static inline uint8_t G_TjcPlotScale_MapBipolar(float value,
                                                float full_scale)
{
    const float minimum = (float)G_TJC_PLOT_CODE_MIN;
    const float maximum = (float)G_TJC_PLOT_CODE_MAX;
    const float center = 0.5f * (minimum + maximum);
    const float half_span = 0.5f * (maximum - minimum);
    float normalized;
    float mapped;

    if (!isfinite(value) ||
        !isfinite(full_scale) ||
        !(full_scale > 0.0f)) {
        return (uint8_t)(center + 0.5f);
    }

    normalized = value / full_scale;
    if (normalized < -1.0f) normalized = -1.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    mapped = center + half_span * normalized;
    return (uint8_t)(mapped + 0.5f);
}

static inline uint8_t G_TjcPlotScale_MapUnipolar(float value,
                                                 float full_scale)
{
    float normalized;
    float mapped;

    if (!isfinite(value) ||
        !isfinite(full_scale) ||
        !(value > 0.0f) ||
        !(full_scale > 0.0f)) {
        return 0U;
    }

    normalized = value / full_scale;
    if (normalized > 1.0f) normalized = 1.0f;
    mapped = (float)G_TJC_PLOT_CODE_MAX * normalized;
    return (uint8_t)(mapped + 0.5f);
}

#endif
