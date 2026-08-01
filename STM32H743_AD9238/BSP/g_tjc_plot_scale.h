#ifndef G_TJC_PLOT_SCALE_H
#define G_TJC_PLOT_SCALE_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/*
 * 7 寸 800x480 串口屏中的 s0 波形控件为 760x232，ID=1、通道=0。
 * add/addt 数据码按控件纵向像素使用，尺寸变化时必须同步修改这里。
 */
#define G_TJC_PLOT_POINTS      760U
#define G_TJC_PLOT_HEIGHT      232U
#define G_TJC_PLOT_MARGIN        8U
#define G_TJC_PLOT_CODE_MIN    G_TJC_PLOT_MARGIN
#define G_TJC_PLOT_CODE_MAX    \
    (G_TJC_PLOT_HEIGHT - 1U - G_TJC_PLOT_MARGIN)
#define G_TJC_PLOT_X_MIN         1U
#define G_TJC_PLOT_X_MAX        (G_TJC_PLOT_POINTS - 2U)
#define G_TJC_TWO_PI_F           6.28318530717958647692f

#if G_TJC_PLOT_HEIGHT > 256U
#error "TJC waveform raw data is limited to one byte per point"
#endif

#if (2U * G_TJC_PLOT_MARGIN + 1U) >= G_TJC_PLOT_HEIGHT
#error "G TJC plot margin leaves no drawable vertical range"
#endif

/*
 * 时域窗口采用半开区间 [0, cycles)，不能把 cycles 本身作为最后一点。
 * 否则左右两端会重复同一相位，峰值恰好位于周期边界时会看起来多出
 * 一个周期特征。cycles 只允许题目界面的 1 或 3，其余值回退到 1。
 */
static inline float G_TjcPlotScale_TimePhaseTurns(uint32_t point,
                                                  uint8_t cycles)
{
    const uint8_t display_cycles = (cycles == 3U) ? 3U : 1U;
    const uint32_t bounded_point =
        (point < G_TJC_PLOT_POINTS) ? point : (G_TJC_PLOT_POINTS - 1U);

    return (float)display_cycles * (float)bounded_point /
           (float)G_TJC_PLOT_POINTS;
}

/*
 * 把基波正弦相位锁定到屏幕左边缘的上升过零点。返回 0～1 周的偏移，
 * 只改变显示窗口起点，不改变谐波相对相位、测量值或频谱结果。
 */
static inline float G_TjcPlotScale_TriggerOffsetTurns(
    float fundamental_phase_rad)
{
    float turns;

    if (!isfinite(fundamental_phase_rad)) return 0.0f;
    turns = -fundamental_phase_rad / G_TJC_TWO_PI_F;
    turns -= floorf(turns);
    return turns;
}

/* 按10mV向上取整坐标满量程，并限制在题目硬件的显示范围内。 */
static inline uint16_t G_TjcPlotScale_SelectAmplitudeAxis(
    float required_mv,
    uint16_t minimum_mv,
    uint16_t maximum_mv)
{
    uint32_t units;
    uint32_t scale;

    if (!isfinite(required_mv) || !(required_mv > 0.0f)) {
        return minimum_mv;
    }
    if (required_mv >= (float)maximum_mv) return maximum_mv;

    units = (uint32_t)(required_mv / 10.0f);
    if ((float)(10U * units) < required_mv) units++;
    scale = 10U * units;
    if (scale < minimum_mv) scale = minimum_mv;
    if (scale > maximum_mv) scale = maximum_mv;
    return (uint16_t)scale;
}

/* 根据最高频率逐帧选择最小可容纳的横轴档位。 */
static inline uint32_t G_TjcPlotScale_SelectFrequencyAxis(float maximum_hz)
{
    if (!isfinite(maximum_hz) || (maximum_hz <= 50000.0f)) {
        return 50000U;
    }
    if (maximum_hz <= 100000.0f) return 100000U;
    if (maximum_hz <= 200000.0f) return 200000U;
    return 500000U;
}

/*
 * 仅对显示重建波形去直流均值，让 0 mV 与屏幕中线一致。
 * 测量结果、FFT 分量和原始采样均不受此处理影响。
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

/* 将 -full_scale～+full_scale 线性映射到波形控件安全绘图区。 */
static inline uint8_t G_TjcPlotScale_MapBipolar(float value,
                                                float full_scale)
{
    const float center = 0.5f * (float)(G_TJC_PLOT_HEIGHT - 1U);
    const float half_span = center;
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
    /*
     * 坐标斜率按控件完整 0..219 高度计算，因此轴上数值与像素位置
     * 一致；正常波形由 15% 自动余量保证不会到边。这里只在异常或
     * 超量程时限到 8 像素安全边界，避免曲线贴框后看不见。
     */
    if (mapped < (float)G_TJC_PLOT_CODE_MIN) {
        mapped = (float)G_TJC_PLOT_CODE_MIN;
    }
    if (mapped > (float)G_TJC_PLOT_CODE_MAX) {
        mapped = (float)G_TJC_PLOT_CODE_MAX;
    }
    return (uint8_t)(mapped + 0.5f);
}

/* 将 0～full_scale 的频谱峰值线性映射到纵轴安全高度。 */
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
    /*
     * 纵轴 0..full_scale 对应控件完整 0..219 像素。15% 自动余量
     * 使正常最高谱线低于安全上边界，只有过载才会被限幅到 211。
     */
    mapped = (float)(G_TJC_PLOT_HEIGHT - 1U) * normalized;
    if (mapped > (float)G_TJC_PLOT_CODE_MAX) {
        mapped = (float)G_TJC_PLOT_CODE_MAX;
    }
    return (uint8_t)(mapped + 0.5f);
}

/*
 * 频率 0～full_scale 映射到曲线左右安全边界。使用四舍五入使半量程
 * 精确落在 660 点控件的视觉中线，修改控件宽度时无需改业务代码。
 */
static inline uint32_t G_TjcPlotScale_MapFrequencyX(float frequency_hz,
                                                    float full_scale_hz)
{
    float normalized;
    float mapped;

    if (!isfinite(frequency_hz) ||
        !isfinite(full_scale_hz) ||
        !(frequency_hz > 0.0f) ||
        !(full_scale_hz > 0.0f)) {
        return G_TJC_PLOT_X_MIN;
    }

    normalized = frequency_hz / full_scale_hz;
    if (normalized > 1.0f) normalized = 1.0f;
    mapped = normalized *
             (float)(G_TJC_PLOT_X_MAX - G_TJC_PLOT_X_MIN);
    return G_TJC_PLOT_X_MIN + (uint32_t)(mapped + 0.5f);
}

#endif
