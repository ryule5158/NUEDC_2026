/**
 * @file    FilterEx.c
 * @brief   Implementation of allocation-free filtering extensions.
 */
#include "FilterEx.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define FILTEREX_PI       3.14159265358979323846f /* 单精度圆周率。 */
#define FILTEREX_TWO_PI   6.28318530717958647692f /* 单精度二倍圆周率。 */
#define FILTEREX_EPSILON  1.0e-20f               /* 防止数值除零的最小量。 */

/* 判断单精度数是否为有限值。 */
static int FilterEx_IsFinite(float x)
{
    return isfinite(x) ? 1 : 0;
}

/* 检查采样率与目标频率是否满足奈奎斯特范围。 */
static int FilterEx_IsValidFrequency(float sample_rate_hz, float frequency_hz)
{
    return FilterEx_IsFinite(sample_rate_hz)
        && FilterEx_IsFinite(frequency_hz)
        && sample_rate_hz > 0.0f
        && frequency_hz > 0.0f
        && frequency_hz < 0.5f * sample_rate_hz;
}

/* ========================================================================== */
/* Lightweight streaming filters                                              */
/* ========================================================================== */

/* 按采样率和截止频率初始化一阶低通。 */
FilterEx_Status_t FilterEx_OnePoleInit(FilterEx_OnePole_t *filter,
                                       float sample_rate_hz,
                                       float cutoff_hz)
{
    if (filter == NULL) return FILTEREX_ERROR_NULL;
    if (!FilterEx_IsValidFrequency(sample_rate_hz, cutoff_hz)) {
        return FILTEREX_ERROR_PARAMETER;
    }

    filter->pole = expf(-FILTEREX_TWO_PI * cutoff_hz / sample_rate_hz);
    filter->y = 0.0f;
    return FILTEREX_OK;
}

/* 使用指定初值复位一阶低通。 */
void FilterEx_OnePoleReset(FilterEx_OnePole_t *filter, float seed)
{
    if (filter == NULL) return;
    filter->y = seed;
}

/* 批量处理一阶低通输入样本。 */
void FilterEx_OnePoleProcessBlock(FilterEx_OnePole_t *filter,
                                  const float *input, float *output,
                                  uint32_t length)
{
    if (filter == NULL || input == NULL || output == NULL) return;
    for (uint32_t i = 0u; i < length; ++i) {
        output[i] = FilterEx_OnePoleProcess(filter, input[i]);
    }
}

/* 按采样率和截止频率初始化直流阻断器。 */
FilterEx_Status_t FilterEx_DCBlockInit(FilterEx_DCBlock_t *filter,
                                       float sample_rate_hz,
                                       float cutoff_hz)
{
    if (filter == NULL) return FILTEREX_ERROR_NULL;
    if (!FilterEx_IsValidFrequency(sample_rate_hz, cutoff_hz)) {
        return FILTEREX_ERROR_PARAMETER;
    }

    filter->pole = expf(-FILTEREX_TWO_PI * cutoff_hz / sample_rate_hz);
    filter->x_prev = 0.0f;
    filter->y_prev = 0.0f;
    return FILTEREX_OK;
}

/* 使用指定输入初值复位直流阻断器。 */
void FilterEx_DCBlockReset(FilterEx_DCBlock_t *filter, float input_seed)
{
    if (filter == NULL) return;
    filter->x_prev = input_seed;
    filter->y_prev = 0.0f;
}

/* 批量处理直流阻断输入样本。 */
void FilterEx_DCBlockProcessBlock(FilterEx_DCBlock_t *filter,
                                  const float *input, float *output,
                                  uint32_t length)
{
    if (filter == NULL || input == NULL || output == NULL) return;
    for (uint32_t i = 0u; i < length; ++i) {
        output[i] = FilterEx_DCBlockProcess(filter, input[i]);
    }
}

/* 将毫秒时间常数换算为一阶滤波极点。 */
static float FilterEx_TimeConstantPole(float sample_rate_hz, float time_ms)
{
    if (time_ms <= 0.0f) return 0.0f;
    return expf(-1000.0f / (sample_rate_hz * time_ms));
}

/* 初始化全波包络跟随器。 */
FilterEx_Status_t FilterEx_EnvelopeInit(FilterEx_Envelope_t *filter,
                                        float sample_rate_hz,
                                        float attack_ms,
                                        float release_ms)
{
    if (filter == NULL) return FILTEREX_ERROR_NULL;
    if (!FilterEx_IsFinite(sample_rate_hz) || sample_rate_hz <= 0.0f
        || !FilterEx_IsFinite(attack_ms) || attack_ms < 0.0f
        || !FilterEx_IsFinite(release_ms) || release_ms < 0.0f) {
        return FILTEREX_ERROR_PARAMETER;
    }

    filter->attack_pole = FilterEx_TimeConstantPole(sample_rate_hz, attack_ms);
    filter->release_pole = FilterEx_TimeConstantPole(sample_rate_hz, release_ms);
    filter->envelope = 0.0f;
    return FILTEREX_OK;
}

/* 使用指定初值复位包络跟随器。 */
void FilterEx_EnvelopeReset(FilterEx_Envelope_t *filter, float seed)
{
    if (filter == NULL) return;
    filter->envelope = (seed >= 0.0f) ? seed : -seed;
}

/* 批量处理包络跟随输入样本。 */
void FilterEx_EnvelopeProcessBlock(FilterEx_Envelope_t *filter,
                                   const float *input, float *output,
                                   uint32_t length)
{
    if (filter == NULL || input == NULL || output == NULL) return;
    for (uint32_t i = 0u; i < length; ++i) {
        output[i] = FilterEx_EnvelopeProcess(filter, input[i]);
    }
}

/* 使用调用者缓冲区初始化滑动平均。 */
FilterEx_Status_t FilterEx_MovingAverageInit(FilterEx_MovingAverage_t *filter,
                                             float *storage,
                                             uint32_t storage_length,
                                             uint32_t window)
{
    if (filter == NULL || storage == NULL) return FILTEREX_ERROR_NULL;
    if (window == 0u) return FILTEREX_ERROR_PARAMETER;
    if (storage_length < window) return FILTEREX_ERROR_CAPACITY;

    filter->ring = storage;
    filter->window = window;
    filter->index = 0u;
    filter->count = 0u;
    filter->sum = 0.0f;
    memset(storage, 0, window * sizeof(float));
    return FILTEREX_OK;
}

/* 清空滑动平均历史状态。 */
void FilterEx_MovingAverageReset(FilterEx_MovingAverage_t *filter)
{
    if (filter == NULL || filter->ring == NULL || filter->window == 0u) return;
    memset(filter->ring, 0, filter->window * sizeof(float));
    filter->index = 0u;
    filter->count = 0u;
    filter->sum = 0.0f;
}

/* 批量处理滑动平均输入样本。 */
void FilterEx_MovingAverageProcessBlock(FilterEx_MovingAverage_t *filter,
                                        const float *input, float *output,
                                        uint32_t length)
{
    if (filter == NULL || input == NULL || output == NULL) return;
    for (uint32_t i = 0u; i < length; ++i) {
        output[i] = FilterEx_MovingAverageProcess(filter, input[i]);
    }
}

/* 使用调用者缓冲区初始化流式均方根。 */
FilterEx_Status_t FilterEx_RMSInit(FilterEx_RMS_t *filter,
                                   float *storage,
                                   uint32_t storage_length,
                                   uint32_t window)
{
    if (filter == NULL || storage == NULL) return FILTEREX_ERROR_NULL;
    if (window == 0u) return FILTEREX_ERROR_PARAMETER;
    if (storage_length < window) return FILTEREX_ERROR_CAPACITY;

    filter->ring = storage;
    filter->window = window;
    filter->index = 0u;
    filter->count = 0u;
    filter->sum_squares = 0.0f;
    memset(storage, 0, window * sizeof(float));
    return FILTEREX_OK;
}

/* 清空流式均方根历史状态。 */
void FilterEx_RMSReset(FilterEx_RMS_t *filter)
{
    if (filter == NULL || filter->ring == NULL || filter->window == 0u) return;
    memset(filter->ring, 0, filter->window * sizeof(float));
    filter->index = 0u;
    filter->count = 0u;
    filter->sum_squares = 0.0f;
}

/* 处理一个流式均方根输入样本。 */
float FilterEx_RMSProcess(FilterEx_RMS_t *filter, float x)
{
    const float square = x * x;
    if (filter->count < filter->window) {
        filter->ring[filter->index] = square;
        filter->sum_squares += square;
        filter->count++;
    } else {
        filter->sum_squares += square - filter->ring[filter->index];
        filter->ring[filter->index] = square;
    }

    filter->index++;
    if (filter->index == filter->window) filter->index = 0u;

    /* Long float accumulations can undershoot zero by a few ulps after subtraction. */
    const float mean_square = filter->sum_squares / (float)filter->count;
    return (mean_square > 0.0f) ? sqrtf(mean_square) : 0.0f;
}

/* 批量处理流式均方根输入样本。 */
void FilterEx_RMSProcessBlock(FilterEx_RMS_t *filter,
                              const float *input, float *output,
                              uint32_t length)
{
    if (filter == NULL || input == NULL || output == NULL) return;
    for (uint32_t i = 0u; i < length; ++i) {
        output[i] = FilterEx_RMSProcess(filter, input[i]);
    }
}

/* ========================================================================== */
/* Fractional delay                                                          */
/* ========================================================================== */

/* 初始化分数延时器。 */
FilterEx_Status_t FilterEx_FractionalDelayInit(
    FilterEx_FractionalDelay_t *delay,
    float *storage,
    uint32_t storage_length)
{
    if (delay == NULL || storage == NULL) return FILTEREX_ERROR_NULL;
    if (storage_length < 2u) return FILTEREX_ERROR_CAPACITY;

    delay->ring = storage;
    delay->capacity = storage_length;
    delay->integer_delay = 0u;
    delay->fraction = 0.0f;
    FilterEx_FractionalDelayReset(delay);
    return FILTEREX_OK;
}

/* 直接设置分数延时。 */
FilterEx_Status_t FilterEx_FractionalDelaySetSamples(
    FilterEx_FractionalDelay_t *delay,
    float delay_samples)
{
    if (delay == NULL || delay->ring == NULL) return FILTEREX_ERROR_NULL;
    if (delay->capacity < 2u || !FilterEx_IsFinite(delay_samples)
        || delay_samples < 0.0f) {
        return FILTEREX_ERROR_PARAMETER;
    }
    if (delay_samples > (float)(delay->capacity - 2u)) {
        return FILTEREX_ERROR_CAPACITY;
    }

    delay->integer_delay = (uint32_t)floorf(delay_samples);
    delay->fraction = delay_samples - (float)delay->integer_delay;
    return FILTEREX_OK;
}

/* 按采样率、点频和目标相移设置分数延时。 */
FilterEx_Status_t FilterEx_FractionalDelaySetPhase(
    FilterEx_FractionalDelay_t *delay,
    float sample_rate_hz,
    float frequency_hz,
    float phase_deg)
{
    float delay_samples;

    if (delay == NULL || delay->ring == NULL) return FILTEREX_ERROR_NULL;
    if (!FilterEx_IsValidFrequency(sample_rate_hz, frequency_hz)
        || !FilterEx_IsFinite(phase_deg)
        || phase_deg < 0.0f || phase_deg > 180.0f) {
        return FILTEREX_ERROR_PARAMETER;
    }

    delay_samples = sample_rate_hz * phase_deg
                  / (360.0f * frequency_hz);
    return FilterEx_FractionalDelaySetSamples(delay, delay_samples);
}

/* 清空分数延时器历史数据。 */
void FilterEx_FractionalDelayReset(FilterEx_FractionalDelay_t *delay)
{
    if (delay == NULL || delay->ring == NULL || delay->capacity < 2u) return;
    memset(delay->ring, 0, delay->capacity * sizeof(float));
    delay->write_index = 0u;
}

/* 批量处理分数延时输入。 */
void FilterEx_FractionalDelayProcessBlock(
    FilterEx_FractionalDelay_t *delay,
    const float *input,
    float *output,
    uint32_t length)
{
    if (delay == NULL || input == NULL || output == NULL) return;
    for (uint32_t i = 0u; i < length; ++i) {
        output[i] = FilterEx_FractionalDelayProcess(delay, input[i]);
    }
}

/* 按奇数窗口长度初始化中值滤波器。 */
FilterEx_Status_t FilterEx_MedianInit(FilterEx_Median_t *filter, uint32_t window)
{
    if (filter == NULL) return FILTEREX_ERROR_NULL;
    if (window == 0u || window > FILTEREX_MEDIAN_MAX_WINDOW
        || (window & 1u) == 0u) {
        return FILTEREX_ERROR_PARAMETER;
    }

    filter->window = (uint8_t)window;
    FilterEx_MedianReset(filter);
    return FILTEREX_OK;
}

/* 清空中值滤波历史状态。 */
void FilterEx_MedianReset(FilterEx_Median_t *filter)
{
    if (filter == NULL) return;
    memset(filter->ring, 0, sizeof(filter->ring));
    memset(filter->sorted, 0, sizeof(filter->sorted));
    filter->index = 0u;
    filter->count = 0u;
}

/* 将新样本插入已排序窗口。 */
static void FilterEx_MedianInsert(float *sorted, uint32_t count, float x)
{
    uint32_t pos = count;
    while (pos > 0u && sorted[pos - 1u] > x) {
        sorted[pos] = sorted[pos - 1u];
        --pos;
    }
    sorted[pos] = x;
}

/* 处理一个中值滤波输入样本。 */
float FilterEx_MedianProcess(FilterEx_Median_t *filter, float x)
{
    if (filter->count < filter->window) {
        filter->ring[filter->index] = x;
        FilterEx_MedianInsert(filter->sorted, filter->count, x);
        filter->count++;
    } else {
        const float old = filter->ring[filter->index];
        uint32_t remove = 0u;

        filter->ring[filter->index] = x;
        while (remove < filter->count && filter->sorted[remove] != old) ++remove;
        if (remove == filter->count) remove = filter->count - 1u;
        for (uint32_t i = remove; i + 1u < filter->count; ++i) {
            filter->sorted[i] = filter->sorted[i + 1u];
        }
        FilterEx_MedianInsert(filter->sorted, filter->count - 1u, x);
    }

    filter->index++;
    if (filter->index == filter->window) filter->index = 0u;

    if ((filter->count & 1u) != 0u) {
        return filter->sorted[filter->count / 2u];
    }
    return 0.5f * (filter->sorted[filter->count / 2u - 1u]
                 + filter->sorted[filter->count / 2u]);
}

/* 批量处理中值滤波输入样本。 */
void FilterEx_MedianProcessBlock(FilterEx_Median_t *filter,
                                 const float *input, float *output,
                                 uint32_t length)
{
    if (filter == NULL || input == NULL || output == NULL) return;
    for (uint32_t i = 0u; i < length; ++i) {
        output[i] = FilterEx_MedianProcess(filter, input[i]);
    }
}

/* ========================================================================== */
/* General FIR and multirate processing                                       */
/* ========================================================================== */

/* 计算理想低通脉冲响应的一个抽头。 */
static float FilterEx_IdealLowpass(float normalized_cutoff, int32_t offset)
{
    if (offset == 0) return 2.0f * normalized_cutoff;
    return sinf(FILTEREX_TWO_PI * normalized_cutoff * (float)offset)
         / (FILTEREX_PI * (float)offset);
}

/* 计算指定窗函数在一个抽头处的权值。 */
static float FilterEx_WindowValue(FilterEx_Window_t window,
                                  uint32_t index, uint32_t taps)
{
    const float phase = FILTEREX_TWO_PI * (float)index / (float)(taps - 1u);
    switch (window) {
    case FILTEREX_WINDOW_HANN:
        return 0.5f - 0.5f * cosf(phase);
    case FILTEREX_WINDOW_HAMMING:
        return 0.54f - 0.46f * cosf(phase);
    case FILTEREX_WINDOW_BLACKMAN:
        return 0.42f - 0.5f * cosf(phase) + 0.08f * cosf(2.0f * phase);
    case FILTEREX_WINDOW_RECTANGULAR:
    default:
        return 1.0f;
    }
}

/* 设计奇数抽头线性相位 FIR 系数。 */
FilterEx_Status_t FilterEx_FIRDesign(float *coefficients, uint32_t taps,
                                     FilterEx_FIRType_t type,
                                     float sample_rate_hz,
                                     float frequency1_hz,
                                     float frequency2_hz,
                                     FilterEx_Window_t window)
{
    if (coefficients == NULL) return FILTEREX_ERROR_NULL;
    if (taps < 3u || (taps & 1u) == 0u
        || type > FILTEREX_FIR_BANDSTOP
        || window > FILTEREX_WINDOW_BLACKMAN
        || !FilterEx_IsValidFrequency(sample_rate_hz, frequency1_hz)) {
        return FILTEREX_ERROR_PARAMETER;
    }
    if ((type == FILTEREX_FIR_BANDPASS || type == FILTEREX_FIR_BANDSTOP)
        && (!FilterEx_IsValidFrequency(sample_rate_hz, frequency2_hz)
            || frequency2_hz <= frequency1_hz)) {
        return FILTEREX_ERROR_PARAMETER;
    }

    const float f1 = frequency1_hz / sample_rate_hz;
    const float f2 = frequency2_hz / sample_rate_hz;
    const int32_t midpoint = (int32_t)(taps / 2u);

    for (uint32_t n = 0u; n < taps; ++n) {
        const int32_t offset = (int32_t)n - midpoint;
        float ideal;
        switch (type) {
        case FILTEREX_FIR_HIGHPASS:
            ideal = ((offset == 0) ? 1.0f : 0.0f)
                  - FilterEx_IdealLowpass(f1, offset);
            break;
        case FILTEREX_FIR_BANDPASS:
            ideal = FilterEx_IdealLowpass(f2, offset)
                  - FilterEx_IdealLowpass(f1, offset);
            break;
        case FILTEREX_FIR_BANDSTOP:
            ideal = ((offset == 0) ? 1.0f : 0.0f)
                  - (FilterEx_IdealLowpass(f2, offset)
                     - FilterEx_IdealLowpass(f1, offset));
            break;
        case FILTEREX_FIR_LOWPASS:
        default:
            ideal = FilterEx_IdealLowpass(f1, offset);
            break;
        }
        coefficients[n] = ideal * FilterEx_WindowValue(window, n, taps);
    }

    /* Normalize in the relevant passband without disturbing linear phase. */
    float reference = 0.0f;
    if (type == FILTEREX_FIR_HIGHPASS) {
        for (uint32_t n = 0u; n < taps; ++n) {
            reference += ((n & 1u) != 0u) ? -coefficients[n] : coefficients[n];
        }
        reference = fabsf(reference);
    } else if (type == FILTEREX_FIR_BANDPASS) {
        const float center = 0.5f * (frequency1_hz + frequency2_hz);
        const float omega = FILTEREX_TWO_PI * center / sample_rate_hz;
        for (uint32_t n = 0u; n < taps; ++n) {
            const int32_t offset = (int32_t)n - midpoint;
            reference += coefficients[n] * cosf(omega * (float)offset);
        }
        reference = fabsf(reference);
    } else {
        for (uint32_t n = 0u; n < taps; ++n) reference += coefficients[n];
        reference = fabsf(reference);
    }

    if (!FilterEx_IsFinite(reference) || reference < FILTEREX_EPSILON) {
        return FILTEREX_ERROR_PARAMETER;
    }
    const float scale = 1.0f / reference;
    for (uint32_t n = 0u; n < taps; ++n) coefficients[n] *= scale;
    return FILTEREX_OK;
}

/* 计算 FIR 在指定频率处的幅值响应。 */
float FilterEx_FIRMagnitude(const float *coefficients, uint32_t taps,
                            float sample_rate_hz, float frequency_hz)
{
    if (coefficients == NULL || taps == 0u || sample_rate_hz <= 0.0f
        || frequency_hz < 0.0f || frequency_hz > 0.5f * sample_rate_hz) {
        return 0.0f;
    }

    const float omega = FILTEREX_TWO_PI * frequency_hz / sample_rate_hz;
    float real = 0.0f;
    float imag = 0.0f;
    for (uint32_t n = 0u; n < taps; ++n) {
        const float phase = omega * (float)n;
        real += coefficients[n] * cosf(phase);
        imag -= coefficients[n] * sinf(phase);
    }
    return sqrtf(real * real + imag * imag);
}

/* 使用指定系数和状态缓冲区初始化 FIR。 */
FilterEx_Status_t FilterEx_FIRInit(FilterEx_FIR_t *filter,
                                   const float *coefficients,
                                   float *state, uint32_t state_length,
                                   uint32_t taps)
{
    if (filter == NULL || coefficients == NULL || state == NULL) {
        return FILTEREX_ERROR_NULL;
    }
    if (taps == 0u) return FILTEREX_ERROR_PARAMETER;
    if (state_length < taps) return FILTEREX_ERROR_CAPACITY;

    filter->coefficients = coefficients;
    filter->state = state;
    filter->taps = taps;
    filter->write_index = 0u;
    memset(state, 0, taps * sizeof(float));
    return FILTEREX_OK;
}

/* 清空 FIR 历史状态。 */
void FilterEx_FIRReset(FilterEx_FIR_t *filter)
{
    if (filter == NULL || filter->state == NULL || filter->taps == 0u) return;
    memset(filter->state, 0, filter->taps * sizeof(float));
    filter->write_index = 0u;
}

/* 批量处理 FIR 输入样本。 */
void FilterEx_FIRProcessBlock(FilterEx_FIR_t *filter,
                              const float *input, float *output,
                              uint32_t length)
{
    if (filter == NULL || input == NULL || output == NULL) return;
    for (uint32_t i = 0u; i < length; ++i) {
        output[i] = FilterEx_FIRProcess(filter, input[i]);
    }
}

/* 初始化 FIR 抽取器。 */
FilterEx_Status_t FilterEx_DecimatorInit(FilterEx_Decimator_t *decimator,
                                         uint32_t factor,
                                         const float *coefficients,
                                         float *state,
                                         uint32_t state_length,
                                         uint32_t taps)
{
    if (decimator == NULL) return FILTEREX_ERROR_NULL;
    if (factor < 2u) return FILTEREX_ERROR_PARAMETER;

    const FilterEx_Status_t status = FilterEx_FIRInit(&decimator->fir,
                                                       coefficients, state,
                                                       state_length, taps);
    if (status != FILTEREX_OK) return status;
    decimator->factor = factor;
    decimator->phase = 0u;
    return FILTEREX_OK;
}

/* 清空抽取器历史状态并复位相位。 */
void FilterEx_DecimatorReset(FilterEx_Decimator_t *decimator)
{
    if (decimator == NULL) return;
    FilterEx_FIRReset(&decimator->fir);
    decimator->phase = 0u;
}

/* 计算给定输入长度可产生的抽取输出点数。 */
uint32_t FilterEx_DecimatorOutputCount(const FilterEx_Decimator_t *decimator,
                                       uint32_t input_length)
{
    if (decimator == NULL || decimator->factor < 2u || input_length == 0u) {
        return 0u;
    }
    const uint32_t first = (decimator->phase == 0u)
                         ? 0u : decimator->factor - decimator->phase;
    if (first >= input_length) return 0u;
    return 1u + (input_length - 1u - first) / decimator->factor;
}

/* 计算 FIR 当前历史窗口的卷积结果。 */
static float FilterEx_FIRCurrent(const FilterEx_FIR_t *filter)
{
    uint32_t k = 0u;
    uint32_t state_index = filter->write_index;
    float y = 0.0f;

    for (; k <= filter->write_index; ++k) {
        y += filter->coefficients[k] * filter->state[state_index--];
    }
    state_index = filter->taps - 1u;
    for (; k < filter->taps; ++k) {
        y += filter->coefficients[k] * filter->state[state_index--];
    }
    return y;
}

/* 完成一块 FIR 抽取并返回实际输出点数。 */
FilterEx_Status_t FilterEx_DecimatorProcess(FilterEx_Decimator_t *decimator,
                                            const float *input,
                                            uint32_t input_length,
                                            float *output,
                                            uint32_t output_capacity,
                                            uint32_t *output_length)
{
    if (decimator == NULL || output_length == NULL) return FILTEREX_ERROR_NULL;
    *output_length = 0u;
    if (input_length == 0u) return FILTEREX_OK;
    if (input == NULL || output == NULL) return FILTEREX_ERROR_NULL;

    const uint32_t required = FilterEx_DecimatorOutputCount(decimator, input_length);
    if (output_capacity < required) return FILTEREX_ERROR_CAPACITY;

    uint32_t produced = 0u;
    for (uint32_t i = 0u; i < input_length; ++i) {
        FilterEx_FIR_t *fir = &decimator->fir;
        fir->state[fir->write_index] = input[i];

        /* Convolution is the expensive part; skip it on discarded phases. */
        if (decimator->phase == 0u) output[produced++] = FilterEx_FIRCurrent(fir);

        fir->write_index++;
        if (fir->write_index == fir->taps) fir->write_index = 0u;
        decimator->phase++;
        if (decimator->phase == decimator->factor) decimator->phase = 0u;
    }

    *output_length = produced;
    return FILTEREX_OK;
}

/* ========================================================================== */
/* Biquad and second-order-section cascades                                    */
/* ========================================================================== */

/* 清空双二阶节延迟状态。 */
void FilterEx_BiquadReset(FilterEx_Biquad_t *filter)
{
    if (filter == NULL) return;
    filter->z1 = 0.0f;
    filter->z2 = 0.0f;
}

/* 将双二阶节设置为直通。 */
void FilterEx_BiquadMakeBypass(FilterEx_Biquad_t *filter)
{
    if (filter == NULL) return;
    filter->b0 = 1.0f;
    filter->b1 = 0.0f;
    filter->b2 = 0.0f;
    filter->a1 = 0.0f;
    filter->a2 = 0.0f;
    FilterEx_BiquadReset(filter);
}

/* 检查双二阶节极点是否稳定。 */
int FilterEx_BiquadIsStable(const FilterEx_Biquad_t *filter)
{
    if (filter == NULL
        || !FilterEx_IsFinite(filter->a1)
        || !FilterEx_IsFinite(filter->a2)) {
        return 0;
    }

    /* Jury test for roots of z^2 + a1*z + a2 strictly inside the unit circle. */
    return fabsf(filter->a2) < 1.0f
        && (1.0f + filter->a1 + filter->a2) > 0.0f
        && (1.0f - filter->a1 + filter->a2) > 0.0f;
}

/* 归一化并写入双二阶节系数。 */
FilterEx_Status_t FilterEx_BiquadSet(FilterEx_Biquad_t *filter,
                                     float b0, float b1, float b2,
                                     float a0, float a1, float a2)
{
    if (filter == NULL) return FILTEREX_ERROR_NULL;
    if (!FilterEx_IsFinite(b0) || !FilterEx_IsFinite(b1)
        || !FilterEx_IsFinite(b2) || !FilterEx_IsFinite(a0)
        || !FilterEx_IsFinite(a1) || !FilterEx_IsFinite(a2)
        || fabsf(a0) < FILTEREX_EPSILON) {
        return FILTEREX_ERROR_PARAMETER;
    }

    const float inverse_a0 = 1.0f / a0;
    FilterEx_Biquad_t candidate;
    candidate.b0 = b0 * inverse_a0;
    candidate.b1 = b1 * inverse_a0;
    candidate.b2 = b2 * inverse_a0;
    candidate.a1 = a1 * inverse_a0;
    candidate.a2 = a2 * inverse_a0;
    candidate.z1 = 0.0f;
    candidate.z2 = 0.0f;

    if (!FilterEx_IsFinite(candidate.b0) || !FilterEx_IsFinite(candidate.b1)
        || !FilterEx_IsFinite(candidate.b2) || !FilterEx_IsFinite(candidate.a1)
        || !FilterEx_IsFinite(candidate.a2)) {
        return FILTEREX_ERROR_PARAMETER;
    }
    if (!FilterEx_BiquadIsStable(&candidate)) return FILTEREX_ERROR_UNSTABLE;

    *filter = candidate;
    return FILTEREX_OK;
}

/* 按 RBJ 公式设计双二阶节系数。 */
FilterEx_Status_t FilterEx_BiquadDesign(FilterEx_Biquad_t *filter,
                                        FilterEx_BiquadType_t type,
                                        float sample_rate_hz,
                                        float frequency_hz,
                                        float q_or_slope,
                                        float gain_db)
{
    if (filter == NULL) return FILTEREX_ERROR_NULL;
    if (type > FILTEREX_BIQUAD_HIGH_SHELF
        || !FilterEx_IsValidFrequency(sample_rate_hz, frequency_hz)
        || !FilterEx_IsFinite(q_or_slope) || q_or_slope <= 0.0f
        || !FilterEx_IsFinite(gain_db) || fabsf(gain_db) > 120.0f) {
        return FILTEREX_ERROR_PARAMETER;
    }

    const float omega = FILTEREX_TWO_PI * frequency_hz / sample_rate_hz;
    const float sine = sinf(omega);
    const float cosine = cosf(omega);
    float b0, b1, b2, a0, a1, a2;

    if (type == FILTEREX_BIQUAD_LOW_SHELF
        || type == FILTEREX_BIQUAD_HIGH_SHELF) {
        const float A = powf(10.0f, gain_db / 40.0f);
        const float radicand = (A + 1.0f / A) * (1.0f / q_or_slope - 1.0f)
                              + 2.0f;
        if (radicand <= 0.0f || !FilterEx_IsFinite(radicand)) {
            return FILTEREX_ERROR_PARAMETER;
        }
        const float alpha = 0.5f * sine * sqrtf(radicand);
        const float two_root_A_alpha = 2.0f * sqrtf(A) * alpha;

        if (type == FILTEREX_BIQUAD_LOW_SHELF) {
            b0 = A * ((A + 1.0f) - (A - 1.0f) * cosine + two_root_A_alpha);
            b1 = 2.0f * A * ((A - 1.0f) - (A + 1.0f) * cosine);
            b2 = A * ((A + 1.0f) - (A - 1.0f) * cosine - two_root_A_alpha);
            a0 = (A + 1.0f) + (A - 1.0f) * cosine + two_root_A_alpha;
            a1 = -2.0f * ((A - 1.0f) + (A + 1.0f) * cosine);
            a2 = (A + 1.0f) + (A - 1.0f) * cosine - two_root_A_alpha;
        } else {
            b0 = A * ((A + 1.0f) + (A - 1.0f) * cosine + two_root_A_alpha);
            b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cosine);
            b2 = A * ((A + 1.0f) + (A - 1.0f) * cosine - two_root_A_alpha);
            a0 = (A + 1.0f) - (A - 1.0f) * cosine + two_root_A_alpha;
            a1 = 2.0f * ((A - 1.0f) - (A + 1.0f) * cosine);
            a2 = (A + 1.0f) - (A - 1.0f) * cosine - two_root_A_alpha;
        }
        return FilterEx_BiquadSet(filter, b0, b1, b2, a0, a1, a2);
    }

    const float alpha = sine / (2.0f * q_or_slope);
    a0 = 1.0f + alpha;
    a1 = -2.0f * cosine;
    a2 = 1.0f - alpha;

    switch (type) {
    case FILTEREX_BIQUAD_HIGHPASS:
        b0 = 0.5f * (1.0f + cosine);
        b1 = -(1.0f + cosine);
        b2 = b0;
        break;
    case FILTEREX_BIQUAD_BANDPASS:
        b0 = alpha;
        b1 = 0.0f;
        b2 = -alpha;
        break;
    case FILTEREX_BIQUAD_NOTCH:
        b0 = 1.0f;
        b1 = -2.0f * cosine;
        b2 = 1.0f;
        break;
    case FILTEREX_BIQUAD_ALLPASS:
        b0 = 1.0f - alpha;
        b1 = -2.0f * cosine;
        b2 = 1.0f + alpha;
        break;
    case FILTEREX_BIQUAD_PEAK: {
        const float A = powf(10.0f, gain_db / 40.0f);
        b0 = 1.0f + alpha * A;
        b1 = -2.0f * cosine;
        b2 = 1.0f - alpha * A;
        a0 = 1.0f + alpha / A;
        a1 = -2.0f * cosine;
        a2 = 1.0f - alpha / A;
        break;
    }
    case FILTEREX_BIQUAD_LOWPASS:
    default:
        b0 = 0.5f * (1.0f - cosine);
        b1 = 1.0f - cosine;
        b2 = b0;
        break;
    }

    return FilterEx_BiquadSet(filter, b0, b1, b2, a0, a1, a2);
}

/* 批量处理双二阶节输入样本。 */
void FilterEx_BiquadProcessBlock(FilterEx_Biquad_t *filter,
                                 const float *input, float *output,
                                 uint32_t length)
{
    if (filter == NULL || input == NULL || output == NULL) return;
    for (uint32_t i = 0u; i < length; ++i) {
        output[i] = FilterEx_BiquadProcess(filter, input[i]);
    }
}

/* 计算双二阶节在指定频率处的幅值响应。 */
float FilterEx_BiquadMagnitude(const FilterEx_Biquad_t *filter,
                               float sample_rate_hz, float frequency_hz)
{
    if (filter == NULL || sample_rate_hz <= 0.0f || frequency_hz < 0.0f
        || frequency_hz > 0.5f * sample_rate_hz) {
        return 0.0f;
    }

    const float omega = FILTEREX_TWO_PI * frequency_hz / sample_rate_hz;
    const float c1 = cosf(omega);
    const float s1 = sinf(omega);
    const float c2 = cosf(2.0f * omega);
    const float s2 = sinf(2.0f * omega);
    const float numerator_real = filter->b0 + filter->b1 * c1 + filter->b2 * c2;
    const float numerator_imag = -filter->b1 * s1 - filter->b2 * s2;
    const float denominator_real = 1.0f + filter->a1 * c1 + filter->a2 * c2;
    const float denominator_imag = -filter->a1 * s1 - filter->a2 * s2;
    const float numerator_power = numerator_real * numerator_real
                                + numerator_imag * numerator_imag;
    const float denominator_power = denominator_real * denominator_real
                                  + denominator_imag * denominator_imag;
    if (denominator_power < FILTEREX_EPSILON) return FLT_MAX;
    return sqrtf(numerator_power / denominator_power);
}

/* 初始化空的双二阶节级联。 */
void FilterEx_SOSInit(FilterEx_SOS_t *filter)
{
    if (filter == NULL) return;
    memset(filter, 0, sizeof(*filter));
}

/* 清空级联中各节的延迟状态。 */
void FilterEx_SOSReset(FilterEx_SOS_t *filter)
{
    if (filter == NULL) return;
    for (uint32_t i = 0u; i < filter->section_count; ++i) {
        FilterEx_BiquadReset(&filter->section[i]);
    }
}

/* 向级联末尾追加一个双二阶节。 */
FilterEx_Status_t FilterEx_SOSAppend(FilterEx_SOS_t *filter,
                                     const FilterEx_Biquad_t *section)
{
    if (filter == NULL || section == NULL) return FILTEREX_ERROR_NULL;
    if (filter->section_count >= FILTEREX_MAX_SOS_SECTIONS) {
        return FILTEREX_ERROR_CAPACITY;
    }
    if (!FilterEx_BiquadIsStable(section)) return FILTEREX_ERROR_UNSTABLE;

    FilterEx_Biquad_t *destination = &filter->section[filter->section_count];
    *destination = *section;
    destination->z1 = 0.0f;
    destination->z2 = 0.0f;
    filter->section_count++;
    return FILTEREX_OK;
}

/* 按行载入并归一化 SOS 系数。 */
FilterEx_Status_t FilterEx_SOSLoad(FilterEx_SOS_t *filter,
                                   const float *coefficients,
                                   uint32_t section_count)
{
    if (filter == NULL || coefficients == NULL) return FILTEREX_ERROR_NULL;
    if (section_count == 0u) return FILTEREX_ERROR_PARAMETER;
    if (section_count > FILTEREX_MAX_SOS_SECTIONS) return FILTEREX_ERROR_CAPACITY;

    FilterEx_SOS_t candidate;
    FilterEx_SOSInit(&candidate);
    for (uint32_t i = 0u; i < section_count; ++i) {
        FilterEx_Status_t status = FilterEx_BiquadSet(&candidate.section[i],
                                                      coefficients[6u * i + 0u],
                                                      coefficients[6u * i + 1u],
                                                      coefficients[6u * i + 2u],
                                                      coefficients[6u * i + 3u],
                                                      coefficients[6u * i + 4u],
                                                      coefficients[6u * i + 5u]);
        if (status != FILTEREX_OK) return status;
        candidate.section_count++;
    }

    *filter = candidate;
    return FILTEREX_OK;
}

/* 批量处理 SOS 级联输入样本。 */
void FilterEx_SOSProcessBlock(FilterEx_SOS_t *filter,
                              const float *input, float *output,
                              uint32_t length)
{
    if (filter == NULL || input == NULL || output == NULL) return;
    for (uint32_t i = 0u; i < length; ++i) {
        output[i] = FilterEx_SOSProcess(filter, input[i]);
    }
}

/* 计算 SOS 级联在指定频率处的幅值响应。 */
float FilterEx_SOSMagnitude(const FilterEx_SOS_t *filter,
                            float sample_rate_hz, float frequency_hz)
{
    if (filter == NULL) return 0.0f;
    float magnitude = 1.0f;
    for (uint32_t i = 0u; i < filter->section_count; ++i) {
        magnitude *= FilterEx_BiquadMagnitude(&filter->section[i],
                                               sample_rate_hz, frequency_hz);
    }
    return magnitude;
}

/* 设计一个一阶巴特沃斯节。 */
static FilterEx_Status_t FilterEx_FirstOrderButterworth(FilterEx_Biquad_t *section,
                                                        FilterEx_ButterworthType_t type,
                                                        float sample_rate_hz,
                                                        float cutoff_hz)
{
    const float K = tanf(FILTEREX_PI * cutoff_hz / sample_rate_hz);
    const float normalization = 1.0f / (1.0f + K);
    const float a1 = (K - 1.0f) * normalization;

    if (type == FILTEREX_BUTTERWORTH_HIGHPASS) {
        return FilterEx_BiquadSet(section, normalization, -normalization, 0.0f,
                                  1.0f, a1, 0.0f);
    }
    return FilterEx_BiquadSet(section, K * normalization, K * normalization, 0.0f,
                              1.0f, a1, 0.0f);
}

/* 将指定阶数的巴特沃斯低通或高通设计为稳定 SOS。 */
FilterEx_Status_t FilterEx_ButterworthDesign(FilterEx_SOS_t *filter,
                                             FilterEx_ButterworthType_t type,
                                             float sample_rate_hz,
                                             float cutoff_hz,
                                             uint32_t order)
{
    if (filter == NULL) return FILTEREX_ERROR_NULL;
    if (type > FILTEREX_BUTTERWORTH_HIGHPASS
        || !FilterEx_IsValidFrequency(sample_rate_hz, cutoff_hz)
        || order == 0u || order > 2u * FILTEREX_MAX_SOS_SECTIONS) {
        return FILTEREX_ERROR_PARAMETER;
    }

    FilterEx_SOS_t candidate;
    FilterEx_SOSInit(&candidate);
    FilterEx_Status_t status;

    if ((order & 1u) != 0u) {
        FilterEx_Biquad_t first_order;
        status = FilterEx_FirstOrderButterworth(&first_order, type,
                                                sample_rate_hz, cutoff_hz);
        if (status != FILTEREX_OK) return status;
        status = FilterEx_SOSAppend(&candidate, &first_order);
        if (status != FILTEREX_OK) return status;
    }

    const uint32_t biquad_count = order / 2u;
    for (uint32_t k = 1u; k <= biquad_count; ++k) {
        float q;
        if ((order & 1u) == 0u) {
            q = 1.0f / (2.0f * cosf((2.0f * (float)k - 1.0f)
                                     * FILTEREX_PI / (2.0f * (float)order)));
        } else {
            q = 1.0f / (2.0f * cosf((float)k * FILTEREX_PI / (float)order));
        }

        FilterEx_Biquad_t section;
        const FilterEx_BiquadType_t section_type =
            (type == FILTEREX_BUTTERWORTH_HIGHPASS)
            ? FILTEREX_BIQUAD_HIGHPASS : FILTEREX_BIQUAD_LOWPASS;
        status = FilterEx_BiquadDesign(&section, section_type,
                                       sample_rate_hz, cutoff_hz, q, 0.0f);
        if (status != FILTEREX_OK) return status;
        status = FilterEx_SOSAppend(&candidate, &section);
        if (status != FILTEREX_OK) return status;
    }

    *filter = candidate;
    return FILTEREX_OK;
}

/* 以高通后接低通的方式设计宽带带通 SOS。 */
FilterEx_Status_t FilterEx_WideBandPassDesign(FilterEx_SOS_t *filter,
                                              float sample_rate_hz,
                                              float low_edge_hz,
                                              float high_edge_hz,
                                              uint32_t order_per_edge)
{
    if (filter == NULL) return FILTEREX_ERROR_NULL;
    if (!FilterEx_IsValidFrequency(sample_rate_hz, low_edge_hz)
        || !FilterEx_IsValidFrequency(sample_rate_hz, high_edge_hz)
        || high_edge_hz <= low_edge_hz || order_per_edge == 0u) {
        return FILTEREX_ERROR_PARAMETER;
    }

    FilterEx_SOS_t highpass;
    FilterEx_SOS_t lowpass;
    FilterEx_Status_t status = FilterEx_ButterworthDesign(
        &highpass, FILTEREX_BUTTERWORTH_HIGHPASS,
        sample_rate_hz, low_edge_hz, order_per_edge);
    if (status != FILTEREX_OK) return status;
    status = FilterEx_ButterworthDesign(
        &lowpass, FILTEREX_BUTTERWORTH_LOWPASS,
        sample_rate_hz, high_edge_hz, order_per_edge);
    if (status != FILTEREX_OK) return status;

    const uint32_t total = (uint32_t)highpass.section_count
                         + (uint32_t)lowpass.section_count;
    if (total > FILTEREX_MAX_SOS_SECTIONS) return FILTEREX_ERROR_CAPACITY;

    FilterEx_SOS_t candidate;
    FilterEx_SOSInit(&candidate);
    for (uint32_t i = 0u; i < highpass.section_count; ++i) {
        status = FilterEx_SOSAppend(&candidate, &highpass.section[i]);
        if (status != FILTEREX_OK) return status;
    }
    for (uint32_t i = 0u; i < lowpass.section_count; ++i) {
        status = FilterEx_SOSAppend(&candidate, &lowpass.section[i]);
        if (status != FILTEREX_OK) return status;
    }

    /* Make the geometric-center gain unity; pole locations and stability are unchanged. */
    const float center_hz = sqrtf(low_edge_hz * high_edge_hz);
    const float center_gain = FilterEx_SOSMagnitude(&candidate,
                                                     sample_rate_hz, center_hz);
    if (!FilterEx_IsFinite(center_gain) || center_gain < FILTEREX_EPSILON) {
        return FILTEREX_ERROR_PARAMETER;
    }
    const float gain_correction = 1.0f / center_gain;
    candidate.section[0].b0 *= gain_correction;
    candidate.section[0].b1 *= gain_correction;
    candidate.section[0].b2 *= gain_correction;

    *filter = candidate;
    return FILTEREX_OK;
}
