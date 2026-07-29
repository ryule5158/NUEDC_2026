/**
 * @file    FilterEx.h
 * @brief   Stateful, allocation-free filtering extensions for embedded signal work.
 *
 * FilterEx is additive: it does not replace or alter any API in Filter.h.
 * All objects are caller-owned, all hot paths use float, and coefficient design
 * is separated from sample processing so trigonometric functions never run in
 * an ISR/sample loop.
 */
/* 扩展滤波器模块头文件保护宏。 */
#ifndef FILTER_EX_H
#define FILTER_EX_H /* 扩展滤波器模块头文件保护宏。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FILTEREX_MAX_SOS_SECTIONS   8u  /* SOS 级联允许的最大节数。 */
#define FILTEREX_MEDIAN_MAX_WINDOW 31u  /* 中值滤波允许的最大窗口。 */

/* 扩展滤波器公共状态码。 */
typedef enum {
    FILTEREX_OK                =  0,
    FILTEREX_ERROR_NULL        = -1,
    FILTEREX_ERROR_PARAMETER   = -2,
    FILTEREX_ERROR_CAPACITY    = -3,
    FILTEREX_ERROR_UNSTABLE    = -4
} FilterEx_Status_t;

/* ========================================================================== */
/* Lightweight streaming filters                                              */
/* ========================================================================== */

/* 一阶低通滤波器状态。 */
typedef struct {
    float pole;
    float y;
} FilterEx_OnePole_t;

/* 按采样率和截止频率初始化一阶低通。 */
FilterEx_Status_t FilterEx_OnePoleInit(FilterEx_OnePole_t *filter,
                                       float sample_rate_hz,
                                       float cutoff_hz);
/* 使用指定初值复位一阶低通。 */
void FilterEx_OnePoleReset(FilterEx_OnePole_t *filter, float seed);

/* 处理一个一阶低通输入样本。 */
static inline float FilterEx_OnePoleProcess(FilterEx_OnePole_t *filter, float x)
{
    filter->y += (1.0f - filter->pole) * (x - filter->y);
    return filter->y;
}

/* 批量处理一阶低通输入样本。 */
void FilterEx_OnePoleProcessBlock(FilterEx_OnePole_t *filter,
                                  const float *input, float *output,
                                  uint32_t length);

/* 带可调低频极点的直流阻断器状态。 */
typedef struct {
    float pole;
    float x_prev;
    float y_prev;
} FilterEx_DCBlock_t;

/* 按采样率和截止频率初始化直流阻断器。 */
FilterEx_Status_t FilterEx_DCBlockInit(FilterEx_DCBlock_t *filter,
                                       float sample_rate_hz,
                                       float cutoff_hz);
/* 使用指定输入初值复位直流阻断器。 */
void FilterEx_DCBlockReset(FilterEx_DCBlock_t *filter, float input_seed);

/* 处理一个直流阻断输入样本。 */
static inline float FilterEx_DCBlockProcess(FilterEx_DCBlock_t *filter, float x)
{
    const float y = x - filter->x_prev + filter->pole * filter->y_prev;
    filter->x_prev = x;
    filter->y_prev = y;
    return y;
}

/* 批量处理直流阻断输入样本。 */
void FilterEx_DCBlockProcessBlock(FilterEx_DCBlock_t *filter,
                                  const float *input, float *output,
                                  uint32_t length);

/* 具有独立上升、释放时间常数的全波包络跟随器。 */
typedef struct {
    float attack_pole;
    float release_pole;
    float envelope;
} FilterEx_Envelope_t;

/* 初始化全波包络跟随器。 */
FilterEx_Status_t FilterEx_EnvelopeInit(FilterEx_Envelope_t *filter,
                                        float sample_rate_hz,
                                        float attack_ms,
                                        float release_ms);
/* 使用指定初值复位包络跟随器。 */
void FilterEx_EnvelopeReset(FilterEx_Envelope_t *filter, float seed);

/* 处理一个包络跟随输入样本。 */
static inline float FilterEx_EnvelopeProcess(FilterEx_Envelope_t *filter, float x)
{
    const float target = (x >= 0.0f) ? x : -x;
    const float pole = (target > filter->envelope)
                     ? filter->attack_pole : filter->release_pole;
    filter->envelope += (1.0f - pole) * (target - filter->envelope);
    return filter->envelope;
}

/* 批量处理包络跟随输入样本。 */
void FilterEx_EnvelopeProcessBlock(FilterEx_Envelope_t *filter,
                                   const float *input, float *output,
                                   uint32_t length);

/* O(1) 流式滑动平均状态，环形缓冲区由调用者提供。 */
typedef struct {
    float *ring;
    float sum;
    uint32_t window;
    uint32_t index;
    uint32_t count;
} FilterEx_MovingAverage_t;

/* 使用调用者缓冲区初始化滑动平均。 */
FilterEx_Status_t FilterEx_MovingAverageInit(FilterEx_MovingAverage_t *filter,
                                             float *storage,
                                             uint32_t storage_length,
                                             uint32_t window);
/* 清空滑动平均历史状态。 */
void FilterEx_MovingAverageReset(FilterEx_MovingAverage_t *filter);

/* 处理一个滑动平均输入样本。 */
static inline float FilterEx_MovingAverageProcess(FilterEx_MovingAverage_t *filter,
                                                  float x)
{
    if (filter->count < filter->window) {
        filter->ring[filter->index] = x;
        filter->sum += x;
        filter->count++;
    } else {
        filter->sum += x - filter->ring[filter->index];
        filter->ring[filter->index] = x;
    }

    filter->index++;
    if (filter->index == filter->window) filter->index = 0u;
    return filter->sum / (float)filter->count;
}

/* 批量处理滑动平均输入样本。 */
void FilterEx_MovingAverageProcessBlock(FilterEx_MovingAverage_t *filter,
                                        const float *input, float *output,
                                        uint32_t length);

/* 使用调用者窗口缓冲区的流式均方根状态。 */
typedef struct {
    float *ring;
    float sum_squares;
    uint32_t window;
    uint32_t index;
    uint32_t count;
} FilterEx_RMS_t;

/* 使用调用者缓冲区初始化流式均方根。 */
FilterEx_Status_t FilterEx_RMSInit(FilterEx_RMS_t *filter,
                                   float *storage,
                                   uint32_t storage_length,
                                   uint32_t window);
/* 清空流式均方根历史状态。 */
void FilterEx_RMSReset(FilterEx_RMS_t *filter);
/* 处理一个流式均方根输入样本。 */
float FilterEx_RMSProcess(FilterEx_RMS_t *filter, float x);
/* 批量处理流式均方根输入样本。 */
void FilterEx_RMSProcessBlock(FilterEx_RMS_t *filter,
                              const float *input, float *output,
                              uint32_t length);

/* ========================================================================== */
/* Fractional delay                                                          */
/* ========================================================================== */

/* 流式分数延时器状态，环形历史缓冲区由调用者提供。 */
typedef struct {
    float *ring;            /* 输入样本的环形历史缓冲区。 */
    uint32_t capacity;      /* 环形缓冲区容量，单位为样本点。 */
    uint32_t write_index;   /* 当前输入样本的写入位置。 */
    uint32_t integer_delay; /* 延时的整数采样点部分。 */
    float fraction;         /* 延时的小数采样点部分，范围0~1。 */
} FilterEx_FractionalDelay_t;

/* 初始化分数延时器，storage_length至少为2。 */
FilterEx_Status_t FilterEx_FractionalDelayInit(
    FilterEx_FractionalDelay_t *delay,
    float *storage,
    uint32_t storage_length);

/* 直接设置分数延时，最大值为capacity-2个采样点。 */
FilterEx_Status_t FilterEx_FractionalDelaySetSamples(
    FilterEx_FractionalDelay_t *delay,
    float delay_samples);

/* 按采样率、当前点频和目标相移设置延时，phase_deg范围为0~180。 */
FilterEx_Status_t FilterEx_FractionalDelaySetPhase(
    FilterEx_FractionalDelay_t *delay,
    float sample_rate_hz,
    float frequency_hz,
    float phase_deg);

/* 清空分数延时器历史数据，随后前若干点为启动暂态。 */
void FilterEx_FractionalDelayReset(FilterEx_FractionalDelay_t *delay);

/* 处理一个输入采样点并返回线性插值后的延时输出。 */
static inline float FilterEx_FractionalDelayProcess(
    FilterEx_FractionalDelay_t *delay,
    float x)
{
    uint32_t index0;
    uint32_t index1;
    float y;

    delay->ring[delay->write_index] = x;
    index0 = delay->write_index + delay->capacity - delay->integer_delay;
    if (index0 >= delay->capacity) index0 -= delay->capacity;
    index1 = (index0 == 0u) ? (delay->capacity - 1u) : (index0 - 1u);
    y = (1.0f - delay->fraction) * delay->ring[index0]
      + delay->fraction * delay->ring[index1];

    delay->write_index++;
    if (delay->write_index == delay->capacity) delay->write_index = 0u;
    return y;
}

/* 批量处理分数延时输入，input和output允许为同一数组。 */
void FilterEx_FractionalDelayProcessBlock(
    FilterEx_FractionalDelay_t *delay,
    const float *input,
    float *output,
    uint32_t length);

/* 无堆内存的 O(window) 流式中值滤波状态。 */
typedef struct {
    float ring[FILTEREX_MEDIAN_MAX_WINDOW];
    float sorted[FILTEREX_MEDIAN_MAX_WINDOW];
    uint8_t window;
    uint8_t index;
    uint8_t count;
} FilterEx_Median_t;

/* 按奇数窗口长度初始化中值滤波器。 */
FilterEx_Status_t FilterEx_MedianInit(FilterEx_Median_t *filter, uint32_t window);
/* 清空中值滤波历史状态。 */
void FilterEx_MedianReset(FilterEx_Median_t *filter);
/* 处理一个中值滤波输入样本。 */
float FilterEx_MedianProcess(FilterEx_Median_t *filter, float x);
/* 批量处理中值滤波输入样本。 */
void FilterEx_MedianProcessBlock(FilterEx_Median_t *filter,
                                 const float *input, float *output,
                                 uint32_t length);

/* ========================================================================== */
/* General FIR and multirate processing                                       */
/* ========================================================================== */

/* FIR 滤波器响应类型。 */
typedef enum {
    FILTEREX_FIR_LOWPASS = 0,
    FILTEREX_FIR_HIGHPASS,
    FILTEREX_FIR_BANDPASS,
    FILTEREX_FIR_BANDSTOP
} FilterEx_FIRType_t;

/* FIR 设计使用的窗函数类型。 */
typedef enum {
    FILTEREX_WINDOW_RECTANGULAR = 0,
    FILTEREX_WINDOW_HANN,
    FILTEREX_WINDOW_HAMMING,
    FILTEREX_WINDOW_BLACKMAN
} FilterEx_Window_t;

/* 设计奇数抽头线性相位 FIR 系数。 */
FilterEx_Status_t FilterEx_FIRDesign(float *coefficients, uint32_t taps,
                                     FilterEx_FIRType_t type,
                                     float sample_rate_hz,
                                     float frequency1_hz,
                                     float frequency2_hz,
                                     FilterEx_Window_t window);

/* 计算 FIR 在指定频率处的幅值响应。 */
float FilterEx_FIRMagnitude(const float *coefficients, uint32_t taps,
                            float sample_rate_hz, float frequency_hz);

/* 使用调用者状态缓冲区的流式环形 FIR。 */
typedef struct {
    const float *coefficients;
    float *state;
    uint32_t taps;
    uint32_t write_index;
} FilterEx_FIR_t;

/* 使用指定系数和状态缓冲区初始化 FIR。 */
FilterEx_Status_t FilterEx_FIRInit(FilterEx_FIR_t *filter,
                                   const float *coefficients,
                                   float *state, uint32_t state_length,
                                   uint32_t taps);
/* 清空 FIR 历史状态。 */
void FilterEx_FIRReset(FilterEx_FIR_t *filter);

/* 处理一个 FIR 输入样本。 */
static inline float FilterEx_FIRProcess(FilterEx_FIR_t *filter, float x)
{
    uint32_t k = 0u;
    uint32_t state_index = filter->write_index;
    float y = 0.0f;

    filter->state[state_index] = x;
    for (; k <= filter->write_index; ++k) {
        y += filter->coefficients[k] * filter->state[state_index--];
    }
    state_index = filter->taps - 1u;
    for (; k < filter->taps; ++k) {
        y += filter->coefficients[k] * filter->state[state_index--];
    }

    filter->write_index++;
    if (filter->write_index == filter->taps) filter->write_index = 0u;
    return y;
}

/* 批量处理 FIR 输入样本。 */
void FilterEx_FIRProcessBlock(FilterEx_FIR_t *filter,
                              const float *input, float *output,
                              uint32_t length);

/* 仅在需要输出时计算卷积的 FIR 抽取器。 */
typedef struct {
    FilterEx_FIR_t fir;
    uint32_t factor;
    uint32_t phase;
} FilterEx_Decimator_t;

/* 初始化 FIR 抽取器。 */
FilterEx_Status_t FilterEx_DecimatorInit(FilterEx_Decimator_t *decimator,
                                         uint32_t factor,
                                         const float *coefficients,
                                         float *state,
                                         uint32_t state_length,
                                         uint32_t taps);
/* 清空抽取器历史状态并复位相位。 */
void FilterEx_DecimatorReset(FilterEx_Decimator_t *decimator);
/* 计算给定输入长度可产生的抽取输出点数。 */
uint32_t FilterEx_DecimatorOutputCount(const FilterEx_Decimator_t *decimator,
                                       uint32_t input_length);
/* 完成一块 FIR 抽取并返回实际输出点数。 */
FilterEx_Status_t FilterEx_DecimatorProcess(FilterEx_Decimator_t *decimator,
                                            const float *input,
                                            uint32_t input_length,
                                            float *output,
                                            uint32_t output_capacity,
                                            uint32_t *output_length);

/* ========================================================================== */
/* Biquad and second-order-section cascades                                    */
/* ========================================================================== */

/* 双二阶节响应类型。 */
typedef enum {
    FILTEREX_BIQUAD_LOWPASS = 0,
    FILTEREX_BIQUAD_HIGHPASS,
    FILTEREX_BIQUAD_BANDPASS,
    FILTEREX_BIQUAD_NOTCH,
    FILTEREX_BIQUAD_ALLPASS,
    FILTEREX_BIQUAD_PEAK,
    FILTEREX_BIQUAD_LOW_SHELF,
    FILTEREX_BIQUAD_HIGH_SHELF
} FilterEx_BiquadType_t;

/* 归一化 DF2T 双二阶节状态，分母采用 1+a1z^-1+a2z^-2。 */
typedef struct {
    float b0, b1, b2;
    float a1, a2;
    float z1, z2;
} FilterEx_Biquad_t;

/* 清空双二阶节延迟状态。 */
void FilterEx_BiquadReset(FilterEx_Biquad_t *filter);
/* 将双二阶节设置为直通。 */
void FilterEx_BiquadMakeBypass(FilterEx_Biquad_t *filter);
/* 检查双二阶节极点是否稳定。 */
int FilterEx_BiquadIsStable(const FilterEx_Biquad_t *filter);

/* 归一化并写入双二阶节系数。 */
FilterEx_Status_t FilterEx_BiquadSet(FilterEx_Biquad_t *filter,
                                     float b0, float b1, float b2,
                                     float a0, float a1, float a2);

/* 按 RBJ 公式设计双二阶节系数。 */
FilterEx_Status_t FilterEx_BiquadDesign(FilterEx_Biquad_t *filter,
                                        FilterEx_BiquadType_t type,
                                        float sample_rate_hz,
                                        float frequency_hz,
                                        float q_or_slope,
                                        float gain_db);

/* 处理一个双二阶节输入样本。 */
static inline float FilterEx_BiquadProcess(FilterEx_Biquad_t *filter, float x)
{
    const float y = filter->b0 * x + filter->z1;
    filter->z1 = filter->b1 * x - filter->a1 * y + filter->z2;
    filter->z2 = filter->b2 * x - filter->a2 * y;
    return y;
}

/* 批量处理双二阶节输入样本。 */
void FilterEx_BiquadProcessBlock(FilterEx_Biquad_t *filter,
                                 const float *input, float *output,
                                 uint32_t length);
/* 计算双二阶节在指定频率处的幅值响应。 */
float FilterEx_BiquadMagnitude(const FilterEx_Biquad_t *filter,
                               float sample_rate_hz, float frequency_hz);

/* 双二阶节级联状态。 */
typedef struct {
    FilterEx_Biquad_t section[FILTEREX_MAX_SOS_SECTIONS];
    uint8_t section_count;
} FilterEx_SOS_t;

/* 初始化空的双二阶节级联。 */
void FilterEx_SOSInit(FilterEx_SOS_t *filter);
/* 清空级联中各节的延迟状态。 */
void FilterEx_SOSReset(FilterEx_SOS_t *filter);
/* 向级联末尾追加一个双二阶节。 */
FilterEx_Status_t FilterEx_SOSAppend(FilterEx_SOS_t *filter,
                                     const FilterEx_Biquad_t *section);

/* 按 [b0,b1,b2,a0,a1,a2] 行格式载入 SOS 系数。 */
FilterEx_Status_t FilterEx_SOSLoad(FilterEx_SOS_t *filter,
                                   const float *coefficients,
                                   uint32_t section_count);

/* 处理一个 SOS 级联输入样本。 */
static inline float FilterEx_SOSProcess(FilterEx_SOS_t *filter, float x)
{
    float y = x;
    for (uint32_t i = 0u; i < filter->section_count; ++i) {
        y = FilterEx_BiquadProcess(&filter->section[i], y);
    }
    return y;
}

/* 批量处理 SOS 级联输入样本。 */
void FilterEx_SOSProcessBlock(FilterEx_SOS_t *filter,
                              const float *input, float *output,
                              uint32_t length);
/* 计算 SOS 级联在指定频率处的幅值响应。 */
float FilterEx_SOSMagnitude(const FilterEx_SOS_t *filter,
                            float sample_rate_hz, float frequency_hz);

/* 巴特沃斯低通或高通设计类型。 */
typedef enum {
    FILTEREX_BUTTERWORTH_LOWPASS = 0,
    FILTEREX_BUTTERWORTH_HIGHPASS
} FilterEx_ButterworthType_t;

/* 将 1～16 阶巴特沃斯低通或高通设计为稳定 SOS。 */
FilterEx_Status_t FilterEx_ButterworthDesign(FilterEx_SOS_t *filter,
                                             FilterEx_ButterworthType_t type,
                                             float sample_rate_hz,
                                             float cutoff_hz,
                                             uint32_t order);

/* 以高通后接低通的方式设计宽带带通 SOS。 */
FilterEx_Status_t FilterEx_WideBandPassDesign(FilterEx_SOS_t *filter,
                                              float sample_rate_hz,
                                              float low_edge_hz,
                                              float high_edge_hz,
                                              uint32_t order_per_edge);

#ifdef __cplusplus
}
#endif

#endif /* FILTER_EX_H */
