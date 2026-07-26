#include "FFT.h"
#include <stddef.h>
#include <stdio.h>

#define FFT_PI 3.14159265358979323846f /* 单精度圆周率。 */
#define FFT_RENORMALIZE_INTERVAL 32U /* 旋转因子归一化间隔。 */

static const float s_simulation_frequency_hz = 1000.0f; /* 仿真正弦频率。 */

/* FFT运行参数与静态工作区。 */
float FFT_Fs = 20000.0f;
float FFT_input[FFT_length] = {0};
float FFT_Buffer[FFT_length * 2U] = {0};
float FFT_normalized_output[FFT_length / 2U + 1U] = {0};
float FFT_output[FFT_length] = {0};
float FFT_mag_max = 0.0f;
uint32_t mag_max_index = 0U;
volatile float FFT_Amplitude = 0.0f;
volatile float FFT_Frequency = 0.0f;

/* 推进单位旋转，并周期性校正浮点漂移。 */
static void FFT_RotateUnit(float *cosine,
                           float *sine,
                           float step_cosine,
                           float step_sine,
                           uint32_t sample_count)
{
    float next_cosine = (*cosine * step_cosine) - (*sine * step_sine);

    *sine = (*sine * step_cosine) + (*cosine * step_sine);
    *cosine = next_cosine;

    if ((sample_count % FFT_RENORMALIZE_INTERVAL) == 0U) {
        float magnitude = sqrtf(((*cosine) * (*cosine)) +
                                ((*sine) * (*sine)));

        if (magnitude > 1.0e-12f) {
            *cosine /= magnitude;
            *sine /= magnitude;
        }
    }
}

/* 复制一帧实数输入数据。 */
void FFT_Input(const float *arr)
{
    if (arr == NULL) return;
    for (uint32_t i = 0U; i < FFT_length; i++) {
        FFT_input[i] = arr[i];
    }
}

/* 生成固定频率正弦测试帧。 */
void Sin_Simulation(void)
{
    for (uint32_t i = 0U; i < FFT_length; i++) {
        FFT_input[i] = sinf(2.0f * FFT_PI * s_simulation_frequency_hz *
                            (float)i / FFT_Fs);
    }
}

/* 对输入帧去除直流、加Hann窗并执行512点FFT。 */
void FFT_Start(void)
{
    float mean = 0.0f;
    float win_sum = 0.0f;
    float window_cosine = 1.0f;
    float window_sine = 0.0f;
    float window_step = 2.0f * FFT_PI / (float)(FFT_length - 1U);
    float window_step_cosine = cosf(window_step);
    float window_step_sine = sinf(window_step);

    for (uint32_t i = 0U; i < FFT_length; i++) {
        mean += FFT_input[i];
    }
    mean /= (float)FFT_length;

    for (uint32_t i = 0U; i < FFT_length; i++) {
        float win = 0.5f - (0.5f * window_cosine);

        win_sum += win;
        FFT_Buffer[2U * i] = (FFT_input[i] - mean) * win;
        FFT_Buffer[(2U * i) + 1U] = 0.0f;
        FFT_RotateUnit(&window_cosine,
                       &window_sine,
                       window_step_cosine,
                       window_step_sine,
                       i + 1U);
    }

    arm_cfft_f32(&arm_cfft_sR_f32_len512, FFT_Buffer, 0, 1);
    arm_cmplx_mag_f32(FFT_Buffer, FFT_output, FFT_length);

    for (uint32_t i = 0U; i < FFT_length; i++) {
        FFT_output[i] = FFT_output[i] * (float)FFT_length / win_sum;
    }
}

/* 查找非直流最大谱峰并更新频率、幅值结果。 */
void Process_FFT_mag(void)
{
    uint32_t relative_index = 0U;
    float offset;

    FFT_Frequency = 0.0f;
    FFT_Amplitude = 0.0f;
    arm_max_f32(&FFT_output[1],
                (FFT_length / 2U) - 1U,
                &FFT_mag_max,
                &relative_index);
    if (FFT_mag_max > 0.0f) {
        mag_max_index = relative_index + 1U;
        offset = FFT_InterpolateHannPeak(mag_max_index, &FFT_mag_max);
        FFT_Frequency = ((float)mag_max_index + offset) *
                        FFT_Fs / (float)FFT_length;
        FFT_Amplitude = FFT_mag_max * 2.0f / (float)FFT_length;
    } else {
        mag_max_index = 0U;
        FFT_mag_max = 0.0f;
    }

    printf("001:%.3f\r\n", (double)FFT_Frequency);
    printf("002:%.3f\r\n", (double)FFT_mag_max);
    printf("003:%.3f\r\n", (double)FFT_Amplitude);
}

/* 将双边幅度谱转换为不改动原谱的单边幅度谱。 */
void Normalize_FFT_To_Single_Side(void)
{
    float scale = 1.0f / (float)FFT_length;
    uint32_t single_side_length = FFT_length / 2U + 1U;

    for (uint32_t i = 0U; i < single_side_length; i++) {
        if (i == 0U) {
            FFT_normalized_output[i] = FFT_output[i] * scale;
        } else if (i == FFT_length / 2U) {
            FFT_normalized_output[i] = FFT_output[i] * scale;
        } else {
            FFT_normalized_output[i] = FFT_output[i] * 2.0f * scale;
        }
    }
}

/* 对Hann窗谱峰做对数抛物线插值，并补偿非整bin幅度损失。 */
float FFT_InterpolateHannPeak(uint32_t bin, float *corrected_magnitude)
{
    float offset = 0.0f;
    float magnitude = 0.0f;

    if (bin >= FFT_length) {
        if (corrected_magnitude != NULL) *corrected_magnitude = 0.0f;
        return 0.0f;
    }

    magnitude = FFT_output[bin];
    if (corrected_magnitude != NULL) *corrected_magnitude = magnitude;
    if ((bin == 0U) || ((bin + 1U) >= (FFT_length / 2U))) {
        return 0.0f;
    }

    if ((FFT_output[bin - 1U] > 0.0f) && (magnitude > 0.0f) &&
        (FFT_output[bin + 1U] > 0.0f)) {
        float left = logf(FFT_output[bin - 1U]);
        float center = logf(magnitude);
        float right = logf(FFT_output[bin + 1U]);
        float denominator = left - (2.0f * center) + right;

        if (fabsf(denominator) > 1.0e-12f) {
            offset = 0.5f * (left - right) / denominator;
            if (offset > 0.5f) offset = 0.5f;
            if (offset < -0.5f) offset = -0.5f;
        }
    }

    if ((corrected_magnitude != NULL) && (magnitude > 0.0f)) {
        float absolute_offset = fabsf(offset);
        float response = 1.0f;

        if (absolute_offset > 1.0e-6f) {
            float angle = FFT_PI * absolute_offset;

            response = fabsf(sinf(angle) /
                       (angle * (1.0f - absolute_offset * absolute_offset)));
        }
        *corrected_magnitude = (response > 1.0e-6f) ?
                               (magnitude / response) : magnitude;
    }
    return offset;
}

/* 用Goertzel算法计算指定频率的Hann窗幅值。 */
float FFT_AmpAtFreq(const float *data, uint32_t len, float fs, float freq)
{
    float mean = 0.0f;
    float nyquist = 0.5f * fs;

    if ((data == NULL) || (len < 2U) || (fs <= 0.0f) ||
        (freq <= 0.0f) || (freq > nyquist)) {
        return 0.0f;
    }

    for (uint32_t i = 0U; i < len; i++) {
        mean += data[i];
    }
    mean /= (float)len;

    float w = 2.0f * FFT_PI * freq / fs;
    float coeff = 2.0f * cosf(w);

    float q0 = 0.0f, q1 = 0.0f, q2 = 0.0f;
    float win_sum = 0.0f;
    float window_cosine = 1.0f;
    float window_sine = 0.0f;
    float window_step = 2.0f * FFT_PI / (float)(len - 1U);
    float window_step_cosine = cosf(window_step);
    float window_step_sine = sinf(window_step);

    for (uint32_t i = 0U; i < len; i++) {
        float win = 0.5f - (0.5f * window_cosine);

        win_sum += win;
        q0 = (data[i] - mean) * win + coeff * q1 - q2;
        q2 = q1;
        q1 = q0;
        FFT_RotateUnit(&window_cosine,
                       &window_sine,
                       window_step_cosine,
                       window_step_sine,
                       i + 1U);
    }

    float power = q1 * q1 + q2 * q2 - coeff * q1 * q2;

    if (power < 0.0f) {
        power = 0.0f;
    }

    return ((freq >= nyquist) ? 1.0f : 2.0f) * sqrtf(power) / win_sum;
}

/* 用Goertzel算法逐次测量基波与谐波幅值。 */
uint8_t FFT_GetHarmonics(const float *data, uint32_t len, float fs, float f0,
                         float max_freq, float freq_out[], float amp_out[])
{
    uint8_t count = 0;

    if ((data == NULL) || (freq_out == NULL) || (amp_out == NULL) ||
        (len < 2U) || (fs <= 0.0f) || (f0 < 1.0f) ||
        (max_freq < f0)) return 0U;
    if (max_freq > (0.5f * fs)) max_freq = 0.5f * fs;

    for (uint32_t harmonic = 1U;
         ((float)harmonic * f0 <= max_freq) &&
         (count < FFT_MAX_HARMONICS);
         harmonic++) {
        freq_out[count] = (float)harmonic * f0;
        amp_out[count] = FFT_AmpAtFreq(data,
                                       len,
                                       fs,
                                       (float)harmonic * f0);
        count++;
    }

    return count;
}

/* 从最近一次FFT结果中提取基波与谐波参数。 */
uint8_t FFT_GetHarmonics_FromFFT(float fs, float f0, float max_freq,
                                 float freq_out[], float amp_out[])
{
    uint8_t count = 0;
    float df = fs / (float)FFT_length;

    if ((freq_out == NULL) || (amp_out == NULL) || (fs <= 0.0f) ||
        (f0 < 1.0f) || (max_freq < f0)) return 0U;
    if (max_freq > (0.5f * fs)) max_freq = 0.5f * fs;

    for (uint32_t harmonic = 1U;
         ((float)harmonic * f0 <= max_freq) &&
         (count < FFT_MAX_HARMONICS);
         harmonic++) {
        float freq = (float)harmonic * f0;
        uint32_t k = (uint32_t)(freq / df + 0.5f);

        if (k == 0U || k > FFT_length / 2U) break;

        uint32_t k_best = k;
        float mag = FFT_output[k];

        if ((k > 1U) && (FFT_output[k - 1U] > mag)) {
            mag = FFT_output[k - 1U];
            k_best = k - 1U;
        }

        if ((k + 1U <= FFT_length / 2U) &&
            (FFT_output[k + 1U] > mag)) {
            mag = FFT_output[k + 1U];
            k_best = k + 1U;
        }

        float offset = FFT_InterpolateHannPeak(k_best, &mag);

        freq_out[count] = ((float)k_best + offset) * df;
        amp_out[count] = mag *
            ((k_best == FFT_length / 2U) ? 1.0f : 2.0f) /
            (float)FFT_length;

        count++;
    }

    return count;
}

/* 打印最近一次FFT的原始谱和峰值结果。 */
void FFT_Data_Print(void)
{
    printf("\r\nRaw Results\r\n");
    for (uint32_t i = 0U; i < FFT_length; i++) {
        printf("%.3f,", (double)FFT_output[i]);
    }
    printf("\r\nFreq:%.3f\r\n", (double)FFT_Frequency);
    printf("mag_max:%.3f\r\n", (double)FFT_mag_max);
    printf("Amp:%.3f\r\n", (double)FFT_Amplitude);
}

/* ==================================================================== */
/*  窗函数库实现                                                         */
/* ==================================================================== */
/* 生成指定窗函数，并可返回相干增益与等效噪声带宽。 */
void Window_Generate(float *w, uint32_t n, Window_Type_t type,
                     float *cg, float *enbw)
{
    if (w == NULL || n == 0u) return;

    const float n_minus_one = (n > 1U) ? (float)(n - 1U) : 1.0f;

    for (uint32_t i = 0U; i < n; i++) {
        float a = 2.0f * FFT_PI * (float)i / n_minus_one;
        float val;
        switch (type) {
        case WIN_HANN:
            val = 0.5f - 0.5f * cosf(a);
            break;
        case WIN_HAMMING:
            val = 0.54f - 0.46f * cosf(a);
            break;
        case WIN_BLACKMAN:
            val = 0.42f - 0.5f * cosf(a) + 0.08f * cosf(2.0f * a);
            break;
        case WIN_BLACKMAN_HARRIS:               /* 旁瓣 -92dB */
            val = 0.35875f - 0.48829f * cosf(a)
                + 0.14128f * cosf(2.0f * a) - 0.01168f * cosf(3.0f * a);
            break;
        case WIN_FLATTOP:                        /* Matlab flattopwin */
            val = 0.21557895f - 0.41663158f * cosf(a)
                + 0.277263158f * cosf(2.0f * a) - 0.083578947f * cosf(3.0f * a)
                + 0.006947368f * cosf(4.0f * a);
            break;
        case WIN_RECT:
        default:
            val = 1.0f;
            break;
        }
        w[i] = val;
    }

    if (cg != NULL || enbw != NULL) {
        float s1 = 0.0f, s2 = 0.0f;
        for (uint32_t i = 0U; i < n; i++) {
            s1 += w[i];
            s2 += w[i] * w[i];
        }
        if (cg)   *cg   = s1 / (float)n;
        if (enbw) *enbw = (s1 > 0.0f) ? ((float)n * s2 / (s1 * s1)) : 1.0f;
    }
}

/* 将窗系数原地乘到输入数组。 */
void Window_Apply(float *x, const float *w, uint32_t n)
{
    if (x == NULL || w == NULL) return;
    for (uint32_t i = 0U; i < n; i++) {
        x[i] *= w[i];
    }
}
