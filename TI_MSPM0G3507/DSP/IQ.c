/**
 * @file IQ.c
 * @brief 已知频率信号的IQ幅值与相位分析。
 */
#include "IQ.h"
#include <math.h>
#include <stddef.h>

#define IQ_TWO_PI 6.28318530717958647692f /* 两倍圆周率。 */
#define IQ_PI 3.14159265358979323846f /* 单精度圆周率。 */
#define IQ_RENORMALIZE_INTERVAL 32U /* 旋转因子归一化间隔。 */

/* 在指定频率处计算已去直流信号的同相与正交分量。 */
static void IQ_Compute(const float *data,
                       uint32_t length,
                       float sample_rate_hz,
                       float frequency_hz,
                       float *out_i,
                       float *out_q)
{
    float mean = 0.0f;
    float accumulator_i = 0.0f;
    float accumulator_q = 0.0f;
    float cosine = 1.0f;
    float sine = 0.0f;

    if (out_i != NULL) {
        *out_i = 0.0f;
    }
    if (out_q != NULL) {
        *out_q = 0.0f;
    }
    if ((data == NULL) || (out_i == NULL) || (out_q == NULL) ||
        (length == 0U) || (sample_rate_hz <= 0.0f) ||
        (frequency_hz <= 0.0f) ||
        (frequency_hz >= (0.5f * sample_rate_hz))) {
        return;
    }

    for (uint32_t index = 0U; index < length; index++) {
        mean += data[index];
    }
    mean /= (float)length;

    {
        float angle_step = IQ_TWO_PI * frequency_hz / sample_rate_hz;
        float step_cosine = cosf(angle_step);
        float step_sine = sinf(angle_step);

        for (uint32_t index = 0U; index < length; index++) {
            float sample = data[index] - mean;
            float next_cosine =
                (cosine * step_cosine) - (sine * step_sine);

            accumulator_i += sample * cosine;
            accumulator_q += sample * sine;
            sine = (sine * step_cosine) + (cosine * step_sine);
            cosine = next_cosine;

            if (((index + 1U) % IQ_RENORMALIZE_INTERVAL) == 0U) {
                float magnitude = sqrtf((cosine * cosine) + (sine * sine));

                if (magnitude > 0.0f) {
                    cosine /= magnitude;
                    sine /= magnitude;
                }
            }
        }
    }

    *out_i = accumulator_i * (2.0f / (float)length);
    *out_q = -accumulator_q * (2.0f / (float)length);
}

/* 提取指定频率的幅值与相位。 */
void IQ_Demodulate(const float *data,
                   uint32_t len,
                   float fs,
                   float freq,
                   IQ_Result_t *res)
{
    float component_i;
    float component_q;

    if (res == NULL) {
        return;
    }

    IQ_Compute(data, len, fs, freq, &component_i, &component_q);
    res->I = component_i;
    res->Q = component_q;
    res->amplitude =
        sqrtf((component_i * component_i) + (component_q * component_q));
    res->phase = atan2f(component_q, component_i);
}

/* 计算信号通道相对参考通道的归一化相位差。 */
float IQ_PhaseDiff(const float *ref,
                   const float *sig,
                   uint32_t len,
                   float fs,
                   float freq)
{
    float reference_i;
    float reference_q;
    float signal_i;
    float signal_q;
    float difference;

    IQ_Compute(ref, len, fs, freq, &reference_i, &reference_q);
    IQ_Compute(sig, len, fs, freq, &signal_i, &signal_q);
    difference = atan2f(signal_q, signal_i) -
                 atan2f(reference_q, reference_i);

    while (difference > IQ_PI) {
        difference -= IQ_TWO_PI;
    }
    while (difference <= -IQ_PI) {
        difference += IQ_TWO_PI;
    }
    return difference;
}

/* 将弧度转换为角度。 */
float IQ_Rad2Deg(float rad)
{
    return rad * (180.0f / IQ_PI);
}
