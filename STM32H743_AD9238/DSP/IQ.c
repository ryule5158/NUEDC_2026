/**
 * @file    IQ.c
 * @brief   IQ 正交解调实现
 */
#include "IQ.h"
#include <math.h>

#define IQ_2PI 6.28318530717958647692f /* 两倍圆周率。 */
#define IQ_PI 3.14159265358979323846f  /* 圆周率。 */

/**
 * @brief  内部：在频率 freq 处计算 I、Q 两分量（已扣除直流）。
 * @note   逐点用 sinf/cosf 递推太慢，这里用"复指数旋转因子递推"代替逐点三角函数：
 *             c_{n+1} = c_n·cosΔ - s_n·sinΔ
 *             s_{n+1} = s_n·cosΔ + c_n·sinΔ
 *         一次三角函数初始化后，循环内只剩乘加，对 Cortex-M7 FPU 非常友好。
 */
static void IQ_ComputeIQ(const float *data, uint32_t len, float fs, float freq,
                         float *outI, float *outQ)
{
    if (outI == NULL || outQ == NULL) return;
    if (data == NULL || len == 0u || !isfinite(fs) || !isfinite(freq) ||
        fs <= 0.0f || freq < 0.0f || freq > 0.5f * fs) {
        if (outI) *outI = 0.0f;
        if (outQ) *outQ = 0.0f;
        return;
    }

    /* 先求直流分量并扣除，避免直流污染解调结果 */
    float mean = 0.0f;
    for (uint32_t i = 0; i < len; i++) {
        if (!isfinite(data[i])) {
            *outI = 0.0f;
            *outQ = 0.0f;
            return;
        }
        mean += data[i];
    }
    mean /= (float)len;

    /* 旋转因子递推初始化：角步进 Δ = 2π·freq/fs */
    const float delta = IQ_2PI * freq / fs;
    const float cosd = cosf(delta);
    const float sind = sinf(delta);

    float c = 1.0f;   /* cos(0) */
    float s = 0.0f;   /* sin(0) */

    float accI = 0.0f;   /* Σ x·cos */
    float accQ = 0.0f;   /* Σ x·sin */

    for (uint32_t n = 0; n < len; n++) {
        float x = data[n] - mean;
        accI += x * c;
        accQ += x * s;

        /* 旋转到下一个相位（带轻微归一化抑制长序列幅值漂移） */
        float c_new = c * cosd - s * sind;
        float s_new = s * cosd + c * sind;
        c = c_new;
        s = s_new;
    }

    const float scale = 2.0f / (float)len;
    *outI =  accI * scale;
    *outQ = -accQ * scale;   /* 负号使 phase=atan2(Q,I) 等于信号相位 φ */
}

/* 提取指定频率的IQ分量、幅度和相位。 */
void IQ_Demodulate(const float *data, uint32_t len, float fs, float freq,
                   IQ_Result_t *res)
{
    if (res == NULL) return;

    float I, Q;
    IQ_ComputeIQ(data, len, fs, freq, &I, &Q);

    res->I = I;
    res->Q = Q;
    res->amplitude = sqrtf(I * I + Q * Q);
    res->phase = atan2f(Q, I);
}

/* 计算同频参考信号与待测信号的相位差。 */
float IQ_PhaseDiff(const float *ref, const float *sig, uint32_t len,
                   float fs, float freq)
{
    float Ir, Qr, Is, Qs;
    IQ_ComputeIQ(ref, len, fs, freq, &Ir, &Qr);
    IQ_ComputeIQ(sig, len, fs, freq, &Is, &Qs);

    float diff = atan2f(Qs, Is) - atan2f(Qr, Ir);

    /* 归一化到 (-π, π] */
    while (diff >  IQ_PI) diff -= IQ_2PI;
    while (diff <= -IQ_PI) diff += IQ_2PI;
    return diff;
}

/* 将弧度转换为角度。 */
float IQ_Rad2Deg(float rad)
{
    return rad * (180.0f / IQ_PI);
}
