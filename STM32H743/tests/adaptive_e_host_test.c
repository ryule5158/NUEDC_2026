/*
 * E题自适应滤波主链路主机回归测试。
 *
 * 直接链接DSP/Adaptive.c中的Contest_LMSProcess()，验证：
 * 1. B为参考输入、D为移相后的C=A+B时，E能恢复A；
 * 2. A、B频差为100 Hz时，B残余是否小于1%；
 * 3. 仅输入B时，收敛到1%残余的时间是否小于1秒。
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "Adaptive.h"

#define E_FS_HZ             1000000.0f /* 软件移相与LMS的采样率。 */
#define E_PI                3.14159265358979323846
#define E_DELAY_CAPACITY    64u        /* 覆盖10 kHz、180°所需的50点延时。 */
#define E_LMS_TAPS          2u         /* 两个相邻B采样足以表示单频任意相位。 */
#define E_LMS_MU            0.0002f    /* 兼顾100 Hz频差保真和收敛速度的步长。 */
#define E_TOTAL_SAMPLES     1200000u   /* 总仿真时长1.2秒。 */
#define E_MEASURE_START     700000u    /* 只统计收敛后的0.5秒数据。 */

/* 线性分数延时器状态，仅用于模拟待封装的软件移相器。 */
typedef struct {
    float buffer[E_DELAY_CAPACITY]; /* 环形历史数据。 */
    uint32_t write_index;           /* 当前写位置。 */
    uint32_t integer_delay;         /* 整数延时点数。 */
    float fraction;                 /* 小数延时部分。 */
} E_FractionalDelay_t;

/* 指定频率正弦分量的最小二乘统计量。 */
typedef struct {
    double ss, cc, sc; /* 正弦、余弦基函数内积。 */
    double sy, cy;     /* 信号与正弦、余弦基函数内积。 */
} E_SineFit_t;

/* 清零线性分数延时器。 */
static int E_DelayInit(E_FractionalDelay_t *delay, float delay_samples)
{
    if (delay == NULL || !isfinite(delay_samples) || delay_samples < 0.0f ||
        delay_samples > (float)(E_DELAY_CAPACITY - 2u)) {
        return -1;
    }

    delay->integer_delay = (uint32_t)floorf(delay_samples);
    delay->fraction = delay_samples - (float)delay->integer_delay;
    delay->write_index = 0u;
    for (uint32_t i = 0u; i < E_DELAY_CAPACITY; i++) {
        delay->buffer[i] = 0.0f;
    }
    return 0;
}

/* 写入一个样本并返回线性插值后的延时样本。 */
static float E_DelayProcess(E_FractionalDelay_t *delay, float input)
{
    const uint32_t index0 = (delay->write_index + E_DELAY_CAPACITY
                           - delay->integer_delay) % E_DELAY_CAPACITY;
    const uint32_t index1 = (index0 + E_DELAY_CAPACITY - 1u) % E_DELAY_CAPACITY;
    float output;

    delay->buffer[delay->write_index] = input;
    output = (1.0f - delay->fraction) * delay->buffer[index0]
           + delay->fraction * delay->buffer[index1];
    delay->write_index = (delay->write_index + 1u) % E_DELAY_CAPACITY;
    return output;
}

/* 清零指定频率分量统计量。 */
static void E_SineFitInit(E_SineFit_t *fit)
{
    fit->ss = 0.0;
    fit->cc = 0.0;
    fit->sc = 0.0;
    fit->sy = 0.0;
    fit->cy = 0.0;
}

/* 向指定频率分量统计量加入一个样本。 */
static void E_SineFitPush(E_SineFit_t *fit, float freq_hz,
                          uint32_t sample_index, float sample)
{
    const double phase = 2.0 * E_PI * (double)freq_hz
                       * (double)sample_index / (double)E_FS_HZ;
    const double sine = sin(phase);
    const double cosine = cos(phase);

    fit->ss += sine * sine;
    fit->cc += cosine * cosine;
    fit->sc += sine * cosine;
    fit->sy += (double)sample * sine;
    fit->cy += (double)sample * cosine;
}

/* 返回指定频率分量的幅值。 */
static double E_SineFitAmplitude(const E_SineFit_t *fit)
{
    const double determinant = fit->ss * fit->cc - fit->sc * fit->sc;
    const double sine_gain = (fit->sy * fit->cc - fit->cy * fit->sc) / determinant;
    const double cosine_gain = (fit->cy * fit->ss - fit->sy * fit->sc) / determinant;
    return sqrt(sine_gain * sine_gain + cosine_gain * cosine_gain);
}

/* 验证给定频率组合、相移下，E对A的恢复和B的抑制。 */
static int E_RunRecoveryCase(float freq_a, float freq_b, float phase_deg)
{
    Contest_LMS_t lms;
    float weights[E_LMS_TAPS]; /* LMS权重。 */
    float states[E_LMS_TAPS];  /* LMS输入历史。 */
    E_FractionalDelay_t delay;
    E_SineFit_t e_fit_a, d_fit_a, e_fit_b, d_fit_b;
    const float delay_samples = E_FS_HZ * phase_deg / (360.0f * freq_b);

    if (Contest_LMSInit(&lms, weights, states, E_LMS_TAPS, E_LMS_MU, 0.0f) != 0 ||
        E_DelayInit(&delay, delay_samples) != 0) {
        return -1;
    }
    E_SineFitInit(&e_fit_a);
    E_SineFitInit(&d_fit_a);
    E_SineFitInit(&e_fit_b);
    E_SineFitInit(&d_fit_b);

    for (uint32_t index = 0u; index < E_TOTAL_SAMPLES; index++) {
        const float time_s = (float)index / E_FS_HZ;
        const float signal_a = 0.75f * sinf(2.0f * (float)E_PI * freq_a * time_s + 0.31f);
        const float signal_b = 0.65f * sinf(2.0f * (float)E_PI * freq_b * time_s - 0.27f);
        const float signal_d = E_DelayProcess(&delay, signal_a + signal_b);
        float signal_e;

        (void)Contest_LMSProcess(&lms, signal_b, signal_d, &signal_e);
        if (index >= E_MEASURE_START) {
            E_SineFitPush(&e_fit_a, freq_a, index, signal_e);
            E_SineFitPush(&d_fit_a, freq_a, index, signal_d);
            E_SineFitPush(&e_fit_b, freq_b, index, signal_e);
            E_SineFitPush(&d_fit_b, freq_b, index, signal_d);
        }
    }

    {
        const double a_gain = E_SineFitAmplitude(&e_fit_a) / E_SineFitAmplitude(&d_fit_a);
        const double b_residual = E_SineFitAmplitude(&e_fit_b) / E_SineFitAmplitude(&d_fit_b);
        printf("恢复测试: A=%7.1fHz, B=%7.1fHz, 相移=%6.1f°, A幅值比=%.5f, B残余=%.5f%%\n",
               (double)freq_a, (double)freq_b, (double)phase_deg,
               a_gain, b_residual * 100.0);
        if (a_gain < 0.90 || a_gain > 1.10 || b_residual > 0.01) {
            return -1;
        }
    }
    return 0;
}

/* 验证关闭后重新加入B时，达到1%残余所需的时间。 */
static int E_RunResponseCase(float freq_b, float phase_deg)
{
    Contest_LMS_t lms;
    float weights[E_LMS_TAPS]; /* LMS权重。 */
    float states[E_LMS_TAPS];  /* LMS输入历史。 */
    E_FractionalDelay_t delay;
    E_SineFit_t fit;
    const float delay_samples = E_FS_HZ * phase_deg / (360.0f * freq_b);
    const uint32_t block_samples = (uint32_t)fmaxf(1000.0f, 20.0f * E_FS_HZ / freq_b);
    uint32_t settled_at = 0u;

    if (Contest_LMSInit(&lms, weights, states, E_LMS_TAPS, E_LMS_MU, 0.0f) != 0 ||
        E_DelayInit(&delay, delay_samples) != 0) {
        return -1;
    }
    E_SineFitInit(&fit);

    for (uint32_t index = 0u; index < 1000000u; index++) {
        const float time_s = (float)index / E_FS_HZ;
        const float signal_b = 0.65f * sinf(2.0f * (float)E_PI * freq_b * time_s - 0.27f);
        const float signal_d = E_DelayProcess(&delay, signal_b);
        float signal_e;

        (void)Contest_LMSProcess(&lms, signal_b, signal_d, &signal_e);
        E_SineFitPush(&fit, freq_b, index, signal_e);
        if ((index + 1u) % block_samples == 0u) {
            const double residual = E_SineFitAmplitude(&fit) / 0.65;
            if (residual < 0.01) {
                settled_at = index + 1u;
                break;
            }
            E_SineFitInit(&fit);
        }
    }

    printf("响应测试: B=%7.1fHz, 相移=%6.1f°, 达到1%%残余时间=%.3fs\n",
           (double)freq_b, (double)phase_deg, (double)settled_at / E_FS_HZ);
    return (settled_at > 0u && settled_at <= 1000000u) ? 0 : -1;
}

int main(void)
{
    int failed = 0;

    /* 三组均满足E题基本要求的频率差100 Hz，覆盖低、中、高频。 */
    failed |= E_RunRecoveryCase(10100.0f, 10000.0f, 180.0f);
    failed |= E_RunRecoveryCase(50100.0f, 50000.0f, 137.0f);
    failed |= E_RunRecoveryCase(99900.0f, 100000.0f, 90.0f);

    /* E题发挥部分的响应时间目标。 */
    failed |= E_RunResponseCase(10000.0f, 137.0f);
    failed |= E_RunResponseCase(50000.0f, 137.0f);
    failed |= E_RunResponseCase(100000.0f, 137.0f);

    if (failed != 0) {
        fprintf(stderr, "E题自适应滤波主链路测试失败。\n");
        return EXIT_FAILURE;
    }
    puts("结论: 现有Contest_LMSProcess在本测试条件下满足E题的恢复、残余和响应时间目标。");
    return EXIT_SUCCESS;
}
