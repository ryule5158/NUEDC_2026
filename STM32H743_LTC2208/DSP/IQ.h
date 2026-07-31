/**
 * @file    IQ.h
 * @brief   IQ 正交解调 — 已知频率下提取幅度+相位 / 两通道相位差
 * @note    原理：把信号分别与同频的 cos、sin 参考相乘再平均（数字鉴相/锁相思想），
 *          得到正交分量 I、Q：
 *              I =  (2/N) * Σ x[n]·cos(2π f n/fs)
 *              Q = -(2/N) * Σ x[n]·sin(2π f n/fs)
 *          对 x[n]=A·cos(2π f n/fs + φ) 有  I≈A·cosφ, Q≈A·sinφ，故
 *              幅度 = sqrt(I²+Q²)，相位 = atan2(Q, I)。
 *          相比工程已有的 FFT_AmpAtFreq(Goertzel，只给幅度)，本模块额外给出"相位"，
 *          且可直接算两路同频信号的相位差（相位差测量题常用）。
 * @warning 默认假设窗内为整数个周期或近似整周期；非整周期会有泄漏误差。
 *          相位差测量时两路误差同向抵消，结果仍很准。
 * @date    2026-06-18
 */

#ifndef __IQ_H
#define __IQ_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ---------------------------------------------------------- */
#include "main.h"
#include "arm_math.h"
#include <stdint.h>

/* IQ 解调结果结构体 ------------------------------------------------- */
typedef struct {
    float I;          /* 同相分量 I */
    float Q;          /* 正交分量 Q */
    float amplitude;  /* 幅度 = sqrt(I²+Q²)（单频正弦的峰值幅度） */
    float phase;      /* 相位 (弧度, -π~π) = atan2(Q, I) */
} IQ_Result_t;

/* 接口 -------------------------------------------------------------- */

/**
 * @brief  对一段信号在指定频率处做 IQ 正交解调，得到幅度与相位。
 * @param  data  输入采样数组
 * @param  len   采样点数
 * @param  fs    采样率 (Hz)
 * @param  freq  解调参考频率 (Hz)，一般取被测信号频率
 * @param  res   [out] 解调结果（I/Q/幅度/相位）
 * @note   内部会先扣除直流分量，避免直流抬高低频处的 I 分量。
 */
void IQ_Demodulate(const float *data, uint32_t len, float fs, float freq,
                   IQ_Result_t *res);

/**
 * @brief  计算两路同频信号的相位差 (sig 相对 ref)。
 * @param  ref   参考通道采样数组
 * @param  sig   被测通道采样数组
 * @param  len   采样点数（两路需等长、同步采集）
 * @param  fs    采样率 (Hz)
 * @param  freq  信号频率 (Hz)
 * @return 相位差 (弧度)，已归一化到 (-π, π]，正值表示 sig 超前 ref。
 */
float IQ_PhaseDiff(const float *ref, const float *sig, uint32_t len,
                   float fs, float freq);

/**
 * @brief  弧度转角度（便于打印/显示）。
 */
float IQ_Rad2Deg(float rad);

#ifdef __cplusplus
}
#endif

#endif /* __IQ_H */
