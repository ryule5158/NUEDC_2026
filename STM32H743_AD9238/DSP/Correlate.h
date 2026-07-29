/**
 * @file    Correlate.h
 * @brief   相关分析 — 互相关(时延/相位估计) + 自相关(基频/周期检测)
 * @note    采用时域直接法并限定最大滞后 max_lag，复杂度 O(len·max_lag)，
 *          对"只关心小范围时延/周期"的赛题足够快，且无需 FFT、无动态内存。
 *          典型用途：
 *            - 两路信号时延测量 / 相位差(经时延换算)；
 *            - 噪声中周期信号的基频估计(自相关首峰)。
 * @date    2026-06-18
 */

#ifndef __CORRELATE_H
#define __CORRELATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/**
 * @brief  互相关(有偏估计)：r[m] = (1/len)Σ x[n]·y[n-m]，m = -max_lag..+max_lag。
 * @param  x,y      两路等长信号
 * @param  len      长度
 * @param  max_lag  最大滞后(>0)
 * @param  out      [out] 长度 2*max_lag+1，out[0] 对应 lag=-max_lag，out[max_lag] 对应 lag=0
 */
void Corr_Cross(const float *x, const float *y, uint32_t len,
                int32_t max_lag, float *out);

/**
 * @brief  时延估计：找互相关峰所在滞后(带抛物线插值)，换算成秒。
 * @param  x,y        两路信号(求 y 相对 x 的时延)
 * @param  len        长度
 * @param  max_lag    搜索范围(采样)
 * @param  fs         采样率 (Hz)
 * @param  delay_sec  [out,可空] 时延(秒)，正值表示 y 滞后于 x
 * @return 峰值滞后(采样，带小数)。
 */
float Corr_TimeDelay(const float *x, const float *y, uint32_t len,
                     int32_t max_lag, float fs, float *delay_sec);

/**
 * @brief  自相关(有偏)：r[m] = (1/len)Σ x[n]·x[n-m]，m = 0..max_lag。
 * @param  x        信号
 * @param  len      长度
 * @param  max_lag  最大滞后
 * @param  out      [out] 长度 max_lag+1，out[0]=lag0(能量)
 */
void Corr_Auto(const float *x, uint32_t len, uint32_t max_lag, float *out);

/**
 * @brief  基频/周期估计(自相关首峰法)，适合噪声中的周期信号。
 * @param  x      信号
 * @param  len    长度
 * @param  fs     采样率 (Hz)
 * @param  fmin   预期最低频率 (Hz)，限定搜索上限滞后 = fs/fmin
 * @param  fmax   预期最高频率 (Hz)，限定搜索下限滞后 = fs/fmax
 * @return 基频 (Hz)；找不到返回 0。
 */
float Corr_Pitch(const float *x, uint32_t len, float fs, float fmin, float fmax);

#ifdef __cplusplus
}
#endif

#endif /* __CORRELATE_H */
