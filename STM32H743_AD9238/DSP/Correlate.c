/**
 * @file    Correlate.c
 * @brief   相关分析实现（时域直接法，限定最大滞后）
 */
#include "Correlate.h"
#include <math.h>

/* 计算两路信号在限定滞后范围内的互相关序列。 */
void Corr_Cross(const float *x, const float *y, uint32_t len,
                int32_t max_lag, float *out)
{
    if (x == NULL || y == NULL || out == NULL || len == 0u || max_lag <= 0) return;

    const float inv = 1.0f / (float)len;
    for (int32_t m = -max_lag; m <= max_lag; m++) {
        /* 有效区间 n ∈ [max(0,m), min(len, len+m)) */
        int32_t n0 = (m > 0) ? m : 0;
        int32_t n1 = (m < 0) ? ((int32_t)len + m) : (int32_t)len;
        float acc = 0.0f;
        for (int32_t n = n0; n < n1; n++) {
            acc += x[n] * y[n - m];
        }
        out[m + max_lag] = acc * inv;
    }
}

/* 通过互相关峰值估计两路信号的时延。 */
float Corr_TimeDelay(const float *x, const float *y, uint32_t len,
                     int32_t max_lag, float fs, float *delay_sec)
{
    if (delay_sec) *delay_sec = 0.0f;
    if (x == NULL || y == NULL || len < 2u || max_lag <= 0 ||
        !isfinite(fs) || fs <= 0.0f) return 0.0f;
    if ((uint32_t)max_lag >= len) max_lag = (int32_t)(len - 1u);

    /* 不缓存整条相关序列：边算边找峰 + 记录峰两侧值用于插值 */
    float prev_acc = 0.0f;     /* 上一滞后(m-1)的相关值 */
    float best = -1e30f, best_a = 0.0f, best_b = 0.0f, best_c = 0.0f;
    int32_t best_lag = -max_lag;
    const float inv = 1.0f / (float)len;

    for (int32_t m = -max_lag; m <= max_lag; m++) {
        int32_t n0 = (m > 0) ? m : 0;
        int32_t n1 = (m < 0) ? ((int32_t)len + m) : (int32_t)len;
        float acc = 0.0f;
        for (int32_t n = n0; n < n1; n++) {
            acc += x[n] * y[n - m];
        }
        acc *= inv;

        /* 若上一轮刚刷新了峰，则本轮(m=best_lag+1)的值即峰右邻 */
        if (m == best_lag + 1) {
            best_c = acc;
        }
        if (acc > best) {
            best = acc;
            best_lag = m;
            best_a = prev_acc;   /* 峰左邻(m-1) */
            best_b = acc;
            best_c = acc;        /* 暂置，待下一轮覆盖；峰在末端时退化 */
        }
        prev_acc = acc;
    }

    /* 抛物线插值细化峰位置 */
    float delta = 0.0f;
    float denom = best_a - 2.0f * best_b + best_c;
    if (fabsf(denom) > 1e-12f) {
        delta = 0.5f * (best_a - best_c) / denom;
        if (delta > 0.5f)  delta = 0.5f;
        if (delta < -0.5f) delta = -0.5f;
    }
    float lag = (float)best_lag + delta;
    if (delay_sec) *delay_sec = lag / fs;
    return lag;
}

/* 计算输入信号从零到最大滞后的自相关序列。 */
void Corr_Auto(const float *x, uint32_t len, uint32_t max_lag, float *out)
{
    if (x == NULL || out == NULL || len == 0u) return;
    if (max_lag >= len) max_lag = len - 1u;

    const float inv = 1.0f / (float)len;
    for (uint32_t m = 0; m <= max_lag; m++) {
        float acc = 0.0f;
        for (uint32_t n = m; n < len; n++) {
            acc += x[n] * x[n - m];
        }
        out[m] = acc * inv;
    }
}

/* 内部：计算单个滞后的自相关值 */
static float Corr_AutoAt(const float *x, uint32_t len, uint32_t lag)
{
    float acc = 0.0f;
    for (uint32_t n = lag; n < len; n++) {
        acc += x[n] * x[n - lag];
    }
    return acc / (float)len;
}

/* 在指定频率范围内通过自相关估计基频。 */
float Corr_Pitch(const float *x, uint32_t len, float fs, float fmin, float fmax)
{
    if (x == NULL || len < 4u || fs <= 0.0f || fmin <= 0.0f || fmax <= fmin) return 0.0f;

    uint32_t lag_lo = (uint32_t)(fs / fmax);     /* 高频 -> 小滞后 */
    uint32_t lag_hi = (uint32_t)(fs / fmin);     /* 低频 -> 大滞后 */
    if (lag_lo < 1u) lag_lo = 1u;
    if (lag_hi >= len) lag_hi = len - 1u;
    if (lag_hi <= lag_lo) return 0.0f;

    /* 在 [lag_lo, lag_hi] 找自相关最大峰(排除 lag0) */
    float best = -1e30f;
    uint32_t best_lag = lag_lo;
    for (uint32_t lag = lag_lo; lag <= lag_hi; lag++) {
        float r = Corr_AutoAt(x, len, lag);
        if (r > best) { best = r; best_lag = lag; }
    }

    /* 抛物线插值 */
    float lag = (float)best_lag;
    if (best_lag > lag_lo && best_lag < lag_hi) {
        float a = Corr_AutoAt(x, len, best_lag - 1u);
        float b = best;
        float c = Corr_AutoAt(x, len, best_lag + 1u);
        float denom = a - 2.0f * b + c;
        if (fabsf(denom) > 1e-12f) {
            float delta = 0.5f * (a - c) / denom;
            if (delta > 0.5f)  delta = 0.5f;
            if (delta < -0.5f) delta = -0.5f;
            lag += delta;
        }
    }

    return (lag > 0.0f) ? (fs / lag) : 0.0f;
}
