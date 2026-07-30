/**
 * @file    Adaptive.c
 * @brief   自适应滤波、在线统计和线性校准实现。
 */
#include "Adaptive.h"
#include <math.h>
#include <stddef.h>

#define CC_2PI 6.28318530717958647692f /* 两倍圆周率。 */

/* 清零在线统计量。 */
void Contest_StatsInit(Contest_OnlineStats_t *s)
{
    if (s == NULL) return;
    s->n = 0u;
    s->mean = 0.0f;
    s->m2 = 0.0f;
    s->min_v = 0.0f;
    s->max_v = 0.0f;
}

/* 向在线统计量加入一个样本。 */
void Contest_StatsPush(Contest_OnlineStats_t *s, float x)
{
    if (s == NULL || !isfinite(x)) return;
    if (s->n == 0u) {
        s->n = 1u;
        s->mean = x;
        s->m2 = 0.0f;
        s->min_v = x;
        s->max_v = x;
        return;
    }
    s->n++;
    float delta = x - s->mean;
    s->mean += delta / (float)s->n;
    s->m2 += delta * (x - s->mean);
    if (x < s->min_v) s->min_v = x;
    if (x > s->max_v) s->max_v = x;
}

/* 返回样本方差。 */
float Contest_StatsVariance(const Contest_OnlineStats_t *s)
{
    if (s == NULL || s->n < 2u) return 0.0f;
    return s->m2 / (float)(s->n - 1u);
}

/* 返回样本标准差。 */
float Contest_StatsStd(const Contest_OnlineStats_t *s)
{
    float v = Contest_StatsVariance(s);
    return (v > 0.0f) ? sqrtf(v) : 0.0f;
}

/* 用参考值拟合一次线性校准参数。 */
int Contest_CalibrateLine(const float *raw, const float *ref, uint32_t n,
                          Contest_LineCal_t *cal)
{
    if (raw == NULL || ref == NULL || cal == NULL || n < 2u) return -1;
    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        if (!isfinite(raw[i]) || !isfinite(ref[i])) return -1;
        sx += raw[i];
        sy += ref[i];
        sxx += raw[i] * raw[i];
        sxy += raw[i] * ref[i];
    }
    float nf = (float)n;
    float den = nf * sxx - sx * sx;
    if (fabsf(den) < 1.0e-20f) return -1;
    cal->gain = (nf * sxy - sx * sy) / den;
    cal->offset = (sy - cal->gain * sx) / nf;

    float ymean = sy / nf;
    float ss_tot = 0.0f, ss_err = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        float fit = cal->gain * raw[i] + cal->offset;
        float dt = ref[i] - ymean;
        float de = ref[i] - fit;
        ss_tot += dt * dt;
        ss_err += de * de;
    }
    cal->r2 = (ss_tot > 1.0e-20f) ? (1.0f - ss_err / ss_tot) : 1.0f;
    return 0;
}

/* 对原始值应用一次线性校准。 */
float Contest_CalApply(const Contest_LineCal_t *cal, float raw)
{
    if (cal == NULL) return raw;
    return cal->gain * raw + cal->offset;
}

/* 按标准差阈值剔除异常样本并返回有效点数。 */
uint32_t Contest_RejectOutliers(const float *in, float *out, uint32_t len,
                                float sigma_limit)
{
    if (in == NULL || out == NULL || len == 0u) return 0u;
    if (sigma_limit <= 0.0f) sigma_limit = 3.0f;

    Contest_OnlineStats_t s;
    Contest_StatsInit(&s);
    for (uint32_t i = 0; i < len; i++) Contest_StatsPush(&s, in[i]);
    if (s.n == 0u) return 0u;
    float sd = Contest_StatsStd(&s);
    if (sd <= 1.0e-12f) {
        uint32_t nout = 0u;
        for (uint32_t i = 0; i < len; i++) {
            if (isfinite(in[i])) out[nout++] = in[i];
        }
        return nout;
    }

    uint32_t nout = 0u;
    float lim = sigma_limit * sd;
    for (uint32_t i = 0; i < len; i++) {
        if (fabsf(in[i] - s.mean) <= lim) {
            out[nout++] = in[i];
        }
    }
    return nout;
}

/* 初始化通用归一化LMS滤波器。 */
int Contest_LMSInit(Contest_LMS_t *lms, float *w, float *state,
                    uint32_t taps, float mu, float leak)
{
    if (lms == NULL || w == NULL || state == NULL || taps == 0u ||
        !isfinite(mu) || !isfinite(leak)) return -1;
    if (mu < 0.0f) mu = 0.0f;
    if (leak < 0.0f) leak = 0.0f;
    if (leak > 1.0f) leak = 1.0f;
    lms->taps = taps;
    lms->mu = mu;
    lms->leak = leak;
    lms->w = w;
    lms->state = state;
    for (uint32_t i = 0; i < taps; i++) {
        lms->w[i] = 0.0f;
        lms->state[i] = 0.0f;
    }
    return 0;
}

/* 处理一个LMS样本并返回滤波输出。 */
float Contest_LMSProcess(Contest_LMS_t *lms, float x, float desired,
                         float *err_out)
{
    if (lms == NULL || lms->w == NULL || lms->state == NULL || lms->taps == 0u) {
        if (err_out) *err_out = desired;
        return 0.0f;
    }
    if (!isfinite(x) || !isfinite(desired)) {
        if (err_out) *err_out = 0.0f;
        return 0.0f;
    }

    for (uint32_t i = lms->taps - 1u; i > 0u; i--) {
        lms->state[i] = lms->state[i - 1u];
    }
    lms->state[0] = x;

    float y = 0.0f;
    float pwr = 1.0e-9f;
    for (uint32_t i = 0; i < lms->taps; i++) {
        y += lms->w[i] * lms->state[i];
        pwr += lms->state[i] * lms->state[i];
    }
    float e = desired - y;
    float step = lms->mu * e / pwr;
    float keep = 1.0f - lms->leak;
    for (uint32_t i = 0; i < lms->taps; i++) {
        lms->w[i] = keep * lms->w[i] + step * lms->state[i];
    }
    if (err_out) *err_out = e;
    return y;
}

/* ==================================================================== */
/*  双权重LMS正弦分量分离。                                               */
/* ==================================================================== */

/* 初始化LMS正弦分量分离器。 */
int Contest_LMS_SineSepInit(Contest_LMS_SineSep_t *lms, float fs,
                            float freq, float mu)
{
    if (lms == NULL || !isfinite(fs) || !isfinite(freq) || !isfinite(mu) ||
        fs <= 0.0f || freq <= 0.0f || freq >= 0.5f * fs) {
        return -1;
    }
    if (mu <= 0.0f)  mu = 0.01f;
    if (mu > 1.0f)   mu = 1.0f;

    lms->fs    = fs;
    lms->freq  = freq;
    lms->mu    = mu;
    lms->w_cos = 0.0f;
    lms->w_sin = 0.0f;

    lms->delta = CC_2PI * freq / fs;
    lms->cosd  = cosf(lms->delta);
    lms->sind  = sinf(lms->delta);
    lms->c     = 1.0f;   /* cos(0) */
    lms->s     = 0.0f;   /* sin(0) */
    return 0;
}

/* 处理一个混合样本并返回目标频率分量。 */
float Contest_LMS_SineSepProcess(Contest_LMS_SineSep_t *lms, float mixture,
                                 float *err_out)
{
    if (lms == NULL) {
        if (err_out) *err_out = mixture;
        return 0.0f;
    }
    if (!isfinite(mixture)) {
        if (err_out) *err_out = 0.0f;
        return 0.0f;
    }

    /* Forward pass: y = w_cos * cos(phase) + w_sin * sin(phase) */
    float y = lms->w_cos * lms->c + lms->w_sin * lms->s;

    /* Error */
    float e = mixture - y;

    /* Normalised LMS weight update.
     * Reference power = c^2 + s^2 ≈ 1.0 (unit-amplitude rotation);
     * the epsilon guards against division by zero. */
    float pwr  = lms->c * lms->c + lms->s * lms->s + 1.0e-9f;
    float step = lms->mu * e / pwr;
    lms->w_cos += step * lms->c;
    lms->w_sin += step * lms->s;

    /* Advance reference phase via rotation matrix (same trick as IQ.c):
     *   [c(n+1)]   [cosd  -sind] [c(n)]
     *   [s(n+1)] = [sind   cosd] [s(n)]
     * Avoids per-sample sinf/cosf — only 4 muls + 2 adds in the hot path. */
    float c_new = lms->c * lms->cosd - lms->s * lms->sind;
    float s_new = lms->s * lms->cosd + lms->c * lms->sind;
    lms->c = c_new;
    lms->s = s_new;

    if (err_out) *err_out = e;
    return y;
}

/* 从收敛权重读取幅度和相位。 */
void Contest_LMS_SineSepGetParams(const Contest_LMS_SineSep_t *lms,
                                  float *amplitude, float *phase_rad)
{
    if (lms == NULL) {
        if (amplitude) *amplitude = 0.0f;
        if (phase_rad) *phase_rad = 0.0f;
        return;
    }
    if (amplitude) {
        *amplitude = sqrtf(lms->w_cos * lms->w_cos
                         + lms->w_sin * lms->w_sin);
    }
    if (phase_rad) {
        *phase_rad = atan2f(-lms->w_sin, lms->w_cos);
    }
}

/* 清零正弦分离权重并复位参考相位。 */
void Contest_LMS_SineSepReset(Contest_LMS_SineSep_t *lms)
{
    if (lms == NULL) return;
    lms->w_cos = 0.0f;
    lms->w_sin = 0.0f;
    lms->c     = 1.0f;
    lms->s     = 0.0f;
}

/* 初始化二阶陷波器。 */
int Contest_NotchInit(Contest_Notch_t *n, float fs, float freq, float radius)
{
    if (n == NULL || !isfinite(fs) || !isfinite(freq) || !isfinite(radius) ||
        fs <= 0.0f || freq <= 0.0f || freq >= 0.5f * fs) return -1;
    if (radius < 0.80f) radius = 0.80f;
    if (radius > 0.9999f) radius = 0.9999f;
    float w = CC_2PI * freq / fs;
    float c = cosf(w);
    n->fs = fs;
    n->freq = freq;
    n->radius = radius;
    n->b0 = 1.0f;
    n->b1 = -2.0f * c;
    n->b2 = 1.0f;
    n->a1 = -2.0f * radius * c;
    n->a2 = radius * radius;
    n->z1 = 0.0f;
    n->z2 = 0.0f;
    return 0;
}

/* 处理一个陷波滤波样本。 */
float Contest_NotchProcess(Contest_Notch_t *n, float x)
{
    if (n == NULL || !isfinite(x)) return x;
    float y = n->b0 * x + n->z1;
    n->z1 = n->b1 * x - n->a1 * y + n->z2;
    n->z2 = n->b2 * x - n->a2 * y;
    return y;
}

/* 清零陷波器状态。 */
void Contest_NotchReset(Contest_Notch_t *n)
{
    if (n == NULL) return;
    n->z1 = 0.0f;
    n->z2 = 0.0f;
}
