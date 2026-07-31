/**
 * @file    ModelFit.c
 * @brief   Unknown RLC model fitting from measured frequency response.
 */
#include "ModelFit.h"
#include <math.h>
#include <stddef.h>

#define MODEL_FIT_ORDER          5u       /* 线性方程未知量个数。 */
#define MODEL_FIT_MIN_PIVOT      1.0e-18  /* 高斯消元允许的最小主元。 */
#define MODEL_FIT_MAG_FLOOR_REL  0.03f    /* 相位低信噪比加权的相对幅值基准。 */
#define MODEL_FIT_WEIGHT_MIN     0.08f    /* 测量点的最小拟合权重。 */

/* 将数值限制到给定闭区间。 */
static float ModelFit_ClampF(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/* 使用全主元行选择求解五元线性方程组。 */
static int ModelFit_Solve5(double a[MODEL_FIT_ORDER][MODEL_FIT_ORDER + 1u],
                           double x[MODEL_FIT_ORDER])
{
    for (uint32_t col = 0u; col < MODEL_FIT_ORDER; col++) {
        uint32_t piv = col;
        double maxv = fabs(a[col][col]);

        for (uint32_t r = col + 1u; r < MODEL_FIT_ORDER; r++) {
            double v = fabs(a[r][col]);
            if (v > maxv) {
                maxv = v;
                piv = r;
            }
        }

        if (maxv < MODEL_FIT_MIN_PIVOT) {
            return -1;
        }

        if (piv != col) {
            for (uint32_t c = col; c <= MODEL_FIT_ORDER; c++) {
                double t = a[col][c];
                a[col][c] = a[piv][c];
                a[piv][c] = t;
            }
        }

        const double diag = a[col][col];
        for (uint32_t r = 0u; r < MODEL_FIT_ORDER; r++) {
            if (r == col) continue;
            const double factor = a[r][col] / diag;
            for (uint32_t c = col; c <= MODEL_FIT_ORDER; c++) {
                a[r][c] -= factor * a[col][c];
            }
        }
    }

    for (uint32_t i = 0u; i < MODEL_FIT_ORDER; i++) {
        x[i] = a[i][MODEL_FIT_ORDER] / a[i][i];
    }
    return 0;
}

/* 将一行加权观测累加到正规方程。 */
static void ModelFit_AccumulateRow(double normal[MODEL_FIT_ORDER][MODEL_FIT_ORDER + 1u],
                                   const double row[MODEL_FIT_ORDER],
                                   double rhs,
                                   double weight)
{
    for (uint32_t r = 0u; r < MODEL_FIT_ORDER; r++) {
        const double wr = weight * row[r];
        for (uint32_t c = 0u; c < MODEL_FIT_ORDER; c++) {
            normal[r][c] += wr * row[c];
        }
        normal[r][MODEL_FIT_ORDER] += wr * rhs;
    }
}

/* 根据幅频和相频测量值拟合归一化二阶模型。 */
int ModelFit_FitSecondOrder(const float *freq_hz,
                            const float *gain_mag,
                            const float *phase_rad,
                            uint32_t n,
                            float norm_hz,
                            ModelFit_SecondOrder_t *model)
{
    if (freq_hz == NULL || gain_mag == NULL || phase_rad == NULL ||
        model == NULL || n < 5u || !isfinite(norm_hz) || norm_hz <= 0.0f) {
        return -1;
    }

    float max_mag = 0.0f;
    uint32_t valid_count = 0u;
    for (uint32_t k = 0u; k < n; k++) {
        if (isfinite(freq_hz[k]) && isfinite(gain_mag[k]) &&
            isfinite(phase_rad[k]) && freq_hz[k] > 0.0f && gain_mag[k] > 0.0f) {
            valid_count++;
        }
        if (isfinite(gain_mag[k]) && gain_mag[k] > max_mag) {
            max_mag = gain_mag[k];
        }
    }
    if (max_mag <= 0.0f || valid_count < 5u) {
        return -1;
    }

    const float mag_floor = max_mag * MODEL_FIT_MAG_FLOOR_REL;
    double normal[MODEL_FIT_ORDER][MODEL_FIT_ORDER + 1u] = {{0.0}};

    for (uint32_t k = 0u; k < n; k++) {
        const float mag = gain_mag[k];
        if (!isfinite(freq_hz[k]) || !isfinite(mag) || !isfinite(phase_rad[k]) ||
            freq_hz[k] <= 0.0f || mag <= 0.0f) {
            continue;
        }

        const double x = (double)(freq_hz[k] / norm_hz);
        const double x2 = x * x;
        const double r = (double)(mag * cosf(phase_rad[k]));
        const double im = (double)(mag * sinf(phase_rad[k]));

        /*
         * Low-output points have poor phase SNR. Keep them, but reduce their
         * ability to dominate the denominator fit.
         */
        float wf = (mag * mag) / (mag * mag + mag_floor * mag_floor);
        wf = ModelFit_ClampF(wf, MODEL_FIT_WEIGHT_MIN, 1.0f);
        const double w = (double)wf;

        /* Real equation:
         * c0 - c2*x^2 + Im(H)*x*d1 + Re(H)*x^2*d2 = Re(H)
         */
        const double row_re[MODEL_FIT_ORDER] = {
            1.0, 0.0, -x2, im * x, r * x2
        };
        ModelFit_AccumulateRow(normal, row_re, r, w);

        /* Imag equation:
         * c1*x - Re(H)*x*d1 + Im(H)*x^2*d2 = Im(H)
         */
        const double row_im[MODEL_FIT_ORDER] = {
            0.0, x, 0.0, -r * x, im * x2
        };
        ModelFit_AccumulateRow(normal, row_im, im, w);
    }

    double sol[MODEL_FIT_ORDER];
    if (ModelFit_Solve5(normal, sol) != 0) {
        return -1;
    }

    model->c0 = (float)sol[0];
    model->c1 = (float)sol[1];
    model->c2 = (float)sol[2];
    model->d1 = (float)sol[3];
    model->d2 = (float)sol[4];
    if (!isfinite(model->c0) || !isfinite(model->c1) || !isfinite(model->c2) ||
        !isfinite(model->d1) || !isfinite(model->d2)) {
        return -1;
    }
    model->norm_hz = norm_hz;
    model->type = 0u;

    double err2 = 0.0;
    uint32_t used = 0u;
    for (uint32_t k = 0u; k < n; k++) {
        if (!isfinite(freq_hz[k]) || !isfinite(gain_mag[k]) ||
            !isfinite(phase_rad[k]) || freq_hz[k] <= 0.0f || gain_mag[k] <= 0.0f) {
            continue;
        }
        ModelFit_Complex_t h = ModelFit_EvalComplex(model, freq_hz[k]);
        const double r = (double)(gain_mag[k] * cosf(phase_rad[k]));
        const double im = (double)(gain_mag[k] * sinf(phase_rad[k]));
        const double dr = (double)h.real - r;
        const double di = (double)h.imag - im;
        err2 += dr * dr + di * di;
        used++;
    }
    model->fit_rms = (used > 0u) ? (float)sqrt(err2 / (double)used) : 0.0f;

    return 0;
}

/* 计算模型在指定频率处的复数响应。 */
ModelFit_Complex_t ModelFit_EvalComplex(const ModelFit_SecondOrder_t *model,
                                        float freq_hz)
{
    ModelFit_Complex_t out = {0.0f, 0.0f};
    if (model == NULL || !isfinite(model->norm_hz) || !isfinite(freq_hz) ||
        !isfinite(model->c0) || !isfinite(model->c1) || !isfinite(model->c2) ||
        !isfinite(model->d1) || !isfinite(model->d2) || model->norm_hz <= 0.0f) {
        return out;
    }

    const float x = freq_hz / model->norm_hz;
    const float x2 = x * x;

    const float nr = model->c0 - model->c2 * x2;
    const float ni = model->c1 * x;
    const float dr = 1.0f - model->d2 * x2;
    const float di = model->d1 * x;
    const float den = dr * dr + di * di;

    if (den <= 1.0e-20f) {
        return out;
    }

    out.real = (nr * dr + ni * di) / den;
    out.imag = (ni * dr - nr * di) / den;
    return out;
}

/* 计算模型在指定频率处的幅值响应。 */
float ModelFit_EvalMag(const ModelFit_SecondOrder_t *model, float freq_hz)
{
    ModelFit_Complex_t h = ModelFit_EvalComplex(model, freq_hz);
    return sqrtf(h.real * h.real + h.imag * h.imag);
}

