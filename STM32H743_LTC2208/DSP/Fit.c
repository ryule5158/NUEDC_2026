/**
 * @file    Fit.c
 * @brief   最小二乘拟合实现
 */
#include "Fit.h"
#include <math.h>

/* 用最小二乘法拟合直线并可选返回决定系数。 */
int Fit_Linear(const float *x, const float *y, uint32_t n,
               float *a, float *b, float *r2)
{
    if (x == NULL || y == NULL || a == NULL || b == NULL || n < 2u) return -1;

    float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
    for (uint32_t i = 0; i < n; i++) {
        if (!isfinite(x[i]) || !isfinite(y[i])) return -1;
        sx  += x[i];
        sy  += y[i];
        sxx += x[i] * x[i];
        sxy += x[i] * y[i];
    }
    float nf = (float)n;
    float denom = nf * sxx - sx * sx;
    if (fabsf(denom) < 1e-20f) return -1;     /* x 全相同，无法拟合 */

    *a = (nf * sxy - sx * sy) / denom;
    *b = (sy - (*a) * sx) / nf;

    if (r2 != NULL) {
        float mean_y = sy / nf;
        float ss_tot = 0.0f, ss_res = 0.0f;
        for (uint32_t i = 0; i < n; i++) {
            float fit = (*a) * x[i] + (*b);
            float dt = y[i] - mean_y;
            float dr = y[i] - fit;
            ss_tot += dt * dt;
            ss_res += dr * dr;
        }
        *r2 = (ss_tot > 1e-20f) ? (1.0f - ss_res / ss_tot) : 1.0f;
    }
    return 0;
}

/* 用正规方程拟合指定次数的多项式。 */
int Fit_Poly(const float *x, const float *y, uint32_t n, uint32_t degree,
             float *coeffs)
{
    if (x == NULL || y == NULL || coeffs == NULL) return -1;
    if (degree < 1u || degree > FIT_MAX_DEGREE) return -1;
    if (n <= degree) return -1;

    const uint32_t m = degree + 1u;          /* 未知数个数 */

    /* 正规方程 A·c = B，其中 A[i][j] = Σ x^(i+j)，B[i] = Σ y·x^i */
    /* 先求幂和 Σx^k (k=0..2*degree) 与 Σy·x^i (i=0..degree) */
    float powsum[2 * FIT_MAX_DEGREE + 1];
    float ysum[FIT_MAX_DEGREE + 1];
    for (uint32_t k = 0; k <= 2u * degree; k++) powsum[k] = 0.0f;
    for (uint32_t i = 0; i < m; i++)           ysum[i]   = 0.0f;

    for (uint32_t p = 0; p < n; p++) {
        float xv = x[p];
        if (!isfinite(xv) || !isfinite(y[p])) return -1;
        float xp = 1.0f;                       /* x^k */
        for (uint32_t k = 0; k <= 2u * degree; k++) {
            powsum[k] += xp;
            if (k <= degree) ysum[k] += y[p] * xp;
            xp *= xv;
        }
    }

    /* 组装增广矩阵 [A | B]，尺寸 m x (m+1) */
    float aug[FIT_MAX_DEGREE + 1][FIT_MAX_DEGREE + 2];
    for (uint32_t i = 0; i < m; i++) {
        for (uint32_t j = 0; j < m; j++) {
            aug[i][j] = powsum[i + j];
        }
        aug[i][m] = ysum[i];
    }

    /* 高斯消元(列主元) */
    for (uint32_t col = 0; col < m; col++) {
        /* 选主元 */
        uint32_t piv = col;
        float maxv = fabsf(aug[col][col]);
        for (uint32_t r = col + 1u; r < m; r++) {
            float v = fabsf(aug[r][col]);
            if (v > maxv) { maxv = v; piv = r; }
        }
        if (maxv < 1e-20f) return -1;          /* 奇异，拟合失败 */

        /* 交换行 */
        if (piv != col) {
            for (uint32_t j = 0; j <= m; j++) {
                float t = aug[col][j];
                aug[col][j] = aug[piv][j];
                aug[piv][j] = t;
            }
        }

        /* 消元 */
        float diag = aug[col][col];
        for (uint32_t r = 0; r < m; r++) {
            if (r == col) continue;
            float factor = aug[r][col] / diag;
            for (uint32_t j = col; j <= m; j++) {
                aug[r][j] -= factor * aug[col][j];
            }
        }
    }

    /* 回代(此时对角化后直接除) */
    for (uint32_t i = 0; i < m; i++) {
        coeffs[i] = aug[i][m] / aug[i][i];
    }
    return 0;
}

/* 使用霍纳法计算多项式在指定横坐标处的值。 */
float Fit_PolyEval(const float *coeffs, uint32_t degree, float x)
{
    if (coeffs == NULL) return 0.0f;
    /* 霍纳法：从最高次往下 */
    float y = coeffs[degree];
    for (int32_t i = (int32_t)degree - 1; i >= 0; i--) {
        y = y * x + coeffs[i];
    }
    return y;
}
