/**
 * @file    Fit.h
 * @brief   最小二乘曲线拟合 — 直线 / 多项式（仪器校准、标定常用）
 * @note    典型用途：把"原始测量值 → 真实物理量"做线性/多项式校正；
 *          幅频特性曲线拟合；传感器标定。多项式用正规方程 + 高斯消元，
 *          阶数限定 <= FIT_MAX_DEGREE，无动态内存。
 * @date    2026-06-18
 */

#ifndef __FIT_H
#define __FIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define FIT_MAX_DEGREE   6   /* 多项式最高阶数 */

/**
 * @brief  一元线性最小二乘拟合 y = a*x + b。
 * @param  x,y  数据点
 * @param  n    点数(>=2)
 * @param  a    [out] 斜率
 * @param  b    [out] 截距
 * @param  r2   [out,可空] 决定系数 R²(0~1，越接近 1 拟合越好)
 * @return 0 成功，-1 失败(点数不足或 x 全相同)
 */
int Fit_Linear(const float *x, const float *y, uint32_t n,
               float *a, float *b, float *r2);

/**
 * @brief  多项式最小二乘拟合 y = c0 + c1*x + c2*x² + ... + cd*x^d。
 * @param  x,y     数据点
 * @param  n       点数(需 > degree)
 * @param  degree  多项式阶数(1~FIT_MAX_DEGREE)
 * @param  coeffs  [out] 系数数组，长度 degree+1，coeffs[0]=c0 ...
 * @return 0 成功，-1 失败
 * @note   为改善数值条件，建议先把 x 归一化(如减均值/除量程)再拟合。
 */
int Fit_Poly(const float *x, const float *y, uint32_t n, uint32_t degree,
             float *coeffs);

/**
 * @brief  多项式求值(霍纳法)。
 * @param  coeffs  系数(coeffs[0]=常数项)
 * @param  degree  阶数
 * @param  x       自变量
 * @return 多项式在 x 处的值。
 */
float Fit_PolyEval(const float *coeffs, uint32_t degree, float x);

#ifdef __cplusplus
}
#endif

#endif /* __FIT_H */
