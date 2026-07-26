/**
 * @file    ModelFit.h
 * @brief   根据实测频率响应拟合通用二阶模型。
 */
#ifndef __MODEL_FIT_H
#define __MODEL_FIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    MODEL_FILTER_SECOND_ORDER = 0, /* 通用二阶连续系统模型。 */
    MODEL_FILTER_INVALID = 255 /* 无效模型。 */
} ModelFit_FilterType_t;

/* 归一化二阶连续系统模型参数。 */
typedef struct {
    /*
     * Normalized continuous model:
     *
     *            c0 + c1*(j*x) + c2*(j*x)^2
     * H(jx) = ---------------------------------
     *            1  + d1*(j*x) + d2*(j*x)^2
     *
     * x = f_hz / norm_hz.
     */
    float c0; /* 分子常数项。 */
    float c1; /* 分子一阶项。 */
    float c2; /* 分子二阶项。 */
    float d1; /* 分母一阶项。 */
    float d2; /* 分母二阶项。 */
    float norm_hz; /* 归一化频率，单位Hz。 */
    float fit_rms; /* 复数频响拟合均方根误差。 */
    uint8_t type; /* ModelFit_FilterType_t模型类型。 */
} ModelFit_SecondOrder_t;

/* 单精度复数结果。 */
typedef struct {
    float real; /* 实部。 */
    float imag; /* 虚部。 */
} ModelFit_Complex_t;

/* 从幅频、相频样本拟合二阶模型，成功返回0。 */
int ModelFit_FitSecondOrder(const float *freq_hz,
                            const float *gain_mag,
                            const float *phase_rad,
                            uint32_t n,
                            float norm_hz,
                            ModelFit_SecondOrder_t *model);

/* 计算模型在指定频率处的复数响应。 */
ModelFit_Complex_t ModelFit_EvalComplex(const ModelFit_SecondOrder_t *model,
                                        float freq_hz);

/* 计算模型在指定频率处的幅值。 */
float ModelFit_EvalMag(const ModelFit_SecondOrder_t *model, float freq_hz);


#ifdef __cplusplus
}
#endif

#endif /* __MODEL_FIT_H */
