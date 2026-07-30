/**
 * @file    ModelFit.h
 * @brief   Unknown RLC model fitting from measured frequency response.
 */
/* 模型拟合模块头文件保护宏。 */
#ifndef __MODEL_FIT_H
#define __MODEL_FIT_H /* 模型拟合模块头文件保护宏。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 模型识别结果类型。 */
typedef enum {
    MODEL_FILTER_INVALID = 255
} ModelFit_FilterType_t;

/* 归一化二阶连续系统模型及拟合质量。 */
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
    float c0;
    float c1;
    float c2;
    float d1;
    float d2;
    float norm_hz;
    float fit_rms;
    uint8_t type;
} ModelFit_SecondOrder_t;

/* 单精度复数。 */
typedef struct {
    float real;
    float imag;
} ModelFit_Complex_t;

/* 根据幅频和相频测量值拟合归一化二阶模型。 */
int ModelFit_FitSecondOrder(const float *freq_hz,
                            const float *gain_mag,
                            const float *phase_rad,
                            uint32_t n,
                            float norm_hz,
                            ModelFit_SecondOrder_t *model);

/* 计算模型在指定频率处的复数响应。 */
ModelFit_Complex_t ModelFit_EvalComplex(const ModelFit_SecondOrder_t *model,
                                        float freq_hz);

/* 计算模型在指定频率处的幅值响应。 */
float ModelFit_EvalMag(const ModelFit_SecondOrder_t *model, float freq_hz);


#ifdef __cplusplus
}
#endif

#endif /* __MODEL_FIT_H */
