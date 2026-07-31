/**
 * @file    Adaptive.h
 * @brief   自适应滤波、在线统计和线性校准接口。
 */
#ifndef __ADAPTIVE_H
#define __ADAPTIVE_H /* 自适应处理头文件包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 在线统计量。 */
typedef struct {
    uint32_t n;    /* 有效样本数。 */
    float mean;    /* 样本均值。 */
    float m2;      /* 均值平方差累加量。 */
    float min_v;   /* 最小样本值。 */
    float max_v;   /* 最大样本值。 */
} Contest_OnlineStats_t;

/* 一次线性校准结果。 */
typedef struct {
    float gain;    /* 校准增益。 */
    float offset;  /* 校准偏置。 */
    float r2;      /* 拟合决定系数。 */
} Contest_LineCal_t;

/* 通用归一化LMS滤波器状态。 */
typedef struct {
    uint32_t taps; /* 滤波器抽头数。 */
    float mu;      /* LMS步长。 */
    float leak;    /* 权重泄漏系数。 */
    float *w;      /* 权重数组。 */
    float *state;  /* 输入状态数组。 */
} Contest_LMS_t;

/**
 * @brief  已知频率正弦分量的双权重LMS分离状态。
 * @note   收敛后权重可换算目标分量的幅度和相位。
 */
typedef struct {
    float fs;           /* 采样率，单位Hz。 */
    float freq;         /* 目标频率，单位Hz。 */
    float mu;           /* LMS步长。 */
    float w_cos;        /* 余弦参考权重。 */
    float w_sin;        /* 正弦参考权重。 */
    float delta;        /* 每点相位增量。 */
    float cosd, sind;   /* 相位旋转系数。 */
    float c, s;         /* 当前正交参考值。 */
} Contest_LMS_SineSep_t;

/* 二阶陷波器状态。 */
typedef struct {
    float fs;           /* 采样率，单位Hz。 */
    float freq;         /* 陷波频率，单位Hz。 */
    float radius;       /* 极点半径。 */
    float b0, b1, b2;   /* 分子系数。 */
    float a1, a2;       /* 分母系数。 */
    float z1, z2;       /* 滤波状态。 */
} Contest_Notch_t;

/* 清零在线统计量。 */
void Contest_StatsInit(Contest_OnlineStats_t *s);
/* 向在线统计量加入一个样本。 */
void Contest_StatsPush(Contest_OnlineStats_t *s, float x);
/* 返回样本方差。 */
float Contest_StatsVariance(const Contest_OnlineStats_t *s);
/* 返回样本标准差。 */
float Contest_StatsStd(const Contest_OnlineStats_t *s);

/* 用参考值拟合一次线性校准参数。 */
int Contest_CalibrateLine(const float *raw, const float *ref, uint32_t n,
                          Contest_LineCal_t *cal);
/* 对原始值应用一次线性校准。 */
float Contest_CalApply(const Contest_LineCal_t *cal, float raw);

/* 按标准差阈值剔除异常样本并返回有效点数。 */
uint32_t Contest_RejectOutliers(const float *in, float *out, uint32_t len,
                                float sigma_limit);

/* 初始化通用归一化LMS滤波器。 */
int Contest_LMSInit(Contest_LMS_t *lms, float *w, float *state,
                    uint32_t taps, float mu, float leak);
/* 处理一个LMS样本并返回滤波输出。 */
float Contest_LMSProcess(Contest_LMS_t *lms, float x, float desired,
                         float *err_out);

/**
 * @brief   初始化LMS正弦分量分离器。
 * @param   lms 状态指针。
 * @param   fs 采样率，单位Hz。
 * @param   freq 目标频率，单位Hz。
 * @param   mu LMS步长。
 * @return  成功返回0，参数无效返回-1。
 */
int Contest_LMS_SineSepInit(Contest_LMS_SineSep_t *lms, float fs,
                            float freq, float mu);

/**
 * @brief   处理一个混合信号样本并返回目标频率分量。
 * @param   lms 状态指针。
 * @param   mixture 混合信号样本。
 * @param   err_out 可选的残差信号输出。
 * @return  当前目标频率分量。
 */
float Contest_LMS_SineSepProcess(Contest_LMS_SineSep_t *lms, float mixture,
                                 float *err_out);

/**
 * @brief   从收敛权重读取幅度和相位。
 * @param   lms 状态指针。
 * @param   amplitude 可选的幅度输出。
 * @param   phase_rad 可选的弧度相位输出。
 */
void Contest_LMS_SineSepGetParams(const Contest_LMS_SineSep_t *lms,
                                  float *amplitude, float *phase_rad);

/**
 * @brief   清零正弦分离权重并复位参考相位。
 */
void Contest_LMS_SineSepReset(Contest_LMS_SineSep_t *lms);

/* 初始化二阶陷波器。 */
int Contest_NotchInit(Contest_Notch_t *n, float fs, float freq, float radius);
/* 处理一个陷波滤波样本。 */
float Contest_NotchProcess(Contest_Notch_t *n, float x);
/* 清零陷波器状态。 */
void Contest_NotchReset(Contest_Notch_t *n);

#ifdef __cplusplus
}
#endif

#endif /* __ADAPTIVE_H */
