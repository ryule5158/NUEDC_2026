#ifndef __DSP_PROMAX_H
#define __DSP_PROMAX_H /* DSP ProMax聚合分析接口包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "FFT.h"
#include "Measure.h"
#include <stdint.h>

/* DSP ProMax分析状态。 */
typedef enum
{
  DSP_PROMAX_OK = 0,          /* 时域和频域分析均已完成。 */
  DSP_PROMAX_ERROR_PARAM,     /* 输入指针、长度或采样率无效。 */
  DSP_PROMAX_ERROR_LENGTH,    /* 时域分析完成，但长度不是支持的 FFT 点数。 */
  DSP_PROMAX_ERROR_FFT        /* FFT 或单边谱归一化失败。 */
} DSP_ProMaxStatusTypeDef;

/* DSP ProMax 可配置分析参数。 */
typedef struct
{
  Measure_THDConfig_t thd;    /* THD 搜索范围、最高次数、主瓣宽度和门限。 */
  uint32_t quality_harmonics; /* SNR/SINAD 中排除的最高谐波次数。 */
  uint32_t quality_leak_bins; /* 质量指标对每个谱峰聚合的半宽。 */
} DSP_ProMaxConfigTypeDef;

/* DSP ProMax一站式分析结果。 */
typedef struct
{
  uint8_t time_valid;         /* 时域结果有效标志。 */
  uint8_t fft_valid;          /* 频域结果有效标志。 */
  uint8_t harmonic_count;     /* 已提取的谐波数量。 */

  float dc;                   /* 直流分量。 */
  float rms;                  /* 含直流的真有效值。 */
  float acrms;                /* 去除直流后的交流有效值。 */
  float vpp;                  /* 峰峰值。 */

  Measure_Wave_t wave;        /* 频率、周期、占空比和边沿等时域参数。 */
  Measure_Peak_t peak;        /* FFT主峰频率、幅度和插值后频点。 */
  Measure_Quality_t quality;  /* THD、SNR、SINAD和ENOB。 */
  Measure_THDResult_t thd;    /* THD%、H1~Hn归一化幅度、有效性和误差状态。 */

  float harmonic_freq[FFT_MAX_HARMONICS]; /* 各次谐波频率。 */
  float harmonic_amp[FFT_MAX_HARMONICS];  /* 各次谐波幅度。 */
} DSP_ProMaxResultTypeDef;

/* 填充默认配置：Hann 主瓣 ±2 bin，自动测量到 5 次谐波。 */
void DSP_ProMax_ConfigDefault(DSP_ProMaxConfigTypeDef *config);

/*
 * 可配置的一站式分析。len 支持 FFT_StartN() 接受的 64~4096 点二次幂长度。
 * 使用 config 可把基波搜索范围约束在赛题规定频带，避免带外杂散被误判为基波。
 */
DSP_ProMaxStatusTypeDef DSP_ProMax_AnalyzeEx(const float *data,
                                             uint32_t len,
                                             float fs_hz,
                                             const DSP_ProMaxConfigTypeDef *config,
                                             DSP_ProMaxResultTypeDef *result);

/*
 * 一次完成时域与频域分析。
 * data为输入采样，len为采样点数，fs_hz为采样率，result返回分析结果。
 * 使用默认 THD 配置，支持 64~4096 点二次幂 FFT；其他长度只返回时域结果。
 */
DSP_ProMaxStatusTypeDef DSP_ProMax_Analyze(const float *data,
                                           uint32_t len,
                                           float fs_hz,
                                           DSP_ProMaxResultTypeDef *result);

/* 获取最近一次分析结果的只读指针。 */
const DSP_ProMaxResultTypeDef *DSP_ProMax_GetLastResult(void);

/* 通过printf输出关键分析结果，result为NULL时输出最近一次结果。 */
void DSP_ProMax_PrintResult(const DSP_ProMaxResultTypeDef *result);

#ifdef __cplusplus
}
#endif

#endif /* __DSP_PROMAX_H */
