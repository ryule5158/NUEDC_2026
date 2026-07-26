/**
 * @file    ADC_FFT.h
 * @brief   ADC采样帧的FFT、基频与谐波分析接口。
 */

#ifndef __ADC_FFT_H
#define __ADC_FFT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FFT.h"
#include <stdint.h>

#define ADC_FFT_SAMPLE_LENGTH FFT_length /* 固定输入点数。 */
#define ADC_FFT_MAX_HARMONICS FFT_MAX_HARMONICS /* 最大谐波数量。 */
#define ADC_FFT_STATUS_OK 0U /* 分析成功。 */
#define ADC_FFT_STATUS_ERROR 1U /* 参数或信号分析失败。 */

/* 单帧ADC频谱分析结果。 */
typedef struct {
    float fundamental_hz; /* 插值后的基频，单位Hz。 */
    uint8_t harmonic_count; /* Goertzel谐波数量。 */
    uint8_t harmonic_fft_count; /* FFT谱谐波数量。 */
    float harmonic_freq[ADC_FFT_MAX_HARMONICS]; /* Goertzel谐波频率。 */
    float harmonic_amp[ADC_FFT_MAX_HARMONICS]; /* Goertzel谐波幅值。 */
    float harmonic_freq_fft[ADC_FFT_MAX_HARMONICS]; /* FFT谐波频率。 */
    float harmonic_amp_fft[ADC_FFT_MAX_HARMONICS]; /* FFT谐波幅值。 */
    uint32_t frame_count; /* 成功分析的累计帧数。 */
    uint8_t status; /* 最近一次分析状态。 */
} ADC_FFT_Result_t;

/* 兼容原ADC采样封装的结果类型。 */
typedef ADC_FFT_Result_t ADC_SampleResult_t;

/* 设置默认谐波搜索上限。 */
void ADC_FFT_SetMaxHarmonic(float max_harmonic_hz);
/* 使用默认频率上限分析一帧。 */
uint8_t ADC_FFT_Run(const float *data, uint32_t len, float sample_rate_hz);
/* 使用指定频率上限分析一帧。 */
uint8_t ADC_FFT_RunWithMaxHarmonic(const float *data,
                                   uint32_t len,
                                   float sample_rate_hz,
                                   float max_harmonic_hz);
/* 获取最近一次分析结果。 */
const ADC_FFT_Result_t* ADC_FFT_GetResult(void);
/* 打印指定结果，传NULL时打印最近结果。 */
void ADC_FFT_PrintResult(const ADC_FFT_Result_t *result);

/* BSP/adc_app兼容接口。 */
void ADC_Sample_RunFft(void);
void ADC_Sample_PrintResult(void);
void ADC_Sample_SetFftMaxHarmonic(float max_harmonic_hz);
const ADC_SampleResult_t* ADC_Sample_GetResult(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_FFT_H */
