/**
 * @file    ADC_FFT.c
 * @brief   ADC采样缓冲区通用FFT分析实现。
 */

#include "ADC_FFT.h"
#include "adc_app.h"
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static ADC_FFT_Result_t s_result; /* 最近一次FFT分析结果。 */
static float s_max_harmonic_hz = 1000.0f; /* 默认最高谐波频率。 */

/* 在当前频谱中寻找基波。 */
static float FindFundamentalFft(void);
/* 清空结果并记录分析失败。 */
static uint8_t ADC_FFT_SetError(void);
/* 将浮点数按指定比例四舍五入为整数。 */
static int32_t ADC_FFT_ScaleFloat(float value, float scale);
/* 打印带指定小数位数的定点数。 */
static void ADC_FFT_PrintFixed(int32_t scaled_value, uint32_t decimals);

/* 设置默认最高谐波频率。 */
void ADC_FFT_SetMaxHarmonic(float max_harmonic_hz)
{
    if (max_harmonic_hz > 0.0f) {
        s_max_harmonic_hz = max_harmonic_hz;
    }
}

/* 使用默认最高谐波频率分析一帧数据。 */
uint8_t ADC_FFT_Run(const float *data, uint32_t len, float sample_rate_hz)
{
    return ADC_FFT_RunWithMaxHarmonic(data, len, sample_rate_hz, s_max_harmonic_hz);
}

/* 使用指定最高谐波频率分析一帧数据。 */
uint8_t ADC_FFT_RunWithMaxHarmonic(const float *data,
                                   uint32_t len,
                                   float sample_rate_hz,
                                   float max_harmonic_hz)
{
    if (data == NULL || len != ADC_FFT_SAMPLE_LENGTH ||
        !isfinite(sample_rate_hz) || !isfinite(max_harmonic_hz) ||
        sample_rate_hz <= 0.0f || max_harmonic_hz <= 0.0f) {
        return ADC_FFT_SetError();
    }

    FFT_Fs = sample_rate_hz;
    if (max_harmonic_hz > FFT_Fs * 0.5f) {
        max_harmonic_hz = FFT_Fs * 0.5f;
    }

    FFT_Input((float *)data);
    FFT_Start();

    s_result.fundamental_hz = FindFundamentalFft();
    if (!(s_result.fundamental_hz > 0.0f)) {
        return ADC_FFT_SetError();
    }

    s_result.harmonic_count = FFT_GetHarmonics((float *)data,
                                               len,
                                               FFT_Fs,
                                               s_result.fundamental_hz,
                                               max_harmonic_hz,
                                               s_result.harmonic_freq,
                                               s_result.harmonic_amp);

    s_result.harmonic_fft_count = FFT_GetHarmonics_FromFFT(FFT_Fs,
                                                           s_result.fundamental_hz,
                                                           max_harmonic_hz,
                                                           s_result.harmonic_freq_fft,
                                                           s_result.harmonic_amp_fft);

    s_result.status = ADC_FFT_STATUS_OK;
    s_result.frame_count++;
    return s_result.status;
}

/* 获取最近一次FFT分析结果。 */
const ADC_FFT_Result_t* ADC_FFT_GetResult(void)
{
    return &s_result;
}

/* 打印FFT分析结果。 */
void ADC_FFT_PrintResult(const ADC_FFT_Result_t *result)
{
    uint8_t count;

    if (result == NULL) {
        result = &s_result;
    }

    count = result->harmonic_count;
    if (result->harmonic_fft_count < count) {
        count = result->harmonic_fft_count;
    }

    printf("Q4: Fs=");
    ADC_FFT_PrintFixed(ADC_FFT_ScaleFloat(FFT_Fs, 100.0f), 2U);
    printf("Hz, F0=");
    ADC_FFT_PrintFixed(ADC_FFT_ScaleFloat(result->fundamental_hz, 100.0f), 2U);
    printf("Hz\r\n");

    for (uint8_t i = 0U; i < count; i++) {
        printf("H%u: ", (unsigned int)(i + 1U));
        ADC_FFT_PrintFixed(ADC_FFT_ScaleFloat(result->harmonic_freq[i], 1000.0f), 3U);
        printf("Hz, Amp=");
        ADC_FFT_PrintFixed(ADC_FFT_ScaleFloat(result->harmonic_amp[i], 1000000.0f), 6U);
        printf("V\r\n");

        printf("F_H%u: ", (unsigned int)(i + 1U));
        ADC_FFT_PrintFixed(ADC_FFT_ScaleFloat(result->harmonic_freq_fft[i], 1000.0f), 3U);
        printf("Hz, Amp=");
        ADC_FFT_PrintFixed(ADC_FFT_ScaleFloat(result->harmonic_amp_fft[i], 1000000.0f), 6U);
        printf("V\r\n");
    }
}

/* 设置片内ADC包装层的最高谐波频率。 */
void ADC_Sample_SetFftMaxHarmonic(float max_harmonic_hz)
{
    ADC_FFT_SetMaxHarmonic(max_harmonic_hz);
}

/* 分析片内ADC最近一帧数据。 */
void ADC_Sample_RunFft(void)
{
    HAL_StatusTypeDef status = ADC_Sample_ConvertReadyData();

    if ((status == HAL_ERROR) ||
        ((status == HAL_BUSY) && (ADC_Sample_VoltageReady() == 0U))) {
        s_result.status = ADC_FFT_STATUS_ERROR;
        return;
    }

    (void)ADC_FFT_Run(ADC_Sample_GetVoltageData(),
                      ADC_Sample_GetSampleCount(),
                      ADC_Sample_GetSampleRateHz());
}

/* 获取片内ADC最近一次FFT结果。 */
const ADC_SampleResult_t* ADC_Sample_GetResult(void)
{
    return ADC_FFT_GetResult();
}

/* 打印片内ADC最近一次FFT结果。 */
void ADC_Sample_PrintResult(void)
{
    ADC_FFT_PrintResult(ADC_FFT_GetResult());
}

/* 在当前频谱中寻找基波。 */
static float FindFundamentalFft(void)
{
    float    df      = FFT_Fs / (float)FFT_length;
    uint32_t k_start = (uint32_t)(50.0f / df + 1.0f);
    uint32_t k_end   = (uint32_t)(1000.0f / df);

    if (k_end >= (FFT_length / 2U)) {
        k_end = (FFT_length / 2U) - 1U;
    }

    if (k_start >= k_end) {
        return 0.0f;
    }

    uint32_t k_max     = k_start;
    float    max_value = FFT_output[k_start];

    for (uint32_t k = k_start + 1U; k <= k_end; k++) {
        if (FFT_output[k] > max_value) {
            max_value = FFT_output[k];
            k_max     = k;
        }
    }

    /* 静音帧的所有 bin 都为 0；不能把搜索下限误报成“基波”。 */
    if (!isfinite(max_value) || max_value <= 1.0e-12f) {
        return 0.0f;
    }

    return (float)k_max * df;
}

/* 清空结果并记录分析失败。 */
static uint8_t ADC_FFT_SetError(void)
{
    const uint32_t frame_count = s_result.frame_count;
    memset(&s_result, 0, sizeof(s_result));
    s_result.frame_count = frame_count;
    s_result.status = ADC_FFT_STATUS_ERROR;
    return s_result.status;
}

/* 将浮点数按指定比例四舍五入为整数。 */
static int32_t ADC_FFT_ScaleFloat(float value, float scale)
{
    if (value >= 0.0f) {
        return (int32_t)(value * scale + 0.5f);
    }

    return (int32_t)(value * scale - 0.5f);
}

/* 打印带指定小数位数的定点数。 */
static void ADC_FFT_PrintFixed(int32_t scaled_value, uint32_t decimals)
{
    uint32_t divisor = 1U;
    int32_t whole;
    int32_t frac;

    for (uint32_t i = 0U; i < decimals; i++) {
        divisor *= 10U;
    }

    whole = scaled_value / (int32_t)divisor;
    frac = scaled_value % (int32_t)divisor;
    if (frac < 0) {
        frac = -frac;
    }

    if ((scaled_value < 0) && (whole == 0)) {
        printf("-0");
    } else {
        printf("%ld", (long)whole);
    }

    if (decimals == 0U) {
        return;
    }

    printf(".");
    for (uint32_t pad = divisor / 10U; pad > 1U; pad /= 10U) {
        if ((uint32_t)frac >= pad) {
            break;
        }
        printf("0");
    }
    printf("%ld", (long)frac);
}
