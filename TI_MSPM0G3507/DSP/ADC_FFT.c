/**
 * @file    ADC_FFT.c
 * @brief   Generic FFT analysis for ADC sample buffers.
 */

#include "ADC_FFT.h"
#include "adc_app.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

static ADC_FFT_Result_t s_result; /* 最近一次FFT分析结果。 */
static float s_max_harmonic_hz = 500000.0f; /* 默认搜索至片上ADC最高奈奎斯特频率。 */

/* 在指定频率上限内搜索并插值基频。 */
static float FindFundamentalFft(float max_frequency_hz);

/* 清空本帧分析数据，同时保留累计帧数。 */
static void ClearResult(uint8_t status)
{
    uint32_t frame_count = s_result.frame_count;

    memset(&s_result, 0, sizeof(s_result));
    s_result.frame_count = frame_count;
    s_result.status = status;
}

void ADC_FFT_SetMaxHarmonic(float max_harmonic_hz)
{
    if (max_harmonic_hz > 0.0f) {
        s_max_harmonic_hz = max_harmonic_hz;
    }
}

uint8_t ADC_FFT_Run(const float *data, uint32_t len, float sample_rate_hz)
{
    return ADC_FFT_RunWithMaxHarmonic(data, len, sample_rate_hz, s_max_harmonic_hz);
}

uint8_t ADC_FFT_RunWithMaxHarmonic(const float *data,
                                   uint32_t len,
                                   float sample_rate_hz,
                                   float max_harmonic_hz)
{
    if (data == NULL || len != ADC_FFT_SAMPLE_LENGTH ||
        sample_rate_hz <= 0.0f || max_harmonic_hz <= 0.0f) {
        ClearResult(ADC_FFT_STATUS_ERROR);
        return s_result.status;
    }

    FFT_Fs = sample_rate_hz;
    if (max_harmonic_hz > FFT_Fs * 0.5f) {
        max_harmonic_hz = FFT_Fs * 0.5f;
    }

    FFT_Input(data);
    FFT_Start();

    s_result.fundamental_hz = FindFundamentalFft(max_harmonic_hz);
    if (s_result.fundamental_hz <= 0.0f) {
        ClearResult(ADC_FFT_STATUS_ERROR);
        return s_result.status;
    }

    s_result.harmonic_count = FFT_GetHarmonics(data,
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

    if ((s_result.harmonic_count == 0U) ||
        (s_result.harmonic_fft_count == 0U)) {
        ClearResult(ADC_FFT_STATUS_ERROR);
        return s_result.status;
    }

    s_result.status = ADC_FFT_STATUS_OK;
    s_result.frame_count++;
    return s_result.status;
}

const ADC_FFT_Result_t* ADC_FFT_GetResult(void)
{
    return &s_result;
}

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

    printf("Q4: Fs=%.2fHz, F0=%.2fHz\r\n",
           (double)FFT_Fs,
           (double)result->fundamental_hz);

    for (uint8_t i = 0U; i < count; i++) {
        printf("H%u: %.8fHz, Amp=%.8fV\r\n",
               (unsigned int)(i + 1U),
               (double)result->harmonic_freq[i],
               (double)result->harmonic_amp[i]);
        printf("F_H%u: %.8fHz, Amp=%.8fV\r\n",
               (unsigned int)(i + 1U),
               (double)result->harmonic_freq_fft[i],
               (double)result->harmonic_amp_fft[i]);
    }
}

void ADC_Sample_SetFftMaxHarmonic(float max_harmonic_hz)
{
    ADC_FFT_SetMaxHarmonic(max_harmonic_hz);
}

void ADC_Sample_RunFft(void)
{
    (void)ADC_FFT_Run(ADC_Sample_GetVoltageData(),
                      ADC_Sample_GetSampleCount(),
                      ADC_Sample_GetSampleRateHz());
}

const ADC_SampleResult_t* ADC_Sample_GetResult(void)
{
    return ADC_FFT_GetResult();
}

void ADC_Sample_PrintResult(void)
{
    ADC_FFT_PrintResult(ADC_FFT_GetResult());
}

/* 跳过直流分量，在有效频带内搜索最大谱峰并作抛物线插值。 */
static float FindFundamentalFft(float max_frequency_hz)
{
    float df = FFT_Fs / (float)FFT_length;
    uint32_t k_start = 1U;
    uint32_t k_end = (uint32_t)(max_frequency_hz / df);
    uint32_t k_max;
    float max_value;

    if (k_end >= (FFT_length / 2U)) {
        k_end = (FFT_length / 2U) - 1U;
    }

    if (k_start > k_end) {
        return 0.0f;
    }

    k_max = k_start;
    max_value = FFT_output[k_start];

    for (uint32_t k = k_start + 1U; k <= k_end; k++) {
        if (FFT_output[k] > max_value) {
            max_value = FFT_output[k];
            k_max     = k;
        }
    }

    if (max_value <= 0.0f) {
        return 0.0f;
    }

    if ((k_max > 0U) && ((k_max + 1U) < (FFT_length / 2U))) {
        float offset = FFT_InterpolateHannPeak(k_max, NULL);
        float frequency_hz = ((float)k_max + offset) * df;

        return (frequency_hz < max_frequency_hz) ?
               frequency_hz : max_frequency_hz;
    }

    return (float)k_max * df;
}
