/**
 * @file    ADC_FFT.c
 * @brief   Generic FFT analysis for ADC sample buffers.
 */

#include "ADC_FFT.h"
#include "adc_app.h"
#include <stddef.h>
#include <stdio.h>

static ADC_FFT_Result_t s_result;
static float s_max_harmonic_hz = 1000.0f;

static float FindFundamentalFft(void);

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
        s_result.status = ADC_FFT_STATUS_ERROR;
        return s_result.status;
    }

    FFT_Fs = sample_rate_hz;
    if (max_harmonic_hz > FFT_Fs * 0.5f) {
        max_harmonic_hz = FFT_Fs * 0.5f;
    }

    FFT_Input((float *)data);
    FFT_Start();

    s_result.fundamental_hz = FindFundamentalFft();

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

    return (float)k_max * df;
}
