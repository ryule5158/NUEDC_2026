#ifndef ADC_APP_H
#define ADC_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * Both ADCs are triggered by the same TIMG12 event. DMA channel 0 stores the
 * voltage samples and DMA channel 1 stores the current samples.
 */
#define ADC_APP_SAMPLE_LENGTH       512U
#define ADC_APP_HALF_LENGTH         (ADC_APP_SAMPLE_LENGTH / 2U)
#define ADC_APP_CHANNEL_COUNT       2U
#define ADC_APP_PRINT_RAW_OFF       0U
#define ADC_APP_PRINT_RAW_ON        1U
#define ADC_APP_MIN_SAMPLE_RATE_HZ  1000.0f
#define ADC_APP_MAX_SAMPLE_RATE_HZ  1000000.0f

typedef enum {
    ADC_APP_CHANNEL_VOLTAGE = 0,
    ADC_APP_CHANNEL_CURRENT = 1,
} ADC_AppChannel_t;

typedef enum {
    ADC_MODE_ONESHOT = 0,
    ADC_MODE_CONTINUOUS = 1,
} ADC_SampleMode_t;

typedef enum {
    ADC_HALF_NONE = 0,
    ADC_HALF_FIRST = 1,
    ADC_HALF_SECOND = 2,
} ADC_HalfReady_t;

HAL_StatusTypeDef ADC_Sample_Init(float vref_v,
                                  float sample_rate_hz,
                                  float max_harmonic_hz,
                                  uint8_t print_raw,
                                  ADC_SampleMode_t mode);
HAL_StatusTypeDef ADC_Sample_Start(void);
void ADC_Sample_Stop(void);
void ADC_Sample_Process(void);

void ADC_Sample_ConfigSync(uint8_t enable);
HAL_StatusTypeDef ADC_Sample_SetActiveChannel(uint8_t channel);
uint8_t ADC_Sample_GetActiveChannel(void);
float ADC_Sample_GetSampleRateHz(void);
uint8_t ADC_Sample_IsRunning(void);

uint8_t ADC_Sample_DataReady(void);
void ADC_Sample_Acknowledge(void);
ADC_HalfReady_t ADC_Sample_GetReadyHalf(void);
uint16_t ADC_Sample_GetReadyOffset(void);
uint16_t ADC_Sample_GetReadyLength(void);
uint32_t ADC_Sample_GetFrameSequence(void);
void ADC_Sample_BufferLock(void);
void ADC_Sample_BufferUnlock(void);
uint8_t ADC_Sample_IsOverrun(void);
uint8_t ADC_Sample_HasError(void);

const uint16_t *ADC_Sample_GetRawData(void);
const uint16_t *ADC_Sample_GetRawDataByChannel(uint8_t channel);
const uint16_t *ADC_Sample_GetReadyRawDataByChannel(uint8_t channel);
const float *ADC_Sample_GetVoltageData(void);
const float *ADC_Sample_GetVoltageDataByChannel(uint8_t channel);
void ADC_Sample_ConvertHalf(ADC_HalfReady_t half);
void ADC_Sample_PrintRaw(void);

#ifdef __cplusplus
}
#endif

#endif /* ADC_APP_H */
