#include "ti_msp_dl_config.h"
#include "adc_app.h"
#include "ad9910_app.h"
#include <stdio.h>

#define ADC_TEST_SAMPLE_RATE_HZ     100000.0f /* 每通道测试采样率。 */
#define ADC_TEST_SAMPLE_COUNT           512U /* 每通道测试采样点数。 */
#define ADC_TEST_FRAME_DELAY_MS         300U /* 两帧打印间隔。 */
#define ADC_TEST_TIMEOUT_MARGIN_MS       10U /* DMA采集超时余量。 */

/* 打印一帧双通道ADC统计值和原始码。 */
static void ADC_TestPrintFrame(uint32_t frame)
{
    const uint16_t *voltage_raw;
    const uint16_t *current_raw;
    uint16_t sample_count = ADC_Sample_GetSampleCount();
    uint32_t voltage_sum = 0U;
    uint32_t current_sum = 0U;
    uint16_t voltage_min = 4095U;
    uint16_t voltage_max = 0U;
    uint16_t current_min = 4095U;
    uint16_t current_max = 0U;
    uint32_t index;

    voltage_raw =
        ADC_Sample_GetRawDataByChannel(ADC_APP_CHANNEL_VOLTAGE);
    current_raw =
        ADC_Sample_GetRawDataByChannel(ADC_APP_CHANNEL_CURRENT);

    for (index = 0U; index < sample_count; index++) {
        uint16_t voltage = voltage_raw[index];
        uint16_t current = current_raw[index];

        voltage_sum += voltage;
        current_sum += current;

        if (voltage < voltage_min) {
            voltage_min = voltage;
        }
        if (voltage > voltage_max) {
            voltage_max = voltage;
        }
        if (current < current_min) {
            current_min = current;
        }
        if (current > current_max) {
            current_max = current;
        }
    }

    printf("#FRAME=%lu FS=%lu N=%u "
           "PB25_MIN=%u PB25_MAX=%u PB25_MEAN=%lu "
           "PB17_MIN=%u PB17_MAX=%u PB17_MEAN=%lu\r\n",
           (unsigned long)frame,
           (unsigned long)(ADC_Sample_GetSampleRateHz() + 0.5f),
           (unsigned int)sample_count,
           (unsigned int)voltage_min,
           (unsigned int)voltage_max,
           (unsigned long)(voltage_sum / sample_count),
           (unsigned int)current_min,
           (unsigned int)current_max,
           (unsigned long)(current_sum / sample_count));
    printf("#DATA=PB25,PB17\r\n");

    for (index = 0U; index < sample_count; index++) {
        printf("%u,%u\r\n",
               (unsigned int)voltage_raw[index],
               (unsigned int)current_raw[index]);
    }

    printf("#END\r\n");
}

/* 初始化AD9910与双ADC，并持续输出采样帧。 */
int main(void)
{
    ADC_SampleConfig_t adc_config;
    uint32_t frame = 0U;
    uint32_t capture_start_ms;
    uint32_t capture_timeout_ms;
    uint8_t capture_timeout;

    SYSCFG_DL_init();
    NUEDC_HAL_Init();
    AD9910_AppInit();
    AD9910_AppOutputSine(1000, 16383);

    printf("\r\nMSPM0G3507 ADC SIMPLE TEST\r\n");
    printf("UART0 PA10/PA11 115200-8-N-1\r\n");
    printf("PB25=ADC0_CH4, PB17=ADC1_CH4, RANGE=0..3.3V\r\n");
    printf("FORMAT: PB25_RAW,PB17_RAW; 0..4095\r\n");

    ADC_Sample_GetDefaultConfig(&adc_config);
    adc_config.sample_rate_hz = ADC_TEST_SAMPLE_RATE_HZ;
    adc_config.sample_count = ADC_TEST_SAMPLE_COUNT;
    adc_config.mode = ADC_MODE_ONESHOT;
    adc_config.quality = ADC_QUALITY_RAW;
    if (ADC_Sample_Init(&adc_config) != HAL_OK) {
        printf("#ERROR=ADC_INIT\r\n");
        while (1) {
            __WFI();
        }
    }
    capture_timeout_ms =
        (uint32_t)(((float)ADC_Sample_GetSampleCount() * 1000.0f) /
                   ADC_Sample_GetSampleRateHz()) +
        ADC_TEST_TIMEOUT_MARGIN_MS;

    while (1) {
        if (ADC_Sample_Start() != HAL_OK) {
            printf("#ERROR=ADC_START\r\n");
            HAL_Delay(ADC_TEST_FRAME_DELAY_MS);
            continue;
        }

        capture_start_ms = HAL_GetTick();
        capture_timeout = 0U;
        while (ADC_Sample_DataReady() == 0U) {
            if (ADC_Sample_HasError() != 0U) {
                break;
            }
            if ((HAL_GetTick() - capture_start_ms) >= capture_timeout_ms) {
                capture_timeout = 1U;
                break;
            }
            __WFI();
        }

        if (capture_timeout != 0U) {
            printf("#ERROR=ADC_TIMEOUT DMA0_LEFT=%u DMA1_LEFT=%u\r\n",
                   (unsigned int)DL_DMA_getTransferSize(
                       DMA, DMA_ADC_VOLTAGE_CHAN_ID),
                   (unsigned int)DL_DMA_getTransferSize(
                       DMA, DMA_ADC_CURRENT_CHAN_ID));
            ADC_Sample_Stop();
        } else if (ADC_Sample_HasError() != 0U) {
            printf("#ERROR=ADC_DMA\r\n");
            ADC_Sample_Stop();
        } else {
            frame++;
            ADC_TestPrintFrame(frame);
        }

        ADC_Sample_Acknowledge();
        HAL_Delay(ADC_TEST_FRAME_DELAY_MS);
    }
}
