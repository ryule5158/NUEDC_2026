#include "ti_msp_dl_config.h"
#include "adc_app.h"
#include "ad9910_app.h"
#include <stdio.h>

#define ADC_TEST_SAMPLE_RATE_HZ    100000.0f
#define ADC_TEST_MAX_INPUT_HZ       10000.0f
#define ADC_TEST_FRAME_DELAY_MS       300U

static void ADC_TestPrintFrame(uint32_t frame)
{
    const uint16_t *voltage_raw;
    const uint16_t *current_raw;
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

    for (index = 0U; index < ADC_APP_SAMPLE_LENGTH; index++) {
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

    printf("#FRAME=%lu FS=100000 N=%u "
           "PB25_MIN=%u PB25_MAX=%u PB25_MEAN=%lu "
           "PB17_MIN=%u PB17_MAX=%u PB17_MEAN=%lu\r\n",
           (unsigned long)frame,
           (unsigned int)ADC_APP_SAMPLE_LENGTH,
           (unsigned int)voltage_min,
           (unsigned int)voltage_max,
           (unsigned long)(voltage_sum / ADC_APP_SAMPLE_LENGTH),
           (unsigned int)current_min,
           (unsigned int)current_max,
           (unsigned long)(current_sum / ADC_APP_SAMPLE_LENGTH));
    printf("#DATA=PB25,PB17\r\n");

    for (index = 0U; index < ADC_APP_SAMPLE_LENGTH; index++) {
        printf("%u,%u\r\n",
               (unsigned int)voltage_raw[index],
               (unsigned int)current_raw[index]);
    }

  printf("#END\r\n");
}

int main(void)
{
    uint32_t frame = 0U;

    SYSCFG_DL_init();
    NUEDC_HAL_Init();
	AD9910_AppInit();
	AD9910_AppOutputSine(1000, 16383);

    printf("\r\nMSPM0G3507 ADC SIMPLE TEST\r\n");
    printf("UART0 PA10/PA11 115200-8-N-1\r\n");
    printf("PB25=ADC0_CH4, PB17=ADC1_CH4, RANGE=0..3.3V\r\n");
    printf("FORMAT: PB25_RAW,PB17_RAW; 0..4095\r\n");

    if (ADC_Sample_Init(3.3f,
                        ADC_TEST_SAMPLE_RATE_HZ,
                        ADC_TEST_MAX_INPUT_HZ,
                        ADC_APP_PRINT_RAW_OFF,
                        ADC_MODE_ONESHOT) != HAL_OK) {
        printf("#ERROR=ADC_INIT\r\n");
        while (1) {
            __WFI();
        }
    }

    while (1) {
        if (ADC_Sample_Start() != HAL_OK) {
            printf("#ERROR=ADC_START\r\n");
            HAL_Delay(ADC_TEST_FRAME_DELAY_MS);
            continue;
        }

        while (ADC_Sample_DataReady() == 0U) {
            if (ADC_Sample_HasError() != 0U) {
                break;
            }
            __WFI();
        }

        if (ADC_Sample_HasError() != 0U) {
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
