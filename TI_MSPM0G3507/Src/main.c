#include "ti_msp_dl_config.h"
#include "adc_app.h"
#include "ad9910_app.h"
#include <stdio.h>

#define ADC_TEST_SAMPLE_RATE_HZ     1000000.0f
#define ADC_TEST_SAMPLE_COUNT           512U

int main(void)
{
    ADC_SampleConfig_t adc_config;

    SYSCFG_DL_init();
    NUEDC_HAL_Init();

    ADC_Sample_GetDefaultConfig(&adc_config);
    adc_config.sample_rate_hz = ADC_TEST_SAMPLE_RATE_HZ;
    adc_config.sample_count = ADC_TEST_SAMPLE_COUNT;
    adc_config.mode = ADC_MODE_ONESHOT;
    adc_config.quality = ADC_QUALITY_MEDIAN3;
    if (ADC_Sample_Init(&adc_config) != HAL_OK) {
        while (1) {
            __WFI();
        }
    }

    if (ADC_Sample_Start() != HAL_OK) {
        while (1) {
            __WFI();
        }
    }

    ADC_Sample_PrintVoltages();
    ADC_Sample_PrintFrequency();

    while (1) {
        __WFI();
    }
}
