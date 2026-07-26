#include "adc.h"
#include "pll_selftest_config.h"

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
static uint32_t s_adc12_clock_users;

static void ADC_CommonInit(ADC_HandleTypeDef *hadc)
{
    hadc->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
    hadc->Init.Resolution = ADC_RESOLUTION_16B;
    hadc->Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait = DISABLE;
    hadc->Init.ContinuousConvMode = DISABLE;
    hadc->Init.NbrOfConversion = 1;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    hadc->Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc->Init.OversamplingMode = DISABLE;
    hadc->Init.Oversampling.Ratio = 1;
}

void MX_ADC2_Init(void)
{
#if PLL_SELFTEST_ENABLE
    ADC_ChannelConfTypeDef config = {0};

    hadc2.Instance = ADC2;
    ADC_CommonInit(&hadc2);
    /* The slave is started by ADC1 multimode; its own trigger is ignored. */
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) Error_Handler();

    config.Channel = ADC_CHANNEL_8;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset = 0;
    config.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc2, &config) != HAL_OK) Error_Handler();
#endif
}

void MX_ADC1_Init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef config = {0};

    hadc1.Instance = ADC1;
#if PLL_SELFTEST_ENABLE
    ADC_CommonInit(&hadc1);
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    multimode.Mode = ADC_DUALMODE_REGSIMULT;
    multimode.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS;
    multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
        Error_Handler();
    }

    config.Channel = ADC_CHANNEL_4;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset = 0;
    config.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK) Error_Handler();
#else
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
    hadc1.Init.Resolution = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 2;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_ONESHOT;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Init.Oversampling.Ratio = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) Error_Handler();

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
        Error_Handler();
    }
    config.Channel = ADC_CHANNEL_4;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset = 0;
    config.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK) Error_Handler();
    config.Channel = ADC_CHANNEL_8;
    config.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK) Error_Handler();
#endif
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adc_handle)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};

    if ((adc_handle->Instance != ADC1) && (adc_handle->Instance != ADC2)) return;

    if (s_adc12_clock_users == 0U) {
        clock.PeriphClockSelection = RCC_PERIPHCLK_ADC;
        clock.PLL2.PLL2M = 2;
        clock.PLL2.PLL2N = 12;
        clock.PLL2.PLL2P = 2;
        clock.PLL2.PLL2Q = 2;
        clock.PLL2.PLL2R = 2;
        clock.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
        clock.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
        clock.PLL2.PLL2FRACN = 0;
        clock.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
        if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) Error_Handler();
        __HAL_RCC_ADC12_CLK_ENABLE();
    }
    s_adc12_clock_users++;
    __HAL_RCC_GPIOC_CLK_ENABLE();

#if PLL_SELFTEST_ENABLE
    gpio.Pin = (adc_handle->Instance == ADC1) ? GPIO_PIN_4 : GPIO_PIN_5;
#else
    if (adc_handle->Instance != ADC1) return;
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
#endif
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

    if (adc_handle->Instance == ADC1) {
        hdma_adc1.Instance = DMA1_Stream0;
        hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
#if PLL_SELFTEST_ENABLE
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
#else
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode = DMA_NORMAL;
        hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
#endif
        hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) Error_Handler();
        __HAL_LINKDMA(adc_handle, DMA_Handle, hdma_adc1);
    }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *adc_handle)
{
    if ((adc_handle->Instance != ADC1) && (adc_handle->Instance != ADC2)) return;

    if (adc_handle->Instance == ADC1) {
#if PLL_SELFTEST_ENABLE
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);
#else
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4 | GPIO_PIN_5);
#endif
        HAL_DMA_DeInit(adc_handle->DMA_Handle);
    } else {
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_5);
    }

    if (s_adc12_clock_users > 0U) s_adc12_clock_users--;
    if (s_adc12_clock_users == 0U) __HAL_RCC_ADC12_CLK_DISABLE();
}
