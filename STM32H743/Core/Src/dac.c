#include "dac.h"
#include "pll_selftest_config.h"

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

void MX_DAC1_Init(void)
{
#if PLL_SELFTEST_ENABLE
    DAC_ChannelConfTypeDef config = {0};

    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK) Error_Handler();

    config.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    config.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
    config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    config.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    config.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &config, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
#endif
}

void HAL_DAC_MspInit(DAC_HandleTypeDef *dac_handle)
{
#if PLL_SELFTEST_ENABLE
    GPIO_InitTypeDef gpio = {0};

    if ((dac_handle == NULL) || (dac_handle->Instance != DAC1)) return;
    __HAL_RCC_DAC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA4 is DAC1_OUT1 in this temporary PLL hardware test. */
    gpio.Pin = GPIO_PIN_4;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    hdma_dac1_ch1.Instance = DMA1_Stream1;
    hdma_dac1_ch1.Init.Request = DMA_REQUEST_DAC1_CH1;
    hdma_dac1_ch1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_dac1_ch1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_dac1_ch1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_dac1_ch1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_dac1_ch1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_dac1_ch1.Init.Mode = DMA_CIRCULAR;
    hdma_dac1_ch1.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_dac1_ch1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_dac1_ch1) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(dac_handle, DMA_Handle1, hdma_dac1_ch1);
#endif
}

void HAL_DAC_MspDeInit(DAC_HandleTypeDef *dac_handle)
{
#if PLL_SELFTEST_ENABLE
    if ((dac_handle == NULL) || (dac_handle->Instance != DAC1)) return;
    __HAL_RCC_DAC12_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
    HAL_DMA_DeInit(dac_handle->DMA_Handle1);
#endif
}
