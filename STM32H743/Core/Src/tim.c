#include "tim.h"
#include "pll_selftest_config.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

void MX_TIM3_Init(void)
{
    TIM_ClockConfigTypeDef clock = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 12U - 1U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 1000U - 1U;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) Error_Handler();
    clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &clock) != HAL_OK) Error_Handler();
    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &master) != HAL_OK) {
        Error_Handler();
    }
}

void MX_TIM6_Init(void)
{
#if PLL_SELFTEST_ENABLE
    TIM_MasterConfigTypeDef master = {0};

    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 0U;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = (PLL_TEST_TIMER_CLOCK_HZ / PLL_TEST_SAMPLE_RATE_HZ) - 1U;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();
    master.MasterOutputTrigger = TIM_TRGO_UPDATE;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &master) != HAL_OK) {
        Error_Handler();
    }
#endif
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
#if PLL_SELFTEST_ENABLE
    else if (timer->Instance == TIM6) {
        __HAL_RCC_TIM6_CLK_ENABLE();
    }
#endif
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_DISABLE();
    }
#if PLL_SELFTEST_ENABLE
    else if (timer->Instance == TIM6) {
        __HAL_RCC_TIM6_CLK_DISABLE();
    }
#endif
}
