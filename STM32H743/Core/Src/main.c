#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "g_contest_config.h"
#include "g_contest_app.h"
#include "fpga_boot_restore.h"
#include "pll_selftest_config.h"
#include "pll_test_app.h"
#include <string.h>

#if !G_CONTEST_ENABLE
static FpgaBootRestore_Config s_fpga_boot_config;

static void App_FpgaBootLog(void *user, const char *message)
{
  size_t length;

  (void)user;
  if (message == NULL) return;
  length = strlen(message);
  if (length > 0xFFFFU) length = 0xFFFFU;
  (void)HAL_UART_Transmit(&huart1,
                          (const uint8_t *)message,
                          (uint16_t)length,
                          30U);
}
#endif

void SystemClock_Config(void);

int main(void)
{
#if PLL_SELFTEST_ENABLE || G_CONTEST_ENABLE
  SCB_EnableICache();
  SCB_EnableDCache();
#endif

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
#if G_CONTEST_ENABLE
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  if (GContest_Init() != HAL_OK) Error_Handler();
#else
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
#if PLL_SELFTEST_ENABLE
  MX_ADC2_Init();
#endif
  MX_ADC1_Init();
  MX_TIM3_Init();
#if PLL_SELFTEST_ENABLE
  MX_TIM6_Init();
#endif

  FpgaBootRestore_GetDefaultConfig(&s_fpga_boot_config);
  s_fpga_boot_config.log = App_FpgaBootLog;
  if (FpgaBootRestore_Start(&s_fpga_boot_config) != FPGA_PROMAX_OK)
  {
    Error_Handler();
  }
  if (FpgaBootRestore_WaitReady(
        0U, FPGA_BOOT_POST_READY_GUARD_MS) != FPGA_PROMAX_OK)
  {
    Error_Handler();
  }
#if PLL_SELFTEST_ENABLE
  if (PllTest_Init() != HAL_OK) Error_Handler();
#endif
#endif

  while (1)
  {
#if G_CONTEST_ENABLE
    GContest_Process();
#else
    FpgaBootRestore_Process();
#if PLL_SELFTEST_ENABLE
    PllTest_Process();
#endif
#endif
  }
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  出现不可恢复错误时停止运行。
  * @retval 无。
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  记录参数断言失败的源文件与行号。
  * @param  file 源文件名指针。
  * @param  line 源文件行号。
  * @retval 无。
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
