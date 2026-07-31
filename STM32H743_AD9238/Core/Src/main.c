#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "g_contest_app.h"
#include "g_contest_cli.h"
#include "g_tjc_ui.h"

#include <stdio.h>

void SystemClock_Config(void);

int main(void)
{
  G_TjcUiMode last_display_mode = G_TJC_MODE_WAVE_1T;
  uint8_t has_display_result = 0U;

  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();

  if (GContest_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (GContestCli_Init() != HAL_OK)
  {
    Error_Handler();
  }
  if (G_TjcUi_Init() != HAL_OK)
  {
    Error_Handler();
  }

  while (1)
  {
    G_TjcUiMode display_mode;
    G_TjcUiPage page_event;
    uint8_t waveform_cycles = 1U;
    uint8_t measurement_requested = 0U;

    GContestCli_Poll();
    if (G_TjcUi_TakePageEvent(&page_event) != 0U)
    {
      if (page_event == G_TJC_PAGE_MAIN)
      {
        const PeriodicAnalyzer_Result *cached_result =
            GContest_GetLastResult();

        if ((has_display_result != 0U) &&
            (cached_result != NULL) &&
            (cached_result->valid != 0U) &&
            (cached_result->status == PERIODIC_ANALYZER_OK))
        {
          /*
           * main 控件为页面私有对象；从 index/set 返回时重新发布最近
           * 一帧，用户不会看到恢复成占位符的空页面。
           */
          (void)G_TjcUi_Publish(cached_result, last_display_mode);
        }
        else
        {
          (void)G_TjcUi_ShowIdle();
        }
      }
    }
    if (G_TjcUi_TakeRequest(&display_mode) != 0U)
    {
      waveform_cycles =
          (display_mode == G_TJC_MODE_WAVE_3T) ? 3U : 1U;
      measurement_requested = 1U;
    }
    else if (GContestCli_TakeMeasurementRequest(&waveform_cycles) != 0U)
    {
      display_mode = (waveform_cycles == 3U) ?
                     G_TJC_MODE_WAVE_3T :
                     G_TJC_MODE_WAVE_1T;
      measurement_requested = 1U;
    }

    if (measurement_requested != 0U)
    {
      const uint32_t previous_frame = GContest_GetFrameNumber();
      const PeriodicAnalyzer_Result *result;

      (void)G_TjcUi_ShowAcquiring(display_mode);
      GContest_Process(waveform_cycles);
      result = GContest_GetLastResult();
      if ((GContest_GetFrameNumber() != previous_frame) &&
          (result != NULL) &&
          (result->valid != 0U) &&
          (result->status == PERIODIC_ANALYZER_OK))
      {
        HAL_StatusTypeDef display_status;
        const uint32_t display_start_ms = HAL_GetTick();

        /*
         * 先完成坐标轴、曲线和数值的屏幕发布，再格式化并排队完整
         * 串口报告。USART1 的线速不再位于屏幕响应关键路径。
         */
        display_status = G_TjcUi_Publish(result, display_mode);
        last_display_mode = display_mode;
        has_display_result = 1U;
        printf("[UI] mode=%u publish=%lums status=%d\r\n",
               (unsigned)display_mode,
               (unsigned long)(HAL_GetTick() - display_start_ms),
               (int)display_status);
        GContest_PrintLastReport(waveform_cycles);
      }
      else if ((result != NULL) &&
               (result->status == PERIODIC_ANALYZER_E_NO_SIGNAL))
      {
        (void)G_TjcUi_ShowError("NO SIGNAL");
      }
      else
      {
        (void)G_TjcUi_ShowError("MEASURE ERROR");
      }
    }
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
