#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "g_contest_app.h"
#include "g_contest_cli.h"
#include "g_tjc_ui.h"

#include <stdio.h>
#include <string.h>

#define G_BACKEND_RETRY_MS   1000U
#define G_INTERFACE_RETRY_MS 1000U
#define G_POWER_READY_TIMEOUT_MS 100U

HAL_StatusTypeDef SystemClock_Config(void);

static uint8_t Main_TimeReached(uint32_t now, uint32_t deadline)
{
  return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static void Main_DisableUi(uint8_t *ui_ready,
                           uint8_t *ui_auto_retry,
                           uint32_t *ui_retry_deadline)
{
  if ((ui_ready == NULL) ||
      (ui_auto_retry == NULL) ||
      (ui_retry_deadline == NULL))
  {
    return;
  }

  *ui_ready = 0U;
  if (G_TjcUi_CanReinitialize() != 0U)
  {
    *ui_auto_retry = 1U;
    *ui_retry_deadline = HAL_GetTick() + G_INTERFACE_RETRY_MS;
    printf("[TJC] link error; safe background recovery enabled\r\n");
  }
  else
  {
    *ui_auto_retry = 0U;
    printf("[TJC] transparent-transfer state is uncertain; "
           "automatic ASCII recovery disabled; reset the screen/board\r\n");
  }
}

int main(void)
{
  uint8_t cli_ready = 0U;
  uint8_t ui_ready = 0U;
  uint8_t ui_auto_retry = 1U;
  uint8_t clock_degraded = 0U;
  uint8_t clock_fallback_failed = 0U;
  uint32_t cli_retry_deadline = 0U;
  uint32_t ui_retry_deadline = 0U;
  uint32_t backend_retry_deadline = 0U;

  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  if (SystemClock_Config() != HAL_OK)
  {
    clock_degraded = 1U;
    /*
     * Return to the reset HSI clock instead of entering a silent pre-UART
     * fatal loop.  HAL_RCC_DeInit also updates SysTick for HSI.
     */
    if (HAL_RCC_DeInit() != HAL_OK)
    {
      clock_fallback_failed = 1U;
    }
  }
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk |
                SCB_SHCSR_BUSFAULTENA_Msk |
                SCB_SHCSR_USGFAULTENA_Msk;

  MX_GPIO_Init();

  /*
   * Command RX must be armed before the FPGA is probed.  An FPGA boot delay,
   * wiring fault, or protocol mismatch must not make the whole unit silent.
   */
  if (MX_USART1_UART_Init() == HAL_OK)
  {
    printf("\r\n[BOOT] USART1 diagnostic ready, 115200 8N1\r\n");
    if (clock_degraded != 0U)
    {
      printf("[BOOT] primary HSE/PLL clock failed; HSI fallback=%s "
             "SystemCoreClock=%luHz\r\n",
             (clock_fallback_failed == 0U) ? "ACTIVE" : "FAILED",
             (unsigned long)SystemCoreClock);
    }
    if (GContestCli_Init() == HAL_OK)
    {
      cli_ready = 1U;
      printf("[BOOT] USART1 command parser ready\r\n");
    }
    else
    {
      cli_retry_deadline = HAL_GetTick() + G_INTERFACE_RETRY_MS;
      printf("[BOOT] USART1 command parser unavailable; "
             "background retry enabled\r\n");
    }
  }
  else
  {
    cli_retry_deadline = HAL_GetTick() + G_INTERFACE_RETRY_MS;
  }

  if (MX_USART2_UART_Init() == HAL_OK)
  {
    printf("[BOOT] USART2 screen link initialized\r\n");
    if (G_TjcUi_Init() == HAL_OK)
    {
      ui_ready = 1U;
      printf("[BOOT] serial-screen protocol ready\r\n");
    }
    else
    {
      ui_retry_deadline = HAL_GetTick() + G_INTERFACE_RETRY_MS;
      printf("[BOOT] serial-screen protocol unavailable; "
             "background retry enabled\r\n");
    }
  }
  else
  {
    ui_retry_deadline = HAL_GetTick() + G_INTERFACE_RETRY_MS;
    printf("[BOOT] USART2 screen hardware unavailable; "
           "background retry enabled\r\n");
  }

  printf("[BOOT] initializing SPI2 FPGA link\r\n");
  if (MX_SPI2_Init() == HAL_OK)
  {
    printf("[BOOT] SPI2 initialized\r\n");
    printf("[BOOT] initializing G measurement backend\r\n");
    if (GContest_Init() == HAL_OK)
    {
      printf("[BOOT] G measurement backend ready\r\n");
    }
    else
    {
      backend_retry_deadline = HAL_GetTick() + G_BACKEND_RETRY_MS;
      printf("[BOOT] G backend is not ready; UI/CLI remain alive and "
             "background retry is enabled\r\n");
    }
  }
  else
  {
    backend_retry_deadline = HAL_GetTick() + G_BACKEND_RETRY_MS;
    printf("[BOOT] SPI2 hardware is not ready; UI/CLI remain alive and "
           "background retry is enabled\r\n");
  }
  if (GContest_IsReady() == 0U)
  {
    if ((ui_ready != 0U) &&
        (G_TjcUi_ShowError("FPGA WAIT") != HAL_OK))
    {
      Main_DisableUi(&ui_ready,
                     &ui_auto_retry,
                     &ui_retry_deadline);
    }
  }
  printf("[BOOT] startup complete; entering main loop\r\n");

  while (1)
  {
    G_TjcUiMode display_mode;
    uint8_t waveform_cycles = 1U;
    uint8_t measurement_requested = 0U;
    uint32_t now = HAL_GetTick();

    if ((cli_ready == 0U) &&
        (Main_TimeReached(now, cli_retry_deadline) != 0U))
    {
      if (MX_USART1_UART_Init() == HAL_OK)
      {
        printf("\r\n[BOOT] USART1 diagnostic hardware recovered\r\n");
        if (clock_degraded != 0U)
        {
          printf("[BOOT] primary HSE/PLL clock failed; HSI fallback=%s "
                 "SystemCoreClock=%luHz\r\n",
                 (clock_fallback_failed == 0U) ? "ACTIVE" : "FAILED",
                 (unsigned long)SystemCoreClock);
        }
        if (GContestCli_Init() == HAL_OK)
        {
          cli_ready = 1U;
          printf("[BOOT] USART1 command parser recovered\r\n");
        }
        else
        {
          cli_retry_deadline = HAL_GetTick() + G_INTERFACE_RETRY_MS;
        }
      }
      else
      {
        cli_retry_deadline = HAL_GetTick() + G_INTERFACE_RETRY_MS;
      }
    }
    if (cli_ready != 0U)
    {
      GContestCli_Poll();
    }

    if ((ui_ready != 0U) && (G_TjcUi_IsHealthy() == 0U))
    {
      Main_DisableUi(&ui_ready,
                     &ui_auto_retry,
                     &ui_retry_deadline);
    }

    if ((ui_ready == 0U) &&
        (ui_auto_retry != 0U) &&
        (Main_TimeReached(now, ui_retry_deadline) != 0U))
    {
      if ((MX_USART2_UART_Init() == HAL_OK) &&
          (G_TjcUi_Init() == HAL_OK))
      {
        ui_ready = 1U;
        ui_auto_retry = 1U;
        printf("[BOOT] serial-screen protocol recovered\r\n");
        if ((GContest_IsReady() == 0U) &&
            (G_TjcUi_ShowError("FPGA WAIT") != HAL_OK))
        {
          Main_DisableUi(&ui_ready,
                         &ui_auto_retry,
                         &ui_retry_deadline);
        }
      }
      else
      {
        if (G_TjcUi_CanReinitialize() != 0U)
        {
          ui_retry_deadline = HAL_GetTick() + G_INTERFACE_RETRY_MS;
        }
        else
        {
          ui_auto_retry = 0U;
          printf("[TJC] unsafe transparent-transfer state; "
                 "automatic recovery stopped\r\n");
        }
      }
    }

    if ((GContest_IsReady() == 0U) &&
        (Main_TimeReached(now, backend_retry_deadline) != 0U))
    {
      printf("[BOOT] retrying G measurement backend\r\n");
      if (MX_SPI2_Init() != HAL_OK)
      {
        printf("[BOOT] SPI2 hardware retry failed\r\n");
        backend_retry_deadline = HAL_GetTick() + G_BACKEND_RETRY_MS;
      }
      else if (GContest_Init() == HAL_OK)
      {
        printf("[BOOT] G measurement backend recovered\r\n");
        if ((ui_ready != 0U) &&
            (G_TjcUi_ShowError("READY") != HAL_OK))
        {
          Main_DisableUi(&ui_ready,
                         &ui_auto_retry,
                         &ui_retry_deadline);
        }
      }
      else
      {
        backend_retry_deadline = HAL_GetTick() + G_BACKEND_RETRY_MS;
      }
    }

    if ((cli_ready != 0U) &&
        (GContestCli_TakeCalibrationRequest() != 0U))
    {
      if (GContest_IsReady() != 0U)
      {
        (void)GContest_CaptureCalibrationRaw();
      }
      else
      {
        printf("[CAL] request rejected: FPGA/ADC backend is not ready\r\n");
      }
      continue;
    }
    if ((cli_ready != 0U) &&
        (GContestCli_IsCalibrationMode() != 0U))
    {
      /* Calibration owns the ADC; discard any queued screen measurement. */
      if (ui_ready != 0U)
      {
        (void)G_TjcUi_TakeRequest(&display_mode);
      }
      continue;
    }

    /* The serial screen is the default UI; USART1 remains available too. */
    if ((ui_ready != 0U) &&
        (G_TjcUi_TakeRequest(&display_mode) != 0U))
    {
      waveform_cycles =
          (display_mode == G_TJC_MODE_WAVE_3T) ? 3U : 1U;
      measurement_requested = 1U;
    }
    else if ((cli_ready != 0U) &&
             (GContestCli_TakeMeasurementRequest(&waveform_cycles) != 0U))
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
      HAL_StatusTypeDef ui_status = HAL_OK;

      if (GContest_IsReady() == 0U)
      {
        printf("[G] request rejected: FPGA/ADC backend is not ready; "
               "automatic retry continues\r\n");
        if ((ui_ready != 0U) &&
            (G_TjcUi_ShowError("FPGA WAIT") != HAL_OK))
        {
          Main_DisableUi(&ui_ready,
                         &ui_auto_retry,
                         &ui_retry_deadline);
        }
        continue;
      }

      if (ui_ready != 0U)
      {
        ui_status = G_TjcUi_ShowAcquiring(display_mode);
      }
      GContest_Process(waveform_cycles);
      result = GContest_GetLastResult();

      if (ui_ready != 0U)
      {
        if (ui_status == HAL_OK)
        {
          if ((GContest_GetFrameNumber() != previous_frame) &&
              (result != NULL) &&
              (result->valid != 0U) &&
              (result->status == PERIODIC_ANALYZER_OK))
          {
            ui_status = G_TjcUi_Publish(result, display_mode);
          }
          else if ((result != NULL) &&
                   (result->status == PERIODIC_ANALYZER_E_NO_SIGNAL))
          {
            ui_status = G_TjcUi_ShowError("NO SIGNAL");
          }
          else if (GContest_IsReady() == 0U)
          {
            ui_status = G_TjcUi_ShowError("FPGA WAIT");
          }
          else
          {
            ui_status = G_TjcUi_ShowError("MEASURE ERROR");
          }
        }
        if (ui_status != HAL_OK)
        {
          Main_DisableUi(&ui_ready,
                         &ui_auto_retry,
                         &ui_retry_deadline);
        }
      }
      if (GContest_IsReady() == 0U)
      {
        backend_retry_deadline = HAL_GetTick();
      }
    }
  }
}

HAL_StatusTypeDef SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  uint32_t start_tick;

  /** Supply configuration update enable */
  if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /*
   * HAL_PWREx_ConfigSupply() can return immediately when the supply selection
   * is already locked.  Verify the actual regulator-ready flag explicitly.
   */
  start_tick = HAL_GetTick();
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_ACTVOSRDY))
  {
    if ((HAL_GetTick() - start_tick) >= G_POWER_READY_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  /** Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  start_tick = HAL_GetTick();
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY))
  {
    if ((HAL_GetTick() - start_tick) >= G_POWER_READY_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  /** Initialize the RCC oscillators. */
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
    return HAL_ERROR;
  }

  /** Initialize the CPU, AHB and APB clocks. */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_D3PCLK1 |
                                RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    return HAL_ERROR;
  }
  return HAL_OK;
}

void Error_HandlerContext(const char *file,
                          uint32_t line,
                          const char *reason)
{
  static volatile uint32_t fatal_sequence;
  char message[224];

  if (file == NULL)
  {
    file = "unknown";
  }
  if (reason == NULL)
  {
    reason = "unspecified";
  }

  /*
   * The low-level writer is bounded and does not use HAL_GetTick(), IRQs, or
   * the HAL UART state machine.  Fatal diagnostics therefore cannot wait
   * forever on a broken UART.
   */
  while (1)
  {
    int length;

    fatal_sequence++;
    length = snprintf(message,
                      sizeof(message),
                      "[FATAL] seq=%lu reason=%s file=%s line=%lu "
                      "tick=%lu; firmware halted\r\n",
                      (unsigned long)fatal_sequence,
                      reason,
                      file,
                      (unsigned long)line,
                      (unsigned long)HAL_GetTick());

    if (length > 0)
    {
      uint32_t send_length = (uint32_t)length;
      if (send_length >= sizeof(message))
      {
        send_length = sizeof(message) - 1U;
      }
      (void)USART1_DebugWrite((const uint8_t *)message, send_length);
    }

    for (volatile uint32_t delay = 0U;
         delay < 24000000U;
         ++delay)
    {
      __NOP();
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  Error_HandlerContext((const char *)file, line, "assert_failed");
}
#endif
