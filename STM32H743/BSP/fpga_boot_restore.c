#include "fpga_boot_restore.h"
#include "fpga_promax_link.h"
#include "stm32h7xx_hal.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define FPGA_BOOT_LOG_SIZE 192U
#define FPGA_BOOT_AD9708_ID_ADDRESS 0x0AU
#define FPGA_BOOT_AD9280_ID_ADDRESS 0x2AU
#define FPGA_BOOT_PROMAX_ID_ADDRESS 0x30U

/*
 * 默认 bank0：直通；bank1：8 点滑动平均；bank2：一阶差分；
 * bank3：二阶差分。所有系数均为 Q1.16，且每次恢复都会补零到 32 抽头。
 */
static const int32_t s_fir_passthrough_q16[1] = {
  65536
};
static const int32_t s_fir_average_q16[8] = {
  8192, 8192, 8192, 8192, 8192, 8192, 8192, 8192
};
static const int32_t s_fir_edge_q16[2] = {
  32768, -32768
};
static const int32_t s_fir_curvature_q16[3] = {
  16384, -32768, 16384
};
static const FpgaBootRestore_FirBank s_default_fir_banks[
  FPGA_PROMAX_MF_BANKS] = {
  {0U, s_fir_passthrough_q16, 1U},
  {1U, s_fir_average_q16, 8U},
  {2U, s_fir_edge_q16, 2U},
  {3U, s_fir_curvature_q16, 3U}
};

/*
 * 默认监测基波的 1～8 次谐波。它只是未知信号搜索前的确定初值，
 * PLL 得到基波后应通过 SetToneBank() 更新，不依赖这些固定频点锁相。
 */
static const uint32_t s_default_tones_hz[FPGA_PROMAX_DDC_CHANNELS] = {
  FPGA_BOOT_INITIAL_FUNDAMENTAL_HZ,
  FPGA_BOOT_INITIAL_FUNDAMENTAL_HZ * 2U,
  FPGA_BOOT_INITIAL_FUNDAMENTAL_HZ * 3U,
  FPGA_BOOT_INITIAL_FUNDAMENTAL_HZ * 4U,
  FPGA_BOOT_INITIAL_FUNDAMENTAL_HZ * 5U,
  FPGA_BOOT_INITIAL_FUNDAMENTAL_HZ * 6U,
  FPGA_BOOT_INITIAL_FUNDAMENTAL_HZ * 7U,
  FPGA_BOOT_INITIAL_FUNDAMENTAL_HZ * 8U
};

static FpgaBootRestore_Config s_config;
static FpgaBootRestore_Status s_status;
static FpgaPromax s_device;
static uint32_t s_tone_cache_hz[FPGA_PROMAX_DDC_CHANNELS];
static char s_log_buffer[FPGA_BOOT_LOG_SIZE];
static uint8_t s_started;
static uint8_t s_device_initialized;

static uint8_t FpgaBootRestore_TimeReached(uint32_t now, uint32_t deadline);
static FpgaPromax_Result FpgaBootRestore_ValidateConfig(
  const FpgaBootRestore_Config *config);
static FpgaPromax_Result FpgaBootRestore_LoadAllFir(void);
static void FpgaBootRestore_Log(const char *format, ...);
static void FpgaBootRestore_LogRawDeviceIds(void);
static void FpgaBootRestore_RecordFailure(FpgaPromax_Result result);
static FpgaPromax_Result FpgaBootRestore_StopBestEffort(void);

void FpgaBootRestore_GetDefaultConfig(FpgaBootRestore_Config *config)
{
  if (config == NULL)
  {
    return;
  }

  memset(config, 0, sizeof(*config));
  config->adc_sample_rate_hz = FPGA_BOOT_ADC_SAMPLE_RATE_HZ;
  config->qspi_settle_ms = FPGA_BOOT_QSPI_SETTLE_MS;
  config->retry_interval_ms = FPGA_BOOT_RETRY_INTERVAL_MS;
  config->required_capability =
    FPGA_PROMAX_CAPABILITY_REALTIME | FPGA_PROMAX_CAPABILITY_WAVE_STATS;
  config->wave_block_log2 = FPGA_BOOT_WAVE_BLOCK_LOG2;
  config->run_after_restore = 1U;
  config->fir_banks = s_default_fir_banks;
  config->fir_bank_count = FPGA_PROMAX_MF_BANKS;
  config->tone_frequencies_hz = s_default_tones_hz;
  config->tone_count = FPGA_PROMAX_DDC_CHANNELS;
}

FpgaPromax_Result FpgaBootRestore_Start(
  const FpgaBootRestore_Config *config)
{
  FpgaPromax_Result result;

  result = FpgaBootRestore_ValidateConfig(config);
  if (result != FPGA_PROMAX_OK)
  {
    return result;
  }

  memset(&s_device, 0, sizeof(s_device));
  memset(&s_status, 0, sizeof(s_status));
  s_config = *config;
  memcpy(s_tone_cache_hz,
         config->tone_frequencies_hz,
         config->tone_count * sizeof(s_tone_cache_hz[0]));
  s_config.tone_frequencies_hz = s_tone_cache_hz;
  s_started = 1U;
  s_device_initialized = 0U;
  s_status.state = FPGA_BOOT_STATE_WAIT_QSPI;
  s_status.stage = FPGA_BOOT_STAGE_NONE;
  s_status.last_result = FPGA_PROMAX_OK;
  s_status.next_action_tick = HAL_GetTick() + s_config.qspi_settle_ms;

  FpgaBootRestore_Log(
    "[FPGA_BOOT] wait_qspi=%lu ms, automatic volatile restore armed\r\n",
    (unsigned long)s_config.qspi_settle_ms);
  return FPGA_PROMAX_OK;
}

void FpgaBootRestore_Process(void)
{
  uint32_t now;

  if ((s_started == 0U) ||
      (s_status.state == FPGA_BOOT_STATE_IDLE) ||
      (s_status.state == FPGA_BOOT_STATE_READY) ||
      (s_status.state == FPGA_BOOT_STATE_RESTORING))
  {
    return;
  }

  now = HAL_GetTick();
  if (FpgaBootRestore_TimeReached(now, s_status.next_action_tick) == 0U)
  {
    return;
  }

  (void)FpgaBootRestore_RestoreNow();
}

FpgaPromax_Result FpgaBootRestore_WaitReady(uint32_t timeout_ms,
                                                uint32_t stable_guard_ms)
{
  uint32_t start_tick;
  FpgaPromax_Result result;

  if (s_started == 0U)
  {
    return FPGA_PROMAX_E_ARGUMENT;
  }

  start_tick = HAL_GetTick();
  for (;;)
  {
    FpgaBootRestore_Process();
    if (s_status.ready != 0U)
    {
      /* An independently powered FPGA may answer briefly and then be reset by
       * its QSPI configuration sequence. Re-run the complete transaction after
       * a guard interval so RAM coefficients and registers are never stale. */
      if (stable_guard_ms != 0U)
      {
        HAL_Delay(stable_guard_ms);
      }
      result = FpgaBootRestore_RestoreNow();
      if (result == FPGA_PROMAX_OK)
      {
        return FPGA_PROMAX_OK;
      }
    }
    else if ((s_status.state == FPGA_BOOT_STATE_IDLE) &&
             (s_status.last_result != FPGA_PROMAX_OK))
    {
      return s_status.last_result;
    }

    if ((timeout_ms != 0U) &&
        ((uint32_t)(HAL_GetTick() - start_tick) >= timeout_ms))
    {
      return (s_status.last_result == FPGA_PROMAX_OK) ?
             FPGA_PROMAX_E_TIMEOUT : s_status.last_result;
    }
    HAL_Delay(10U);
  }
}
FpgaPromax_Result FpgaBootRestore_RestoreNow(void)
{
  FpgaPromax_Result result;
  uint32_t status_word;

  if (s_started == 0U)
  {
    return FPGA_PROMAX_E_ARGUMENT;
  }

  s_status.state = FPGA_BOOT_STATE_RESTORING;
  s_status.ready = 0U;
  s_status.attempt_count++;
  s_status.fpga_status = 0U;
  s_device_initialized = 0U;

  s_status.stage = FPGA_BOOT_STAGE_INIT_LINK;
  result = FPGA_ProMax_Init(&s_device);
  s_status.fpga_device_id = s_device.device_id;
  if (result != FPGA_PROMAX_OK)
  {
    FpgaBootRestore_LogRawDeviceIds();
    FpgaBootRestore_RecordFailure(result);
    return result;
  }
  s_device_initialized = 1U;
  s_status.fpga_version = s_device.version;
  s_status.fpga_capability = s_device.capability;

  s_status.stage = FPGA_BOOT_STAGE_CHECK_CAPABILITY;
  if ((s_device.capability & s_config.required_capability) !=
      s_config.required_capability)
  {
    FpgaBootRestore_RecordFailure(FPGA_PROMAX_E_UNSUPPORTED);
    return FPGA_PROMAX_E_UNSUPPORTED;
  }

  s_status.stage = FPGA_BOOT_STAGE_STOP;
  result = FpgaPromax_SetRun(&s_device, 0U);
  if (result != FPGA_PROMAX_OK)
  {
    FpgaBootRestore_RecordFailure(result);
    return result;
  }

  s_status.stage = FPGA_BOOT_STAGE_LOAD_FIR;
  result = FpgaBootRestore_LoadAllFir();
  if (result != FPGA_PROMAX_OK)
  {
    FpgaBootRestore_RecordFailure(result);
    return result;
  }

  s_status.stage = FPGA_BOOT_STAGE_CONFIGURE_TONES;
  result = FpgaPromax_ConfigureToneBank(&s_device,
                                        s_config.tone_frequencies_hz,
                                        s_config.tone_count,
                                        s_config.adc_sample_rate_hz);
  if (result != FPGA_PROMAX_OK)
  {
    FpgaBootRestore_RecordFailure(result);
    return result;
  }

  s_status.stage = FPGA_BOOT_STAGE_CONFIGURE_WAVE_STATS;
  result = FpgaPromax_SetWaveBlockLog2(&s_device,
                                       s_config.wave_block_log2);
  if (result != FPGA_PROMAX_OK)
  {
    FpgaBootRestore_RecordFailure(result);
    return result;
  }

  s_status.stage = FPGA_BOOT_STAGE_CLEAR_STATE;
  result = FpgaPromax_ClearState(&s_device);
  if (result != FPGA_PROMAX_OK)
  {
    FpgaBootRestore_RecordFailure(result);
    return result;
  }

  s_status.stage = FPGA_BOOT_STAGE_START;
  result = FpgaPromax_SetRun(&s_device, s_config.run_after_restore);
  if (result != FPGA_PROMAX_OK)
  {
    FpgaBootRestore_RecordFailure(result);
    return result;
  }

  s_status.stage = FPGA_BOOT_STAGE_VERIFY_RUN;
  result = FpgaPromax_GetStatus(&s_device, &status_word);
  if (result != FPGA_PROMAX_OK)
  {
    FpgaBootRestore_RecordFailure(result);
    return result;
  }
  s_status.fpga_status = status_word;
  if ((((status_word & FPGA_PROMAX_STATUS_RUN) != 0U) ? 1U : 0U) !=
      s_config.run_after_restore)
  {
    FpgaBootRestore_RecordFailure(FPGA_PROMAX_E_FPGA_STATUS);
    return FPGA_PROMAX_E_FPGA_STATUS;
  }

  s_status.state = FPGA_BOOT_STATE_READY;
  s_status.stage = FPGA_BOOT_STAGE_NONE;
  s_status.last_result = FPGA_PROMAX_OK;
  s_status.ready = 1U;
  FpgaBootRestore_Log(
    "[FPGA_BOOT] READY attempt=%lu id=0x%08lX version=0x%08lX "
    "cap=0x%08lX status=0x%08lX fir=%lu tones=%lu fs=%lu\r\n",
    (unsigned long)s_status.attempt_count,
    (unsigned long)s_status.fpga_device_id,
    (unsigned long)s_status.fpga_version,
    (unsigned long)s_status.fpga_capability,
    (unsigned long)s_status.fpga_status,
    (unsigned long)s_config.fir_bank_count,
    (unsigned long)s_config.tone_count,
    (unsigned long)s_config.adc_sample_rate_hz);
  return FPGA_PROMAX_OK;
}

FpgaPromax_Result FpgaBootRestore_SetToneBank(
  const uint32_t *frequencies_hz,
  size_t count)
{
  FpgaPromax_Result result;

  if ((s_started == 0U) || (s_status.ready == 0U) ||
      (frequencies_hz == NULL) || (count == 0U) ||
      (count > FPGA_PROMAX_DDC_CHANNELS))
  {
    return FPGA_PROMAX_E_ARGUMENT;
  }

  result = FpgaPromax_SetRun(&s_device, 0U);
  if (result != FPGA_PROMAX_OK)
  {
    return result;
  }
  result = FpgaPromax_ConfigureToneBank(&s_device,
                                        frequencies_hz,
                                        count,
                                        s_config.adc_sample_rate_hz);
  if (result == FPGA_PROMAX_OK)
  {
    result = FpgaPromax_ClearState(&s_device);
  }
  if (result == FPGA_PROMAX_OK)
  {
    result = FpgaPromax_SetRun(&s_device, s_config.run_after_restore);
  }
  if (result != FPGA_PROMAX_OK)
  {
    (void)FpgaBootRestore_StopBestEffort();
    return result;
  }

  memcpy(s_tone_cache_hz,
         frequencies_hz,
         count * sizeof(s_tone_cache_hz[0]));
  s_config.tone_count = count;
  s_status.fpga_status =
    (s_config.run_after_restore != 0U) ? FPGA_PROMAX_STATUS_RUN : 0U;
  return FPGA_PROMAX_OK;
}

void FpgaBootRestore_RequestRecovery(void)
{
  if (s_started == 0U)
  {
    return;
  }

  s_status.ready = 0U;
  s_status.state = FPGA_BOOT_STATE_RETRY_WAIT;
  s_status.stage = FPGA_BOOT_STAGE_NONE;
  s_status.next_action_tick = HAL_GetTick();
}

const FpgaBootRestore_Status *FpgaBootRestore_GetStatus(void)
{
  return &s_status;
}

FpgaPromax *FpgaBootRestore_GetDevice(void)
{
  return (s_status.ready != 0U) ? &s_device : NULL;
}

const char *FpgaBootRestore_StageName(FpgaBootRestore_Stage stage)
{
  switch (stage)
  {
    case FPGA_BOOT_STAGE_INIT_LINK:
      return "FPGA_ProMax_Init";
    case FPGA_BOOT_STAGE_CHECK_CAPABILITY:
      return "CheckCapability";
    case FPGA_BOOT_STAGE_STOP:
      return "FpgaPromax_SetRun(0)";
    case FPGA_BOOT_STAGE_LOAD_FIR:
      return "FpgaPromax_LoadFirBank";
    case FPGA_BOOT_STAGE_CONFIGURE_TONES:
      return "FpgaPromax_ConfigureToneBank";
    case FPGA_BOOT_STAGE_CONFIGURE_WAVE_STATS:
      return "FpgaPromax_SetWaveBlockLog2";
    case FPGA_BOOT_STAGE_CLEAR_STATE:
      return "FpgaPromax_ClearState";
    case FPGA_BOOT_STAGE_START:
      return "FpgaPromax_SetRun(1)";
    case FPGA_BOOT_STAGE_VERIFY_RUN:
      return "FpgaPromax_GetStatus";
    default:
      return "none";
  }
}

static uint8_t FpgaBootRestore_TimeReached(uint32_t now, uint32_t deadline)
{
  return (((int32_t)(now - deadline)) >= 0) ? 1U : 0U;
}

static FpgaPromax_Result FpgaBootRestore_ValidateConfig(
  const FpgaBootRestore_Config *config)
{
  uint8_t bank_seen;
  size_t index;

  if ((config == NULL) || (config->adc_sample_rate_hz == 0U) ||
      (config->retry_interval_ms == 0U) ||
      (config->fir_banks == NULL) ||
      (config->fir_bank_count != FPGA_PROMAX_MF_BANKS) ||
      (config->tone_frequencies_hz == NULL) ||
      (config->tone_count == 0U) ||
      (config->tone_count > FPGA_PROMAX_DDC_CHANNELS) ||
      (config->wave_block_log2 < FPGA_PROMAX_WAVE_MIN_BLOCK_LOG2) ||
      (config->wave_block_log2 > FPGA_PROMAX_WAVE_MAX_BLOCK_LOG2) ||
      (config->run_after_restore > 1U))
  {
    return FPGA_PROMAX_E_ARGUMENT;
  }

  bank_seen = 0U;
  for (index = 0U; index < config->fir_bank_count; index++)
  {
    const FpgaBootRestore_FirBank *bank;
    uint8_t bank_mask;

    bank = &config->fir_banks[index];
    if ((bank->bank >= FPGA_PROMAX_MF_BANKS) ||
        (bank->coeff_q16 == NULL) || (bank->count == 0U) ||
        (bank->count > FPGA_PROMAX_MF_TAPS))
    {
      return FPGA_PROMAX_E_ARGUMENT;
    }
    bank_mask = (uint8_t)(1U << bank->bank);
    if ((bank_seen & bank_mask) != 0U)
    {
      return FPGA_PROMAX_E_ARGUMENT;
    }
    bank_seen = (uint8_t)(bank_seen | bank_mask);
  }
  if (bank_seen != (uint8_t)((1U << FPGA_PROMAX_MF_BANKS) - 1U))
  {
    return FPGA_PROMAX_E_ARGUMENT;
  }
  return FPGA_PROMAX_OK;
}

static FpgaPromax_Result FpgaBootRestore_LoadAllFir(void)
{
  FpgaPromax_Result result;
  size_t index;

  for (index = 0U; index < s_config.fir_bank_count; index++)
  {
    const FpgaBootRestore_FirBank *bank;

    bank = &s_config.fir_banks[index];
    result = FpgaPromax_LoadFirBank(&s_device,
                                    bank->bank,
                                    bank->coeff_q16,
                                    bank->count);
    if (result != FPGA_PROMAX_OK)
    {
      return result;
    }
  }
  return FPGA_PROMAX_OK;
}

static void FpgaBootRestore_Log(const char *format, ...)
{
  va_list args;
  int length;

  if ((s_config.log == NULL) || (format == NULL))
  {
    return;
  }

  va_start(args, format);
  length = vsnprintf(s_log_buffer, sizeof(s_log_buffer), format, args);
  va_end(args);
  if (length <= 0)
  {
    return;
  }
  s_log_buffer[sizeof(s_log_buffer) - 1U] = '\0';
  s_config.log(s_config.log_user, s_log_buffer);
}

/*
 * Read three independent fixed identifiers before ADC DMA starts. Their
 * combination separates a dead SPI read path from a missing ProMax address
 * range in the FPGA runtime image.
 */
static void FpgaBootRestore_LogRawDeviceIds(void)
{
  FpgaPromax_Result ad9708_result;
  FpgaPromax_Result ad9280_result;
  FpgaPromax_Result promax_result;
  uint32_t ad9708_id;
  uint32_t ad9280_id;
  uint32_t promax_id;

  ad9708_id = 0U;
  ad9280_id = 0U;
  promax_id = 0U;
  ad9708_result = FPGA_ProMax_ReadPhysicalRegister(
    FPGA_BOOT_AD9708_ID_ADDRESS,
    &ad9708_id);
  ad9280_result = FPGA_ProMax_ReadPhysicalRegister(
    FPGA_BOOT_AD9280_ID_ADDRESS,
    &ad9280_id);
  promax_result = FPGA_ProMax_ReadPhysicalRegister(
    FPGA_BOOT_PROMAX_ID_ADDRESS,
    &promax_id);

  FpgaBootRestore_Log(
    "[FPGA_BOOT] RAW_ID ad9708=0x%08lX(%d) ad9280=0x%08lX(%d) "
    "promax=0x%08lX(%d)\r\n",
    (unsigned long)ad9708_id,
    (int)ad9708_result,
    (unsigned long)ad9280_id,
    (int)ad9280_result,
    (unsigned long)promax_id,
    (int)promax_result);
}
static void FpgaBootRestore_RecordFailure(FpgaPromax_Result result)
{
  FpgaBootRestore_Stage failed_stage;
  int32_t transport_status;
  uint32_t retry_delay_ms;
  uint8_t protocol_status;

  failed_stage = s_status.stage;
  transport_status = s_device.last_transport_status;
  protocol_status = s_device.last_protocol_status;
  s_status.last_result = result;
  s_status.ready = 0U;
  (void)FpgaBootRestore_StopBestEffort();
  retry_delay_ms = s_config.retry_interval_ms;
  if ((result == FPGA_PROMAX_E_UNSUPPORTED) ||
      (result == FPGA_PROMAX_E_ARGUMENT) ||
      (result == FPGA_PROMAX_E_RANGE))
  {
    /* 确定性配置错误不会自行变好，停止后台重试以保护 ADC 主循环。 */
    s_status.state = FPGA_BOOT_STATE_IDLE;
    s_status.next_action_tick = 0U;
    retry_delay_ms = 0U;
  }
  else
  {
    s_status.state = FPGA_BOOT_STATE_RETRY_WAIT;
    s_status.next_action_tick = HAL_GetTick() + retry_delay_ms;
  }
  s_status.stage = failed_stage;
  FpgaBootRestore_Log(
    "[FPGA_BOOT] FAIL attempt=%lu stage=%s result=%d id=0x%08lX "
    "transport=%ld protocol=%u; retry_in=%lu ms\r\n",
    (unsigned long)s_status.attempt_count,
    FpgaBootRestore_StageName(failed_stage),
    (int)result,
    (unsigned long)s_status.fpga_device_id,
    (long)transport_status,
    (unsigned int)protocol_status,
    (unsigned long)retry_delay_ms);
}

static FpgaPromax_Result FpgaBootRestore_StopBestEffort(void)
{
  if (s_device_initialized == 0U)
  {
    return FPGA_PROMAX_E_TRANSPORT;
  }
  return FpgaPromax_SetRun(&s_device, 0U);
}
