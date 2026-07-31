#include "ads8688_app.h"
#include <stdio.h>

/* 本层提供九通道轮询、单通道块采集及调试打印接口。 */

static ADS8688_HandleTypeDef s_ads8688;             /* ADS8688应用层驱动句柄。 */
static ADS8688_AppDataTypeDef s_ads8688_app_data;   /* 最近一轮通道扫描数据。 */
static ADS8688_AppCaptureTypeDef s_ads8688_capture; /* 最近一次块采集信息。 */
static uint16_t s_ads8688_capture_raw[ADS8688_APP_SAMPLE_LENGTH]; /* 原始采样码缓冲区。 */
static float s_ads8688_capture_voltage_mv[ADS8688_APP_SAMPLE_LENGTH]; /* 电压采样缓冲区。 */
static uint32_t s_ads8688_print_tick;                /* 上次打印的系统节拍。 */

/* 获取通道的短名称。 */
static const char *ADS8688_AppChannelName(ADS8688_ChannelTypeDef channel);
/* 获取驱动状态的短名称。 */
static const char *ADS8688_AppStatusName(ADS8688_StatusTypeDef status);
/* 按需初始化ADS8688应用层。 */
static ADS8688_StatusTypeDef ADS8688_AppEnsureInit(void);
/* 获取指定通道当前量程。 */
static ADS8688_RangeTypeDef ADS8688_AppGetChannelRange(ADS8688_ChannelTypeDef channel);
/* 将块采集原始码换算为电压。 */
static void ADS8688_AppConvertCapture(ADS8688_ChannelTypeDef channel,
                                      uint32_t sample_count);
/* 将浮点数按比例转换为定点整数。 */
static int32_t ADS8688_AppScaleFloat(float value, float scale);
/* 打印带小数位的定点数。 */
static void ADS8688_AppPrintFixed(int32_t scaled_value, uint32_t decimals);

/* 初始化ADS8688应用层。 */
ADS8688_StatusTypeDef ADS8688_AppInit(ADS8688_RangeTypeDef range,
                                      float vref_mv)
{
  ADS8688_ConfigTypeDef cfg;
  ADS8688_StatusTypeDef status;

  if (vref_mv <= 0.0f)
  {
    vref_mv = ADS8688_APP_DEFAULT_VREF_MV;
  }

  ADS8688_GetDefaultConfig(&cfg);
  cfg.vref_mv = vref_mv;

  status = ADS8688_Init(&s_ads8688, &cfg);
  if (status == ADS8688_OK)
  {
    status = ADS8688_SetAllRanges(&s_ads8688, range);
  }

  s_ads8688_app_data.status = status;
  s_ads8688_app_data.scan_count = 0U;
  s_ads8688_app_data.initialized = (status == ADS8688_OK) ? 1U : 0U;
  s_ads8688_capture.status = status;
  s_ads8688_capture.channel = ADS8688_APP_DEFAULT_CAPTURE_CH;
  s_ads8688_capture.range = ADS8688_AppGetChannelRange(ADS8688_APP_DEFAULT_CAPTURE_CH);
  s_ads8688_capture.sample_count = 0U;
  s_ads8688_capture.target_sample_rate_hz = 0.0f;
  s_ads8688_capture.actual_sample_rate_hz = 0.0f;
  s_ads8688_capture.ready = 0U;
  s_ads8688_print_tick = HAL_GetTick();

#if (ADS8688_APP_ENABLE_PRINTF == 1U)
  if (status == ADS8688_OK)
  {
    printf("ADS8688 init ok, range=%u, vref=", (unsigned int)range);
    ADS8688_AppPrintFixed(ADS8688_AppScaleFloat(vref_mv, 10.0f), 1U);
    printf("mV\r\n");
  }
  else
  {
    printf("ADS8688 init failed: %s(%d)\r\n",
           ADS8688_AppStatusName(status),
           (int)status);
  }
#endif

  return status;
}

/* 扫描全部九个输入通道。 */
ADS8688_StatusTypeDef ADS8688_AppScanAll(void)
{
  ADS8688_StatusTypeDef status;

  status = ADS8688_AppEnsureInit();
  if (status != ADS8688_OK)
  {
    return status;
  }

  status = ADS8688_ReadAllChannels(&s_ads8688, s_ads8688_app_data.samples);
  s_ads8688_app_data.status = status;
  if (status == ADS8688_OK)
  {
    s_ads8688_app_data.scan_count++;
  }

  return status;
}

/* 读取指定输入通道。 */
ADS8688_StatusTypeDef ADS8688_AppReadChannel(ADS8688_ChannelTypeDef channel,
                                              ADS8688_SampleTypeDef *sample)
{
  ADS8688_StatusTypeDef status;

  if (sample == NULL)
  {
    return ADS8688_ERROR_PARAM;
  }

  status = ADS8688_AppEnsureInit();
  if (status != ADS8688_OK)
  {
    return status;
  }

  status = ADS8688_ReadChannel(&s_ads8688, channel, sample);
  s_ads8688_app_data.status = status;
  return status;
}

/* 采集指定通道的连续波形块。 */
ADS8688_StatusTypeDef ADS8688_AppCaptureChannel(ADS8688_ChannelTypeDef channel,
                                                uint32_t sample_count,
                                                float sample_rate_hz)
{
  ADS8688_StatusTypeDef status;

  if ((sample_count == 0U) || (sample_count > ADS8688_APP_SAMPLE_LENGTH))
  {
    return ADS8688_ERROR_PARAM;
  }

  status = ADS8688_AppEnsureInit();
  if (status != ADS8688_OK)
  {
    return status;
  }

  s_ads8688_capture.ready = 0U;
  s_ads8688_capture.channel = channel;
  s_ads8688_capture.range = ADS8688_AppGetChannelRange(channel);
  s_ads8688_capture.sample_count = sample_count;
  s_ads8688_capture.target_sample_rate_hz = sample_rate_hz;
  s_ads8688_capture.actual_sample_rate_hz = 0.0f;

  status = ADS8688_CaptureChannel(&s_ads8688,
                                  channel,
                                  s_ads8688_capture_raw,
                                  sample_count,
                                  sample_rate_hz,
                                  &s_ads8688_capture.actual_sample_rate_hz);
  s_ads8688_capture.status = status;
  s_ads8688_app_data.status = status;

  if (status == ADS8688_OK)
  {
    ADS8688_AppConvertCapture(channel, sample_count);
    s_ads8688_capture.ready = 1U;
  }

  return status;
}

/* 按默认参数完成一次块采集。 */
ADS8688_StatusTypeDef ADS8688_AppCaptureDefault(void)
{
  return ADS8688_AppCaptureChannel(ADS8688_APP_DEFAULT_CAPTURE_CH,
                                   ADS8688_APP_SAMPLE_LENGTH,
                                   ADS8688_APP_FASTEST_RATE);
}

/* 查询块采集数据是否就绪。 */
uint8_t ADS8688_AppCaptureReady(void)
{
  return s_ads8688_capture.ready;
}

/* 获取块采集原始码缓冲区。 */
const uint16_t *ADS8688_AppGetCaptureRawData(void)
{
  return s_ads8688_capture_raw;
}

/* 获取块采集电压缓冲区。 */
const float *ADS8688_AppGetCaptureVoltageData(void)
{
  return s_ads8688_capture_voltage_mv;
}

/* 获取块采集有效点数。 */
uint32_t ADS8688_AppGetCaptureLength(void)
{
  return s_ads8688_capture.sample_count;
}

/* 获取块采集实际采样率。 */
float ADS8688_AppGetCaptureSampleRateHz(void)
{
  return s_ads8688_capture.actual_sample_rate_hz;
}

/* 获取块采集状态信息。 */
const ADS8688_AppCaptureTypeDef *ADS8688_AppGetCaptureInfo(void)
{
  return &s_ads8688_capture;
}

/* 周期扫描并按配置打印采样结果。 */
void ADS8688_AppProcess(void)
{
  ADS8688_StatusTypeDef status;
  uint8_t i;

  status = ADS8688_AppScanAll();

#if (ADS8688_APP_ENABLE_PRINTF == 1U)
  if ((HAL_GetTick() - s_ads8688_print_tick) < ADS8688_APP_PRINT_INTERVAL_MS)
  {
    return;
  }

  s_ads8688_print_tick = HAL_GetTick();
  if (status != ADS8688_OK)
  {
    printf("ADS8688 scan failed: %s(%d)\r\n",
           ADS8688_AppStatusName(status),
           (int)status);
    return;
  }

  printf("ADS8688 scan %lu:", (unsigned long)s_ads8688_app_data.scan_count);
  for (i = 0U; i < ADS8688_CHANNEL_COUNT; i++)
  {
    printf(" %s=%u/",
           ADS8688_AppChannelName(s_ads8688_app_data.samples[i].channel),
           (unsigned int)s_ads8688_app_data.samples[i].code);
    ADS8688_AppPrintFixed(ADS8688_AppScaleFloat(s_ads8688_app_data.samples[i].voltage_mv,
                                                100.0f),
                          2U);
    printf("mV");
  }
  printf("\r\n");
#else
  (void)status;
  (void)i;
#endif
}

/* 获取应用层轮询数据。 */
const ADS8688_AppDataTypeDef *ADS8688_AppGetData(void)
{
  return &s_ads8688_app_data;
}

/* 获取ADS8688底层句柄。 */
ADS8688_HandleTypeDef *ADS8688_AppGetHandle(void)
{
  return &s_ads8688;
}

/* 按需初始化ADS8688应用层。 */
static ADS8688_StatusTypeDef ADS8688_AppEnsureInit(void)
{
  if (s_ads8688_app_data.initialized != 0U)
  {
    return ADS8688_OK;
  }

  return ADS8688_AppInit(ADS8688_APP_DEFAULT_RANGE,
                         ADS8688_APP_DEFAULT_VREF_MV);
}

/* 获取指定通道当前量程。 */
static ADS8688_RangeTypeDef ADS8688_AppGetChannelRange(ADS8688_ChannelTypeDef channel)
{
  if (channel == ADS8688_CHANNEL_AUX)
  {
    return ADS8688_RANGE_AUX_4V096;
  }

  if ((uint8_t)channel < ADS8688_ANALOG_CHANNEL_COUNT)
  {
    return s_ads8688.ranges[(uint8_t)channel];
  }

  return ADS8688_APP_DEFAULT_RANGE;
}

/* 将块采集原始码换算为电压。 */
static void ADS8688_AppConvertCapture(ADS8688_ChannelTypeDef channel,
                                      uint32_t sample_count)
{
  ADS8688_RangeTypeDef range = ADS8688_AppGetChannelRange(channel);
  uint32_t i;

  for (i = 0U; i < sample_count; i++)
  {
    if (channel == ADS8688_CHANNEL_AUX)
    {
      s_ads8688_capture_voltage_mv[i] =
          ADS8688_AuxCodeToVoltageMv(s_ads8688_capture_raw[i],
                                     s_ads8688.cfg.vref_mv);
    }
    else
    {
      s_ads8688_capture_voltage_mv[i] =
          ADS8688_CodeToVoltageMv(s_ads8688_capture_raw[i],
                                  range,
                                  s_ads8688.cfg.vref_mv);
    }
  }
}

/* 获取通道的短名称。 */
static const char *ADS8688_AppChannelName(ADS8688_ChannelTypeDef channel)
{
  switch (channel)
  {
    case ADS8688_CHANNEL_0:   return "CH0";
    case ADS8688_CHANNEL_1:   return "CH1";
    case ADS8688_CHANNEL_2:   return "CH2";
    case ADS8688_CHANNEL_3:   return "CH3";
    case ADS8688_CHANNEL_4:   return "CH4";
    case ADS8688_CHANNEL_5:   return "CH5";
    case ADS8688_CHANNEL_6:   return "CH6";
    case ADS8688_CHANNEL_7:   return "CH7";
    case ADS8688_CHANNEL_AUX: return "AUX";
    default:                  return "UNK";
  }
}

/* 获取驱动状态的短名称。 */
static const char *ADS8688_AppStatusName(ADS8688_StatusTypeDef status)
{
  switch (status)
  {
    case ADS8688_OK:                 return "OK";
    case ADS8688_ERROR:              return "ERROR";
    case ADS8688_ERROR_PARAM:        return "PARAM";
    case ADS8688_ERROR_SPI:          return "SPI";
    case ADS8688_ERROR_SPI_CONFIG:   return "SPI_CONFIG";
    case ADS8688_ERROR_UNSUPPORTED:  return "UNSUPPORTED";
    default:                         return "UNKNOWN";
  }
}

/* 将浮点数按比例转换为定点整数。 */
static int32_t ADS8688_AppScaleFloat(float value, float scale)
{
  if (value >= 0.0f)
  {
    return (int32_t)(value * scale + 0.5f);
  }

  return (int32_t)(value * scale - 0.5f);
}

/* 打印带小数位的定点数。 */
static void ADS8688_AppPrintFixed(int32_t scaled_value, uint32_t decimals)
{
  uint32_t divisor = 1U;
  int32_t whole;
  int32_t frac;

  for (uint32_t i = 0U; i < decimals; i++)
  {
    divisor *= 10U;
  }

  whole = scaled_value / (int32_t)divisor;
  frac = scaled_value % (int32_t)divisor;
  if (frac < 0)
  {
    frac = -frac;
  }

  if ((scaled_value < 0) && (whole == 0))
  {
    printf("-0");
  }
  else
  {
    printf("%ld", (long)whole);
  }

  if (decimals == 0U)
  {
    return;
  }

  printf(".");
  for (uint32_t pad = divisor / 10U; pad > 1U; pad /= 10U)
  {
    if ((uint32_t)frac >= pad)
    {
      break;
    }
    printf("0");
  }
  printf("%ld", (long)frac);
}
