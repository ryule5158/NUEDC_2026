#ifndef __ADS8688_APP_H
#define __ADS8688_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ADS8688.h"

#define ADS8688_APP_DEFAULT_VREF_MV        ADS8688_DEFAULT_VREF_MV /* 默认参考电压。 */
#define ADS8688_APP_DEFAULT_RANGE          ADS8688_RANGE_BIPOLAR_10V24 /* 默认输入量程。 */
#define ADS8688_APP_ENABLE_PRINTF          1U    /* 1启用轮询结果打印。 */
#define ADS8688_APP_PRINT_INTERVAL_MS      200U  /* 轮询打印间隔，单位ms。 */
#define ADS8688_APP_SAMPLE_LENGTH          4096U /* 应用层最大采样点数。 */
#define ADS8688_APP_DEFAULT_CAPTURE_CH     ADS8688_CHANNEL_0 /* 默认采集通道。 */
#define ADS8688_APP_FASTEST_RATE           0.0f  /* 最快采集的特殊采样率值。 */

/* ADS8688应用层轮询数据。 */
typedef struct
{
  ADS8688_StatusTypeDef status; /* 最近一次操作状态。 */
  ADS8688_SampleTypeDef samples[ADS8688_CHANNEL_COUNT]; /* 最近一轮各通道数据。 */
  uint32_t scan_count;         /* 已完成扫描轮数。 */
  uint8_t initialized;         /* 应用层初始化标志。 */
} ADS8688_AppDataTypeDef;

/* ADS8688应用层块采集信息。 */
typedef struct
{
  ADS8688_StatusTypeDef status;       /* 块采集状态。 */
  ADS8688_ChannelTypeDef channel;     /* 块采集通道。 */
  ADS8688_RangeTypeDef range;         /* 采集时通道量程。 */
  uint32_t sample_count;              /* 有效采样点数。 */
  float target_sample_rate_hz;        /* 目标采样率，单位Hz。 */
  float actual_sample_rate_hz;        /* 实际采样率，单位Hz。 */
  uint8_t ready;                      /* 数据就绪标志。 */
} ADS8688_AppCaptureTypeDef;

/* 初始化ADS8688应用层。 */
ADS8688_StatusTypeDef ADS8688_AppInit(ADS8688_RangeTypeDef range,
                                      float vref_mv);
/* 扫描全部九个输入通道。 */
ADS8688_StatusTypeDef ADS8688_AppScanAll(void);
/* 读取指定输入通道。 */
ADS8688_StatusTypeDef ADS8688_AppReadChannel(ADS8688_ChannelTypeDef channel,
                                              ADS8688_SampleTypeDef *sample);
/* 采集指定通道的连续波形块。 */
ADS8688_StatusTypeDef ADS8688_AppCaptureChannel(ADS8688_ChannelTypeDef channel,
                                                uint32_t sample_count,
                                                float sample_rate_hz);
/* 按默认参数完成一次块采集。 */
ADS8688_StatusTypeDef ADS8688_AppCaptureDefault(void);
/* 查询块采集数据是否就绪。 */
uint8_t ADS8688_AppCaptureReady(void);
/* 获取块采集原始码缓冲区。 */
const uint16_t *ADS8688_AppGetCaptureRawData(void);
/* 获取块采集电压缓冲区。 */
const float *ADS8688_AppGetCaptureVoltageData(void);
/* 获取块采集有效点数。 */
uint32_t ADS8688_AppGetCaptureLength(void);
/* 获取块采集实际采样率。 */
float ADS8688_AppGetCaptureSampleRateHz(void);
/* 获取块采集状态信息。 */
const ADS8688_AppCaptureTypeDef *ADS8688_AppGetCaptureInfo(void);
/* 周期扫描并按配置打印采样结果。 */
void ADS8688_AppProcess(void);
/* 获取应用层轮询数据。 */
const ADS8688_AppDataTypeDef *ADS8688_AppGetData(void);
/* 获取ADS8688底层句柄。 */
ADS8688_HandleTypeDef *ADS8688_AppGetHandle(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADS8688_APP_H */
