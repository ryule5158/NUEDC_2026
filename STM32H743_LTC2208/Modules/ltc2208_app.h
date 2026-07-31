#ifndef __LTC2208_APP_H
#define __LTC2208_APP_H /* LTC2208应用层头文件包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "ltc2208.h"
#include <stdint.h>

#define LTC2208_APP_DEFAULT_SAMPLES     1024U /* 默认一次采集点数。 */
#define LTC2208_APP_DEFAULT_DECIMATION  1U    /* 默认不抽取。 */
#define LTC2208_APP_DEFAULT_TIMEOUT_MS  100U  /* 默认阻塞采集超时时间。 */

/* LTC2208模块输入电压两点校准数据。 */
typedef struct
{
  float low_code;          /* 低电压标定点的实测平均码。 */
  float high_code;         /* 高电压标定点的实测平均码。 */
  float low_voltage_v;     /* 低电压标定点，单位V。 */
  float high_voltage_v;    /* 高电压标定点，单位V。 */
  float volts_per_code;    /* 相邻ADC码对应的输入电压差。 */
  float intercept_v;       /* 原始码为0时的线性拟合电压。 */
  uint8_t valid;           /* 1表示当前校准参数有效。 */
} LTC2208_VoltageCalibrationTypeDef;

/* Capture phase retained when a blocking capture returns an error. */
typedef enum
{
  LTC2208_APP_CAPTURE_IDLE = 0,
  LTC2208_APP_CAPTURE_START,
  LTC2208_APP_CAPTURE_WAIT,
  LTC2208_APP_CAPTURE_READ,
  LTC2208_APP_CAPTURE_STATUS,
  LTC2208_APP_CAPTURE_COMPLETE
} LTC2208_AppCaptureStageTypeDef;

/* LTC2208应用层最近一次采集结果。 */
typedef struct
{
  LTC2208_StatusTypeDef status;       /* 最近一次应用层操作状态。 */
  LTC2208_CaptureInfoTypeDef capture; /* 最近一次FPGA采集状态。 */
  uint16_t sample_count;             /* 应用层缓存中的有效点数。 */
  uint8_t initialized;               /* 应用层初始化标志。 */
  LTC2208_AppCaptureStageTypeDef capture_stage;
} LTC2208_AppDataTypeDef;

/* 初始化高速ADC应用层并核验STM32-FPGA-LTC2208链路。 */
LTC2208_StatusTypeDef LTC2208_AppInit(void);

/* 立即触发并阻塞采集，点数1~4096，抽取倍数1~65535。 */
LTC2208_StatusTypeDef LTC2208_AppCapture(uint16_t sample_count,
                                       uint16_t decimation,
                                       uint32_t timeout_ms);

/* 按输入电压阈值触发并阻塞采集，不提供触发前数据。 */
LTC2208_StatusTypeDef LTC2208_AppCaptureTriggered(
    uint16_t sample_count,
    uint16_t decimation,
    LTC2208_TriggerTypeDef trigger_mode,
    float trigger_voltage_v,
    uint32_t timeout_ms);

/* 设置两个已知输入电压及其实测平均码，用于线性电压换算。 */
LTC2208_StatusTypeDef LTC2208_AppSetVoltageCalibration(
    float low_code,
    float low_voltage_v,
    float high_code,
    float high_voltage_v);

/* 将一个16位原始采样码换算为校准后的模块输入电压。 */
float LTC2208_AppCodeToVoltage(uint16_t code);

/* 获取应用层4096点 signed-20 Q3 采样缓存，只读使用。 */
const LTC2208_SampleTypeDef *LTC2208_AppGetSamples(void);

/* 获取应用层缓存中的有效采样点数。 */
uint16_t LTC2208_AppGetSampleCount(void);

/* 获取最近一次采集结果，只读使用。 */
const LTC2208_AppDataTypeDef *LTC2208_AppGetData(void);

/* 获取当前两点电压校准数据，只读使用。 */
const LTC2208_VoltageCalibrationTypeDef *
LTC2208_AppGetVoltageCalibration(void);

#ifdef __cplusplus
}
#endif

#endif /* __LTC2208_APP_H */
