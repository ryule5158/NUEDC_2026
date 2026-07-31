#ifndef __AD603_H__
#define __AD603_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "ad5687_app.h"

#define AD603_DEFAULT_MIN_GAIN_MDB     (-20000L) /* 默认最小增益，单位mdB。 */
#define AD603_DEFAULT_MAX_GAIN_MDB       60000L  /* 默认最大增益，单位mdB。 */
#define AD603_DEFAULT_MIN_CONTROL_MV       400U  /* 默认最小增益控制电压，单位mV。 */
#define AD603_DEFAULT_MAX_CONTROL_MV      1446U  /* 默认最大增益控制电压，单位mV。 */

/* AD603驱动返回状态。 */
typedef enum
{
  AD603_OK = 0U,       /* 操作成功。 */
  AD603_ERROR_PARAM,   /* 指针或配置参数无效。 */
  AD603_ERROR_RANGE,   /* 目标增益超出校准范围。 */
  AD603_ERROR_DAC      /* AD5687电压输出失败。 */
} AD603_StatusTypeDef;

/* AD603模块增益与控制电压的两点校准配置。 */
typedef struct
{
  AD5687_OutputTypeDef dac_output; /* 连接AD603 P2-2的AD5687 CV输出。 */
  int32_t min_gain_mdb;            /* 校准最小增益，单位mdB。 */
  int32_t max_gain_mdb;            /* 校准最大增益，单位mdB。 */
  uint16_t min_control_mv;         /* 最小增益对应控制电压，单位mV。 */
  uint16_t max_control_mv;         /* 最大增益对应控制电压，单位mV。 */
} AD603_ConfigTypeDef;

/* AD603模块运行句柄。 */
typedef struct
{
  AD603_ConfigTypeDef config; /* 当前模块校准配置。 */
  int32_t gain_mdb;           /* 最近成功设置的增益，单位mdB。 */
  uint16_t control_mv;        /* 最近成功输出的控制电压，单位mV。 */
  uint8_t initialized;        /* 驱动初始化完成标志。 */
} AD603_HandleTypeDef;

/* 填充用户手册给出的默认增益校准参数。 */
void AD603_GetDefaultConfig(AD603_ConfigTypeDef *config);

/* 初始化AD603并把所选CV口置为最小增益电压。 */
AD603_StatusTypeDef AD603_Init(AD603_HandleTypeDef *handle,
                               const AD603_ConfigTypeDef *config);

/* 按mdB设置模块增益，例如10000表示10dB。 */
AD603_StatusTypeDef AD603_SetGainMdb(AD603_HandleTypeDef *handle,
                                     int32_t gain_mdb);

/* 根据两点校准参数计算目标控制电压。 */
AD603_StatusTypeDef AD603_GainToVoltageMv(const AD603_ConfigTypeDef *config,
                                          int32_t gain_mdb,
                                          uint16_t *voltage_mv);

#ifdef __cplusplus
}
#endif

#endif /* __AD603_H__ */
