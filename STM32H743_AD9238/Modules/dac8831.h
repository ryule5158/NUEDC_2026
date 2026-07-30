#ifndef DAC8831_H
#define DAC8831_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define DAC8831_RESOLUTION_BITS         16U       /* DAC8831 DAC分辨率位数 */
#define DAC8831_MAX_CODE                0xFFFFU   /* DAC8831最大16位DAC码值 */
#define DAC8831_MIDSCALE_CODE           0x8000U   /* DAC8831中点码值 */
#define DAC8831_SOFT_SPI_DELAY_CYCLES   1U       /* DAC8831软件SPI每个时序阶段的延时周期 */
#define DAC8831_DEFAULT_VREF_MV         2500.0f   /* DAC8831默认参考电压, 单位mV */

/* DAC8831驱动状态。 */
typedef enum
{
  DAC8831_OK = 0,       /* 操作成功。 */
  DAC8831_ERROR,        /* 通用操作错误。 */
  DAC8831_ERROR_PARAM,  /* 输入参数无效。 */
  DAC8831_ERROR_SPI     /* 串行通信失败。 */
} DAC8831_StatusTypeDef;

/* DAC8831 LDAC控制方式。 */
typedef enum
{
  DAC8831_LDAC_TIED_LOW = 0, /* LDAC硬件固定为低电平。 */
  DAC8831_LDAC_GPIO_PULSE    /* LDAC由GPIO脉冲更新。 */
} DAC8831_LdacModeTypeDef;

/* DAC8831模拟输出接法。 */
typedef enum
{
  DAC8831_OUTPUT_UNIPOLAR = 0, /* 单极性输出。 */
  DAC8831_OUTPUT_BIPOLAR       /* 双极性输出。 */
} DAC8831_OutputModeTypeDef;

/* DAC8831软件SPI和模拟参数配置。 */
typedef struct
{
  GPIO_TypeDef *sck_port;                /* 串行时钟端口。 */
  uint16_t sck_pin;                      /* 串行时钟引脚。 */
  GPIO_TypeDef *mosi_port;               /* 串行数据端口。 */
  uint16_t mosi_pin;                     /* 串行数据引脚。 */
  GPIO_TypeDef *cs_port;                 /* 片选端口。 */
  uint16_t cs_pin;                       /* 片选引脚。 */
  GPIO_TypeDef *ldac_port;               /* LDAC端口。 */
  uint16_t ldac_pin;                     /* LDAC引脚。 */
  DAC8831_LdacModeTypeDef ldac_mode;     /* LDAC控制方式。 */
  uint16_t spi_delay_cycles;             /* 软件SPI边沿延时周期数。 */
  uint16_t ldac_pulse_delay_cycles;      /* LDAC脉冲延时周期数。 */
  float vref_mv;                         /* 参考电压，单位mV。 */
  DAC8831_OutputModeTypeDef output_mode; /* 模拟输出接法。 */
} DAC8831_ConfigTypeDef;

/* DAC8831驱动句柄。 */
typedef struct
{
  DAC8831_ConfigTypeDef cfg; /* 当前驱动配置。 */
  uint16_t last_code;        /* 最近写入的16位码值。 */
} DAC8831_HandleTypeDef;

/* 获取不绑定引脚的默认配置。 */
void DAC8831_GetDefaultConfig(DAC8831_ConfigTypeDef *cfg);
/* 初始化DAC8831驱动句柄。 */
DAC8831_StatusTypeDef DAC8831_Init(DAC8831_HandleTypeDef *dev,
                                   const DAC8831_ConfigTypeDef *cfg);
/* 写入16位原始DAC码。 */
DAC8831_StatusTypeDef DAC8831_WriteRaw(DAC8831_HandleTypeDef *dev,
                                       uint16_t code);
/* 按毫伏值写入单极性输出。 */
DAC8831_StatusTypeDef DAC8831_WriteUnipolarMv(DAC8831_HandleTypeDef *dev,
                                              float millivolts);
/* 按毫伏值写入双极性输出。 */
DAC8831_StatusTypeDef DAC8831_WriteBipolarMv(DAC8831_HandleTypeDef *dev,
                                             float millivolts);
/* 产生一次LDAC更新脉冲。 */
DAC8831_StatusTypeDef DAC8831_PulseLdac(DAC8831_HandleTypeDef *dev);

/* 将单极性毫伏值换算为16位码。 */
uint16_t DAC8831_UnipolarMvToCode(float millivolts, float vref_mv);
/* 将双极性毫伏值换算为16位码。 */
uint16_t DAC8831_BipolarMvToCode(float millivolts, float vref_mv);
/* 将16位码换算为单极性毫伏值。 */
float DAC8831_CodeToUnipolarMv(uint16_t code, float vref_mv);
/* 将16位码换算为双极性毫伏值。 */
float DAC8831_CodeToBipolarMv(uint16_t code, float vref_mv);

#ifdef __cplusplus
}
#endif

#endif /* DAC8831_H */
