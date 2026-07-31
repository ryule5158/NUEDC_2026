#ifndef __ADS8688_H
#define __ADS8688_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* ADS8688默认四线SPI映射：SCLK=PA5、MOSI=PB5、MISO=PA6、CS=PA4。 */
#ifndef ADS8688_USE_HARDWARE_SPI
#define ADS8688_USE_HARDWARE_SPI              1U /* 1使用硬件SPI，0使用软件SPI。 */
#endif

#ifndef ADS8688_SPI_INSTANCE
#define ADS8688_SPI_INSTANCE                  SPI1 /* 默认硬件SPI实例。 */
#endif

#ifndef ADS8688_SPI_BAUDRATE_PRESCALER
/* 当前SPI123为192MHz，16分频得到12MHz且不超过器件17MHz上限。 */
#define ADS8688_SPI_BAUDRATE_PRESCALER        SPI_BAUDRATEPRESCALER_16 /* SPI时钟分频。 */
#endif

#ifndef ADS8688_CS_GPIO_PORT
#define ADS8688_CS_GPIO_PORT                  GPIOA /* 片选端口。 */
#endif
#ifndef ADS8688_CS_PIN
#define ADS8688_CS_PIN                        GPIO_PIN_4 /* 片选引脚。 */
#endif

#ifndef ADS8688_SCK_GPIO_PORT
#define ADS8688_SCK_GPIO_PORT                 GPIOA /* SPI时钟端口。 */
#endif
#ifndef ADS8688_SCK_PIN
#define ADS8688_SCK_PIN                       GPIO_PIN_5 /* SPI时钟引脚。 */
#endif
#ifndef ADS8688_SCK_AF
#define ADS8688_SCK_AF                        GPIO_AF5_SPI1 /* SPI时钟复用功能。 */
#endif

#ifndef ADS8688_MISO_GPIO_PORT
#define ADS8688_MISO_GPIO_PORT                GPIOA /* SPI数据输入端口。 */
#endif
#ifndef ADS8688_MISO_PIN
#define ADS8688_MISO_PIN                      GPIO_PIN_6 /* SPI数据输入引脚。 */
#endif
#ifndef ADS8688_MISO_AF
#define ADS8688_MISO_AF                       GPIO_AF5_SPI1 /* SPI数据输入复用功能。 */
#endif

#ifndef ADS8688_MOSI_GPIO_PORT
#define ADS8688_MOSI_GPIO_PORT                GPIOB /* SPI数据输出端口。 */
#endif
#ifndef ADS8688_MOSI_PIN
#define ADS8688_MOSI_PIN                      GPIO_PIN_5 /* SPI数据输出引脚。 */
#endif
#ifndef ADS8688_MOSI_AF
#define ADS8688_MOSI_AF                       GPIO_AF5_SPI1 /* SPI数据输出复用功能。 */
#endif

#define ADS8688_CHANNEL_COUNT                 9U       /* 八路模拟通道与AUX总数。 */
#define ADS8688_ANALOG_CHANNEL_COUNT          8U       /* 可编程量程模拟通道数。 */
#define ADS8688_DEFAULT_VREF_MV               4096.0f  /* 默认参考电压，单位mV。 */
#define ADS8688_SPI_TIMEOUT_MS                2U       /* 单帧SPI超时，单位ms。 */
#define ADS8688_SOFT_SPI_DELAY_CYCLES         1U       /* 软件SPI边沿延时周期数。 */
#define ADS8688_MAX_THROUGHPUT_SPS            500000.0f /* 器件最大总吞吐率。 */

/* 快速帧模式直接访问SPI寄存器，适合连续波形采集。 */
#ifndef ADS8688_USE_FAST_SPI_FRAME
#define ADS8688_USE_FAST_SPI_FRAME            1U /* 1启用快速32时钟帧。 */
#endif

#ifndef ADS8688_ENABLE_LOW_LEVEL_DEBUG
#define ADS8688_ENABLE_LOW_LEVEL_DEBUG        1U /* 1启用底层SPI调试输出。 */
#endif

#ifndef ADS8688_LOW_LEVEL_DEBUG_LIMIT
#define ADS8688_LOW_LEVEL_DEBUG_LIMIT         80U /* 底层调试最大打印帧数。 */
#endif

#ifndef ADS8688_CS_SETUP_DELAY_CYCLES
#define ADS8688_CS_SETUP_DELAY_CYCLES         16U /* 片选建立延时周期数。 */
#endif

#ifndef ADS8688_CS_HOLD_DELAY_CYCLES
#define ADS8688_CS_HOLD_DELAY_CYCLES          16U /* 片选保持延时周期数。 */
#endif

/* ADS8688驱动状态。 */
typedef enum
{
  ADS8688_OK = 0,             /* 操作成功。 */
  ADS8688_ERROR,              /* 通用操作错误。 */
  ADS8688_ERROR_PARAM,        /* 输入参数无效。 */
  ADS8688_ERROR_SPI,          /* SPI通信失败。 */
  ADS8688_ERROR_SPI_CONFIG,   /* SPI配置不符合要求。 */
  ADS8688_ERROR_UNSUPPORTED   /* 当前模式不支持该操作。 */
} ADS8688_StatusTypeDef;

/* ADS8688采样通道。 */
typedef enum
{
  ADS8688_CHANNEL_0 = 0U, /* 模拟输入通道0。 */
  ADS8688_CHANNEL_1,      /* 模拟输入通道1。 */
  ADS8688_CHANNEL_2,      /* 模拟输入通道2。 */
  ADS8688_CHANNEL_3,      /* 模拟输入通道3。 */
  ADS8688_CHANNEL_4,      /* 模拟输入通道4。 */
  ADS8688_CHANNEL_5,      /* 模拟输入通道5。 */
  ADS8688_CHANNEL_6,      /* 模拟输入通道6。 */
  ADS8688_CHANNEL_7,      /* 模拟输入通道7。 */
  ADS8688_CHANNEL_AUX     /* 辅助输入通道。 */
} ADS8688_ChannelTypeDef;

/* ADS8688模拟输入量程。 */
typedef enum
{
  ADS8688_RANGE_BIPOLAR_10V24  = 0x00U, /* 双极性±2.5倍参考电压。 */
  ADS8688_RANGE_BIPOLAR_5V12   = 0x01U, /* 双极性±1.25倍参考电压。 */
  ADS8688_RANGE_BIPOLAR_2V56   = 0x02U, /* 双极性±0.625倍参考电压。 */
  ADS8688_RANGE_UNIPOLAR_10V24 = 0x05U, /* 单极性0～2.5倍参考电压。 */
  ADS8688_RANGE_UNIPOLAR_5V12  = 0x06U, /* 单极性0～1.25倍参考电压。 */
  ADS8688_RANGE_AUX_4V096      = 0xFFU  /* AUX的0～VREF换算标识。 */
} ADS8688_RangeTypeDef;

/* ADS8688单通道采样结果。 */
typedef struct
{
  ADS8688_ChannelTypeDef channel; /* 采样通道。 */
  ADS8688_RangeTypeDef range;     /* 当前量程。 */
  uint16_t code;                  /* 16位原始采样码。 */
  float voltage_mv;               /* 换算电压，单位mV。 */
} ADS8688_SampleTypeDef;

/* ADS8688总线、引脚和参考参数配置。 */
typedef struct
{
  SPI_HandleTypeDef *hspi;        /* 外部SPI句柄；NULL时由驱动初始化。 */
  SPI_TypeDef *spi_instance;      /* 硬件SPI实例。 */
  uint32_t spi_prescaler;         /* SPI时钟分频。 */
  uint32_t spi_timeout_ms;        /* SPI单帧超时，单位ms。 */

  GPIO_TypeDef *cs_port;          /* 片选端口。 */
  uint16_t cs_pin;                /* 片选引脚。 */

  GPIO_TypeDef *sck_port;         /* 时钟端口。 */
  uint16_t sck_pin;               /* 时钟引脚。 */
  uint8_t sck_af;                 /* 时钟复用功能。 */

  GPIO_TypeDef *miso_port;        /* 数据输入端口。 */
  uint16_t miso_pin;              /* 数据输入引脚。 */
  uint8_t miso_af;                /* 数据输入复用功能。 */

  GPIO_TypeDef *mosi_port;        /* 数据输出端口。 */
  uint16_t mosi_pin;              /* 数据输出引脚。 */
  uint8_t mosi_af;                /* 数据输出复用功能。 */

  uint16_t soft_spi_delay_cycles; /* 软件SPI边沿延时周期数。 */
  float vref_mv;                  /* 参考电压，单位mV。 */
} ADS8688_ConfigTypeDef;

/* ADS8688驱动句柄。 */
typedef struct
{
  ADS8688_ConfigTypeDef cfg; /* 当前驱动配置。 */
  SPI_HandleTypeDef *active_hspi; /* 实际使用的SPI句柄。 */
  ADS8688_RangeTypeDef ranges[ADS8688_ANALOG_CHANNEL_COUNT]; /* 各通道量程缓存。 */
  uint8_t auto_mask;         /* 自动扫描通道掩码。 */
  uint8_t auto_next_channel; /* 自动扫描下一通道。 */
  uint8_t initialized;       /* 驱动初始化标志。 */
} ADS8688_HandleTypeDef;

/* 获取ADS8688默认配置。 */
void ADS8688_GetDefaultConfig(ADS8688_ConfigTypeDef *cfg);
/* 初始化ADS8688驱动。 */
ADS8688_StatusTypeDef ADS8688_Init(ADS8688_HandleTypeDef *dev,
                                   const ADS8688_ConfigTypeDef *cfg);
/* 软件复位ADS8688。 */
ADS8688_StatusTypeDef ADS8688_Reset(ADS8688_HandleTypeDef *dev);
/* 使ADS8688进入待机模式。 */
ADS8688_StatusTypeDef ADS8688_Standby(ADS8688_HandleTypeDef *dev);
/* 使ADS8688进入掉电模式。 */
ADS8688_StatusTypeDef ADS8688_PowerDown(ADS8688_HandleTypeDef *dev);

/* 写入ADS8688程序寄存器。 */
ADS8688_StatusTypeDef ADS8688_WriteRegister(ADS8688_HandleTypeDef *dev,
                                            uint8_t address,
                                            uint8_t value);
/* 读取ADS8688程序寄存器。 */
ADS8688_StatusTypeDef ADS8688_ReadRegister(ADS8688_HandleTypeDef *dev,
                                           uint8_t address,
                                           uint8_t *value);

/* 设置指定模拟通道量程。 */
ADS8688_StatusTypeDef ADS8688_SetRange(ADS8688_HandleTypeDef *dev,
                                       ADS8688_ChannelTypeDef channel,
                                       ADS8688_RangeTypeDef range);
/* 设置全部模拟通道量程。 */
ADS8688_StatusTypeDef ADS8688_SetAllRanges(ADS8688_HandleTypeDef *dev,
                                           ADS8688_RangeTypeDef range);

/* 手动读取指定通道。 */
ADS8688_StatusTypeDef ADS8688_ReadChannel(ADS8688_HandleTypeDef *dev,
                                          ADS8688_ChannelTypeDef channel,
                                          ADS8688_SampleTypeDef *sample);
/* 按给定通道顺序完成一次手动扫描。 */
ADS8688_StatusTypeDef ADS8688_ReadManualSequence(ADS8688_HandleTypeDef *dev,
                                                 const ADS8688_ChannelTypeDef *channels,
                                                 uint8_t count,
                                                 ADS8688_SampleTypeDef *samples);
/* 依次读取CH0～CH7和AUX。 */
ADS8688_StatusTypeDef ADS8688_ReadAllChannels(ADS8688_HandleTypeDef *dev,
                                              ADS8688_SampleTypeDef samples[ADS8688_CHANNEL_COUNT]);

/* 连续采集单通道波形；采样率≤0时按总线最快速度运行。 */
ADS8688_StatusTypeDef ADS8688_CaptureChannel(ADS8688_HandleTypeDef *dev,
                                             ADS8688_ChannelTypeDef channel,
                                             uint16_t *codes,
                                             uint32_t sample_count,
                                             float sample_rate_hz,
                                             float *actual_sample_rate_hz);
/* 交织采集指定通道序列；返回值采样率为单通道实际速率。 */
ADS8688_StatusTypeDef ADS8688_CaptureSequence(ADS8688_HandleTypeDef *dev,
                                              const ADS8688_ChannelTypeDef *channels,
                                              uint8_t channel_count,
                                              uint16_t *codes,
                                              uint32_t sample_count,
                                              float sample_rate_hz,
                                              float *actual_sample_rate_hz);

/* 设置自动扫描通道掩码。 */
ADS8688_StatusTypeDef ADS8688_SetAutoSequence(ADS8688_HandleTypeDef *dev,
                                              uint8_t channel_mask);
/* 启动ADS8688自动扫描。 */
ADS8688_StatusTypeDef ADS8688_StartAutoScan(ADS8688_HandleTypeDef *dev);
/* 读取自动扫描的下一通道结果。 */
ADS8688_StatusTypeDef ADS8688_ReadAutoNext(ADS8688_HandleTypeDef *dev,
                                           ADS8688_SampleTypeDef *sample);

/* 将模拟通道采样码换算为电压。 */
float ADS8688_CodeToVoltageMv(uint16_t code,
                              ADS8688_RangeTypeDef range,
                              float vref_mv);
/* 将AUX采样码换算为电压。 */
float ADS8688_AuxCodeToVoltageMv(uint16_t code, float vref_mv);

#ifdef __cplusplus
}
#endif

#endif /* __ADS8688_H */
