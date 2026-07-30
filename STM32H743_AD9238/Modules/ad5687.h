#ifndef __AD5687_H
#define __AD5687_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

#define AD5687_RESOLUTION_BITS     12U       /* AD5687 DAC分辨率位数 */
#define AD5687_MAX_CODE            0x0FFFU   /* AD5687最大12位DAC码值 */
#define AD5687_SOFT_SPI_DELAY_CYCLES  1U    /* AD5687软件SPI每个时序阶段的延时周期 */

/* AD5687级联器件选择。 */
typedef enum
{
  AD5687_DEVICE_CV1_CV2 = 0U, /* 近端AD5687，对应测试板CV1和CV2 */
  AD5687_DEVICE_CV3_CV4,      /* 远端AD5687，对应测试板CV3和CV4 */
  AD5687_DEVICE_ALL            /* 两片AD5687同时执行相同命令 */
} AD5687_DeviceTypeDef;

/* AD5687输出通道选择。 */
typedef enum
{
  AD5687_CHANNEL_A   = 0x01U,
  AD5687_CHANNEL_B   = 0x08U,
  AD5687_CHANNEL_ALL = 0x09U
} AD5687_ChannelTypeDef;

/* AD5687掉电模式。 */
typedef enum
{
  AD5687_POWER_NORMAL      = 0x00U,
  AD5687_POWER_1K_TO_GND   = 0x01U,
  AD5687_POWER_100K_TO_GND = 0x02U,
  AD5687_POWER_THREE_STATE = 0x03U
} AD5687_PowerModeTypeDef;

/* AD5687软件SPI驱动句柄。 */
typedef struct
{
  GPIO_TypeDef *sck_port;      /* 串行时钟端口。 */
  uint16_t sck_pin;            /* 串行时钟引脚。 */
  GPIO_TypeDef *mosi_port;     /* 串行数据端口。 */
  uint16_t mosi_pin;           /* 串行数据引脚。 */
  GPIO_TypeDef *cs_port;       /* 级联片选端口。 */
  uint16_t cs_pin;             /* 级联片选引脚。 */
  uint16_t spi_delay_cycles;   /* 软件SPI边沿延时周期数。 */
  uint32_t vref_mv;            /* 参考电压，单位mV。 */
  uint8_t gain;                /* 输出增益，仅支持1或2。 */
} AD5687_HandleTypeDef;

/* 初始化AD5687级联链路。 */
HAL_StatusTypeDef AD5687_Init(AD5687_HandleTypeDef *dev);
/* 写入指定器件和通道的输入寄存器。 */
HAL_StatusTypeDef AD5687_WriteInputRaw(AD5687_HandleTypeDef *dev,
                                       AD5687_DeviceTypeDef device,
                                       AD5687_ChannelTypeDef channel,
                                       uint16_t code);
/* 将指定通道输入寄存器更新到输出。 */
HAL_StatusTypeDef AD5687_Update(AD5687_HandleTypeDef *dev,
                                AD5687_DeviceTypeDef device,
                                AD5687_ChannelTypeDef channel);
/* 写入指定通道并立即更新输出。 */
HAL_StatusTypeDef AD5687_WriteAndUpdateRaw(AD5687_HandleTypeDef *dev,
                                           AD5687_DeviceTypeDef device,
                                           AD5687_ChannelTypeDef channel,
                                           uint16_t code);
/* 按毫伏值设置指定通道输出。 */
HAL_StatusTypeDef AD5687_SetVoltageMv(AD5687_HandleTypeDef *dev,
                                      AD5687_DeviceTypeDef device,
                                      AD5687_ChannelTypeDef channel,
                                      uint32_t voltage_mv);
/* 同时设置一片AD5687的两个输出通道。 */
HAL_StatusTypeDef AD5687_SetBothVoltageMv(AD5687_HandleTypeDef *dev,
                                          AD5687_DeviceTypeDef device,
                                          uint32_t channel_a_mv,
                                          uint32_t channel_b_mv);
/* 设置一片或两片AD5687的掉电模式。 */
HAL_StatusTypeDef AD5687_SetPowerMode(AD5687_HandleTypeDef *dev,
                                      AD5687_DeviceTypeDef device,
                                      AD5687_PowerModeTypeDef channel_a_mode,
                                      AD5687_PowerModeTypeDef channel_b_mode);
/* 软件复位指定AD5687。 */
HAL_StatusTypeDef AD5687_SoftwareReset(AD5687_HandleTypeDef *dev,
                                       AD5687_DeviceTypeDef device);
/* 将目标电压换算为12位DAC码。 */
uint16_t AD5687_VoltageToCodeMv(const AD5687_HandleTypeDef *dev,
                                uint32_t voltage_mv);
/* 将12位DAC码换算为输出电压。 */
uint32_t AD5687_CodeToVoltageMv(const AD5687_HandleTypeDef *dev,
                                uint16_t code);

#ifdef __cplusplus
}
#endif

#endif /* __AD5687_H */
