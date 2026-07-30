#ifndef __ADS8363_H
#define __ADS8363_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define ADS8363_CS_PORT                 GPIOB        /* ADS8363片选CS端口 */
#define ADS8363_CS_PIN                  GPIO_PIN_12  /* ADS8363片选CS引脚, 低电平有效 */
#define ADS8363_RD_CONVST_PORT          GPIOH        /* ADS8363转换启动RD/CONVST端口 */
#define ADS8363_RD_CONVST_PIN           GPIO_PIN_8   /* ADS8363转换启动RD/CONVST引脚, 上升沿启动 */
#define ADS8363_BUSY_PORT               GPIOH        /* ADS8363忙信号BUSY端口 */
#define ADS8363_BUSY_PIN                GPIO_PIN_9   /* ADS8363忙信号BUSY引脚, 高电平表示转换中 */
#define ADS8363_DEFAULT_VREF_MV         2500.0f      /* ADS8363默认外部参考电压, 测试板REF5025/VREF2V5_1约2500mV */
#define ADS8363_SPI_TIMEOUT_MS          2U           /* ADS8363单帧SPI通信超时, 单位ms */
#define ADS8363_BUSY_TIMEOUT_US         100U         /* ADS8363等待BUSY状态超时, 单位us */
#define ADS8363_CONVST_PULSE_CYCLES     32U          /* ADS8363启动转换脉冲保持时间, 单位CPU空循环 */

/* ADS8363驱动状态。 */
typedef enum
{
  ADS8363_OK = 0,              /* ADS8363操作成功 */
  ADS8363_ERROR,               /* ADS8363通用错误 */
  ADS8363_ERROR_PARAM,         /* ADS8363参数错误 */
  ADS8363_ERROR_BUSY_TIMEOUT,  /* ADS8363等待BUSY超时 */
  ADS8363_ERROR_SPI_CONFIG,    /* ADS8363所需SPI2配置不正确 */
  ADS8363_ERROR_SPI            /* ADS8363 SPI通信失败 */
} ADS8363_StatusTypeDef;

/* ADS8363 A侧差分通道选择。 */
typedef enum
{
  ADS8363_DIFF_CH0 = 0U,       /* ADS8363 A侧差分通道0, PCB丝印ADC-A1减ADC-A0 */
  ADS8363_DIFF_CH1 = 3U        /* ADS8363 A侧差分通道1, PCB丝印A3仅偏置减ADC-A2 */
} ADS8363_DiffChannelTypeDef;

/* ADS8363一次双通道采样结果。 */
typedef struct
{
  int16_t a;                   /* ADS8363 SDOA读到的A侧有符号采样码 */
  int16_t b;                   /* ADS8363 B侧采样码, 当前未启用SDOB/SPI5时无效 */
  uint8_t b_valid;             /* B侧数据有效标志, 当前默认始终为0 */
} ADS8363_SamplePairTypeDef;

/* ADS8363接口与换算参数配置。 */
typedef struct
{
  SPI_HandleTypeDef *hspi;     /* ADS8363使用的SPI句柄, 主工程为hspi2 */
  GPIO_TypeDef *cs_port;       /* ADS8363 CS端口 */
  uint16_t cs_pin;             /* ADS8363 CS引脚 */
  GPIO_TypeDef *rd_port;       /* ADS8363 RD/CONVST端口 */
  uint16_t rd_pin;             /* ADS8363 RD/CONVST引脚 */
  GPIO_TypeDef *busy_port;     /* ADS8363 BUSY端口 */
  uint16_t busy_pin;           /* ADS8363 BUSY引脚 */
  uint32_t spi_timeout_ms;     /* SPI单帧超时, 单位ms */
  uint32_t busy_timeout_us;    /* BUSY等待超时, 单位us */
  uint16_t pulse_cycles;       /* RD/CONVST脉冲保持空循环数 */
  float vref_mv;               /* 参考电压, 单位mV, 只影响码值到电压的换算 */
} ADS8363_ConfigTypeDef;

/* ADS8363驱动句柄。 */
typedef struct
{
  ADS8363_ConfigTypeDef cfg;             /* ADS8363底层配置 */
  ADS8363_DiffChannelTypeDef channel;    /* 当前差分通道 */
  uint8_t pipeline_valid_frames;         /* 流水线已预填充帧数 */
} ADS8363_HandleTypeDef;

/************************************************************
 * Function :       ADS8363_GetDefaultConfig
 * Comment  :       获取ADS8363默认配置参数
 * Parameter:       cfg: 配置结构体指针
 * Return   :       null
 * Date     :       2026-06-13 V1
************************************************************/
void ADS8363_GetDefaultConfig(ADS8363_ConfigTypeDef *cfg);

/************************************************************
 * Function :       ADS8363_Init
 * Comment  :       初始化ADS8363并预填充流水线
 * Parameter:       dev: ADS8363句柄, cfg: 配置结构体
 * Return   :       ADS8363状态
 * Date     :       2026-06-13 V1
************************************************************/
ADS8363_StatusTypeDef ADS8363_Init(ADS8363_HandleTypeDef *dev,
                                   const ADS8363_ConfigTypeDef *cfg);

/************************************************************
 * Function :       ADS8363_Reset
 * Comment  :       发送ADS8363软件复位命令
 * Parameter:       dev: ADS8363句柄
 * Return   :       ADS8363状态
 * Date     :       2026-06-13 V1
************************************************************/
ADS8363_StatusTypeDef ADS8363_Reset(ADS8363_HandleTypeDef *dev);

/************************************************************
 * Function :       ADS8363_SetDiffChannel
 * Comment  :       设置ADS8363差分采样通道
 * Parameter:       dev: ADS8363句柄, channel: 差分通道
 * Return   :       ADS8363状态
 * Date     :       2026-06-13 V1
************************************************************/
ADS8363_StatusTypeDef ADS8363_SetDiffChannel(ADS8363_HandleTypeDef *dev,
                                             ADS8363_DiffChannelTypeDef channel);

/************************************************************
 * Function :       ADS8363_EnableInternalReference
 * Comment  :       使能或关闭ADS8363内部参考源, 当前硬件默认使用REF5025外部2.5V参考
 * Parameter:       dev: ADS8363句柄, enable: 0关闭, 非0使能
 * Return   :       ADS8363状态
 * Date     :       2026-06-13 V1
************************************************************/
ADS8363_StatusTypeDef ADS8363_EnableInternalReference(ADS8363_HandleTypeDef *dev,
                                                      uint8_t enable);

/************************************************************
 * Function :       ADS8363_ReadPair
 * Comment  :       切换通道后读取ADS8363一帧有效采样
 * Parameter:       dev: ADS8363句柄, channel: 差分通道, sample: 采样结果
 * Return   :       ADS8363状态
 * Date     :       2026-06-13 V1
************************************************************/
ADS8363_StatusTypeDef ADS8363_ReadPair(ADS8363_HandleTypeDef *dev,
                                       ADS8363_DiffChannelTypeDef channel,
                                       ADS8363_SamplePairTypeDef *sample);

/************************************************************
 * Function :       ADS8363_ReadPairContinuous
 * Comment  :       连续读取ADS8363当前通道采样
 * Parameter:       dev: ADS8363句柄, sample: 采样结果
 * Return   :       ADS8363状态
 * Date     :       2026-06-13 V1
************************************************************/
ADS8363_StatusTypeDef ADS8363_ReadPairContinuous(ADS8363_HandleTypeDef *dev,
                                                 ADS8363_SamplePairTypeDef *sample);

/************************************************************
 * Function :       ADS8363_CodeToVoltageMv
 * Comment  :       将ADS8363有符号码值换算为差分电压
 * Parameter:       code: ADC码值, vref_mv: 参考电压mV
 * Return   :       电压值, 单位mV
 * Date     :       2026-06-13 V1
************************************************************/
float ADS8363_CodeToVoltageMv(int16_t code, float vref_mv);

#ifdef __cplusplus
}
#endif

#endif /* __ADS8363_H */
