#include "ADS8363.h"

#define ADS8363_CFG_C_SHIFT              14U    /* 差分通道字段位移。 */
#define ADS8363_CFG_R_SHIFT              12U    /* 寄存器更新字段位移。 */
#define ADS8363_CFG_PD_SHIFT             10U    /* 电源模式字段位移。 */
#define ADS8363_CFG_R_UPDATE_CHANNEL     (0U << ADS8363_CFG_R_SHIFT) /* 仅更新当前通道。 */
#define ADS8363_CFG_R_UPDATE_ALL         (1U << ADS8363_CFG_R_SHIFT) /* 更新全部通道。 */
#define ADS8363_CFG_NORMAL_POWER         (0U << ADS8363_CFG_PD_SHIFT) /* 正常工作模式。 */
#define ADS8363_CFG_CID                  (1U << 5) /* 启用通道标识输出。 */
#define ADS8363_ACCESS_CONFIG_ONLY       0x0U   /* 仅访问配置寄存器。 */
#define ADS8363_ACCESS_WRITE_REFDAC1     0x2U   /* 写参考DAC1访问码。 */
#define ADS8363_ACCESS_WRITE_REFDAC2     0x5U   /* 写参考DAC2访问码。 */
#define ADS8363_ACCESS_RESET             0x4U   /* 软件复位访问码。 */
#define ADS8363_REFDAC_2V5_ENABLE        0x03FFU /* 内部2.5V参考使能值。 */
#define ADS8363_REFDAC_2V5_DISABLE       0x07FFU /* 内部参考关闭值。 */
#define ADS8363_FRAME_TRAILING_BITS      4U     /* SPI帧尾部空位数。 */

/* 检查ADS8363句柄配置。 */
static ADS8363_StatusTypeDef ADS8363_CheckConfig(ADS8363_HandleTypeDef *dev);
/* 检查SPI外设配置是否满足ADS8363时序。 */
static ADS8363_StatusTypeDef ADS8363_CheckSpiConfig(SPI_HandleTypeDef *hspi);
/* 写入ADS8363配置字。 */
static ADS8363_StatusTypeDef ADS8363_WriteConfig(ADS8363_HandleTypeDef *dev,
                                                 ADS8363_DiffChannelTypeDef channel,
                                                 uint16_t access);
/* 写入ADS8363内部寄存器。 */
static ADS8363_StatusTypeDef ADS8363_WriteRegister(ADS8363_HandleTypeDef *dev,
                                                   uint16_t access,
                                                   uint16_t value);
/* 预填充ADS8363转换流水线。 */
static ADS8363_StatusTypeDef ADS8363_PrimePipeline(ADS8363_HandleTypeDef *dev);
/* 完成一次32位SPI帧交换。 */
static ADS8363_StatusTypeDef ADS8363_TransferFrame(ADS8363_HandleTypeDef *dev,
                                                   uint16_t tx_word,
                                                   uint16_t *rx_word);
/* 等待BUSY达到指定电平。 */
static ADS8363_StatusTypeDef ADS8363_WaitBusyState(ADS8363_HandleTypeDef *dev,
                                                   GPIO_PinState state);
/* 将16位命令字对齐到32位SPI帧。 */
static uint32_t ADS8363_WordToFrame(uint16_t word);
/* 从32位SPI帧提取16位数据字。 */
static uint16_t ADS8363_FrameToWord(uint32_t frame);
/* 组装ADS8363配置字。 */
static uint16_t ADS8363_BuildConfigWord(ADS8363_DiffChannelTypeDef channel,
                                        uint16_t r_bits,
                                        uint16_t access);
/* 提供转换启动脉冲短延时。 */
static void ADS8363_DelayCycles(uint16_t cycles);
/* 启用DWT周期计数器。 */
static void ADS8363_EnableCycleCounter(void);

/* 填充ADS8363默认配置。 */
void ADS8363_GetDefaultConfig(ADS8363_ConfigTypeDef *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  cfg->hspi = NULL;
  cfg->cs_port = NULL;
  cfg->cs_pin = 0U;
  cfg->rd_port = NULL;
  cfg->rd_pin = 0U;
  cfg->busy_port = NULL;
  cfg->busy_pin = 0U;
  cfg->spi_timeout_ms = ADS8363_SPI_TIMEOUT_MS;
  cfg->busy_timeout_us = ADS8363_BUSY_TIMEOUT_US;
  cfg->pulse_cycles = ADS8363_CONVST_PULSE_CYCLES;
  cfg->vref_mv = ADS8363_DEFAULT_VREF_MV;
}

/* 初始化ADS8363并预填充转换流水线。 */
ADS8363_StatusTypeDef ADS8363_Init(ADS8363_HandleTypeDef *dev,
                                   const ADS8363_ConfigTypeDef *cfg)
{
  ADS8363_StatusTypeDef status;

  if ((dev == NULL) || (cfg == NULL))
  {
    return ADS8363_ERROR_PARAM;
  }

  dev->cfg = *cfg;
  dev->channel = ADS8363_DIFF_CH0;
  dev->pipeline_valid_frames = 0U;
  ADS8363_EnableCycleCounter();

  status = ADS8363_CheckConfig(dev);
  if (status != ADS8363_OK)
  {
    return status;
  }

  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(dev->cfg.rd_port, dev->cfg.rd_pin, GPIO_PIN_RESET);

  status = ADS8363_Reset(dev);
  if (status != ADS8363_OK)
  {
    return status;
  }

  status = ADS8363_WriteConfig(dev, ADS8363_DIFF_CH0, ADS8363_ACCESS_CONFIG_ONLY);
  if (status != ADS8363_OK)
  {
    return status;
  }

  status = ADS8363_PrimePipeline(dev);
  if (status != ADS8363_OK)
  {
    return status;
  }

  return ADS8363_OK;
}

/* 发送ADS8363软件复位命令。 */
ADS8363_StatusTypeDef ADS8363_Reset(ADS8363_HandleTypeDef *dev)
{
  ADS8363_StatusTypeDef status;
  uint16_t dummy;

  if (dev == NULL)
  {
    return ADS8363_ERROR_PARAM;
  }

  status = ADS8363_TransferFrame(dev,
                                 ADS8363_BuildConfigWord(ADS8363_DIFF_CH0,
                                                         ADS8363_CFG_R_UPDATE_ALL,
                                                         ADS8363_ACCESS_RESET),
                                 &dummy);
  if (status != ADS8363_OK)
  {
    return status;
  }

  HAL_Delay(1U);
  dev->channel = ADS8363_DIFF_CH0;
  dev->pipeline_valid_frames = 0U;
  return ADS8363_OK;
}

/* 设置ADS8363 A侧差分通道。 */
ADS8363_StatusTypeDef ADS8363_SetDiffChannel(ADS8363_HandleTypeDef *dev,
                                             ADS8363_DiffChannelTypeDef channel)
{
  ADS8363_StatusTypeDef status;
  uint16_t dummy;

  if ((dev == NULL) ||
      ((channel != ADS8363_DIFF_CH0) && (channel != ADS8363_DIFF_CH1)))
  {
    return ADS8363_ERROR_PARAM;
  }

  status = ADS8363_TransferFrame(dev,
                                 ADS8363_BuildConfigWord(channel,
                                                         ADS8363_CFG_R_UPDATE_CHANNEL,
                                                         ADS8363_ACCESS_CONFIG_ONLY),
                                 &dummy);
  if (status != ADS8363_OK)
  {
    return status;
  }

  dev->channel = channel;
  dev->pipeline_valid_frames = 0U;
  return ADS8363_OK;
}

/* 使能或关闭ADS8363内部参考源。 */
ADS8363_StatusTypeDef ADS8363_EnableInternalReference(ADS8363_HandleTypeDef *dev,
                                                      uint8_t enable)
{
  ADS8363_StatusTypeDef status;
  uint16_t refdac;

  if (dev == NULL)
  {
    return ADS8363_ERROR_PARAM;
  }

  refdac = (enable != 0U) ? ADS8363_REFDAC_2V5_ENABLE : ADS8363_REFDAC_2V5_DISABLE;

  status = ADS8363_WriteRegister(dev, ADS8363_ACCESS_WRITE_REFDAC1, refdac);
  if (status != ADS8363_OK)
  {
    return status;
  }

  status = ADS8363_WriteRegister(dev, ADS8363_ACCESS_WRITE_REFDAC2, refdac);
  if (status != ADS8363_OK)
  {
    return status;
  }

  if (enable != 0U)
  {
    HAL_Delay(8U);
  }

  return ADS8363_OK;
}

/* 切换通道并读取一帧有效采样。 */
ADS8363_StatusTypeDef ADS8363_ReadPair(ADS8363_HandleTypeDef *dev,
                                       ADS8363_DiffChannelTypeDef channel,
                                       ADS8363_SamplePairTypeDef *sample)
{
  ADS8363_StatusTypeDef status;

  if ((dev == NULL) || (sample == NULL) ||
      ((channel != ADS8363_DIFF_CH0) && (channel != ADS8363_DIFF_CH1)))
  {
    return ADS8363_ERROR_PARAM;
  }

  if (dev->channel != channel)
  {
    status = ADS8363_SetDiffChannel(dev, channel);
    if (status != ADS8363_OK)
    {
      return status;
    }
  }

  status = ADS8363_PrimePipeline(dev);
  if (status != ADS8363_OK)
  {
    return status;
  }

  return ADS8363_ReadPairContinuous(dev, sample);
}

/* 连续读取当前通道采样。 */
ADS8363_StatusTypeDef ADS8363_ReadPairContinuous(ADS8363_HandleTypeDef *dev,
                                                 ADS8363_SamplePairTypeDef *sample)
{
  ADS8363_StatusTypeDef status;
  uint16_t raw_a;

  if ((dev == NULL) || (sample == NULL))
  {
    return ADS8363_ERROR_PARAM;
  }

  if ((dev->channel != ADS8363_DIFF_CH0) && (dev->channel != ADS8363_DIFF_CH1))
  {
    return ADS8363_ERROR_PARAM;
  }

  status = ADS8363_PrimePipeline(dev);
  if (status != ADS8363_OK)
  {
    return status;
  }

  status = ADS8363_TransferFrame(dev,
                                 ADS8363_BuildConfigWord(dev->channel,
                                                         ADS8363_CFG_R_UPDATE_CHANNEL,
                                                         ADS8363_ACCESS_CONFIG_ONLY),
                                 &raw_a);
  if (status != ADS8363_OK)
  {
    return status;
  }

  sample->a = (int16_t)raw_a;
  sample->b = 0;
  sample->b_valid = 0U;
  dev->pipeline_valid_frames = 2U;

  return ADS8363_OK;
}

/* 将ADS8363有符号码换算为差分电压。 */
float ADS8363_CodeToVoltageMv(int16_t code, float vref_mv)
{
  return ((float)code * vref_mv) / 32768.0f;
}

/* 检查ADS8363句柄配置。 */
static ADS8363_StatusTypeDef ADS8363_CheckConfig(ADS8363_HandleTypeDef *dev)
{
  if ((dev->cfg.hspi == NULL) ||
      (dev->cfg.cs_port == NULL) ||
      (dev->cfg.cs_pin == 0U) ||
      (dev->cfg.rd_port == NULL) ||
      (dev->cfg.rd_pin == 0U) ||
      (dev->cfg.busy_port == NULL) ||
      (dev->cfg.busy_pin == 0U) ||
      (dev->cfg.spi_timeout_ms == 0U) ||
      (dev->cfg.busy_timeout_us == 0U) ||
      (dev->cfg.pulse_cycles == 0U) ||
      (dev->cfg.vref_mv <= 0.0f))
  {
    return ADS8363_ERROR_PARAM;
  }

  return ADS8363_CheckSpiConfig(dev->cfg.hspi);
}

/* 检查SPI外设配置是否满足ADS8363时序。 */
static ADS8363_StatusTypeDef ADS8363_CheckSpiConfig(SPI_HandleTypeDef *hspi)
{
  if (hspi == NULL)
  {
    return ADS8363_ERROR_PARAM;
  }

  if ((hspi->Init.Mode != SPI_MODE_MASTER) ||
      (hspi->Init.Direction != SPI_DIRECTION_2LINES) ||
      (hspi->Init.DataSize != SPI_DATASIZE_20BIT) ||
      (hspi->Init.CLKPolarity != SPI_POLARITY_LOW) ||
      (hspi->Init.CLKPhase != SPI_PHASE_2EDGE) ||
      (hspi->Init.FirstBit != SPI_FIRSTBIT_MSB))
  {
    return ADS8363_ERROR_SPI_CONFIG;
  }

  return ADS8363_OK;
}

/* 写入ADS8363配置字。 */
static ADS8363_StatusTypeDef ADS8363_WriteConfig(ADS8363_HandleTypeDef *dev,
                                                 ADS8363_DiffChannelTypeDef channel,
                                                 uint16_t access)
{
  ADS8363_StatusTypeDef status;
  uint16_t dummy;

  status = ADS8363_TransferFrame(dev,
                                 ADS8363_BuildConfigWord(channel,
                                                         ADS8363_CFG_R_UPDATE_ALL,
                                                         access),
                                 &dummy);
  if (status == ADS8363_OK)
  {
    dev->channel = channel;
    dev->pipeline_valid_frames = 0U;
  }

  return status;
}

/* 写入ADS8363内部寄存器。 */
static ADS8363_StatusTypeDef ADS8363_WriteRegister(ADS8363_HandleTypeDef *dev,
                                                   uint16_t access,
                                                   uint16_t value)
{
  ADS8363_StatusTypeDef status;
  uint16_t dummy;

  status = ADS8363_WriteConfig(dev, dev->channel, access);
  if (status != ADS8363_OK)
  {
    return status;
  }

  status = ADS8363_TransferFrame(dev, value, &dummy);
  if (status != ADS8363_OK)
  {
    return status;
  }

  dev->pipeline_valid_frames = 0U;
  return ADS8363_OK;
}

/* 预填充ADS8363转换流水线。 */
static ADS8363_StatusTypeDef ADS8363_PrimePipeline(ADS8363_HandleTypeDef *dev)
{
  ADS8363_StatusTypeDef status;
  uint16_t dummy;

  if (dev == NULL)
  {
    return ADS8363_ERROR_PARAM;
  }

  while (dev->pipeline_valid_frames < 2U)
  {
    status = ADS8363_TransferFrame(dev,
                                   ADS8363_BuildConfigWord(dev->channel,
                                                           ADS8363_CFG_R_UPDATE_CHANNEL,
                                                           ADS8363_ACCESS_CONFIG_ONLY),
                                   &dummy);
    if (status != ADS8363_OK)
    {
      return status;
    }

    dev->pipeline_valid_frames++;
  }

  return ADS8363_OK;
}

/* 完成一次32位SPI帧交换。 */
static ADS8363_StatusTypeDef ADS8363_TransferFrame(ADS8363_HandleTypeDef *dev,
                                                   uint16_t tx_word,
                                                   uint16_t *rx_word)
{
  uint32_t tx_frame;
  uint32_t rx_frame;
  HAL_StatusTypeDef hal_status;

  if ((dev == NULL) || (rx_word == NULL))
  {
    return ADS8363_ERROR_PARAM;
  }

  if (ADS8363_WaitBusyState(dev, GPIO_PIN_RESET) != ADS8363_OK)
  {
    return ADS8363_ERROR_BUSY_TIMEOUT;
  }

  tx_frame = ADS8363_WordToFrame(tx_word);
  rx_frame = 0U;
  *rx_word = 0U;

  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(dev->cfg.rd_port, dev->cfg.rd_pin, GPIO_PIN_SET);
  ADS8363_DelayCycles(dev->cfg.pulse_cycles);
  HAL_GPIO_WritePin(dev->cfg.rd_port, dev->cfg.rd_pin, GPIO_PIN_RESET);

  if (ADS8363_WaitBusyState(dev, GPIO_PIN_SET) != ADS8363_OK)
  {
    HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);
    return ADS8363_ERROR_BUSY_TIMEOUT;
  }

  hal_status = HAL_SPI_TransmitReceive(dev->cfg.hspi,
                                       (uint8_t *)&tx_frame,
                                       (uint8_t *)&rx_frame,
                                       1U,
                                       dev->cfg.spi_timeout_ms);
  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);

  if (hal_status != HAL_OK)
  {
    return ADS8363_ERROR_SPI;
  }

  if (ADS8363_WaitBusyState(dev, GPIO_PIN_RESET) != ADS8363_OK)
  {
    return ADS8363_ERROR_BUSY_TIMEOUT;
  }

  *rx_word = ADS8363_FrameToWord(rx_frame);
  return ADS8363_OK;
}

/* 等待BUSY达到指定电平。 */
static ADS8363_StatusTypeDef ADS8363_WaitBusyState(ADS8363_HandleTypeDef *dev,
                                                   GPIO_PinState state)
{
  uint32_t start_cycles;
  uint32_t timeout_cycles;

  if (dev == NULL)
  {
    return ADS8363_ERROR_PARAM;
  }

  start_cycles = DWT->CYCCNT;
  timeout_cycles = (HAL_RCC_GetSysClockFreq() / 1000000U) * dev->cfg.busy_timeout_us;

  while (HAL_GPIO_ReadPin(dev->cfg.busy_port, dev->cfg.busy_pin) != state)
  {
    if ((DWT->CYCCNT - start_cycles) > timeout_cycles)
    {
      return ADS8363_ERROR_BUSY_TIMEOUT;
    }
  }

  return ADS8363_OK;
}

/* 将16位命令字对齐到32位SPI帧。 */
static uint32_t ADS8363_WordToFrame(uint16_t word)
{
  return ((uint32_t)word << ADS8363_FRAME_TRAILING_BITS) & 0x000FFFF0U;
}

/* 从32位SPI帧提取16位数据字。 */
static uint16_t ADS8363_FrameToWord(uint32_t frame)
{
  return (uint16_t)((frame >> ADS8363_FRAME_TRAILING_BITS) & 0xFFFFU);
}

/* 组装ADS8363配置字。 */
static uint16_t ADS8363_BuildConfigWord(ADS8363_DiffChannelTypeDef channel,
                                        uint16_t r_bits,
                                        uint16_t access)
{
  uint16_t c_bits;

  c_bits = ((uint16_t)channel & 0x3U) << ADS8363_CFG_C_SHIFT;

  return (uint16_t)(c_bits |
                    r_bits |
                    ADS8363_CFG_NORMAL_POWER |
                    ADS8363_CFG_CID |
                    (access & 0xFU));
}

/* 提供转换启动脉冲短延时。 */
static void ADS8363_DelayCycles(uint16_t cycles)
{
  while (cycles > 0U)
  {
    __NOP();
    cycles--;
  }
}

/* 启用DWT周期计数器。 */
static void ADS8363_EnableCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
