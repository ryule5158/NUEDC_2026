#include "ADS8688.h"
#include "stm32h7xx_ll_spi.h"
#include <stddef.h>
#include <stdio.h>

#define ADS8688_CMD_NO_OP             0x0000U /* 空操作命令。 */
#define ADS8688_CMD_STDBY             0x8200U /* 进入待机命令。 */
#define ADS8688_CMD_PWR_DN            0x8300U /* 进入掉电命令。 */
#define ADS8688_CMD_RST               0x8500U /* 软件复位命令。 */
#define ADS8688_CMD_AUTO_RST          0xA000U /* 启动自动扫描命令。 */
#define ADS8688_CMD_MAN_CH_BASE       0xC000U /* 手动通道命令基值。 */
#define ADS8688_CMD_MAN_AUX           0xE000U /* 手动AUX通道命令。 */

#define ADS8688_REG_AUTO_SEQ_EN       0x01U /* 自动扫描使能寄存器。 */
#define ADS8688_REG_FEATURE_SELECT    0x03U /* 特性选择寄存器。 */
#define ADS8688_REG_RANGE_CH0         0x05U /* 通道0量程寄存器首地址。 */

#define ADS8688_REG_WRITE_BIT         0x0100U /* 寄存器写操作标志。 */
#define ADS8688_COMMAND_FRAME_BITS    32U     /* 普通命令帧位数。 */
#define ADS8688_REGISTER_FRAME_BITS   24U     /* 寄存器访问帧位数。 */

static SPI_HandleTypeDef s_ads8688_hspi; /* 驱动内部硬件SPI句柄。 */
static uint32_t s_ads8688_debug_count;   /* 已打印的底层调试帧数。 */

/* 检查ADS8688配置是否完整。 */
static ADS8688_StatusTypeDef ADS8688_CheckConfig(const ADS8688_ConfigTypeDef *cfg);
/* 按配置初始化ADS8688总线。 */
static ADS8688_StatusTypeDef ADS8688_InitBus(ADS8688_HandleTypeDef *dev);
/* 初始化驱动内部硬件SPI。 */
static ADS8688_StatusTypeDef ADS8688_InitHardwareSpi(ADS8688_HandleTypeDef *dev);
/* 检查外部硬件SPI配置。 */
static ADS8688_StatusTypeDef ADS8688_CheckHardwareSpi(SPI_HandleTypeDef *hspi);
/* 配置硬件SPI复用引脚。 */
static void ADS8688_InitHardwarePins(const ADS8688_ConfigTypeDef *cfg);
/* 配置软件SPI GPIO。 */
static void ADS8688_InitSoftwarePins(const ADS8688_ConfigTypeDef *cfg);
/* 使能指定GPIO端口时钟。 */
static void ADS8688_EnableGpioClock(GPIO_TypeDef *port);
/* 使能指定SPI外设时钟。 */
static ADS8688_StatusTypeDef ADS8688_EnableSpiClock(SPI_TypeDef *instance);
/* 发送普通命令帧并接收采样码。 */
static ADS8688_StatusTypeDef ADS8688_TransferCommand(ADS8688_HandleTypeDef *dev,
                                                     uint16_t command,
                                                     uint16_t *code);
/* 发送连续采集命令帧。 */
static ADS8688_StatusTypeDef ADS8688_TransferCaptureCommand(ADS8688_HandleTypeDef *dev,
                                                            uint16_t command,
                                                            uint16_t *code);
/* 通过寄存器直访快速发送命令帧。 */
static ADS8688_StatusTypeDef ADS8688_HardwareTransferCommandFast(ADS8688_HandleTypeDef *dev,
                                                                 uint16_t command,
                                                                 uint16_t *code);
/* 完成一次程序寄存器访问。 */
static ADS8688_StatusTypeDef ADS8688_TransferRegister(ADS8688_HandleTypeDef *dev,
                                                      uint16_t command,
                                                      uint8_t *value);
/* 通过HAL硬件SPI交换字节流。 */
static ADS8688_StatusTypeDef ADS8688_HardwareTransfer(ADS8688_HandleTypeDef *dev,
                                                      const uint8_t *tx,
                                                      uint8_t *rx,
                                                      uint16_t size);
/* 通过GPIO软件SPI交换指定位数。 */
static ADS8688_StatusTypeDef ADS8688_SoftwareTransfer(ADS8688_HandleTypeDef *dev,
                                                      uint32_t tx,
                                                      uint8_t bits,
                                                      uint32_t *rx);
/* 拉低ADS8688片选并满足建立时间。 */
static void ADS8688_Select(const ADS8688_HandleTypeDef *dev);
/* 满足保持时间后拉高ADS8688片选。 */
static void ADS8688_Deselect(const ADS8688_HandleTypeDef *dev);
/* 提供GPIO时序短延时。 */
static void ADS8688_DelayCycles(uint16_t cycles);
/* 启用DWT周期计数器。 */
static void ADS8688_EnableCycleCounter(void);
/* 将采样率换算为CPU周期。 */
static ADS8688_StatusTypeDef ADS8688_GetSamplePeriodCycles(float sample_rate_hz,
                                                           uint32_t *period_cycles);
/* 等待到指定DWT周期时刻。 */
static void ADS8688_WaitUntilCycle(uint32_t target_cycle);
/* 计算块采集实际单通道采样率。 */
static void ADS8688_SetActualSampleRate(uint32_t start_cycle,
                                        uint32_t end_cycle,
                                        uint32_t sample_count,
                                        float *actual_sample_rate_hz);
/* 快速拉低片选。 */
static void ADS8688_SelectFast(const ADS8688_HandleTypeDef *dev);
/* 快速拉高片选。 */
static void ADS8688_DeselectFast(const ADS8688_HandleTypeDef *dev);
/* 打印ADS8688总线配置。 */
static void ADS8688_DebugPrintConfig(const ADS8688_HandleTypeDef *dev);
/* 打印一次SPI传输调试信息。 */
static void ADS8688_DebugPrintTransfer(const ADS8688_HandleTypeDef *dev,
                                       const char *tag,
                                       const uint8_t *tx,
                                       const uint8_t *rx,
                                       uint16_t size,
                                       HAL_StatusTypeDef hal_status,
                                       uint32_t spi_error,
                                       GPIO_PinState sdo_idle,
                                       GPIO_PinState sdo_selected,
                                       GPIO_PinState sdo_done);
/* 判断是否为可编程量程模拟通道。 */
static uint8_t ADS8688_IsAnalogChannel(ADS8688_ChannelTypeDef channel);
/* 检查通道枚举是否有效。 */
static uint8_t ADS8688_IsValidChannel(ADS8688_ChannelTypeDef channel);
/* 检查量程枚举是否有效。 */
static uint8_t ADS8688_IsValidRange(ADS8688_RangeTypeDef range);
/* 生成手动通道转换命令。 */
static uint16_t ADS8688_ChannelCommand(ADS8688_ChannelTypeDef channel);
/* 查找掩码中的下一使能通道。 */
static uint8_t ADS8688_GetNextEnabledChannel(uint8_t mask, uint8_t after_channel);
/* 填充通道、码值和电压采样结构。 */
static void ADS8688_FillSample(ADS8688_HandleTypeDef *dev,
                               ADS8688_ChannelTypeDef channel,
                               uint16_t code,
                               ADS8688_SampleTypeDef *sample);

/************************************************************
 * Function :       ADS8688_GetDefaultConfig
 * Comment  :       获取默认引脚、SPI和参考电压配置
 * Parameter:       cfg: 配置结构体指针
 * Return   :       null
************************************************************/
void ADS8688_GetDefaultConfig(ADS8688_ConfigTypeDef *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  cfg->hspi = NULL;
  cfg->spi_instance = ADS8688_SPI_INSTANCE;
  cfg->spi_prescaler = ADS8688_SPI_BAUDRATE_PRESCALER;
  cfg->spi_timeout_ms = ADS8688_SPI_TIMEOUT_MS;

  cfg->cs_port = ADS8688_CS_GPIO_PORT;
  cfg->cs_pin = ADS8688_CS_PIN;
  cfg->sck_port = ADS8688_SCK_GPIO_PORT;
  cfg->sck_pin = ADS8688_SCK_PIN;
  cfg->sck_af = ADS8688_SCK_AF;
  cfg->miso_port = ADS8688_MISO_GPIO_PORT;
  cfg->miso_pin = ADS8688_MISO_PIN;
  cfg->miso_af = ADS8688_MISO_AF;
  cfg->mosi_port = ADS8688_MOSI_GPIO_PORT;
  cfg->mosi_pin = ADS8688_MOSI_PIN;
  cfg->mosi_af = ADS8688_MOSI_AF;

  cfg->soft_spi_delay_cycles = ADS8688_SOFT_SPI_DELAY_CYCLES;
  cfg->vref_mv = ADS8688_DEFAULT_VREF_MV;
}

/************************************************************
 * Function :       ADS8688_Init
 * Comment  :       初始化ADS8688驱动, 复位寄存器并记录默认量程
 * Parameter:       dev: 驱动句柄; cfg: 配置参数
 * Return   :       ADS8688状态
************************************************************/
ADS8688_StatusTypeDef ADS8688_Init(ADS8688_HandleTypeDef *dev,
                                   const ADS8688_ConfigTypeDef *cfg)
{
  ADS8688_StatusTypeDef status;
  uint8_t i;

  if ((dev == NULL) || (cfg == NULL))
  {
    return ADS8688_ERROR_PARAM;
  }

  status = ADS8688_CheckConfig(cfg);
  if (status != ADS8688_OK)
  {
    return status;
  }

  dev->cfg = *cfg;
  dev->active_hspi = cfg->hspi;
  dev->auto_mask = 0xFFU;
  dev->auto_next_channel = 0U;
  dev->initialized = 0U;
  s_ads8688_debug_count = 0U;
  for (i = 0U; i < ADS8688_ANALOG_CHANNEL_COUNT; i++)
  {
    dev->ranges[i] = ADS8688_RANGE_BIPOLAR_10V24;
  }

  status = ADS8688_InitBus(dev);
  if (status != ADS8688_OK)
  {
    return status;
  }

  ADS8688_Deselect(dev);
  ADS8688_DebugPrintConfig(dev);
  status = ADS8688_Reset(dev);
  if (status == ADS8688_OK)
  {
    dev->initialized = 1U;
  }

  return status;
}

/* 软件复位ADS8688并清除扫描状态。 */
ADS8688_StatusTypeDef ADS8688_Reset(ADS8688_HandleTypeDef *dev)
{
  ADS8688_StatusTypeDef status;
  uint16_t dummy;
  uint8_t i;

  if (dev == NULL)
  {
    return ADS8688_ERROR_PARAM;
  }

  status = ADS8688_TransferCommand(dev, ADS8688_CMD_RST, &dummy);
  if (status != ADS8688_OK)
  {
    return status;
  }

  HAL_Delay(1U);
  for (i = 0U; i < ADS8688_ANALOG_CHANNEL_COUNT; i++)
  {
    dev->ranges[i] = ADS8688_RANGE_BIPOLAR_10V24;
  }
  dev->auto_mask = 0xFFU;
  dev->auto_next_channel = 0U;

  return ADS8688_OK;
}

/* 使ADS8688进入待机模式。 */
ADS8688_StatusTypeDef ADS8688_Standby(ADS8688_HandleTypeDef *dev)
{
  uint16_t dummy;

  if (dev == NULL)
  {
    return ADS8688_ERROR_PARAM;
  }

  return ADS8688_TransferCommand(dev, ADS8688_CMD_STDBY, &dummy);
}

/* 使ADS8688进入掉电模式。 */
ADS8688_StatusTypeDef ADS8688_PowerDown(ADS8688_HandleTypeDef *dev)
{
  uint16_t dummy;

  if (dev == NULL)
  {
    return ADS8688_ERROR_PARAM;
  }

  return ADS8688_TransferCommand(dev, ADS8688_CMD_PWR_DN, &dummy);
}

/* 写入ADS8688程序寄存器。 */
ADS8688_StatusTypeDef ADS8688_WriteRegister(ADS8688_HandleTypeDef *dev,
                                            uint8_t address,
                                            uint8_t value)
{
  ADS8688_StatusTypeDef status;
  uint16_t command;
  uint8_t readback;

  if ((dev == NULL) || (address > 0x7FU))
  {
    return ADS8688_ERROR_PARAM;
  }

  command = (uint16_t)((((uint16_t)address & 0x7FU) << 9) |
                       ADS8688_REG_WRITE_BIT |
                       value);
  status = ADS8688_TransferRegister(dev, command, &readback);
  if (status != ADS8688_OK)
  {
    return status;
  }

  return (readback == value) ? ADS8688_OK : ADS8688_ERROR_SPI;
}

/* 读取ADS8688程序寄存器。 */
ADS8688_StatusTypeDef ADS8688_ReadRegister(ADS8688_HandleTypeDef *dev,
                                           uint8_t address,
                                           uint8_t *value)
{
  uint16_t command;

  if ((dev == NULL) || (value == NULL) || (address > 0x7FU))
  {
    return ADS8688_ERROR_PARAM;
  }

  command = (uint16_t)(((uint16_t)address & 0x7FU) << 9);
  return ADS8688_TransferRegister(dev, command, value);
}

/* 设置指定模拟通道量程。 */
ADS8688_StatusTypeDef ADS8688_SetRange(ADS8688_HandleTypeDef *dev,
                                       ADS8688_ChannelTypeDef channel,
                                       ADS8688_RangeTypeDef range)
{
  ADS8688_StatusTypeDef status;
  uint8_t address;

  if ((dev == NULL) ||
      (ADS8688_IsAnalogChannel(channel) == 0U) ||
      (ADS8688_IsValidRange(range) == 0U))
  {
    return ADS8688_ERROR_PARAM;
  }

  address = (uint8_t)(ADS8688_REG_RANGE_CH0 + (uint8_t)channel);
  status = ADS8688_WriteRegister(dev, address, (uint8_t)range);
  if (status == ADS8688_OK)
  {
    dev->ranges[(uint8_t)channel] = range;
  }

  return status;
}

/* 设置全部模拟通道量程。 */
ADS8688_StatusTypeDef ADS8688_SetAllRanges(ADS8688_HandleTypeDef *dev,
                                           ADS8688_RangeTypeDef range)
{
  ADS8688_StatusTypeDef status;
  uint8_t ch;

  if ((dev == NULL) || (ADS8688_IsValidRange(range) == 0U))
  {
    return ADS8688_ERROR_PARAM;
  }

  for (ch = 0U; ch < ADS8688_ANALOG_CHANNEL_COUNT; ch++)
  {
    status = ADS8688_SetRange(dev, (ADS8688_ChannelTypeDef)ch, range);
    if (status != ADS8688_OK)
    {
      return status;
    }
  }

  return ADS8688_OK;
}

/* 手动读取指定通道。 */
ADS8688_StatusTypeDef ADS8688_ReadChannel(ADS8688_HandleTypeDef *dev,
                                          ADS8688_ChannelTypeDef channel,
                                          ADS8688_SampleTypeDef *sample)
{
  return ADS8688_ReadManualSequence(dev, &channel, 1U, sample);
}

/************************************************************
 * Function :       ADS8688_ReadManualSequence
 * Comment  :       手动流水线读取任意通道序列, 支持CH0~CH7和AUX共9路
 * Parameter:       channels: 通道表; count: 通道数; samples: 输出样本表
 * Return   :       ADS8688状态
************************************************************/
ADS8688_StatusTypeDef ADS8688_ReadManualSequence(ADS8688_HandleTypeDef *dev,
                                                 const ADS8688_ChannelTypeDef *channels,
                                                 uint8_t count,
                                                 ADS8688_SampleTypeDef *samples)
{
  ADS8688_StatusTypeDef status;
  uint16_t code;
  uint16_t command;
  uint8_t i;

  if ((dev == NULL) || (channels == NULL) || (samples == NULL) || (count == 0U))
  {
    return ADS8688_ERROR_PARAM;
  }

  for (i = 0U; i < count; i++)
  {
    if (ADS8688_IsValidChannel(channels[i]) == 0U)
    {
      return ADS8688_ERROR_PARAM;
    }
  }

  /* 第一帧只装载首通道选择命令, 返回的是旧通道数据, 必须丢弃。 */
  status = ADS8688_TransferCommand(dev, ADS8688_ChannelCommand(channels[0]), &code);
  if (status != ADS8688_OK)
  {
    return status;
  }
  for (i = 0U; i < count; i++)
  {
    command = (i + 1U < count) ? ADS8688_ChannelCommand(channels[i + 1U]) :
                                 ADS8688_CMD_NO_OP;
    status = ADS8688_TransferCommand(dev, command, &code);
    if (status != ADS8688_OK)
    {
      return status;
    }
    ADS8688_FillSample(dev, channels[i], code, &samples[i]);
  }

  return ADS8688_OK;
}

/* 依次读取CH0～CH7和AUX。 */
ADS8688_StatusTypeDef ADS8688_ReadAllChannels(ADS8688_HandleTypeDef *dev,
                                              ADS8688_SampleTypeDef samples[ADS8688_CHANNEL_COUNT])
{
  static const ADS8688_ChannelTypeDef channels[ADS8688_CHANNEL_COUNT] =
  {
    ADS8688_CHANNEL_0,
    ADS8688_CHANNEL_1,
    ADS8688_CHANNEL_2,
    ADS8688_CHANNEL_3,
    ADS8688_CHANNEL_4,
    ADS8688_CHANNEL_5,
    ADS8688_CHANNEL_6,
    ADS8688_CHANNEL_7,
    ADS8688_CHANNEL_AUX
  };

  return ADS8688_ReadManualSequence(dev, channels, ADS8688_CHANNEL_COUNT, samples);
}

/* 连续采集单通道波形。 */
ADS8688_StatusTypeDef ADS8688_CaptureChannel(ADS8688_HandleTypeDef *dev,
                                             ADS8688_ChannelTypeDef channel,
                                             uint16_t *codes,
                                             uint32_t sample_count,
                                             float sample_rate_hz,
                                             float *actual_sample_rate_hz)
{
  return ADS8688_CaptureSequence(dev,
                                 &channel,
                                 1U,
                                 codes,
                                 sample_count,
                                 sample_rate_hz,
                                 actual_sample_rate_hz);
}

/************************************************************
 * Function :       ADS8688_CaptureSequence
 * Comment  :       连续采集指定通道序列，不执行printf或九通道轮询。
 *                  数据按采样点和通道交织排列：
 *                  p0ch0,p0ch1,...,p1ch0,p1ch1,...
 * Parameter:       channels: 通道列表，可包含AUX
 *                  sample_count: 每个通道的采样点数
 *                  sample_rate_hz: 单通道目标采样率，非正数表示最快速度
 * Return   :       ADS8688状态
************************************************************/
ADS8688_StatusTypeDef ADS8688_CaptureSequence(ADS8688_HandleTypeDef *dev,
                                              const ADS8688_ChannelTypeDef *channels,
                                              uint8_t channel_count,
                                              uint16_t *codes,
                                              uint32_t sample_count,
                                              float sample_rate_hz,
                                              float *actual_sample_rate_hz)
{
  ADS8688_StatusTypeDef status;
  uint16_t code;
  uint16_t command;
  uint32_t period_cycles;
  uint32_t next_sample_cycle;
  uint32_t start_cycle;
  uint32_t end_cycle;
  uint32_t point;
  uint8_t ch;

  if (actual_sample_rate_hz != NULL)
  {
    *actual_sample_rate_hz = 0.0f;
  }

  if ((dev == NULL) ||
      (channels == NULL) ||
      (codes == NULL) ||
      (channel_count == 0U) ||
      (channel_count > ADS8688_CHANNEL_COUNT) ||
      (sample_count == 0U))
  {
    return ADS8688_ERROR_PARAM;
  }

  for (ch = 0U; ch < channel_count; ch++)
  {
    if (ADS8688_IsValidChannel(channels[ch]) == 0U)
    {
      return ADS8688_ERROR_PARAM;
    }
  }

  if ((sample_rate_hz > 0.0f) &&
      ((sample_rate_hz * (float)channel_count) > ADS8688_MAX_THROUGHPUT_SPS))
  {
    return ADS8688_ERROR_PARAM;
  }

  status = ADS8688_GetSamplePeriodCycles(sample_rate_hz, &period_cycles);
  if (status != ADS8688_OK)
  {
    return status;
  }

  ADS8688_EnableCycleCounter();

  /* 首帧仅选择首通道；返回值属于前一命令，流水线采集时必须丢弃。 */
  status = ADS8688_TransferCaptureCommand(dev,
                                          ADS8688_ChannelCommand(channels[0]),
                                          &code);
  if (status != ADS8688_OK)
  {
    return status;
  }

  start_cycle = DWT->CYCCNT;
  next_sample_cycle = start_cycle + period_cycles;

  for (point = 0U; point < sample_count; point++)
  {
    if ((period_cycles != 0U) && (point != 0U))
    {
      ADS8688_WaitUntilCycle(next_sample_cycle);
      next_sample_cycle += period_cycles;
    }

    for (ch = 0U; ch < channel_count; ch++)
    {
      if ((point == (sample_count - 1U)) && (ch == (channel_count - 1U)))
      {
        command = ADS8688_CMD_NO_OP;
      }
      else if (ch < (channel_count - 1U))
      {
        command = ADS8688_ChannelCommand(channels[ch + 1U]);
      }
      else
      {
        command = ADS8688_ChannelCommand(channels[0]);
      }

      status = ADS8688_TransferCaptureCommand(dev, command, &code);
      if (status != ADS8688_OK)
      {
        return status;
      }

      codes[(point * (uint32_t)channel_count) + ch] = code;
    }
  }

  end_cycle = DWT->CYCCNT;
  ADS8688_SetActualSampleRate(start_cycle,
                              end_cycle,
                              sample_count,
                              actual_sample_rate_hz);
  return ADS8688_OK;
}

/* 设置自动扫描通道掩码。 */
ADS8688_StatusTypeDef ADS8688_SetAutoSequence(ADS8688_HandleTypeDef *dev,
                                              uint8_t channel_mask)
{
  ADS8688_StatusTypeDef status;

  if ((dev == NULL) || (channel_mask == 0U))
  {
    return ADS8688_ERROR_PARAM;
  }

  status = ADS8688_WriteRegister(dev, ADS8688_REG_AUTO_SEQ_EN, channel_mask);
  if (status == ADS8688_OK)
  {
    dev->auto_mask = channel_mask;
    dev->auto_next_channel = ADS8688_GetNextEnabledChannel(channel_mask, 0xFFU);
  }

  return status;
}

/* 启动ADS8688自动扫描。 */
ADS8688_StatusTypeDef ADS8688_StartAutoScan(ADS8688_HandleTypeDef *dev)
{
  uint16_t dummy;

  if ((dev == NULL) || (dev->auto_mask == 0U))
  {
    return ADS8688_ERROR_PARAM;
  }

  dev->auto_next_channel = ADS8688_GetNextEnabledChannel(dev->auto_mask, 0xFFU);
  return ADS8688_TransferCommand(dev, ADS8688_CMD_AUTO_RST, &dummy);
}

/* 读取自动扫描的下一通道结果。 */
ADS8688_StatusTypeDef ADS8688_ReadAutoNext(ADS8688_HandleTypeDef *dev,
                                           ADS8688_SampleTypeDef *sample)
{
  ADS8688_StatusTypeDef status;
  uint16_t code;
  uint8_t channel;

  if ((dev == NULL) || (sample == NULL) || (dev->auto_mask == 0U))
  {
    return ADS8688_ERROR_PARAM;
  }

  channel = dev->auto_next_channel;
  status = ADS8688_TransferCommand(dev, ADS8688_CMD_NO_OP, &code);
  if (status != ADS8688_OK)
  {
    return status;
  }

  ADS8688_FillSample(dev, (ADS8688_ChannelTypeDef)channel, code, sample);
  dev->auto_next_channel = ADS8688_GetNextEnabledChannel(dev->auto_mask, channel);
  return ADS8688_OK;
}

/* 将模拟通道采样码换算为电压。 */
float ADS8688_CodeToVoltageMv(uint16_t code,
                              ADS8688_RangeTypeDef range,
                              float vref_mv)
{
  float negative_full_scale_mv;
  float full_scale_range_mv;

  if (vref_mv <= 0.0f)
  {
    vref_mv = ADS8688_DEFAULT_VREF_MV;
  }

  switch (range)
  {
    case ADS8688_RANGE_BIPOLAR_10V24:
      negative_full_scale_mv = -2.5f * vref_mv;
      full_scale_range_mv = 5.0f * vref_mv;
      break;

    case ADS8688_RANGE_BIPOLAR_5V12:
      negative_full_scale_mv = -1.25f * vref_mv;
      full_scale_range_mv = 2.5f * vref_mv;
      break;

    case ADS8688_RANGE_BIPOLAR_2V56:
      negative_full_scale_mv = -0.625f * vref_mv;
      full_scale_range_mv = 1.25f * vref_mv;
      break;

    case ADS8688_RANGE_UNIPOLAR_10V24:
      negative_full_scale_mv = 0.0f;
      full_scale_range_mv = 2.5f * vref_mv;
      break;

    case ADS8688_RANGE_UNIPOLAR_5V12:
      negative_full_scale_mv = 0.0f;
      full_scale_range_mv = 1.25f * vref_mv;
      break;

    default:
      negative_full_scale_mv = -2.5f * vref_mv;
      full_scale_range_mv = 5.0f * vref_mv;
      break;
  }

  return negative_full_scale_mv +
         (((float)code * full_scale_range_mv) / 65536.0f);
}

/* 将AUX采样码换算为电压。 */
float ADS8688_AuxCodeToVoltageMv(uint16_t code, float vref_mv)
{
  if (vref_mv <= 0.0f)
  {
    vref_mv = ADS8688_DEFAULT_VREF_MV;
  }

  return ((float)code * vref_mv) / 65536.0f;
}

/* 检查ADS8688配置是否完整。 */
static ADS8688_StatusTypeDef ADS8688_CheckConfig(const ADS8688_ConfigTypeDef *cfg)
{
  if ((cfg == NULL) ||
      (cfg->cs_port == NULL) ||
      (cfg->cs_pin == 0U) ||
      (cfg->spi_timeout_ms == 0U) ||
      (cfg->vref_mv <= 0.0f))
  {
    return ADS8688_ERROR_PARAM;
  }

#if (ADS8688_USE_HARDWARE_SPI == 1U)
  if ((cfg->spi_instance == NULL) ||
      (cfg->sck_port == NULL) ||
      (cfg->sck_pin == 0U) ||
      (cfg->miso_port == NULL) ||
      (cfg->miso_pin == 0U) ||
      (cfg->mosi_port == NULL) ||
      (cfg->mosi_pin == 0U))
  {
    return ADS8688_ERROR_PARAM;
  }
#else
  if ((cfg->sck_port == NULL) ||
      (cfg->sck_pin == 0U) ||
      (cfg->miso_port == NULL) ||
      (cfg->miso_pin == 0U) ||
      (cfg->mosi_port == NULL) ||
      (cfg->mosi_pin == 0U))
  {
    return ADS8688_ERROR_PARAM;
  }
#endif

  return ADS8688_OK;
}

/* 按配置初始化ADS8688总线。 */
static ADS8688_StatusTypeDef ADS8688_InitBus(ADS8688_HandleTypeDef *dev)
{
  if (dev == NULL)
  {
    return ADS8688_ERROR_PARAM;
  }

  ADS8688_EnableGpioClock(dev->cfg.cs_port);

#if (ADS8688_USE_HARDWARE_SPI == 1U)
  ADS8688_InitHardwarePins(&dev->cfg);
  if (dev->active_hspi == NULL)
  {
    return ADS8688_InitHardwareSpi(dev);
  }

  return ADS8688_CheckHardwareSpi(dev->active_hspi);
#else
  ADS8688_InitSoftwarePins(&dev->cfg);
  return ADS8688_OK;
#endif
}

/* 初始化驱动内部硬件SPI。 */
static ADS8688_StatusTypeDef ADS8688_InitHardwareSpi(ADS8688_HandleTypeDef *dev)
{
#if (ADS8688_USE_HARDWARE_SPI == 1U)
  s_ads8688_hspi.Instance = dev->cfg.spi_instance;
  s_ads8688_hspi.Init.Mode = SPI_MODE_MASTER;
  s_ads8688_hspi.Init.Direction = SPI_DIRECTION_2LINES;
  s_ads8688_hspi.Init.DataSize = SPI_DATASIZE_8BIT;
  s_ads8688_hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
  s_ads8688_hspi.Init.CLKPhase = SPI_PHASE_2EDGE;
  s_ads8688_hspi.Init.NSS = SPI_NSS_SOFT;
  s_ads8688_hspi.Init.BaudRatePrescaler = dev->cfg.spi_prescaler;
  s_ads8688_hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  s_ads8688_hspi.Init.TIMode = SPI_TIMODE_DISABLE;
  s_ads8688_hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  s_ads8688_hspi.Init.CRCPolynomial = 0x0U;
  s_ads8688_hspi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  s_ads8688_hspi.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  s_ads8688_hspi.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  s_ads8688_hspi.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  s_ads8688_hspi.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  s_ads8688_hspi.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  s_ads8688_hspi.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  s_ads8688_hspi.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  s_ads8688_hspi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  s_ads8688_hspi.Init.IOSwap = SPI_IO_SWAP_DISABLE;

  if (ADS8688_EnableSpiClock(dev->cfg.spi_instance) != ADS8688_OK)
  {
    return ADS8688_ERROR_PARAM;
  }

  if (HAL_SPI_Init(&s_ads8688_hspi) != HAL_OK)
  {
    return ADS8688_ERROR_SPI_CONFIG;
  }

  dev->active_hspi = &s_ads8688_hspi;
  return ADS8688_OK;
#else
  (void)dev;
  return ADS8688_ERROR_UNSUPPORTED;
#endif
}

/* 检查外部硬件SPI配置。 */
static ADS8688_StatusTypeDef ADS8688_CheckHardwareSpi(SPI_HandleTypeDef *hspi)
{
#if (ADS8688_USE_HARDWARE_SPI == 1U)
  if (hspi == NULL)
  {
    return ADS8688_ERROR_PARAM;
  }

  if ((hspi->Init.Mode != SPI_MODE_MASTER) ||
      (hspi->Init.Direction != SPI_DIRECTION_2LINES) ||
      (hspi->Init.DataSize != SPI_DATASIZE_8BIT) ||
      (hspi->Init.CLKPolarity != SPI_POLARITY_LOW) ||
      (hspi->Init.CLKPhase != SPI_PHASE_2EDGE) ||
      (hspi->Init.FirstBit != SPI_FIRSTBIT_MSB))
  {
    return ADS8688_ERROR_SPI_CONFIG;
  }

  return ADS8688_OK;
#else
  (void)hspi;
  return ADS8688_ERROR_UNSUPPORTED;
#endif
}

/* 配置硬件SPI复用引脚。 */
static void ADS8688_InitHardwarePins(const ADS8688_ConfigTypeDef *cfg)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  ADS8688_EnableGpioClock(cfg->cs_port);
  ADS8688_EnableGpioClock(cfg->sck_port);
  ADS8688_EnableGpioClock(cfg->miso_port);
  ADS8688_EnableGpioClock(cfg->mosi_port);

  HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = cfg->cs_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(cfg->cs_port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = cfg->sck_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = cfg->sck_af;
  HAL_GPIO_Init(cfg->sck_port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = cfg->miso_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = cfg->miso_af;
  HAL_GPIO_Init(cfg->miso_port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = cfg->mosi_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = cfg->mosi_af;
  HAL_GPIO_Init(cfg->mosi_port, &GPIO_InitStruct);
}

/* 配置软件SPI GPIO。 */
static void ADS8688_InitSoftwarePins(const ADS8688_ConfigTypeDef *cfg)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  ADS8688_EnableGpioClock(cfg->cs_port);
  ADS8688_EnableGpioClock(cfg->sck_port);
  ADS8688_EnableGpioClock(cfg->miso_port);
  ADS8688_EnableGpioClock(cfg->mosi_port);

  HAL_GPIO_WritePin(cfg->cs_port, cfg->cs_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(cfg->sck_port, cfg->sck_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(cfg->mosi_port, cfg->mosi_pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Pin = cfg->cs_pin;
  HAL_GPIO_Init(cfg->cs_port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = cfg->sck_pin;
  HAL_GPIO_Init(cfg->sck_port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = cfg->mosi_pin;
  HAL_GPIO_Init(cfg->mosi_port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = cfg->miso_pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(cfg->miso_port, &GPIO_InitStruct);
}

/* 使能指定GPIO端口时钟。 */
static void ADS8688_EnableGpioClock(GPIO_TypeDef *port)
{
  if (port == GPIOA)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else if (port == GPIOB)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  }
  else if (port == GPIOC)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  else if (port == GPIOD)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  }
  else if (port == GPIOE)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();
  }
  else if (port == GPIOH)
  {
    __HAL_RCC_GPIOH_CLK_ENABLE();
  }
#ifdef GPIOF
  else if (port == GPIOF)
  {
    __HAL_RCC_GPIOF_CLK_ENABLE();
  }
#endif
#ifdef GPIOG
  else if (port == GPIOG)
  {
    __HAL_RCC_GPIOG_CLK_ENABLE();
  }
#endif
#ifdef GPIOI
  else if (port == GPIOI)
  {
    __HAL_RCC_GPIOI_CLK_ENABLE();
  }
#endif
#ifdef GPIOJ
  else if (port == GPIOJ)
  {
    __HAL_RCC_GPIOJ_CLK_ENABLE();
  }
#endif
#ifdef GPIOK
  else if (port == GPIOK)
  {
    __HAL_RCC_GPIOK_CLK_ENABLE();
  }
#endif
}

/* 使能指定SPI外设时钟。 */
static ADS8688_StatusTypeDef ADS8688_EnableSpiClock(SPI_TypeDef *instance)
{
  if (instance == SPI1)
  {
    __HAL_RCC_SPI1_CLK_ENABLE();
    return ADS8688_OK;
  }
#ifdef SPI2
  if (instance == SPI2)
  {
    __HAL_RCC_SPI2_CLK_ENABLE();
    return ADS8688_OK;
  }
#endif
#ifdef SPI3
  if (instance == SPI3)
  {
    __HAL_RCC_SPI3_CLK_ENABLE();
    return ADS8688_OK;
  }
#endif
#ifdef SPI4
  if (instance == SPI4)
  {
    __HAL_RCC_SPI4_CLK_ENABLE();
    return ADS8688_OK;
  }
#endif
#ifdef SPI5
  if (instance == SPI5)
  {
    __HAL_RCC_SPI5_CLK_ENABLE();
    return ADS8688_OK;
  }
#endif
#ifdef SPI6
  if (instance == SPI6)
  {
    __HAL_RCC_SPI6_CLK_ENABLE();
    return ADS8688_OK;
  }
#endif

  return ADS8688_ERROR_PARAM;
}

/* 发送普通命令帧并接收采样码。 */
static ADS8688_StatusTypeDef ADS8688_TransferCommand(ADS8688_HandleTypeDef *dev,
                                                     uint16_t command,
                                                     uint16_t *code)
{
#if (ADS8688_USE_HARDWARE_SPI == 1U)
  uint8_t tx[4];
  uint8_t rx[4] = {0U, 0U, 0U, 0U};
  ADS8688_StatusTypeDef status;

  if ((dev == NULL) || (code == NULL))
  {
    return ADS8688_ERROR_PARAM;
  }

  tx[0] = (uint8_t)(command >> 8);
  tx[1] = (uint8_t)(command & 0xFFU);
  tx[2] = 0U;
  tx[3] = 0U;

  status = ADS8688_HardwareTransfer(dev, tx, rx, sizeof(tx));
  if (status != ADS8688_OK)
  {
    return status;
  }

  *code = (uint16_t)(((uint16_t)rx[2] << 8) | rx[3]);
  return ADS8688_OK;
#else
  uint32_t rx_frame;

  if ((dev == NULL) || (code == NULL))
  {
    return ADS8688_ERROR_PARAM;
  }

  if (ADS8688_SoftwareTransfer(dev,
                               (uint32_t)command << 16,
                               ADS8688_COMMAND_FRAME_BITS,
                               &rx_frame) != ADS8688_OK)
  {
    return ADS8688_ERROR_SPI;
  }

  *code = (uint16_t)(rx_frame & 0xFFFFU);
  return ADS8688_OK;
#endif
}

/* 完成一次程序寄存器访问。 */
static ADS8688_StatusTypeDef ADS8688_TransferRegister(ADS8688_HandleTypeDef *dev,
                                                      uint16_t command,
                                                      uint8_t *value)
{
#if (ADS8688_USE_HARDWARE_SPI == 1U)
  uint8_t tx[3];
  uint8_t rx[3] = {0U, 0U, 0U};
  ADS8688_StatusTypeDef status;

  if ((dev == NULL) || (value == NULL))
  {
    return ADS8688_ERROR_PARAM;
  }

  tx[0] = (uint8_t)(command >> 8);
  tx[1] = (uint8_t)(command & 0xFFU);
  tx[2] = 0U;

  status = ADS8688_HardwareTransfer(dev, tx, rx, sizeof(tx));
  if (status != ADS8688_OK)
  {
    return status;
  }

  *value = rx[2];
  return ADS8688_OK;
#else
  uint32_t rx_frame;

  if ((dev == NULL) || (value == NULL))
  {
    return ADS8688_ERROR_PARAM;
  }

  if (ADS8688_SoftwareTransfer(dev,
                               (uint32_t)command << 8,
                               ADS8688_REGISTER_FRAME_BITS,
                               &rx_frame) != ADS8688_OK)
  {
    return ADS8688_ERROR_SPI;
  }

  *value = (uint8_t)(rx_frame & 0xFFU);
  return ADS8688_OK;
#endif
}

/* 发送连续采集命令帧。 */
static ADS8688_StatusTypeDef ADS8688_TransferCaptureCommand(ADS8688_HandleTypeDef *dev,
                                                            uint16_t command,
                                                            uint16_t *code)
{
#if ((ADS8688_USE_HARDWARE_SPI == 1U) && (ADS8688_USE_FAST_SPI_FRAME == 1U))
  return ADS8688_HardwareTransferCommandFast(dev, command, code);
#else
  return ADS8688_TransferCommand(dev, command, code);
#endif
}

/* 通过寄存器直访快速发送命令帧。 */
static ADS8688_StatusTypeDef ADS8688_HardwareTransferCommandFast(ADS8688_HandleTypeDef *dev,
                                                                 uint16_t command,
                                                                 uint16_t *code)
{
#if ((ADS8688_USE_HARDWARE_SPI == 1U) && (ADS8688_USE_FAST_SPI_FRAME == 1U))
  SPI_TypeDef *spi;
  uint8_t tx[4];
  uint8_t rx[4] = {0U, 0U, 0U, 0U};
  uint8_t tx_index = 0U;
  uint8_t rx_index = 0U;
  uint32_t start_cycles;
  uint32_t timeout_cycles;
  GPIO_PinState sdo_idle;
  GPIO_PinState sdo_selected;
  GPIO_PinState sdo_done;

  if ((dev == NULL) || (dev->active_hspi == NULL) || (code == NULL))
  {
    return ADS8688_ERROR_PARAM;
  }

  if (dev->active_hspi->State != HAL_SPI_STATE_READY)
  {
    return ADS8688_ERROR_SPI;
  }

  spi = dev->active_hspi->Instance;
  tx[0] = (uint8_t)(command >> 8);
  tx[1] = (uint8_t)(command & 0xFFU);
  tx[2] = 0U;
  tx[3] = 0U;

  ADS8688_EnableCycleCounter();
  timeout_cycles = (HAL_RCC_GetHCLKFreq() / 1000U) *
                   (dev->cfg.spi_timeout_ms + 1U);
  if (timeout_cycles == 0U)
  {
    timeout_cycles = HAL_RCC_GetHCLKFreq() / 1000U;
  }

  LL_SPI_ClearFlag_EOT(spi);
  LL_SPI_ClearFlag_TXTF(spi);
  LL_SPI_ClearFlag_OVR(spi);
  LL_SPI_ClearFlag_FRE(spi);
  LL_SPI_ClearFlag_UDR(spi);
  MODIFY_REG(spi->CR2, SPI_CR2_TSIZE, 4U);
  __HAL_SPI_ENABLE(dev->active_hspi);

  sdo_idle = HAL_GPIO_ReadPin(dev->cfg.miso_port, dev->cfg.miso_pin);
  ADS8688_SelectFast(dev);
  sdo_selected = HAL_GPIO_ReadPin(dev->cfg.miso_port, dev->cfg.miso_pin);
  LL_SPI_StartMasterTransfer(spi);

  start_cycles = DWT->CYCCNT;
  while (rx_index < 4U)
  {
    if ((tx_index < 4U) && (LL_SPI_IsActiveFlag_TXP(spi) != 0U))
    {
      LL_SPI_TransmitData8(spi, tx[tx_index]);
      tx_index++;
    }

    if (LL_SPI_IsActiveFlag_RXP(spi) != 0U)
    {
      rx[rx_index] = LL_SPI_ReceiveData8(spi);
      rx_index++;
    }

    if ((DWT->CYCCNT - start_cycles) > timeout_cycles)
    {
      sdo_done = HAL_GPIO_ReadPin(dev->cfg.miso_port, dev->cfg.miso_pin);
      ADS8688_DeselectFast(dev);
      __HAL_SPI_DISABLE(dev->active_hspi);
      ADS8688_DebugPrintTransfer(dev,
                                 "FAST_TO_RX",
                                 tx,
                                 rx,
                                 sizeof(tx),
                                 HAL_TIMEOUT,
                                 HAL_SPI_GetError(dev->active_hspi),
                                 sdo_idle,
                                 sdo_selected,
                                 sdo_done);
      return ADS8688_ERROR_SPI;
    }
  }

  while (LL_SPI_IsActiveFlag_EOT(spi) == 0U)
  {
    if ((DWT->CYCCNT - start_cycles) > timeout_cycles)
    {
      sdo_done = HAL_GPIO_ReadPin(dev->cfg.miso_port, dev->cfg.miso_pin);
      ADS8688_DeselectFast(dev);
      __HAL_SPI_DISABLE(dev->active_hspi);
      ADS8688_DebugPrintTransfer(dev,
                                 "FAST_TO_EOT",
                                 tx,
                                 rx,
                                 sizeof(tx),
                                 HAL_TIMEOUT,
                                 HAL_SPI_GetError(dev->active_hspi),
                                 sdo_idle,
                                 sdo_selected,
                                 sdo_done);
      return ADS8688_ERROR_SPI;
    }
  }

  sdo_done = HAL_GPIO_ReadPin(dev->cfg.miso_port, dev->cfg.miso_pin);
  ADS8688_DeselectFast(dev);
  LL_SPI_ClearFlag_EOT(spi);
  LL_SPI_ClearFlag_TXTF(spi);
  __HAL_SPI_DISABLE(dev->active_hspi);

  ADS8688_DebugPrintTransfer(dev,
                             "FAST",
                             tx,
                             rx,
                             sizeof(tx),
                             HAL_OK,
                             HAL_SPI_GetError(dev->active_hspi),
                             sdo_idle,
                             sdo_selected,
                             sdo_done);

  *code = (uint16_t)(((uint16_t)rx[2] << 8) | rx[3]);
  return ADS8688_OK;
#else
  (void)dev;
  (void)command;
  (void)code;
  return ADS8688_ERROR_UNSUPPORTED;
#endif
}

/* 通过HAL硬件SPI交换字节流。 */
static ADS8688_StatusTypeDef ADS8688_HardwareTransfer(ADS8688_HandleTypeDef *dev,
                                                      const uint8_t *tx,
                                                      uint8_t *rx,
                                                      uint16_t size)
{
#if (ADS8688_USE_HARDWARE_SPI == 1U)
  HAL_StatusTypeDef hal_status;
  GPIO_PinState sdo_idle;
  GPIO_PinState sdo_selected;
  GPIO_PinState sdo_done;
  uint32_t spi_error;

  if ((dev == NULL) || (dev->active_hspi == NULL) || (tx == NULL) || (rx == NULL))
  {
    return ADS8688_ERROR_PARAM;
  }

  sdo_idle = HAL_GPIO_ReadPin(dev->cfg.miso_port, dev->cfg.miso_pin);
  ADS8688_Select(dev);
  sdo_selected = HAL_GPIO_ReadPin(dev->cfg.miso_port, dev->cfg.miso_pin);
  hal_status = HAL_SPI_TransmitReceive(dev->active_hspi,
                                       tx,
                                       rx,
                                       size,
                                       dev->cfg.spi_timeout_ms);
  spi_error = HAL_SPI_GetError(dev->active_hspi);
  sdo_done = HAL_GPIO_ReadPin(dev->cfg.miso_port, dev->cfg.miso_pin);
  ADS8688_Deselect(dev);

  ADS8688_DebugPrintTransfer(dev,
                             "HAL",
                             tx,
                             rx,
                             size,
                             hal_status,
                             spi_error,
                             sdo_idle,
                             sdo_selected,
                             sdo_done);

  return (hal_status == HAL_OK) ? ADS8688_OK : ADS8688_ERROR_SPI;
#else
  (void)dev;
  (void)tx;
  (void)rx;
  (void)size;
  return ADS8688_ERROR_UNSUPPORTED;
#endif
}

/* 通过GPIO软件SPI交换指定位数。 */
static ADS8688_StatusTypeDef ADS8688_SoftwareTransfer(ADS8688_HandleTypeDef *dev,
                                                      uint32_t tx,
                                                      uint8_t bits,
                                                      uint32_t *rx)
{
  int8_t bit;
  uint32_t rx_frame = 0U;
  GPIO_PinState tx_state;

  if ((dev == NULL) || (rx == NULL) || (bits == 0U) || (bits > 32U))
  {
    return ADS8688_ERROR_PARAM;
  }

  ADS8688_Select(dev);
  for (bit = (int8_t)bits - 1; bit >= 0; bit--)
  {
    tx_state = (((tx >> (uint8_t)bit) & 0x01U) != 0U) ?
               GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(dev->cfg.mosi_port, dev->cfg.mosi_pin, tx_state);
    ADS8688_DelayCycles(dev->cfg.soft_spi_delay_cycles);

    HAL_GPIO_WritePin(dev->cfg.sck_port, dev->cfg.sck_pin, GPIO_PIN_SET);
    ADS8688_DelayCycles(dev->cfg.soft_spi_delay_cycles);

    rx_frame <<= 1;
    if (HAL_GPIO_ReadPin(dev->cfg.miso_port, dev->cfg.miso_pin) == GPIO_PIN_SET)
    {
      rx_frame |= 0x01U;
    }

    HAL_GPIO_WritePin(dev->cfg.sck_port, dev->cfg.sck_pin, GPIO_PIN_RESET);
    ADS8688_DelayCycles(dev->cfg.soft_spi_delay_cycles);
  }
  ADS8688_Deselect(dev);
  HAL_GPIO_WritePin(dev->cfg.mosi_port, dev->cfg.mosi_pin, GPIO_PIN_RESET);

  *rx = rx_frame;
  return ADS8688_OK;
}

/* 拉低ADS8688片选并满足建立时间。 */
static void ADS8688_Select(const ADS8688_HandleTypeDef *dev)
{
  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_RESET);
  ADS8688_DelayCycles(ADS8688_CS_SETUP_DELAY_CYCLES);
}

/* 满足保持时间后拉高ADS8688片选。 */
static void ADS8688_Deselect(const ADS8688_HandleTypeDef *dev)
{
  ADS8688_DelayCycles(ADS8688_CS_HOLD_DELAY_CYCLES);
  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);
  ADS8688_DelayCycles(ADS8688_CS_HOLD_DELAY_CYCLES);
}

/* 提供GPIO时序短延时。 */
static void ADS8688_DelayCycles(uint16_t cycles)
{
  while (cycles > 0U)
  {
    __NOP();
    cycles--;
  }
}

/* 启用DWT周期计数器。 */
static void ADS8688_EnableCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* 将采样率换算为CPU周期。 */
static ADS8688_StatusTypeDef ADS8688_GetSamplePeriodCycles(float sample_rate_hz,
                                                           uint32_t *period_cycles)
{
  float cycles;

  if (period_cycles == NULL)
  {
    return ADS8688_ERROR_PARAM;
  }

  if (sample_rate_hz <= 0.0f)
  {
    *period_cycles = 0U;
    return ADS8688_OK;
  }

  cycles = (float)HAL_RCC_GetHCLKFreq() / sample_rate_hz;
  if ((cycles < 1.0f) || (cycles > 2147483647.0f))
  {
    return ADS8688_ERROR_PARAM;
  }

  *period_cycles = (uint32_t)(cycles + 0.5f);
  if (*period_cycles == 0U)
  {
    return ADS8688_ERROR_PARAM;
  }

  return ADS8688_OK;
}

/* 等待到指定DWT周期时刻。 */
static void ADS8688_WaitUntilCycle(uint32_t target_cycle)
{
  while ((int32_t)(DWT->CYCCNT - target_cycle) < 0)
  {
  }
}

/* 计算块采集实际单通道采样率。 */
static void ADS8688_SetActualSampleRate(uint32_t start_cycle,
                                        uint32_t end_cycle,
                                        uint32_t sample_count,
                                        float *actual_sample_rate_hz)
{
  uint32_t elapsed_cycles;

  if (actual_sample_rate_hz == NULL)
  {
    return;
  }

  elapsed_cycles = end_cycle - start_cycle;
  if (elapsed_cycles == 0U)
  {
    *actual_sample_rate_hz = 0.0f;
    return;
  }

  *actual_sample_rate_hz = ((float)sample_count * (float)HAL_RCC_GetHCLKFreq()) /
                           (float)elapsed_cycles;
}

/* 快速拉低片选。 */
static void ADS8688_SelectFast(const ADS8688_HandleTypeDef *dev)
{
  dev->cfg.cs_port->BSRR = ((uint32_t)dev->cfg.cs_pin << 16U);
  ADS8688_DelayCycles(ADS8688_CS_SETUP_DELAY_CYCLES);
}

/* 快速拉高片选。 */
static void ADS8688_DeselectFast(const ADS8688_HandleTypeDef *dev)
{
  ADS8688_DelayCycles(ADS8688_CS_HOLD_DELAY_CYCLES);
  dev->cfg.cs_port->BSRR = dev->cfg.cs_pin;
  ADS8688_DelayCycles(ADS8688_CS_HOLD_DELAY_CYCLES);
}

/* 打印ADS8688总线配置。 */
static void ADS8688_DebugPrintConfig(const ADS8688_HandleTypeDef *dev)
{
#if (ADS8688_ENABLE_LOW_LEVEL_DEBUG == 1U)
  if ((dev == NULL) || (dev->active_hspi == NULL))
  {
    return;
  }

  printf("[ADS8688] cfg spi=%p prescaler=%lu cpol=%lu cpha=%lu fast=%u\r\n",
         (void *)dev->active_hspi->Instance,
         (unsigned long)dev->active_hspi->Init.BaudRatePrescaler,
         (unsigned long)dev->active_hspi->Init.CLKPolarity,
         (unsigned long)dev->active_hspi->Init.CLKPhase,
         (unsigned int)ADS8688_USE_FAST_SPI_FRAME);
  printf("[ADS8688] pins CS=%p/0x%04X SCK=%p/0x%04X SDI=%p/0x%04X SDO=%p/0x%04X sdo_idle=%u\r\n",
         (void *)dev->cfg.cs_port,
         (unsigned int)dev->cfg.cs_pin,
         (void *)dev->cfg.sck_port,
         (unsigned int)dev->cfg.sck_pin,
         (void *)dev->cfg.mosi_port,
         (unsigned int)dev->cfg.mosi_pin,
         (void *)dev->cfg.miso_port,
         (unsigned int)dev->cfg.miso_pin,
         (unsigned int)HAL_GPIO_ReadPin(dev->cfg.miso_port, dev->cfg.miso_pin));
#else
  (void)dev;
#endif
}

/* 打印一次SPI传输调试信息。 */
static void ADS8688_DebugPrintTransfer(const ADS8688_HandleTypeDef *dev,
                                       const char *tag,
                                       const uint8_t *tx,
                                       const uint8_t *rx,
                                       uint16_t size,
                                       HAL_StatusTypeDef hal_status,
                                       uint32_t spi_error,
                                       GPIO_PinState sdo_idle,
                                       GPIO_PinState sdo_selected,
                                       GPIO_PinState sdo_done)
{
#if (ADS8688_ENABLE_LOW_LEVEL_DEBUG == 1U)
  uint16_t i;

  (void)dev;
  if ((tag == NULL) || (tx == NULL) || (rx == NULL))
  {
    return;
  }

  if (s_ads8688_debug_count >= ADS8688_LOW_LEVEL_DEBUG_LIMIT)
  {
    return;
  }
  s_ads8688_debug_count++;

  printf("[ADS8688:%s] n=%lu hal=%d spi_err=0x%08lX sdo=%u/%u/%u tx=",
         tag,
         (unsigned long)s_ads8688_debug_count,
         (int)hal_status,
         (unsigned long)spi_error,
         (unsigned int)sdo_idle,
         (unsigned int)sdo_selected,
         (unsigned int)sdo_done);
  for (i = 0U; i < size; i++)
  {
    printf("%02X", (unsigned int)tx[i]);
  }
  printf(" rx=");
  for (i = 0U; i < size; i++)
  {
    printf("%02X", (unsigned int)rx[i]);
  }
  printf("\r\n");
#else
  (void)dev;
  (void)tag;
  (void)tx;
  (void)rx;
  (void)size;
  (void)hal_status;
  (void)spi_error;
  (void)sdo_idle;
  (void)sdo_selected;
  (void)sdo_done;
#endif
}

/* 判断是否为可编程量程模拟通道。 */
static uint8_t ADS8688_IsAnalogChannel(ADS8688_ChannelTypeDef channel)
{
  return (uint8_t)((uint8_t)channel < ADS8688_ANALOG_CHANNEL_COUNT);
}

/* 检查通道枚举是否有效。 */
static uint8_t ADS8688_IsValidChannel(ADS8688_ChannelTypeDef channel)
{
  return (uint8_t)(((uint8_t)channel < ADS8688_ANALOG_CHANNEL_COUNT) ||
                   (channel == ADS8688_CHANNEL_AUX));
}

/* 检查量程枚举是否有效。 */
static uint8_t ADS8688_IsValidRange(ADS8688_RangeTypeDef range)
{
  return (uint8_t)((range == ADS8688_RANGE_BIPOLAR_10V24) ||
                   (range == ADS8688_RANGE_BIPOLAR_5V12) ||
                   (range == ADS8688_RANGE_BIPOLAR_2V56) ||
                   (range == ADS8688_RANGE_UNIPOLAR_10V24) ||
                   (range == ADS8688_RANGE_UNIPOLAR_5V12));
}

/* 生成手动通道转换命令。 */
static uint16_t ADS8688_ChannelCommand(ADS8688_ChannelTypeDef channel)
{
  if (channel == ADS8688_CHANNEL_AUX)
  {
    return ADS8688_CMD_MAN_AUX;
  }

  return (uint16_t)(ADS8688_CMD_MAN_CH_BASE | ((uint16_t)channel << 10));
}

/* 查找掩码中的下一使能通道。 */
static uint8_t ADS8688_GetNextEnabledChannel(uint8_t mask, uint8_t after_channel)
{
  uint8_t ch;

  for (ch = (uint8_t)(after_channel + 1U); ch < ADS8688_ANALOG_CHANNEL_COUNT; ch++)
  {
    if ((mask & (1U << ch)) != 0U)
    {
      return ch;
    }
  }

  for (ch = 0U; ch < ADS8688_ANALOG_CHANNEL_COUNT; ch++)
  {
    if ((mask & (1U << ch)) != 0U)
    {
      return ch;
    }
  }

  return 0U;
}

/* 填充通道、码值和电压采样结构。 */
static void ADS8688_FillSample(ADS8688_HandleTypeDef *dev,
                               ADS8688_ChannelTypeDef channel,
                               uint16_t code,
                               ADS8688_SampleTypeDef *sample)
{
  sample->channel = channel;
  sample->code = code;

  if (channel == ADS8688_CHANNEL_AUX)
  {
    sample->range = ADS8688_RANGE_AUX_4V096;
    sample->voltage_mv = ADS8688_AuxCodeToVoltageMv(code, dev->cfg.vref_mv);
  }
  else
  {
    sample->range = dev->ranges[(uint8_t)channel];
    sample->voltage_mv = ADS8688_CodeToVoltageMv(code,
                                                 sample->range,
                                                 dev->cfg.vref_mv);
  }
}
