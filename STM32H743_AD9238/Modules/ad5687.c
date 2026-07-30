#include "ad5687.h"

#define AD5687_CMD_WRITE_INPUT       0x1U /* 写输入寄存器命令。 */
#define AD5687_CMD_UPDATE_DAC        0x2U /* 更新DAC输出命令。 */
#define AD5687_CMD_WRITE_UPDATE_DAC  0x3U /* 写入并更新DAC命令。 */
#define AD5687_CMD_POWER_DOWN_UP     0x4U /* 掉电控制命令。 */
#define AD5687_CMD_SOFTWARE_RESET    0x6U /* 软件复位命令。 */
#define AD5687_CMD_DAISY_CHAIN       0x8U /* 级联模式控制命令。 */
#define AD5687_CMD_DAISY_NOP         0xFU /* 级联空操作命令。 */

/* 检查AD5687通道选择是否有效。 */
static uint8_t AD5687_IsValidChannel(AD5687_ChannelTypeDef channel);
/* 检查AD5687器件选择是否有效。 */
static uint8_t AD5687_IsValidDevice(AD5687_DeviceTypeDef device);
/* 提供软件SPI短延时。 */
static void AD5687_DelayCycles(uint16_t cycles);
/* 发送一位软件SPI数据。 */
static void AD5687_WriteBit(AD5687_HandleTypeDef *dev, GPIO_PinState bit);
/* 向近端器件发送24位帧。 */
static HAL_StatusTypeDef AD5687_WriteFrame24(AD5687_HandleTypeDef *dev,
                                             uint32_t frame);
/* 向级联器件发送48位帧。 */
static HAL_StatusTypeDef AD5687_WriteFrame48(AD5687_HandleTypeDef *dev,
                                             AD5687_DeviceTypeDef device,
                                             uint32_t frame);
/* 组装并发送DAC控制命令。 */
static HAL_StatusTypeDef AD5687_WriteDacCommand(AD5687_HandleTypeDef *dev,
                                                AD5687_DeviceTypeDef device,
                                                uint8_t command,
                                                AD5687_ChannelTypeDef channel,
                                                uint16_t code);

/************************************************************
 * Function :       AD5687_Init
 * Comment  :       初始化AD5687驱动句柄并发送NOP帧确认SPI通信
 * Parameter:       dev: AD5687驱动句柄指针
 * Return   :       HAL_OK表示成功, HAL_ERROR表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
HAL_StatusTypeDef AD5687_Init(AD5687_HandleTypeDef *dev)
{
  HAL_StatusTypeDef status;

  if ((dev == NULL) ||
      (dev->sck_port == NULL) ||
      (dev->mosi_port == NULL) ||
      (dev->cs_port == NULL) ||
      (dev->sck_pin == 0U) ||
      (dev->mosi_pin == 0U) ||
      (dev->cs_pin == 0U))
  {
    return HAL_ERROR;
  }

  if ((dev->vref_mv == 0U) || ((dev->gain != 1U) && (dev->gain != 2U)))
  {
    return HAL_ERROR;
  }

  if (dev->spi_delay_cycles == 0U)
  {
    dev->spi_delay_cycles = AD5687_SOFT_SPI_DELAY_CYCLES;
  }

  HAL_GPIO_WritePin(dev->sck_port, dev->sck_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(dev->mosi_port, dev->mosi_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

  /* 先开启近端芯片SDO，随后两片芯片才能按48位级联帧通信。 */
  status = AD5687_WriteFrame24(
    dev,
    ((uint32_t)AD5687_CMD_DAISY_CHAIN << 20) | 0x01U);
  if (status != HAL_OK)
  {
    return status;
  }

  return AD5687_WriteFrame48(dev,
                             AD5687_DEVICE_ALL,
                             ((uint32_t)AD5687_CMD_DAISY_NOP << 20));
}

/************************************************************
 * Function :       AD5687_WriteInputRaw
 * Comment  :       向AD5687指定通道输入寄存器写入原始DAC码值
 * Parameter:       dev: AD5687驱动句柄; channel: DAC通道; code: 12位码值
 * Return   :       HAL_OK表示成功, HAL_ERROR表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
HAL_StatusTypeDef AD5687_WriteInputRaw(AD5687_HandleTypeDef *dev,
                                       AD5687_DeviceTypeDef device,
                                       AD5687_ChannelTypeDef channel,
                                       uint16_t code)
{
  return AD5687_WriteDacCommand(dev,
                                device,
                                AD5687_CMD_WRITE_INPUT,
                                channel,
                                code);
}

/************************************************************
 * Function :       AD5687_Update
 * Comment  :       将AD5687指定通道输入寄存器的数据更新到DAC输出
 * Parameter:       dev: AD5687驱动句柄; channel: DAC通道
 * Return   :       HAL_OK表示成功, HAL_ERROR表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
HAL_StatusTypeDef AD5687_Update(AD5687_HandleTypeDef *dev,
                                AD5687_DeviceTypeDef device,
                                AD5687_ChannelTypeDef channel)
{
  return AD5687_WriteDacCommand(dev,
                                device,
                                AD5687_CMD_UPDATE_DAC,
                                channel,
                                0U);
}

/************************************************************
 * Function :       AD5687_WriteAndUpdateRaw
 * Comment  :       向AD5687指定通道写入原始码值并立即更新输出
 * Parameter:       dev: AD5687驱动句柄; channel: DAC通道; code: 12位码值
 * Return   :       HAL_OK表示成功, HAL_ERROR表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
HAL_StatusTypeDef AD5687_WriteAndUpdateRaw(AD5687_HandleTypeDef *dev,
                                           AD5687_DeviceTypeDef device,
                                           AD5687_ChannelTypeDef channel,
                                           uint16_t code)
{
  return AD5687_WriteDacCommand(dev,
                                device,
                                AD5687_CMD_WRITE_UPDATE_DAC,
                                channel,
                                code);
}

/************************************************************
 * Function :       AD5687_SetVoltageMv
 * Comment  :       按毫伏值设置AD5687指定通道输出
 * Parameter:       dev: AD5687驱动句柄; channel: DAC通道; voltage_mv: 目标电压(mV)
 * Return   :       HAL_OK表示成功, HAL_ERROR表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
HAL_StatusTypeDef AD5687_SetVoltageMv(AD5687_HandleTypeDef *dev,
                                      AD5687_DeviceTypeDef device,
                                      AD5687_ChannelTypeDef channel,
                                      uint32_t voltage_mv)
{
  return AD5687_WriteAndUpdateRaw(dev,
                                  device,
                                  channel,
                                  AD5687_VoltageToCodeMv(dev, voltage_mv));
}

/************************************************************
 * Function :       AD5687_SetBothVoltageMv
 * Comment  :       同时设置AD5687 A/B两个通道输出电压
 * Parameter:       dev: AD5687驱动句柄; channel_a_mv: A通道电压(mV); channel_b_mv: B通道电压(mV)
 * Return   :       HAL_OK表示成功, HAL_ERROR表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
HAL_StatusTypeDef AD5687_SetBothVoltageMv(AD5687_HandleTypeDef *dev,
                                          AD5687_DeviceTypeDef device,
                                          uint32_t channel_a_mv,
                                          uint32_t channel_b_mv)
{
  HAL_StatusTypeDef status;

  status = AD5687_WriteInputRaw(dev,
                                device,
                                AD5687_CHANNEL_A,
                                AD5687_VoltageToCodeMv(dev, channel_a_mv));
  if (status != HAL_OK)
  {
    return status;
  }

  status = AD5687_WriteInputRaw(dev,
                                device,
                                AD5687_CHANNEL_B,
                                AD5687_VoltageToCodeMv(dev, channel_b_mv));
  if (status != HAL_OK)
  {
    return status;
  }

  return AD5687_Update(dev, device, AD5687_CHANNEL_ALL);
}

/************************************************************
 * Function :       AD5687_SetPowerMode
 * Comment  :       设置AD5687 A/B通道掉电或正常工作模式
 * Parameter:       dev: AD5687驱动句柄; channel_a_mode: A通道模式; channel_b_mode: B通道模式
 * Return   :       HAL_OK表示成功, HAL_ERROR表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
HAL_StatusTypeDef AD5687_SetPowerMode(AD5687_HandleTypeDef *dev,
                                      AD5687_DeviceTypeDef device,
                                      AD5687_PowerModeTypeDef channel_a_mode,
                                      AD5687_PowerModeTypeDef channel_b_mode)
{
  uint8_t control;
  uint32_t frame;

  if ((dev == NULL) ||
      ((uint8_t)channel_a_mode > (uint8_t)AD5687_POWER_THREE_STATE) ||
      ((uint8_t)channel_b_mode > (uint8_t)AD5687_POWER_THREE_STATE))
  {
    return HAL_ERROR;
  }

  control = (uint8_t)(((uint8_t)channel_b_mode << 6) |
                      0x3CU |
                      (uint8_t)channel_a_mode);
  frame = ((uint32_t)AD5687_CMD_POWER_DOWN_UP << 20) | (uint32_t)control;

  return AD5687_WriteFrame48(dev, device, frame);
}

/************************************************************
 * Function :       AD5687_SoftwareReset
 * Comment  :       发送AD5687软件复位命令
 * Parameter:       dev: AD5687驱动句柄
 * Return   :       HAL_OK表示成功, HAL_ERROR表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
HAL_StatusTypeDef AD5687_SoftwareReset(AD5687_HandleTypeDef *dev,
                                       AD5687_DeviceTypeDef device)
{
  HAL_StatusTypeDef status;

  status = AD5687_WriteFrame48(
    dev,
    device,
    ((uint32_t)AD5687_CMD_SOFTWARE_RESET << 20) | 0x01U);
  if ((status == HAL_OK) && (device != AD5687_DEVICE_CV3_CV4))
  {
    status = AD5687_WriteFrame24(
      dev,
      ((uint32_t)AD5687_CMD_DAISY_CHAIN << 20) | 0x01U);
  }

  return status;
}

/************************************************************
 * Function :       AD5687_VoltageToCodeMv
 * Comment  :       将目标电压毫伏值转换为AD5687 12位DAC码值
 * Parameter:       dev: AD5687驱动句柄; voltage_mv: 目标电压(mV)
 * Return   :       12位DAC码值
 * Date     :       2026-06-10 V1
************************************************************/
uint16_t AD5687_VoltageToCodeMv(const AD5687_HandleTypeDef *dev,
                                uint32_t voltage_mv)
{
  uint32_t full_scale_mv;
  uint64_t numerator;

  if ((dev == NULL) || (dev->vref_mv == 0U) || (dev->gain == 0U))
  {
    return 0U;
  }

  full_scale_mv = dev->vref_mv * (uint32_t)dev->gain;
  if (voltage_mv >= full_scale_mv)
  {
    return AD5687_MAX_CODE;
  }

  numerator = ((uint64_t)voltage_mv * AD5687_MAX_CODE) + (full_scale_mv / 2U);
  return (uint16_t)(numerator / full_scale_mv);
}

/************************************************************
 * Function :       AD5687_CodeToVoltageMv
 * Comment  :       将AD5687 12位DAC码值换算为输出电压毫伏值
 * Parameter:       dev: AD5687驱动句柄; code: 12位DAC码值
 * Return   :       换算后的电压(mV)
 * Date     :       2026-06-10 V1
************************************************************/
uint32_t AD5687_CodeToVoltageMv(const AD5687_HandleTypeDef *dev,
                                uint16_t code)
{
  uint32_t full_scale_mv;
  uint32_t clipped_code;

  if ((dev == NULL) || (dev->vref_mv == 0U) || (dev->gain == 0U))
  {
    return 0U;
  }

  full_scale_mv = dev->vref_mv * (uint32_t)dev->gain;
  clipped_code = (uint32_t)(code & AD5687_MAX_CODE);

  return (uint32_t)((((uint64_t)clipped_code * full_scale_mv) +
                     (AD5687_MAX_CODE / 2U)) /
                    AD5687_MAX_CODE);
}

/* 检查AD5687通道选择是否有效。 */
static uint8_t AD5687_IsValidChannel(AD5687_ChannelTypeDef channel)
{
  return (uint8_t)((channel == AD5687_CHANNEL_A) ||
                   (channel == AD5687_CHANNEL_B) ||
                   (channel == AD5687_CHANNEL_ALL));
}

/* 检查AD5687器件选择是否有效。 */
static uint8_t AD5687_IsValidDevice(AD5687_DeviceTypeDef device)
{
  return (uint8_t)((device == AD5687_DEVICE_CV1_CV2) ||
                   (device == AD5687_DEVICE_CV3_CV4) ||
                   (device == AD5687_DEVICE_ALL));
}

/* 提供软件SPI短延时。 */
static void AD5687_DelayCycles(uint16_t cycles)
{
  while (cycles-- > 0U)
  {
    __NOP();
  }
}

/* 发送一位软件SPI数据。 */
static void AD5687_WriteBit(AD5687_HandleTypeDef *dev, GPIO_PinState bit)
{
  HAL_GPIO_WritePin(dev->mosi_port, dev->mosi_pin, bit);
  AD5687_DelayCycles(dev->spi_delay_cycles);
  HAL_GPIO_WritePin(dev->sck_port, dev->sck_pin, GPIO_PIN_SET);
  AD5687_DelayCycles(dev->spi_delay_cycles);
  HAL_GPIO_WritePin(dev->sck_port, dev->sck_pin, GPIO_PIN_RESET);
  AD5687_DelayCycles(dev->spi_delay_cycles);
}

/* 向近端器件发送24位帧。 */
static HAL_StatusTypeDef AD5687_WriteFrame24(AD5687_HandleTypeDef *dev,
                                             uint32_t frame)
{
  int8_t bit;

  if (dev == NULL)
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
  for (bit = 23; bit >= 0; bit--)
  {
    AD5687_WriteBit(dev,
                    ((frame >> (uint8_t)bit) & 0x01U) != 0U ?
                    GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(dev->mosi_port, dev->mosi_pin, GPIO_PIN_RESET);

  return HAL_OK;
}

/* 向级联器件发送48位帧。 */
static HAL_StatusTypeDef AD5687_WriteFrame48(AD5687_HandleTypeDef *dev,
                                             AD5687_DeviceTypeDef device,
                                             uint32_t frame)
{
  uint32_t device_1_frame;
  uint32_t device_2_frame;
  int8_t bit;

  if ((dev == NULL) ||
      (AD5687_IsValidDevice(device) == 0U))
  {
    return HAL_ERROR;
  }

  device_1_frame =
    (device == AD5687_DEVICE_CV3_CV4) ?
    ((uint32_t)AD5687_CMD_DAISY_NOP << 20) :
    (frame & 0x00FFFFFFUL);
  device_2_frame =
    (device == AD5687_DEVICE_CV1_CV2) ?
    ((uint32_t)AD5687_CMD_DAISY_NOP << 20) :
    (frame & 0x00FFFFFFUL);

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);

  /* 级联链路先发送远端CV3/CV4，再发送近端CV1/CV2。 */
  for (bit = 23; bit >= 0; bit--)
  {
    AD5687_WriteBit(dev,
                    ((device_2_frame >> (uint8_t)bit) & 0x01U) != 0U ?
                    GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  for (bit = 23; bit >= 0; bit--)
  {
    AD5687_WriteBit(dev,
                    ((device_1_frame >> (uint8_t)bit) & 0x01U) != 0U ?
                    GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(dev->mosi_port, dev->mosi_pin, GPIO_PIN_RESET);

  return HAL_OK;
}

/* 组装并发送DAC控制命令。 */
static HAL_StatusTypeDef AD5687_WriteDacCommand(AD5687_HandleTypeDef *dev,
                                                AD5687_DeviceTypeDef device,
                                                uint8_t command,
                                                AD5687_ChannelTypeDef channel,
                                                uint16_t code)
{
  uint32_t frame;

  if (AD5687_IsValidChannel(channel) == 0U)
  {
    return HAL_ERROR;
  }

  frame = ((uint32_t)(command & 0x0FU) << 20) |
          ((uint32_t)((uint8_t)channel & 0x0FU) << 16) |
          ((uint32_t)(code & AD5687_MAX_CODE) << 4);

  return AD5687_WriteFrame48(dev, device, frame);
}
