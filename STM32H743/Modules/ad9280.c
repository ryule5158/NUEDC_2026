#include "ad9280.h"

#define AD9280_REG_CONTROL             0x20U /* 采集启动和终止寄存器地址。 */
#define AD9280_REG_STATUS              0x21U /* 采集状态寄存器地址。 */
#define AD9280_REG_SAMPLE_COUNT        0x22U /* 目标保存点数寄存器地址。 */
#define AD9280_REG_DECIMATION          0x23U /* FPGA抽取倍数寄存器地址。 */
#define AD9280_REG_TRIGGER             0x24U /* 触发模式和阈值寄存器地址。 */
#define AD9280_REG_READ_ADDR           0x25U /* 采样缓存读地址寄存器地址。 */
#define AD9280_REG_READ_DATA           0x26U /* 采样缓存数据寄存器地址。 */
#define AD9280_REG_CAPTURED_COUNT      0x27U /* 实际保存点数寄存器地址。 */
#define AD9280_REG_OVERRANGE_COUNT     0x28U /* 越界采样时钟计数寄存器地址。 */
#define AD9280_REG_SAMPLE_CLK_HZ       0x29U /* ADC物理采样时钟寄存器地址。 */
#define AD9280_REG_DEVICE_ID           0x2AU /* FPGA采集单元设备标识地址。 */
#define AD9280_REG_FIRMWARE_VERSION    0x2BU /* FPGA采集协议版本地址。 */
#define AD9280_REG_MIN_MAX             0x2CU /* 样本最小值和最大值地址。 */
#define AD9280_REG_SUM                 0x2DU /* 样本累加和地址。 */
#define AD9280_REG_LATEST              0x2EU /* 最近物理采样码地址。 */
#define AD9280_REG_FILTER              0x2FU /* bit0 enables the G-mode CIC3/R16 filter. */

#define AD9280_DEVICE_ID               0xAD928001UL /* 预期FPGA采集单元标识。 */
#define AD9280_CONTROL_START           0x00000001UL /* 启动一次采集的控制位。 */
#define AD9280_CONTROL_ABORT           0x00000002UL /* 终止当前采集的控制位。 */
#define AD9280_STATUS_READY            0x00000001UL /* ADC采样时钟锁定状态位。 */
#define AD9280_STATUS_BUSY             0x00000002UL /* FPGA采集忙状态位。 */
#define AD9280_STATUS_DONE             0x00000004UL /* 完整采样块就绪状态位。 */
#define AD9280_STATUS_TRIGGERED        0x00000008UL /* 已触发状态位。 */
#define AD9280_STATUS_OVERRANGE        0x00000010UL /* 采集期间越界状态位。 */
#define AD9280_STATUS_WAIT_TRIGGER     0x00000020UL /* 等待阈值触发状态位。 */
#define AD9280_READ_DATA_ADDR_SHIFT    16U          /* Q8.8缓存返回地址的位移。 */
#define AD9280_READ_DATA_ADDR_MASK     0x00000FFFUL /* 缓存返回地址有效位。 */
#define AD9280_READ_DATA_RESERVED_MASK 0xF0000000UL

static FPGA_LinkHandleTypeDef s_ad9280_link; /* STM32到FPGA的SPI链路句柄。 */
static AD9280_DataTypeDef s_ad9280_data;     /* AD9280底层驱动运行数据。 */
static uint8_t s_ad9280_link_ready;          /* FPGA链路初始化完成标志。 */

/* 检查底层驱动是否已初始化。 */
static AD9280_StatusTypeDef AD9280_RequireInit(void);

/* 将FPGA链路状态转换为AD9280状态。 */
static AD9280_StatusTypeDef AD9280_FromLink(FPGA_LinkStatusTypeDef status);

/* 保存并返回最近一次AD9280状态。 */
static AD9280_StatusTypeDef AD9280_Return(AD9280_StatusTypeDef status);

/* 写一个AD9280采集寄存器。 */
static AD9280_StatusTypeDef AD9280_WriteReg(uint8_t addr, uint32_t value);

/* 读一个AD9280采集寄存器。 */
static AD9280_StatusTypeDef AD9280_ReadReg(uint8_t addr, uint32_t *value);

/* 写寄存器并校验有效位回读值。 */
static AD9280_StatusTypeDef AD9280_WriteVerify(uint8_t addr,
                                               uint32_t value,
                                               uint32_t mask);

/* 结束连续事务并保留最先发生的错误。 */
static AD9280_StatusTypeDef AD9280_EndSession(AD9280_StatusTypeDef status);

/* 检查采集配置是否位于FPGA真实能力范围内。 */
static uint8_t AD9280_ConfigValid(const AD9280_CaptureConfigTypeDef *config);

/* Decode protocol two's-complement fields without implementation-defined casts. */
static int16_t AD9280_DecodeS16(uint32_t value);
static int32_t AD9280_DecodeS32(uint32_t value);

/************************************************************
 * Function :       AD9280_GetDefaultCaptureConfig
 * Comment  :       填充1024点、无抽取、立即触发的默认采集配置
 * Parameter:       config: 采集配置指针
 * Return   :       null
 ************************************************************/
void AD9280_GetDefaultCaptureConfig(AD9280_CaptureConfigTypeDef *config)
{
  if (config == NULL)
  {
    return;
  }

  config->sample_count = 1024U;
  config->decimation = AD9280_DECIMATION_MIN;
  config->trigger_mode = AD9280_TRIGGER_IMMEDIATE;
  config->trigger_threshold = 128U;
}

/************************************************************
 * Function :       AD9280_Init
 * Comment  :       初始化并核验STM32、FPGA和AD9280完整链路
 * Parameter:       hspi: 与FPGA连接的STM32硬件SPI句柄
 * Return   :       AD9280状态
 ************************************************************/
AD9280_StatusTypeDef AD9280_Init(SPI_HandleTypeDef *hspi)
{
  FPGA_LinkConfigTypeDef link_config;
  AD9280_StatusTypeDef status;
  uint32_t status_reg;
  uint32_t start_tick;

  s_ad9280_data = (AD9280_DataTypeDef){0};
  AD9280_GetDefaultCaptureConfig(&s_ad9280_data.config);
  s_ad9280_link_ready = 0U;

  if (hspi == NULL)
  {
    return AD9280_Return(AD9280_ERROR_PARAM);
  }

  FPGA_Link_GetDefaultConfig(&link_config);
  link_config.hspi = hspi;
  status = AD9280_FromLink(FPGA_Link_Init(&s_ad9280_link, &link_config));
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }
  s_ad9280_link_ready = 1U;

  status = AD9280_ReadReg(AD9280_REG_DEVICE_ID,
                          &s_ad9280_data.device_id);
  if ((status != AD9280_OK) ||
      (s_ad9280_data.device_id != AD9280_DEVICE_ID))
  {
    return AD9280_Return((status == AD9280_OK) ?
                         AD9280_ERROR_DEVICE : status);
  }

  status = AD9280_ReadReg(AD9280_REG_FIRMWARE_VERSION,
                          &s_ad9280_data.firmware_version);
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }
  if (s_ad9280_data.firmware_version !=
      AD9280_CAPTURE_PROTOCOL_Q8_8_VERSION)
  {
    /*
     * Firmware 1.1/1.2 returns unsigned 8-bit samples in another bit layout.
     * Reject it here so those words can never be silently decoded as Q8.8.
     */
    return AD9280_Return(AD9280_ERROR_FORMAT);
  }
  s_ad9280_data.sample_word_bits = AD9280_SAMPLE_WORD_BITS;
  s_ad9280_data.sample_fraction_bits = AD9280_SAMPLE_FRACTION_BITS;

  status = AD9280_ReadReg(AD9280_REG_SAMPLE_CLK_HZ,
                          &s_ad9280_data.sample_clock_hz);
  if ((status != AD9280_OK) ||
      (s_ad9280_data.sample_clock_hz != AD9280_SAMPLE_CLOCK_HZ))
  {
    return AD9280_Return((status == AD9280_OK) ?
                         AD9280_ERROR_DEVICE : status);
  }

  start_tick = HAL_GetTick();
  do
  {
    status = AD9280_ReadReg(AD9280_REG_STATUS, &status_reg);
    if (status != AD9280_OK)
    {
      return AD9280_Return(status);
    }
    if ((status_reg & AD9280_STATUS_READY) != 0U)
    {
      s_ad9280_data.initialized = 1U;
      s_ad9280_data.capture.adc_ready = 1U;
      return AD9280_Return(AD9280_OK);
    }
  } while ((HAL_GetTick() - start_tick) < AD9280_READY_TIMEOUT_MS);

  return AD9280_Return(AD9280_ERROR_NOT_READY);
}

/************************************************************
 * Function :       AD9280_StartCapture
 * Comment  :       写入触发和抽取配置并启动一次FPGA高速采集
 * Parameter:       config: 采集配置
 * Return   :       AD9280状态
 ************************************************************/
AD9280_StatusTypeDef AD9280_StartCapture(
    const AD9280_CaptureConfigTypeDef *config)
{
  AD9280_CaptureInfoTypeDef info;
  AD9280_StatusTypeDef status;
  uint32_t trigger;

  status = AD9280_RequireInit();
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }
  if (AD9280_ConfigValid(config) == 0U)
  {
    return AD9280_Return(AD9280_ERROR_PARAM);
  }

  status = AD9280_GetCaptureInfo(&info);
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }
  if (info.adc_ready == 0U)
  {
    return AD9280_Return(AD9280_ERROR_NOT_READY);
  }
  if (info.busy != 0U)
  {
    return AD9280_Return(AD9280_ERROR_BUSY);
  }

  trigger = ((uint32_t)config->trigger_mode << 8) |
            (uint32_t)config->trigger_threshold;
  status = AD9280_FromLink(FPGA_Link_Begin(&s_ad9280_link));
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }

  status = AD9280_WriteVerify(AD9280_REG_SAMPLE_COUNT,
                              config->sample_count,
                              0x00001FFFUL);
  if (status == AD9280_OK)
  {
    status = AD9280_WriteVerify(AD9280_REG_DECIMATION,
                                config->decimation,
                                0x0000FFFFUL);
  }
  if (status == AD9280_OK)
  {
    status = AD9280_WriteVerify(AD9280_REG_TRIGGER,
                                trigger,
                                0x000003FFUL);
  }
  if (status == AD9280_OK)
  {
    status = AD9280_WriteReg(AD9280_REG_CONTROL, AD9280_CONTROL_START);
  }
  status = AD9280_EndSession(status);

  if (status == AD9280_OK)
  {
    s_ad9280_data.config = *config;
    s_ad9280_data.read_progress = 0U;
    s_ad9280_data.capture = (AD9280_CaptureInfoTypeDef){0};
    s_ad9280_data.capture.adc_ready = 1U;
    s_ad9280_data.capture.busy = 1U;
    s_ad9280_data.capture.waiting_trigger =
        (config->trigger_mode == AD9280_TRIGGER_IMMEDIATE) ? 0U : 1U;
    s_ad9280_data.capture.stored_sample_rate_hz =
        (float)s_ad9280_data.sample_clock_hz / (float)config->decimation;
  }
  return AD9280_Return(status);
}

/************************************************************
 * Function :       AD9280_WaitCapture
 * Comment  :       轮询等待阈值触发和完整采集结束
 * Parameter:       timeout_ms: 最大等待时间，单位ms
 * Return   :       AD9280状态
 ************************************************************/
AD9280_StatusTypeDef AD9280_WaitCapture(uint32_t timeout_ms)
{
  AD9280_CaptureInfoTypeDef info;
  AD9280_StatusTypeDef status;
  uint32_t start_tick;

  status = AD9280_RequireInit();
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }
  if (timeout_ms == 0U)
  {
    return AD9280_Return(AD9280_ERROR_PARAM);
  }

  start_tick = HAL_GetTick();
  do
  {
    status = AD9280_GetCaptureInfo(&info);
    if (status != AD9280_OK)
    {
      return AD9280_Return(status);
    }
    if (info.adc_ready == 0U)
    {
      return AD9280_Return(AD9280_ERROR_NOT_READY);
    }
    if (info.done != 0U)
    {
      return AD9280_Return(AD9280_OK);
    }
    if (info.busy == 0U)
    {
      return AD9280_Return(AD9280_ERROR_NO_DATA);
    }
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  return AD9280_Return(AD9280_ERROR_TIMEOUT);
}

/************************************************************
 * Function :       AD9280_ReadCapture
 * Comment  :       连续读取FPGA块RAM中的完整采样块
 * Parameter:       samples: 输出缓存; capacity: 缓存容量;
 *                  sample_count: 实际读取点数
 * Return   :       AD9280状态
 ************************************************************/
AD9280_StatusTypeDef AD9280_ReadCapture(AD9280_SampleTypeDef *samples,
                                        uint16_t capacity,
                                        uint16_t *sample_count)
{
  AD9280_CaptureInfoTypeDef info;
  AD9280_StatusTypeDef status;
  uint32_t value;
  uint16_t index;

  status = AD9280_RequireInit();
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }
  if ((samples == NULL) || (sample_count == NULL))
  {
    return AD9280_Return(AD9280_ERROR_PARAM);
  }
  *sample_count = 0U;
  s_ad9280_data.read_progress = 0U;

  status = AD9280_GetCaptureInfo(&info);
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }
  if ((info.done == 0U) || (info.busy != 0U) ||
      (info.captured_count == 0U))
  {
    return AD9280_Return(AD9280_ERROR_NO_DATA);
  }
  if (capacity < info.captured_count)
  {
    return AD9280_Return(AD9280_ERROR_PARAM);
  }

  status = AD9280_FromLink(FPGA_Link_Begin(&s_ad9280_link));
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }
  status = AD9280_WriteReg(AD9280_REG_READ_ADDR, 0U);
  for (index = 0U;
       (index < info.captured_count) && (status == AD9280_OK);
       index++)
  {
    status = AD9280_ReadReg(AD9280_REG_READ_DATA, &value);
    if ((status == AD9280_OK) &&
        (((value >> AD9280_READ_DATA_ADDR_SHIFT) &
          AD9280_READ_DATA_ADDR_MASK) != index))
    {
      status = AD9280_WriteReg(AD9280_REG_READ_ADDR, index);
      if (status == AD9280_OK)
      {
        status = AD9280_ReadReg(AD9280_REG_READ_DATA, &value);
      }
    }
    if ((status == AD9280_OK) &&
        ((value & AD9280_READ_DATA_RESERVED_MASK) == 0U) &&
        (((value >> AD9280_READ_DATA_ADDR_SHIFT) &
          AD9280_READ_DATA_ADDR_MASK) == index))
    {
      samples[index] = AD9280_DecodeS16(value);
      s_ad9280_data.read_progress = (uint16_t)(index + 1U);
    }
    else if (status == AD9280_OK)
    {
      status = AD9280_ERROR_LINK;
    }
  }
  status = AD9280_EndSession(status);

  if (status == AD9280_OK)
  {
    *sample_count = info.captured_count;
  }
  return AD9280_Return(status);
}

/************************************************************
 * Function :       AD9280_GetCaptureInfo
 * Comment  :       读取FPGA采集状态和稳定统计快照
 * Parameter:       info: 状态输出指针
 * Return   :       AD9280状态
 ************************************************************/
AD9280_StatusTypeDef AD9280_GetCaptureInfo(AD9280_CaptureInfoTypeDef *info)
{
  AD9280_StatusTypeDef status;
  uint32_t status_reg;
  uint32_t count_reg;
  uint32_t overrange_count;
  uint32_t min_max;
  uint32_t sample_sum;
  uint32_t latest;

  status = AD9280_RequireInit();
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }
  if (info == NULL)
  {
    return AD9280_Return(AD9280_ERROR_PARAM);
  }

  status = AD9280_FromLink(FPGA_Link_Begin(&s_ad9280_link));
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }
  status = AD9280_ReadReg(AD9280_REG_STATUS, &status_reg);
  if (status == AD9280_OK)
  {
    status = AD9280_ReadReg(AD9280_REG_CAPTURED_COUNT, &count_reg);
  }
  if (status == AD9280_OK)
  {
    status = AD9280_ReadReg(AD9280_REG_OVERRANGE_COUNT,
                            &overrange_count);
  }
  if (status == AD9280_OK)
  {
    status = AD9280_ReadReg(AD9280_REG_MIN_MAX, &min_max);
  }
  if (status == AD9280_OK)
  {
    status = AD9280_ReadReg(AD9280_REG_SUM, &sample_sum);
  }
  if (status == AD9280_OK)
  {
    status = AD9280_ReadReg(AD9280_REG_LATEST, &latest);
  }
  status = AD9280_EndSession(status);
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }

  info->adc_ready = ((status_reg & AD9280_STATUS_READY) != 0U) ? 1U : 0U;
  info->busy = ((status_reg & AD9280_STATUS_BUSY) != 0U) ? 1U : 0U;
  info->done = ((status_reg & AD9280_STATUS_DONE) != 0U) ? 1U : 0U;
  info->triggered = ((status_reg & AD9280_STATUS_TRIGGERED) != 0U) ? 1U : 0U;
  info->overrange = ((status_reg & AD9280_STATUS_OVERRANGE) != 0U) ? 1U : 0U;
  info->waiting_trigger =
      ((status_reg & AD9280_STATUS_WAIT_TRIGGER) != 0U) ? 1U : 0U;
  info->captured_count = (uint16_t)(count_reg & 0x1FFFU);
  info->overrange_count = overrange_count;
  info->min_sample_q8_8 = AD9280_DecodeS16(min_max);
  info->max_sample_q8_8 = AD9280_DecodeS16(min_max >> 16);
  info->sample_sum_q8_8 = AD9280_DecodeS32(sample_sum);
  info->latest_sample_q8_8 = AD9280_DecodeS16(latest);
  info->mean_sample_q8_8 = (info->captured_count == 0U) ? 0.0f :
      (float)info->sample_sum_q8_8 / (float)info->captured_count;
  info->stored_sample_rate_hz =
      (float)s_ad9280_data.sample_clock_hz /
      (float)s_ad9280_data.config.decimation;
  s_ad9280_data.capture = *info;
  return AD9280_Return(AD9280_OK);
}

/************************************************************
 * Function :       AD9280_AbortCapture
 * Comment  :       终止当前等待触发或高速采集
 * Parameter:       null
 * Return   :       AD9280状态
 ************************************************************/
AD9280_StatusTypeDef AD9280_AbortCapture(void)
{
  AD9280_StatusTypeDef status;

  status = AD9280_RequireInit();
  if (status == AD9280_OK)
  {
    status = AD9280_WriteReg(AD9280_REG_CONTROL, AD9280_CONTROL_ABORT);
  }
  return AD9280_Return(status);
}


AD9280_StatusTypeDef AD9280_SetCic3Filter(uint8_t enable)
{
  AD9280_StatusTypeDef status;

  status = AD9280_RequireInit();
  if (status != AD9280_OK)
  {
    return AD9280_Return(status);
  }
  if (enable > 1U)
  {
    return AD9280_Return(AD9280_ERROR_PARAM);
  }
  if ((enable != 0U) &&
      (s_ad9280_data.firmware_version !=
       AD9280_CAPTURE_PROTOCOL_Q8_8_VERSION))
  {
    return AD9280_Return(AD9280_ERROR_DEVICE);
  }

  status = AD9280_WriteVerify(AD9280_REG_FILTER,
                              (uint32_t)enable,
                              0x00000001UL);
  return AD9280_Return(status);
}
/************************************************************
 * Function :       AD9280_GetData
 * Comment  :       返回AD9280底层运行数据的只读指针
 * Parameter:       null
 * Return   :       底层运行数据只读指针
 ************************************************************/
const AD9280_DataTypeDef *AD9280_GetData(void)
{
  return &s_ad9280_data;
}

/************************************************************
 * Function :       AD9280_RequireInit
 * Comment  :       检查底层驱动初始化状态
 * Parameter:       null
 * Return   :       AD9280状态
 ************************************************************/
static AD9280_StatusTypeDef AD9280_RequireInit(void)
{
  return (s_ad9280_data.initialized != 0U) ?
         AD9280_OK : AD9280_ERROR_NOT_INIT;
}

/************************************************************
 * Function :       AD9280_FromLink
 * Comment  :       将FPGA通信层错误转换为AD9280错误
 * Parameter:       status: FPGA链路状态
 * Return   :       AD9280状态
 ************************************************************/
static AD9280_StatusTypeDef AD9280_FromLink(FPGA_LinkStatusTypeDef status)
{
  if (status == FPGA_LINK_OK)
  {
    return AD9280_OK;
  }
  if (status == FPGA_LINK_ERROR_BUSY)
  {
    return AD9280_ERROR_BUSY;
  }
  return AD9280_ERROR_LINK;
}

/************************************************************
 * Function :       AD9280_Return
 * Comment  :       保存并返回最近一次底层状态
 * Parameter:       status: AD9280状态
 * Return   :       AD9280状态
 ************************************************************/
static AD9280_StatusTypeDef AD9280_Return(AD9280_StatusTypeDef status)
{
  s_ad9280_data.status = status;
  return status;
}

/************************************************************
 * Function :       AD9280_WriteReg
 * Comment  :       写一个FPGA采集寄存器
 * Parameter:       addr: 地址; value: 32位数据
 * Return   :       AD9280状态
 ************************************************************/
static AD9280_StatusTypeDef AD9280_WriteReg(uint8_t addr, uint32_t value)
{
  if (s_ad9280_link_ready == 0U)
  {
    return AD9280_ERROR_NOT_INIT;
  }
  return AD9280_FromLink(FPGA_Link_WriteReg(&s_ad9280_link, addr, value));
}

/************************************************************
 * Function :       AD9280_ReadReg
 * Comment  :       读一个FPGA采集寄存器
 * Parameter:       addr: 地址; value: 32位读数指针
 * Return   :       AD9280状态
 ************************************************************/
static AD9280_StatusTypeDef AD9280_ReadReg(uint8_t addr, uint32_t *value)
{
  if ((s_ad9280_link_ready == 0U) || (value == NULL))
  {
    return AD9280_ERROR_NOT_INIT;
  }
  return AD9280_FromLink(FPGA_Link_ReadReg(&s_ad9280_link, addr, value));
}

/************************************************************
 * Function :       AD9280_WriteVerify
 * Comment  :       写入寄存器并校验有效位回读值
 * Parameter:       addr: 地址; value: 写入值; mask: 有效位掩码
 * Return   :       AD9280状态
 ************************************************************/
static AD9280_StatusTypeDef AD9280_WriteVerify(uint8_t addr,
                                               uint32_t value,
                                               uint32_t mask)
{
  AD9280_StatusTypeDef status;
  uint32_t readback;

  status = AD9280_WriteReg(addr, value);
  if (status == AD9280_OK)
  {
    status = AD9280_ReadReg(addr, &readback);
  }
  if ((status == AD9280_OK) &&
      ((readback & mask) != (value & mask)))
  {
    status = AD9280_ERROR_LINK;
  }
  return status;
}

/************************************************************
 * Function :       AD9280_EndSession
 * Comment  :       结束连续事务并保留首个错误
 * Parameter:       status: 事务内已有状态
 * Return   :       AD9280状态
 ************************************************************/
static AD9280_StatusTypeDef AD9280_EndSession(AD9280_StatusTypeDef status)
{
  AD9280_StatusTypeDef end_status;

  end_status = AD9280_FromLink(FPGA_Link_End(&s_ad9280_link));
  return (status == AD9280_OK) ? end_status : status;
}

/************************************************************
 * Function :       AD9280_ConfigValid
 * Comment  :       检查采集点数、抽取倍数和触发模式
 * Parameter:       config: 采集配置
 * Return   :       1有效，0无效
 ************************************************************/
static uint8_t AD9280_ConfigValid(const AD9280_CaptureConfigTypeDef *config)
{
  if ((config == NULL) ||
      (config->sample_count == 0U) ||
      (config->sample_count > AD9280_BUFFER_MAX_SAMPLES) ||
      (config->decimation < AD9280_DECIMATION_MIN) ||
      (config->decimation > AD9280_DECIMATION_MAX) ||
      (config->trigger_mode > AD9280_TRIGGER_EITHER))
  {
    return 0U;
  }
  return 1U;
}

static int16_t AD9280_DecodeS16(uint32_t value)
{
  uint32_t magnitude;
  value &= 0x0000FFFFUL;
  if (value < 0x00008000UL)
  {
    return (int16_t)value;
  }
  magnitude = 0x00010000UL - value;
  return (int16_t)(-(int32_t)magnitude);
}

static int32_t AD9280_DecodeS32(uint32_t value)
{
  if (value <= 0x7FFFFFFFUL)
  {
    return (int32_t)value;
  }
  return -1 - (int32_t)(0xFFFFFFFFUL - value);
}
