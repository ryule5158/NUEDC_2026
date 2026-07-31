#include "ltc2208.h"

#define LTC2208_REG_CONTROL             0x20U /* 采集启动和终止寄存器地址。 */
#define LTC2208_REG_STATUS              0x21U /* 采集状态寄存器地址。 */
#define LTC2208_REG_SAMPLE_COUNT        0x22U /* 目标保存点数寄存器地址。 */
#define LTC2208_REG_DECIMATION          0x23U /* FPGA抽取倍数寄存器地址。 */
#define LTC2208_REG_TRIGGER             0x24U /* 触发模式和阈值寄存器地址。 */
#define LTC2208_REG_READ_ADDR           0x25U /* 采样缓存读地址寄存器地址。 */
#define LTC2208_REG_READ_DATA           0x26U /* 采样缓存数据寄存器地址。 */
#define LTC2208_REG_CAPTURED_COUNT      0x27U /* 实际保存点数寄存器地址。 */
#define LTC2208_REG_OVERRANGE_COUNT     0x28U /* 越界采样时钟计数寄存器地址。 */
#define LTC2208_REG_SAMPLE_CLK_HZ       0x29U /* ADC物理采样时钟寄存器地址。 */
#define LTC2208_REG_DEVICE_ID           0x2AU /* FPGA采集单元设备标识地址。 */
#define LTC2208_REG_FIRMWARE_VERSION    0x2BU /* FPGA采集协议版本地址。 */
#define LTC2208_REG_MIN_MAX             0x2CU /* 样本最小值和最大值地址。 */
#define LTC2208_REG_SUM                 0x2DU /* 样本累加和地址。 */
#define LTC2208_REG_LATEST              0x2EU /* 最近物理采样码地址。 */
#define LTC2208_REG_FILTER              0x2FU /* bit0 enables CIC5/R65 plus FIR81. */

#define LTC2208_DEVICE_ID               0x22080001UL /* 预期FPGA采集单元标识。 */
#define LTC2208_CONTROL_START           0x00000001UL /* 启动一次采集的控制位。 */
#define LTC2208_CONTROL_ABORT           0x00000002UL /* 终止当前采集的控制位。 */
#define LTC2208_STATUS_READY            0x00000001UL /* ADC采样时钟锁定状态位。 */
#define LTC2208_STATUS_BUSY             0x00000002UL /* FPGA采集忙状态位。 */
#define LTC2208_STATUS_DONE             0x00000004UL /* 完整采样块就绪状态位。 */
#define LTC2208_STATUS_TRIGGERED        0x00000008UL /* 已触发状态位。 */
#define LTC2208_STATUS_OVERRANGE        0x00000010UL /* 采集期间越界状态位。 */
#define LTC2208_STATUS_WAIT_TRIGGER     0x00000020UL /* 等待阈值触发状态位。 */
#define LTC2208_READ_DATA_ADDR_SHIFT    20U          /* Q3缓存返回地址的位移。 */
#define LTC2208_READ_DATA_ADDR_MASK     0x00000FFFUL /* 缓存返回地址有效位。 */
#define LTC2208_READ_DATA_SAMPLE_MASK   0x000FFFFFUL /* signed-20 Q3样本字段。 */

static FPGA_LinkHandleTypeDef s_ltc2208_link; /* STM32到FPGA的SPI链路句柄。 */
static LTC2208_DataTypeDef s_ltc2208_data;     /* LTC2208底层驱动运行数据。 */
static uint8_t s_ltc2208_link_ready;          /* FPGA链路初始化完成标志。 */

/* 检查底层驱动是否已初始化。 */
static LTC2208_StatusTypeDef LTC2208_RequireInit(void);

/* 将FPGA链路状态转换为LTC2208状态。 */
static LTC2208_StatusTypeDef LTC2208_FromLink(FPGA_LinkStatusTypeDef status);

/* 保存并返回最近一次LTC2208状态。 */
static LTC2208_StatusTypeDef LTC2208_Return(LTC2208_StatusTypeDef status);

/* 写一个LTC2208采集寄存器。 */
static LTC2208_StatusTypeDef LTC2208_WriteReg(uint8_t addr, uint32_t value);

/* 读一个LTC2208采集寄存器。 */
static LTC2208_StatusTypeDef LTC2208_ReadReg(uint8_t addr, uint32_t *value);

/* 写寄存器并校验有效位回读值。 */
static LTC2208_StatusTypeDef LTC2208_WriteVerify(uint8_t addr,
                                               uint32_t value,
                                               uint32_t mask);

/* 结束连续事务并保留最先发生的错误。 */
static LTC2208_StatusTypeDef LTC2208_EndSession(LTC2208_StatusTypeDef status);

/* 检查采集配置是否位于FPGA真实能力范围内。 */
static uint8_t LTC2208_ConfigValid(const LTC2208_CaptureConfigTypeDef *config);

/* Decode protocol two's-complement fields without implementation-defined casts. */
static int32_t LTC2208_DecodeS20(uint32_t value);
static int16_t LTC2208_DecodeS16(uint32_t value);
static int32_t LTC2208_DecodeS32(uint32_t value);

/************************************************************
 * Function :       LTC2208_GetDefaultCaptureConfig
 * Comment  :       填充1024点、无抽取、立即触发的默认采集配置
 * Parameter:       config: 采集配置指针
 * Return   :       null
 ************************************************************/
void LTC2208_GetDefaultCaptureConfig(LTC2208_CaptureConfigTypeDef *config)
{
  if (config == NULL)
  {
    return;
  }

  config->sample_count = 1024U;
  config->decimation = LTC2208_DECIMATION_MIN;
  config->trigger_mode = LTC2208_TRIGGER_IMMEDIATE;
  config->trigger_threshold = 32768U;
}

/************************************************************
 * Function :       LTC2208_Init
 * Comment  :       初始化并核验STM32、FPGA和LTC2208完整链路
 * Parameter:       hspi: 与FPGA连接的STM32硬件SPI句柄
 * Return   :       LTC2208状态
 ************************************************************/
LTC2208_StatusTypeDef LTC2208_Init(SPI_HandleTypeDef *hspi)
{
  FPGA_LinkConfigTypeDef link_config;
  LTC2208_StatusTypeDef status;
  uint32_t status_reg;
  uint32_t start_tick;

  s_ltc2208_data = (LTC2208_DataTypeDef){0};
  LTC2208_GetDefaultCaptureConfig(&s_ltc2208_data.config);
  s_ltc2208_link_ready = 0U;

  if (hspi == NULL)
  {
    return LTC2208_Return(LTC2208_ERROR_PARAM);
  }

  FPGA_Link_GetDefaultConfig(&link_config);
  link_config.hspi = hspi;
  status = LTC2208_FromLink(FPGA_Link_Init(&s_ltc2208_link, &link_config));
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }
  s_ltc2208_link_ready = 1U;

  status = LTC2208_ReadReg(LTC2208_REG_DEVICE_ID,
                          &s_ltc2208_data.device_id);
  if ((status != LTC2208_OK) ||
      (s_ltc2208_data.device_id != LTC2208_DEVICE_ID))
  {
    return LTC2208_Return((status == LTC2208_OK) ?
                         LTC2208_ERROR_DEVICE : status);
  }

  status = LTC2208_ReadReg(LTC2208_REG_FIRMWARE_VERSION,
                          &s_ltc2208_data.firmware_version);
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }
  if (s_ltc2208_data.firmware_version !=
      LTC2208_CAPTURE_PROTOCOL_Q3_VERSION)
  {
    /*
     * Earlier firmware returned 12-bit address plus signed-S16 payload. Reject
     * it so that layout can never be silently decoded as signed-20 Q3.
     */
    return LTC2208_Return(LTC2208_ERROR_FORMAT);
  }
  s_ltc2208_data.sample_word_bits = LTC2208_SAMPLE_WORD_BITS;
  s_ltc2208_data.sample_fraction_bits = LTC2208_SAMPLE_FRACTION_BITS;

  status = LTC2208_ReadReg(LTC2208_REG_SAMPLE_CLK_HZ,
                          &s_ltc2208_data.sample_clock_hz);
  if ((status != LTC2208_OK) ||
      (s_ltc2208_data.sample_clock_hz != LTC2208_SAMPLE_CLOCK_HZ))
  {
    return LTC2208_Return((status == LTC2208_OK) ?
                         LTC2208_ERROR_DEVICE : status);
  }

  start_tick = HAL_GetTick();
  do
  {
    status = LTC2208_ReadReg(LTC2208_REG_STATUS, &status_reg);
    if (status != LTC2208_OK)
    {
      return LTC2208_Return(status);
    }
    if ((status_reg & LTC2208_STATUS_READY) != 0U)
    {
      s_ltc2208_data.initialized = 1U;
      s_ltc2208_data.capture.adc_ready = 1U;
      return LTC2208_Return(LTC2208_OK);
    }
  } while ((HAL_GetTick() - start_tick) < LTC2208_READY_TIMEOUT_MS);

  return LTC2208_Return(LTC2208_ERROR_NOT_READY);
}

/************************************************************
 * Function :       LTC2208_StartCapture
 * Comment  :       写入触发和抽取配置并启动一次FPGA高速采集
 * Parameter:       config: 采集配置
 * Return   :       LTC2208状态
 ************************************************************/
LTC2208_StatusTypeDef LTC2208_StartCapture(
    const LTC2208_CaptureConfigTypeDef *config)
{
  LTC2208_CaptureInfoTypeDef info;
  LTC2208_StatusTypeDef status;
  uint32_t trigger;

  status = LTC2208_RequireInit();
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }
  if (LTC2208_ConfigValid(config) == 0U)
  {
    return LTC2208_Return(LTC2208_ERROR_PARAM);
  }

  status = LTC2208_GetCaptureInfo(&info);
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }
  if (info.adc_ready == 0U)
  {
    return LTC2208_Return(LTC2208_ERROR_NOT_READY);
  }
  if (info.busy != 0U)
  {
    return LTC2208_Return(LTC2208_ERROR_BUSY);
  }

  trigger = ((uint32_t)config->trigger_mode << 16) |
            (uint32_t)config->trigger_threshold;
  status = LTC2208_FromLink(FPGA_Link_Begin(&s_ltc2208_link));
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }

  status = LTC2208_WriteVerify(LTC2208_REG_SAMPLE_COUNT,
                              config->sample_count,
                              0x00001FFFUL);
  if (status == LTC2208_OK)
  {
    status = LTC2208_WriteVerify(LTC2208_REG_DECIMATION,
                                config->decimation,
                                0x0000FFFFUL);
  }
  if (status == LTC2208_OK)
  {
    status = LTC2208_WriteVerify(LTC2208_REG_TRIGGER,
                                trigger,
                                0x0003FFFFUL);
  }
  if (status == LTC2208_OK)
  {
    status = LTC2208_WriteReg(LTC2208_REG_CONTROL, LTC2208_CONTROL_START);
  }
  status = LTC2208_EndSession(status);

  if (status == LTC2208_OK)
  {
    s_ltc2208_data.config = *config;
    s_ltc2208_data.read_progress = 0U;
    s_ltc2208_data.capture = (LTC2208_CaptureInfoTypeDef){0};
    s_ltc2208_data.capture.adc_ready = 1U;
    s_ltc2208_data.capture.busy = 1U;
    s_ltc2208_data.capture.waiting_trigger =
        (config->trigger_mode == LTC2208_TRIGGER_IMMEDIATE) ? 0U : 1U;
    s_ltc2208_data.capture.stored_sample_rate_hz =
        (float)s_ltc2208_data.sample_clock_hz / (float)config->decimation;
  }
  return LTC2208_Return(status);
}

/************************************************************
 * Function :       LTC2208_WaitCapture
 * Comment  :       轮询等待阈值触发和完整采集结束
 * Parameter:       timeout_ms: 最大等待时间，单位ms
 * Return   :       LTC2208状态
 ************************************************************/
LTC2208_StatusTypeDef LTC2208_WaitCapture(uint32_t timeout_ms)
{
  LTC2208_CaptureInfoTypeDef info;
  LTC2208_StatusTypeDef status;
  uint32_t start_tick;

  status = LTC2208_RequireInit();
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }
  if (timeout_ms == 0U)
  {
    return LTC2208_Return(LTC2208_ERROR_PARAM);
  }

  start_tick = HAL_GetTick();
  do
  {
    status = LTC2208_GetCaptureInfo(&info);
    if (status != LTC2208_OK)
    {
      return LTC2208_Return(status);
    }
    if (info.adc_ready == 0U)
    {
      return LTC2208_Return(LTC2208_ERROR_NOT_READY);
    }
    if (info.done != 0U)
    {
      return LTC2208_Return(LTC2208_OK);
    }
    if (info.busy == 0U)
    {
      return LTC2208_Return(LTC2208_ERROR_NO_DATA);
    }
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  return LTC2208_Return(LTC2208_ERROR_TIMEOUT);
}

/************************************************************
 * Function :       LTC2208_ReadCapture
 * Comment  :       连续读取FPGA块RAM中的完整采样块
 * Parameter:       samples: 输出缓存; capacity: 缓存容量;
 *                  sample_count: 实际读取点数
 * Return   :       LTC2208状态
 ************************************************************/
LTC2208_StatusTypeDef LTC2208_ReadCapture(LTC2208_SampleTypeDef *samples,
                                        uint16_t capacity,
                                        uint16_t *sample_count)
{
  LTC2208_CaptureInfoTypeDef info;
  LTC2208_StatusTypeDef status;
  uint32_t value;
  uint16_t index;

  status = LTC2208_RequireInit();
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }
  if ((samples == NULL) || (sample_count == NULL))
  {
    return LTC2208_Return(LTC2208_ERROR_PARAM);
  }
  *sample_count = 0U;
  s_ltc2208_data.read_progress = 0U;

  status = LTC2208_GetCaptureInfo(&info);
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }
  if ((info.done == 0U) || (info.busy != 0U) ||
      (info.captured_count == 0U))
  {
    return LTC2208_Return(LTC2208_ERROR_NO_DATA);
  }
  if (capacity < info.captured_count)
  {
    return LTC2208_Return(LTC2208_ERROR_PARAM);
  }

  status = LTC2208_FromLink(FPGA_Link_Begin(&s_ltc2208_link));
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }
  status = LTC2208_WriteReg(LTC2208_REG_READ_ADDR, 0U);
  for (index = 0U;
       (index < info.captured_count) && (status == LTC2208_OK);
       index++)
  {
    status = LTC2208_ReadReg(LTC2208_REG_READ_DATA, &value);
    if ((status == LTC2208_OK) &&
        (((value >> LTC2208_READ_DATA_ADDR_SHIFT) &
          LTC2208_READ_DATA_ADDR_MASK) != index))
    {
      status = LTC2208_WriteReg(LTC2208_REG_READ_ADDR, index);
      if (status == LTC2208_OK)
      {
        status = LTC2208_ReadReg(LTC2208_REG_READ_DATA, &value);
      }
    }
    if ((status == LTC2208_OK) &&
        (((value >> LTC2208_READ_DATA_ADDR_SHIFT) &
          LTC2208_READ_DATA_ADDR_MASK) == index))
    {
      samples[index] = LTC2208_DecodeS20(
          value & LTC2208_READ_DATA_SAMPLE_MASK);
      s_ltc2208_data.read_progress = (uint16_t)(index + 1U);
    }
    else if (status == LTC2208_OK)
    {
      status = LTC2208_ERROR_LINK;
    }
  }
  status = LTC2208_EndSession(status);

  if (status == LTC2208_OK)
  {
    *sample_count = info.captured_count;
  }
  return LTC2208_Return(status);
}

/************************************************************
 * Function :       LTC2208_GetCaptureInfo
 * Comment  :       读取FPGA采集状态和稳定统计快照
 * Parameter:       info: 状态输出指针
 * Return   :       LTC2208状态
 ************************************************************/
LTC2208_StatusTypeDef LTC2208_GetCaptureInfo(LTC2208_CaptureInfoTypeDef *info)
{
  LTC2208_StatusTypeDef status;
  uint32_t status_reg;
  uint32_t count_reg;
  uint32_t overrange_count;
  uint32_t min_max;
  uint32_t sample_sum;
  uint32_t latest;

  status = LTC2208_RequireInit();
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }
  if (info == NULL)
  {
    return LTC2208_Return(LTC2208_ERROR_PARAM);
  }

  status = LTC2208_FromLink(FPGA_Link_Begin(&s_ltc2208_link));
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }
  status = LTC2208_ReadReg(LTC2208_REG_STATUS, &status_reg);
  if (status == LTC2208_OK)
  {
    status = LTC2208_ReadReg(LTC2208_REG_CAPTURED_COUNT, &count_reg);
  }
  if (status == LTC2208_OK)
  {
    status = LTC2208_ReadReg(LTC2208_REG_OVERRANGE_COUNT,
                            &overrange_count);
  }
  if (status == LTC2208_OK)
  {
    status = LTC2208_ReadReg(LTC2208_REG_MIN_MAX, &min_max);
  }
  if (status == LTC2208_OK)
  {
    status = LTC2208_ReadReg(LTC2208_REG_SUM, &sample_sum);
  }
  if (status == LTC2208_OK)
  {
    status = LTC2208_ReadReg(LTC2208_REG_LATEST, &latest);
  }
  status = LTC2208_EndSession(status);
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }

  info->adc_ready = ((status_reg & LTC2208_STATUS_READY) != 0U) ? 1U : 0U;
  info->busy = ((status_reg & LTC2208_STATUS_BUSY) != 0U) ? 1U : 0U;
  info->done = ((status_reg & LTC2208_STATUS_DONE) != 0U) ? 1U : 0U;
  info->triggered = ((status_reg & LTC2208_STATUS_TRIGGERED) != 0U) ? 1U : 0U;
  info->overrange = ((status_reg & LTC2208_STATUS_OVERRANGE) != 0U) ? 1U : 0U;
  info->waiting_trigger =
      ((status_reg & LTC2208_STATUS_WAIT_TRIGGER) != 0U) ? 1U : 0U;
  info->captured_count = (uint16_t)(count_reg & 0x1FFFU);
  info->overrange_count = overrange_count;
  info->min_sample_s16 = LTC2208_DecodeS16(min_max);
  info->max_sample_s16 = LTC2208_DecodeS16(min_max >> 16);
  info->sample_sum_s16 = LTC2208_DecodeS32(sample_sum);
  info->latest_sample_s16 = LTC2208_DecodeS16(latest);
  info->mean_sample_s16 = (info->captured_count == 0U) ? 0.0f :
      (float)info->sample_sum_s16 / (float)info->captured_count;
  info->stored_sample_rate_hz =
      (float)s_ltc2208_data.sample_clock_hz /
      (float)s_ltc2208_data.config.decimation;
  s_ltc2208_data.capture = *info;
  return LTC2208_Return(LTC2208_OK);
}

/************************************************************
 * Function :       LTC2208_AbortCapture
 * Comment  :       终止当前等待触发或高速采集
 * Parameter:       null
 * Return   :       LTC2208状态
 ************************************************************/
LTC2208_StatusTypeDef LTC2208_AbortCapture(void)
{
  LTC2208_StatusTypeDef status;

  status = LTC2208_RequireInit();
  if (status == LTC2208_OK)
  {
    status = LTC2208_WriteReg(LTC2208_REG_CONTROL, LTC2208_CONTROL_ABORT);
  }
  return LTC2208_Return(status);
}


LTC2208_StatusTypeDef LTC2208_SetAntiInterferenceFilter(uint8_t enable)
{
  LTC2208_StatusTypeDef status;

  status = LTC2208_RequireInit();
  if (status != LTC2208_OK)
  {
    return LTC2208_Return(status);
  }
  if (enable > 1U)
  {
    return LTC2208_Return(LTC2208_ERROR_PARAM);
  }
  if ((enable != 0U) &&
      (s_ltc2208_data.firmware_version !=
       LTC2208_CAPTURE_PROTOCOL_Q3_VERSION))
  {
    return LTC2208_Return(LTC2208_ERROR_DEVICE);
  }

  status = LTC2208_WriteVerify(LTC2208_REG_FILTER,
                              (uint32_t)enable,
                              0x00000001UL);
  return LTC2208_Return(status);
}
/************************************************************
 * Function :       LTC2208_GetData
 * Comment  :       返回LTC2208底层运行数据的只读指针
 * Parameter:       null
 * Return   :       底层运行数据只读指针
 ************************************************************/
const LTC2208_DataTypeDef *LTC2208_GetData(void)
{
  return &s_ltc2208_data;
}

/************************************************************
 * Function :       LTC2208_RequireInit
 * Comment  :       检查底层驱动初始化状态
 * Parameter:       null
 * Return   :       LTC2208状态
 ************************************************************/
static LTC2208_StatusTypeDef LTC2208_RequireInit(void)
{
  return (s_ltc2208_data.initialized != 0U) ?
         LTC2208_OK : LTC2208_ERROR_NOT_INIT;
}

/************************************************************
 * Function :       LTC2208_FromLink
 * Comment  :       将FPGA通信层错误转换为LTC2208错误
 * Parameter:       status: FPGA链路状态
 * Return   :       LTC2208状态
 ************************************************************/
static LTC2208_StatusTypeDef LTC2208_FromLink(FPGA_LinkStatusTypeDef status)
{
  if (status == FPGA_LINK_OK)
  {
    return LTC2208_OK;
  }
  if (status == FPGA_LINK_ERROR_BUSY)
  {
    return LTC2208_ERROR_BUSY;
  }
  return LTC2208_ERROR_LINK;
}

/************************************************************
 * Function :       LTC2208_Return
 * Comment  :       保存并返回最近一次底层状态
 * Parameter:       status: LTC2208状态
 * Return   :       LTC2208状态
 ************************************************************/
static LTC2208_StatusTypeDef LTC2208_Return(LTC2208_StatusTypeDef status)
{
  s_ltc2208_data.status = status;
  return status;
}

/************************************************************
 * Function :       LTC2208_WriteReg
 * Comment  :       写一个FPGA采集寄存器
 * Parameter:       addr: 地址; value: 32位数据
 * Return   :       LTC2208状态
 ************************************************************/
static LTC2208_StatusTypeDef LTC2208_WriteReg(uint8_t addr, uint32_t value)
{
  if (s_ltc2208_link_ready == 0U)
  {
    return LTC2208_ERROR_NOT_INIT;
  }
  return LTC2208_FromLink(FPGA_Link_WriteReg(&s_ltc2208_link, addr, value));
}

/************************************************************
 * Function :       LTC2208_ReadReg
 * Comment  :       读一个FPGA采集寄存器
 * Parameter:       addr: 地址; value: 32位读数指针
 * Return   :       LTC2208状态
 ************************************************************/
static LTC2208_StatusTypeDef LTC2208_ReadReg(uint8_t addr, uint32_t *value)
{
  if ((s_ltc2208_link_ready == 0U) || (value == NULL))
  {
    return LTC2208_ERROR_NOT_INIT;
  }
  return LTC2208_FromLink(FPGA_Link_ReadReg(&s_ltc2208_link, addr, value));
}

/************************************************************
 * Function :       LTC2208_WriteVerify
 * Comment  :       写入寄存器并校验有效位回读值
 * Parameter:       addr: 地址; value: 写入值; mask: 有效位掩码
 * Return   :       LTC2208状态
 ************************************************************/
static LTC2208_StatusTypeDef LTC2208_WriteVerify(uint8_t addr,
                                               uint32_t value,
                                               uint32_t mask)
{
  LTC2208_StatusTypeDef status;
  uint32_t readback;

  status = LTC2208_WriteReg(addr, value);
  if (status == LTC2208_OK)
  {
    status = LTC2208_ReadReg(addr, &readback);
  }
  if ((status == LTC2208_OK) &&
      ((readback & mask) != (value & mask)))
  {
    status = LTC2208_ERROR_LINK;
  }
  return status;
}

/************************************************************
 * Function :       LTC2208_EndSession
 * Comment  :       结束连续事务并保留首个错误
 * Parameter:       status: 事务内已有状态
 * Return   :       LTC2208状态
 ************************************************************/
static LTC2208_StatusTypeDef LTC2208_EndSession(LTC2208_StatusTypeDef status)
{
  LTC2208_StatusTypeDef end_status;

  end_status = LTC2208_FromLink(FPGA_Link_End(&s_ltc2208_link));
  return (status == LTC2208_OK) ? end_status : status;
}

/************************************************************
 * Function :       LTC2208_ConfigValid
 * Comment  :       检查采集点数、抽取倍数和触发模式
 * Parameter:       config: 采集配置
 * Return   :       1有效，0无效
 ************************************************************/
static uint8_t LTC2208_ConfigValid(const LTC2208_CaptureConfigTypeDef *config)
{
  if ((config == NULL) ||
      (config->sample_count == 0U) ||
      (config->sample_count > LTC2208_BUFFER_MAX_SAMPLES) ||
      (config->decimation < LTC2208_DECIMATION_MIN) ||
      (config->decimation > LTC2208_DECIMATION_MAX) ||
      (config->trigger_mode > LTC2208_TRIGGER_EITHER) ||
      (config->trigger_threshold > LTC2208_NATIVE_MAX_CODE))
  {
    return 0U;
  }
  return 1U;
}

static int16_t LTC2208_DecodeS16(uint32_t value)
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

static int32_t LTC2208_DecodeS20(uint32_t value)
{
  uint32_t magnitude;
  value &= LTC2208_READ_DATA_SAMPLE_MASK;
  if (value < 0x00080000UL)
  {
    return (int32_t)value;
  }
  magnitude = 0x00100000UL - value;
  return -(int32_t)magnitude;
}

static int32_t LTC2208_DecodeS32(uint32_t value)
{
  if (value <= 0x7FFFFFFFUL)
  {
    return (int32_t)value;
  }
  return -1 - (int32_t)(0xFFFFFFFFUL - value);
}
