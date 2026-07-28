#include "ad9708.h"
#include "arm_math.h"

/* Protocol 0x00020004 atomic hardware-modulation register block. */
#define AD9708_REG_MOD_CONTROL         0x15U
#define AD9708_REG_MOD_FREQ_WORD       0x16U
#define AD9708_REG_AM_GAINS            0x17U
#define AD9708_REG_FM_DEVIATION        0x18U
#define AD9708_REG_MOD_PHASE_OFFSET    0x19U
#define AD9708_FIRMWARE_VERSION_MOD    0x00020004UL
#define AD9708_MOD_REGISTER_MASK       0x00000003UL
#define AD9708_Q15_UNITY               32767UL

#define AD9708_REG_CONTROL             0x00U /* 输出控制寄存器地址。 */
#define AD9708_REG_MODE                0x01U /* 波形模式寄存器地址。 */
#define AD9708_REG_FREQ_WORD           0x02U /* DDS频率字寄存器地址。 */
#define AD9708_REG_CONSTANT            0x03U /* 恒定输出码寄存器地址。 */
#define AD9708_REG_STATUS              0x04U /* FPGA综合状态寄存器地址。 */
#define AD9708_REG_PHASE_OFFSET        0x05U /* DDS相位偏移寄存器地址。 */
#define AD9708_REG_RAM_ADDR            0x06U /* 波形RAM写地址寄存器地址。 */
#define AD9708_REG_RAM_DATA            0x07U /* 波形RAM写数据寄存器地址。 */
#define AD9708_REG_DAC_CLK_HZ          0x09U /* DAC更新时钟寄存器地址。 */
#define AD9708_REG_DEVICE_ID           0x0AU /* FPGA设备标识寄存器地址。 */
#define AD9708_REG_RAM_POINTS          0x0BU /* 波形RAM有效点数寄存器地址。 */
#define AD9708_REG_LEVEL               0x0CU /* 幅度与偏置寄存器地址。 */
#define AD9708_REG_SWEEP_LOW           0x0DU /* 扫频下限频率字寄存器地址。 */
#define AD9708_REG_SWEEP_HIGH          0x0EU /* 扫频上限频率字寄存器地址。 */
#define AD9708_REG_SWEEP_STEP          0x0FU /* 扫频步进频率字寄存器地址。 */
#define AD9708_REG_SWEEP_DWELL         0x10U /* 扫频驻留周期寄存器地址。 */
#define AD9708_REG_SWEEP_CONTROL       0x11U /* 扫频控制寄存器地址。 */
#define AD9708_REG_SWEEP_STATUS        0x12U /* 扫频状态寄存器地址。 */
#define AD9708_REG_CURRENT_FREQ        0x13U /* 当前频率字寄存器地址。 */
#define AD9708_REG_FIRMWARE_VERSION    0x14U /* FPGA固件版本寄存器地址。 */

#define AD9708_DEVICE_ID               0xAD970802UL /* 预期FPGA设备标识。 */
#define AD9708_FIRMWARE_VERSION        0x00020003UL /* 支持Q8.8幅度和偏置误差反馈的FPGA固件版本。 */
#define AD9708_CONTROL_ENABLE          0x00000001UL /* 输出使能控制位。 */
#define AD9708_CONTROL_PHASE_RESET     0x00000002UL /* DDS相位复位控制位。 */
#define AD9708_STATUS_DAC_READY        0x00000001UL /* DAC复位释放状态位。 */
#define AD9708_STATUS_ENABLE           0x00000002UL /* 输出使能状态位。 */
#define AD9708_STATUS_PLL_LOCK         0x00000004UL /* DAC时钟锁定状态位。 */
#define AD9708_STATUS_SWEEP_RUNNING    0x00000008UL /* 扫频运行状态位。 */
#define AD9708_STATUS_SWEEP_DONE       0x00000010UL /* 扫频到达端点状态位。 */
#define AD9708_STATUS_SWEEP_ENABLE     0x00000020UL /* 扫频使能状态位。 */
#define AD9708_SWEEP_STATUS_RUNNING    0x00000001UL /* 扫频状态寄存器运行位。 */
#define AD9708_SWEEP_STATUS_HOLD       0x00000002UL /* 扫频状态寄存器暂停位。 */
#define AD9708_SWEEP_STATUS_DONE       0x00000004UL /* 扫频状态寄存器完成位。 */
#define AD9708_SWEEP_STATUS_DIRECTION  0x00000008UL /* 扫频状态寄存器方向位。 */
#define AD9708_SWEEP_CONTROL_ENABLE    0x00000001UL /* 扫频控制寄存器使能位。 */
#define AD9708_SWEEP_CONTROL_MODE_POS  1U          /* 扫频模式字段最低位。 */
#define AD9708_SWEEP_CONTROL_HOLD      0x00000008UL /* 扫频暂停控制位。 */
#define AD9708_SWEEP_CONTROL_START     0x00000010UL /* 扫频启动脉冲控制位。 */
#define AD9708_SWEEP_CONTROL_DIRECTION 0x00000020UL /* 扫频初始方向控制位。 */
#define AD9708_MODE_MASK               0x00000007UL /* 波形模式有效位掩码。 */
#define AD9708_CODE_MASK               0x000000FFUL /* 8位DAC码有效位掩码。 */
#define AD9708_RAM_ADDR_MASK           0x000003FFUL /* 10位波形RAM地址掩码。 */
#define AD9708_RAM_POINTS_MASK         0x000007FFUL /* 11位波形点数掩码。 */
#define AD9708_LEVEL_MASK              0xFFFFFFFFUL /* Q8.8幅度和偏置字段掩码。 */
#define AD9708_SWEEP_CONTROL_MASK      0x0000002FUL /* 扫频控制有效位掩码。 */
#define AD9708_INTERNAL_TWO_PI          6.28318530717958647692f /* 单位圆周对应的弧度。 */
#define AD9708_INTERNAL_SINC_MIN        (-0.21723362821122166f) /* SINC归一化区间最小值。 */

static FPGA_LinkHandleTypeDef s_ad9708_link; /* STM32到FPGA的SPI链路句柄。 */
static AD9708_DataTypeDef s_ad9708_data;     /* AD9708底层驱动的最新运行状态。 */
static uint8_t s_ad9708_link_ready;          /* FPGA链路已完成初始化标志。 */
static uint8_t s_ad9708_wave_table[AD9708_RAM_MAX_POINTS]; /* 内置RAM波形缓冲区。 */
static AD9708_VoltageCalibrationTypeDef
    s_ad9708_voltage_calibration; /* 当前两点电压校准数据。 */

static AD9708_StatusTypeDef AD9708_RequireInit(void);
static AD9708_StatusTypeDef AD9708_FromLink(FPGA_LinkStatusTypeDef status);
static AD9708_StatusTypeDef AD9708_Return(AD9708_StatusTypeDef status);
static AD9708_StatusTypeDef AD9708_WriteReg(uint8_t addr, uint32_t value);
static AD9708_StatusTypeDef AD9708_ReadReg(uint8_t addr, uint32_t *value);
static AD9708_StatusTypeDef AD9708_WriteVerify(uint8_t addr,
                                               uint32_t value,
                                               uint32_t mask);
static AD9708_StatusTypeDef AD9708_EndSession(AD9708_StatusTypeDef status);
static uint8_t AD9708_FrequencyValid(float output_hz);
static uint32_t AD9708_MakeSweepControl(uint8_t enable,
                                       uint8_t hold,
                                       uint8_t start);

/************************************************************
 * Function :       AD9708_Init
 * Comment  :       初始化并核验STM32、FPGA和高速DAC完整链路
 * Parameter:       hspi: 与FPGA连接的STM32 SPI句柄
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_Init(SPI_HandleTypeDef *hspi)
{
  FPGA_LinkConfigTypeDef cfg;
  AD9708_StatusTypeDef status;
  uint32_t value;

  s_ad9708_data = (AD9708_DataTypeDef){0};
  s_ad9708_data.last_code = AD9708_MID_CODE;
  s_ad9708_data.amplitude = AD9708_MAX_AMPLITUDE;
  s_ad9708_data.amplitude_fraction = 0U;
  s_ad9708_data.offset = AD9708_MID_CODE;
  s_ad9708_data.offset_fraction = 0U;
  s_ad9708_data.ram_points = AD9708_RAM_MAX_POINTS;
  s_ad9708_data.mode = AD9708_MODE_CONSTANT;
  s_ad9708_data.sweep_mode = AD9708_SWEEP_BIDIRECTIONAL;
  s_ad9708_data.sweep_direction = AD9708_SWEEP_UP;
  s_ad9708_link_ready = 0U;

  if (hspi == NULL)
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  FPGA_Link_GetDefaultConfig(&cfg);
  cfg.hspi = hspi;
  status = AD9708_FromLink(FPGA_Link_Init(&s_ad9708_link, &cfg));
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  s_ad9708_link_ready = 1U;

  status = AD9708_ReadReg(AD9708_REG_DEVICE_ID, &value);
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  s_ad9708_data.device_id = value;
  if (value != AD9708_DEVICE_ID)
  {
    return AD9708_Return(AD9708_ERROR_DEVICE);
  }

  status = AD9708_ReadReg(AD9708_REG_FIRMWARE_VERSION, &value);
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  s_ad9708_data.firmware_version = value;
  if ((value != AD9708_FIRMWARE_VERSION) &&
      (value != AD9708_FIRMWARE_VERSION_MOD))
  {
    return AD9708_Return(AD9708_ERROR_DEVICE);
  }
  s_ad9708_data.modulation_supported =
      (value == AD9708_FIRMWARE_VERSION_MOD) ? 1U : 0U;

  status = AD9708_ReadReg(AD9708_REG_DAC_CLK_HZ, &value);
  if ((status != AD9708_OK) || (value != AD9708_DAC_CLK_HZ))
  {
    return AD9708_Return((status == AD9708_OK) ?
                         AD9708_ERROR_DEVICE : status);
  }

  status = AD9708_WaitReady(AD9708_READY_TIMEOUT_MS);
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  s_ad9708_data.initialized = 1U;
  status = AD9708_Reset();
  if (status != AD9708_OK)
  {
    s_ad9708_data.initialized = 0U;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_Reset
 * Comment  :       恢复安全默认配置并将DAC停在中点码
 * Parameter:       null
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_Reset(void)
{
  AD9708_StatusTypeDef status;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  status = AD9708_FromLink(FPGA_Link_Begin(&s_ad9708_link));
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  status = AD9708_WriteVerify(AD9708_REG_SWEEP_CONTROL,
                              0U,
                              AD9708_SWEEP_CONTROL_MASK);
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_CONTROL,
                                0U,
                                AD9708_CONTROL_ENABLE);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_MODE,
                                AD9708_MODE_CONSTANT,
                                AD9708_MODE_MASK);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_FREQ_WORD,
                                0U,
                                0xFFFFFFFFUL);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_CONSTANT,
                                AD9708_MID_CODE,
                                AD9708_CODE_MASK);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_PHASE_OFFSET,
                                0U,
                                0xFFFFFFFFUL);
  }
  if ((status == AD9708_OK) &&
      (s_ad9708_data.modulation_supported != 0U))
  {
    status = AD9708_WriteVerify(AD9708_REG_MOD_FREQ_WORD,
                                0U,
                                0xFFFFFFFFUL);
  }
  if ((status == AD9708_OK) &&
      (s_ad9708_data.modulation_supported != 0U))
  {
    status = AD9708_WriteVerify(AD9708_REG_AM_GAINS,
                                AD9708_Q15_UNITY,
                                0xFFFFFFFFUL);
  }
  if ((status == AD9708_OK) &&
      (s_ad9708_data.modulation_supported != 0U))
  {
    status = AD9708_WriteVerify(AD9708_REG_FM_DEVIATION,
                                0U,
                                0xFFFFFFFFUL);
  }
  if ((status == AD9708_OK) &&
      (s_ad9708_data.modulation_supported != 0U))
  {
    status = AD9708_WriteVerify(AD9708_REG_MOD_PHASE_OFFSET,
                                0U,
                                0xFFFFFFFFUL);
  }
  if ((status == AD9708_OK) &&
      (s_ad9708_data.modulation_supported != 0U))
  {
    /* MOD_CONTROL is deliberately last: it commits all staged fields. */
    status = AD9708_WriteVerify(AD9708_REG_MOD_CONTROL,
                                AD9708_MODULATION_OFF,
                                AD9708_MOD_REGISTER_MASK);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_RAM_POINTS,
                                AD9708_RAM_MAX_POINTS,
                                AD9708_RAM_POINTS_MASK);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(
        AD9708_REG_LEVEL,
        ((uint32_t)AD9708_MAX_AMPLITUDE << 8) | AD9708_MID_CODE,
        AD9708_LEVEL_MASK);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_CONTROL,
                                AD9708_CONTROL_PHASE_RESET,
                                AD9708_CONTROL_ENABLE);
  }
  status = AD9708_EndSession(status);

  if (status == AD9708_OK)
  {
    s_ad9708_data.enabled = 0U;
    s_ad9708_data.sweep_enabled = 0U;
    s_ad9708_data.sweep_running = 0U;
    s_ad9708_data.sweep_hold = 0U;
    s_ad9708_data.sweep_done = 0U;
    s_ad9708_data.last_code = AD9708_MID_CODE;
    s_ad9708_data.amplitude = AD9708_MAX_AMPLITUDE;
    s_ad9708_data.amplitude_fraction = 0U;
    s_ad9708_data.offset = AD9708_MID_CODE;
    s_ad9708_data.offset_fraction = 0U;
    s_ad9708_data.ram_points = AD9708_RAM_MAX_POINTS;
    s_ad9708_data.mode = AD9708_MODE_CONSTANT;
    s_ad9708_data.output_hz = 0.0f;
    s_ad9708_data.current_hz = 0.0f;
    s_ad9708_data.phase_deg = 0.0f;
    s_ad9708_data.freq_word = 0U;
    s_ad9708_data.phase_word = 0U;
    s_ad9708_data.modulation_mode = AD9708_MODULATION_OFF;
    s_ad9708_data.modulation_hz = 0.0f;
    s_ad9708_data.am_depth = 0.0f;
    s_ad9708_data.fm_deviation_hz = 0.0f;
    s_ad9708_data.modulation_phase_deg = 0.0f;
    s_ad9708_data.modulation_freq_word = 0U;
    s_ad9708_data.modulation_phase_word = 0U;
    s_ad9708_data.sweep_low_hz = 0.0f;
    s_ad9708_data.sweep_high_hz = 0.0f;
    s_ad9708_data.sweep_step_hz = 0.0f;
    s_ad9708_data.sweep_dwell_cycles = 0U;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_PollStatus
 * Comment  :       读取FPGA时钟、输出和硬件扫频状态
 * Parameter:       null
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_PollStatus(void)
{
  AD9708_StatusTypeDef status;
  uint32_t status_reg;
  uint32_t sweep_reg;

  if (s_ad9708_link_ready == 0U)
  {
    return AD9708_Return(AD9708_ERROR_NOT_INIT);
  }

  status = AD9708_FromLink(FPGA_Link_Begin(&s_ad9708_link));
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  status = AD9708_ReadReg(AD9708_REG_STATUS, &status_reg);
  if (status == AD9708_OK)
  {
    status = AD9708_ReadReg(AD9708_REG_SWEEP_STATUS, &sweep_reg);
  }
  status = AD9708_EndSession(status);
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  s_ad9708_data.dac_ready =
      ((status_reg & AD9708_STATUS_DAC_READY) != 0U) ? 1U : 0U;
  s_ad9708_data.enabled =
      ((status_reg & AD9708_STATUS_ENABLE) != 0U) ? 1U : 0U;
  s_ad9708_data.pll_locked =
      ((status_reg & AD9708_STATUS_PLL_LOCK) != 0U) ? 1U : 0U;
  s_ad9708_data.sweep_enabled =
      ((status_reg & AD9708_STATUS_SWEEP_ENABLE) != 0U) ? 1U : 0U;
  s_ad9708_data.sweep_running =
      ((sweep_reg & AD9708_SWEEP_STATUS_RUNNING) != 0U) ? 1U : 0U;
  s_ad9708_data.sweep_hold =
      ((sweep_reg & AD9708_SWEEP_STATUS_HOLD) != 0U) ? 1U : 0U;
  s_ad9708_data.sweep_done =
      ((sweep_reg & AD9708_SWEEP_STATUS_DONE) != 0U) ? 1U : 0U;
  s_ad9708_data.sweep_direction =
      ((sweep_reg & AD9708_SWEEP_STATUS_DIRECTION) != 0U) ?
      AD9708_SWEEP_UP : AD9708_SWEEP_DOWN;
  return AD9708_Return(AD9708_OK);
}

/************************************************************
 * Function :       AD9708_WaitReady
 * Comment  :       等待FPGA的125MHz DAC时钟锁定
 * Parameter:       timeout_ms: 最大等待时间，单位ms
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_WaitReady(uint32_t timeout_ms)
{
  AD9708_StatusTypeDef status;
  uint32_t start_tick;

  if ((s_ad9708_link_ready == 0U) || (timeout_ms == 0U))
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  start_tick = HAL_GetTick();
  do
  {
    status = AD9708_PollStatus();
    if (status != AD9708_OK)
    {
      return AD9708_Return(status);
    }
    if ((s_ad9708_data.dac_ready != 0U) &&
        (s_ad9708_data.pll_locked != 0U))
    {
      return AD9708_Return(AD9708_OK);
    }
    HAL_Delay(1U);
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  return AD9708_Return(AD9708_ERROR_PLL);
}

/************************************************************
 * Function :       AD9708_SetEnable
 * Comment  :       启用波形输出或将输出安全停在中点码
 * Parameter:       enable: 0停止，非0启用
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_SetEnable(uint8_t enable)
{
  AD9708_StatusTypeDef status;
  uint32_t control;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  if (enable != 0U)
  {
    status = AD9708_PollStatus();
    if (status != AD9708_OK)
    {
      return AD9708_Return(status);
    }
    if ((s_ad9708_data.dac_ready == 0U) ||
        (s_ad9708_data.pll_locked == 0U))
    {
      return AD9708_Return(AD9708_ERROR_PLL);
    }
  }

  control = (enable != 0U) ? AD9708_CONTROL_ENABLE : 0U;
  status = AD9708_WriteVerify(AD9708_REG_CONTROL,
                              control,
                              AD9708_CONTROL_ENABLE);
  if (status == AD9708_OK)
  {
    s_ad9708_data.enabled = (enable != 0U) ? 1U : 0U;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_ResetPhase
 * Comment  :       将DDS累加器立即重置到已配置相位
 * Parameter:       null
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_ResetPhase(void)
{
  AD9708_StatusTypeDef status;
  uint32_t control;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  control = AD9708_CONTROL_PHASE_RESET;
  if (s_ad9708_data.enabled != 0U)
  {
    control |= AD9708_CONTROL_ENABLE;
  }
  status = AD9708_WriteVerify(AD9708_REG_CONTROL,
                              control,
                              AD9708_CONTROL_ENABLE);
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_SetMode
 * Comment  :       设置FPGA波形发生模式
 * Parameter:       mode: 波形模式
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_SetMode(AD9708_ModeTypeDef mode)
{
  AD9708_StatusTypeDef status;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if (mode > AD9708_MODE_SQUARE)
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  status = AD9708_WriteVerify(AD9708_REG_MODE,
                              (uint32_t)mode,
                              AD9708_MODE_MASK);
  if (status == AD9708_OK)
  {
    s_ad9708_data.mode = mode;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_SetFrequencyHz
 * Comment  :       按Hz设置基础DDS频率并记录量化后的实际值
 * Parameter:       output_hz: 目标频率，范围大于0且不超过50MHz
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_SetFrequencyHz(float output_hz)
{
  AD9708_StatusTypeDef status;

  if (AD9708_FrequencyValid(output_hz) == 0U)
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  status = AD9708_SetFrequencyWord(AD9708_HzToFreqWord(output_hz));
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_SetFrequencyWord
 * Comment  :       直接设置基础DDS频率字
 * Parameter:       freq_word: 32位DDS频率字
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_SetFrequencyWord(uint32_t freq_word)
{
  AD9708_StatusTypeDef status;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if ((freq_word == 0U) ||
      (freq_word > AD9708_HzToFreqWord(AD9708_MAX_OUTPUT_HZ)))
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  status = AD9708_WriteVerify(AD9708_REG_FREQ_WORD,
                              freq_word,
                              0xFFFFFFFFUL);
  if (status == AD9708_OK)
  {
    s_ad9708_data.freq_word = freq_word;
    s_ad9708_data.output_hz = AD9708_FreqWordToHz(freq_word);
    if (s_ad9708_data.sweep_running == 0U)
    {
      s_ad9708_data.current_hz = s_ad9708_data.output_hz;
    }
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_SetPhaseOffsetWord
 * Comment  :       设置32位DDS相位偏移字
 * Parameter:       phase_word: 0~2^32-1对应0~360度
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_SetPhaseOffsetWord(uint32_t phase_word)
{
  AD9708_StatusTypeDef status;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  status = AD9708_WriteVerify(AD9708_REG_PHASE_OFFSET,
                              phase_word,
                              0xFFFFFFFFUL);
  if (status == AD9708_OK)
  {
    s_ad9708_data.phase_word = phase_word;
    s_ad9708_data.phase_deg =
        (float)(((double)phase_word * 360.0) / 4294967296.0);
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_UpdateDdsFast
 * Comment  :       PLL fast path: update only changed DDS registers in one
 *                  SPI session. Link ACK is checked for every write, while
 *                  slow readback verification remains in the setup APIs.
 * Parameter:       output_hz: target frequency when enabled;
 *                  phase_word: 32-bit phase offset;
 *                  enable: zero disables output and forces the FTW to zero.
 * Return   :       AD9708 status
************************************************************/
AD9708_StatusTypeDef AD9708_UpdateDdsFast(float output_hz,
                                          uint32_t phase_word,
                                          uint8_t enable)
{
  AD9708_StatusTypeDef status;
  uint32_t freq_word;
  uint8_t target_enable;
  uint8_t write_frequency;
  uint8_t write_phase;
  uint8_t write_enable;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  target_enable = (enable != 0U) ? 1U : 0U;
  if (target_enable != 0U)
  {
    if (AD9708_FrequencyValid(output_hz) == 0U)
    {
      return AD9708_Return(AD9708_ERROR_PARAM);
    }
    freq_word = AD9708_HzToFreqWord(output_hz);
  }
  else
  {
    freq_word = 0U;
  }

  write_frequency = (freq_word != s_ad9708_data.freq_word) ? 1U : 0U;
  write_phase = (phase_word != s_ad9708_data.phase_word) ? 1U : 0U;
  write_enable = (target_enable != s_ad9708_data.enabled) ? 1U : 0U;
  if ((write_frequency == 0U) &&
      (write_phase == 0U) &&
      (write_enable == 0U))
  {
    return AD9708_Return(AD9708_OK);
  }

  status = AD9708_FromLink(FPGA_Link_Begin(&s_ad9708_link));
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if (write_frequency != 0U)
  {
    status = AD9708_WriteReg(AD9708_REG_FREQ_WORD, freq_word);
  }
  if ((status == AD9708_OK) && (write_phase != 0U))
  {
    status = AD9708_WriteReg(AD9708_REG_PHASE_OFFSET, phase_word);
  }
  if ((status == AD9708_OK) && (write_enable != 0U))
  {
    status = AD9708_WriteReg(
        AD9708_REG_CONTROL,
        (target_enable != 0U) ? AD9708_CONTROL_ENABLE : 0U);
  }
  status = AD9708_EndSession(status);

  if (status == AD9708_OK)
  {
    s_ad9708_data.freq_word = freq_word;
    s_ad9708_data.output_hz =
        (target_enable != 0U) ? AD9708_FreqWordToHz(freq_word) : 0.0f;
    if (s_ad9708_data.sweep_running == 0U)
    {
      s_ad9708_data.current_hz = s_ad9708_data.output_hz;
    }
    s_ad9708_data.phase_word = phase_word;
    s_ad9708_data.phase_deg =
        (float)(((double)phase_word * 360.0) / 4294967296.0);
    s_ad9708_data.enabled = target_enable;
  }
  return AD9708_Return(status);
}

/*
 * Stage every hardware-modulation field and write MOD_CONTROL last. The FPGA
 * uses that final write as one atomic cfg-clock to DAC-clock commit event.
 */
static AD9708_StatusTypeDef AD9708_ConfigureModulationInternal(
    AD9708_ModulationModeTypeDef mode,
    float modulation_hz,
    float parameter,
    uint32_t modulation_phase_word)
{
  AD9708_StatusTypeDef status;
  uint32_t modulation_word = 0U;
  uint32_t deviation_word = 0U;
  uint32_t carrier_gain = AD9708_Q15_UNITY;
  uint32_t side_gain = 0U;
  uint32_t packed_gains;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if (mode > AD9708_MODULATION_FM)
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }
  if (s_ad9708_data.modulation_supported == 0U)
  {
    /* Disabling is a harmless no-op on the legacy 0x00020003 bitstream. */
    if (mode == AD9708_MODULATION_OFF)
    {
      s_ad9708_data.modulation_mode = AD9708_MODULATION_OFF;
      return AD9708_Return(AD9708_OK);
    }
    return AD9708_Return(AD9708_ERROR_DEVICE);
  }

  if (mode != AD9708_MODULATION_OFF)
  {
    if (!(modulation_hz > 0.0f) ||
        (modulation_hz > AD9708_MAX_OUTPUT_HZ))
    {
      return AD9708_Return(AD9708_ERROR_PARAM);
    }
    modulation_word = AD9708_HzToFreqWord(modulation_hz);
    if (modulation_word == 0U)
    {
      return AD9708_Return(AD9708_ERROR_PARAM);
    }
  }

  if (mode == AD9708_MODULATION_AM)
  {
    if (!(parameter >= 0.0f) || !(parameter <= 1.0f))
    {
      return AD9708_Return(AD9708_ERROR_PARAM);
    }
    if ((s_ad9708_data.output_hz > 0.0f) &&
        ((modulation_hz >= s_ad9708_data.output_hz) ||
         ((s_ad9708_data.output_hz + modulation_hz) >
          AD9708_MAX_OUTPUT_HZ)))
    {
      return AD9708_Return(AD9708_ERROR_PARAM);
    }

    /*
     * Normalize A*(1+m*sin) by 1+m. This preserves the requested modulation
     * depth while guaranteeing carrier_gain+side_gain <= Q1.15 full scale.
     */
    carrier_gain = (uint32_t)
        ((32767.0 / (1.0 + (double)parameter)) + 0.5);
    side_gain = (uint32_t)
        (((double)carrier_gain * (double)parameter) + 0.5);
    if ((carrier_gain + side_gain) > AD9708_Q15_UNITY)
    {
      side_gain = AD9708_Q15_UNITY - carrier_gain;
    }
  }
  else if (mode == AD9708_MODULATION_FM)
  {
    if (!(parameter > 0.0f) || (parameter > AD9708_MAX_OUTPUT_HZ))
    {
      return AD9708_Return(AD9708_ERROR_PARAM);
    }
    if ((s_ad9708_data.output_hz > 0.0f) &&
        ((parameter >= s_ad9708_data.output_hz) ||
         ((s_ad9708_data.output_hz + parameter) >
          AD9708_MAX_OUTPUT_HZ)))
    {
      return AD9708_Return(AD9708_ERROR_PARAM);
    }
    deviation_word = AD9708_HzToFreqWord(parameter);
    if (deviation_word == 0U)
    {
      return AD9708_Return(AD9708_ERROR_PARAM);
    }
  }

  packed_gains = (side_gain << 16) | carrier_gain;
  status = AD9708_FromLink(FPGA_Link_Begin(&s_ad9708_link));
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  status = AD9708_WriteVerify(AD9708_REG_MOD_FREQ_WORD,
                              modulation_word,
                              0xFFFFFFFFUL);
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_AM_GAINS,
                                packed_gains,
                                0xFFFFFFFFUL);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_FM_DEVIATION,
                                deviation_word,
                                0xFFFFFFFFUL);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_MOD_PHASE_OFFSET,
                                modulation_phase_word,
                                0xFFFFFFFFUL);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_MOD_CONTROL,
                                (uint32_t)mode,
                                AD9708_MOD_REGISTER_MASK);
  }
  status = AD9708_EndSession(status);

  if (status == AD9708_OK)
  {
    s_ad9708_data.modulation_mode = mode;
    s_ad9708_data.modulation_freq_word = modulation_word;
    s_ad9708_data.modulation_hz =
        (mode == AD9708_MODULATION_OFF) ?
        0.0f : AD9708_FreqWordToHz(modulation_word);
    s_ad9708_data.am_depth =
        (mode == AD9708_MODULATION_AM) ? parameter : 0.0f;
    s_ad9708_data.fm_deviation_hz =
        (mode == AD9708_MODULATION_FM) ?
        AD9708_FreqWordToHz(deviation_word) : 0.0f;
    s_ad9708_data.modulation_phase_word = modulation_phase_word;
    s_ad9708_data.modulation_phase_deg =
        (float)(((double)modulation_phase_word * 360.0) / 4294967296.0);
  }
  return AD9708_Return(status);
}

AD9708_StatusTypeDef AD9708_ConfigureAm(float modulation_hz,
                                        float depth,
                                        uint32_t modulation_phase_word)
{
  return AD9708_ConfigureModulationInternal(AD9708_MODULATION_AM,
                                            modulation_hz,
                                            depth,
                                            modulation_phase_word);
}

AD9708_StatusTypeDef AD9708_ConfigureFm(float modulation_hz,
                                        float deviation_hz,
                                        uint32_t modulation_phase_word)
{
  return AD9708_ConfigureModulationInternal(AD9708_MODULATION_FM,
                                            modulation_hz,
                                            deviation_hz,
                                            modulation_phase_word);
}

AD9708_StatusTypeDef AD9708_DisableModulation(void)
{
  return AD9708_ConfigureModulationInternal(AD9708_MODULATION_OFF,
                                            0.0f,
                                            0.0f,
                                            0U);
}

AD9708_StatusTypeDef AD9708_UpdateModulationPhaseFast(
    uint32_t modulation_phase_word)
{
  AD9708_StatusTypeDef status;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if ((s_ad9708_data.modulation_supported == 0U) ||
      (s_ad9708_data.modulation_mode == AD9708_MODULATION_OFF))
  {
    return AD9708_Return(AD9708_ERROR_DEVICE);
  }
  if (modulation_phase_word == s_ad9708_data.modulation_phase_word)
  {
    return AD9708_Return(AD9708_OK);
  }

  status = AD9708_FromLink(FPGA_Link_Begin(&s_ad9708_link));
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  status = AD9708_WriteReg(AD9708_REG_MOD_PHASE_OFFSET,
                           modulation_phase_word);
  if (status == AD9708_OK)
  {
    status = AD9708_WriteReg(AD9708_REG_MOD_CONTROL,
                             (uint32_t)s_ad9708_data.modulation_mode);
  }
  status = AD9708_EndSession(status);
  if (status == AD9708_OK)
  {
    s_ad9708_data.modulation_phase_word = modulation_phase_word;
    s_ad9708_data.modulation_phase_deg =
        (float)(((double)modulation_phase_word * 360.0) / 4294967296.0);
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_SetLevel
 * Comment  :       设置统一数字幅度和中心偏置
 * Parameter:       amplitude: 0~128; offset: 0~255
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_SetLevel(uint8_t amplitude, uint8_t offset)
{
  return AD9708_SetLevelFine((uint16_t)amplitude << 8,
                             (uint16_t)offset << 8);
}

/************************************************************
 * Function :       AD9708_SetLevelFine
 * Comment  :       设置Q8.8数字幅度和中心偏置
 * Parameter:       amplitude_q8: Q8.8幅度0~128; offset_q8: Q8.8偏置码
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_SetLevelFine(uint16_t amplitude_q8,
                                         uint16_t offset_q8)
{
  AD9708_StatusTypeDef status;
  uint32_t level;
  uint8_t amplitude;
  uint8_t amplitude_fraction;
  uint8_t offset;
  uint8_t offset_fraction;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if ((amplitude_q8 > AD9708_MAX_AMPLITUDE_Q8) ||
      ((amplitude_q8 > 0U) &&
       (amplitude_q8 < AD9708_MIN_AMPLITUDE_Q8)))
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  amplitude = (uint8_t)(amplitude_q8 >> 8);
  amplitude_fraction = (uint8_t)amplitude_q8;
  offset = (uint8_t)(offset_q8 >> 8);
  offset_fraction = (uint8_t)offset_q8;
  level = ((uint32_t)amplitude_fraction << 24) |
          ((uint32_t)offset_fraction << 16) |
          ((uint32_t)amplitude << 8) |
          (uint32_t)offset;
  status = AD9708_WriteVerify(AD9708_REG_LEVEL,
                              level,
                              AD9708_LEVEL_MASK);
  if (status == AD9708_OK)
  {
    s_ad9708_data.amplitude = amplitude;
    s_ad9708_data.amplitude_fraction = amplitude_fraction;
    s_ad9708_data.offset = offset;
    s_ad9708_data.offset_fraction = offset_fraction;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_OutputConstant
 * Comment  :       输出一个不经过幅度缩放的恒定8位DAC码
 * Parameter:       code: 0~255的DAC码
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_OutputConstant(uint8_t code)
{
  AD9708_StatusTypeDef status;

  status = AD9708_StopSweep();
  if (status == AD9708_OK)
  {
    status = AD9708_SetEnable(0U);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_SetMode(AD9708_MODE_CONSTANT);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_CONSTANT,
                                code,
                                AD9708_CODE_MASK);
  }
  if (status == AD9708_OK)
  {
    s_ad9708_data.last_code = code;
    s_ad9708_data.output_hz = 0.0f;
    s_ad9708_data.current_hz = 0.0f;
    s_ad9708_data.freq_word = 0U;
    status = AD9708_SetEnable(1U);
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_LoadWave
 * Comment  :       连续写入2~1024点任意波并核验地址和末点数据
 * Parameter:       samples: 8位采样表; points: 采样点数
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_LoadWave(const uint8_t *samples,
                                     uint16_t points)
{
  AD9708_StatusTypeDef status;
  uint32_t readback;
  uint32_t index;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if ((samples == NULL) ||
      (points < AD9708_RAM_MIN_POINTS) ||
      (points > AD9708_RAM_MAX_POINTS))
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  status = AD9708_StopSweep();
  if (status == AD9708_OK)
  {
    status = AD9708_SetEnable(0U);
  }
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  status = AD9708_FromLink(FPGA_Link_Begin(&s_ad9708_link));
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  status = AD9708_WriteVerify(AD9708_REG_RAM_ADDR,
                              0U,
                              AD9708_RAM_ADDR_MASK);
  for (index = 0U; (index < points) && (status == AD9708_OK); index++)
  {
    status = AD9708_WriteReg(AD9708_REG_RAM_DATA, samples[index]);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_RAM_POINTS,
                                points,
                                AD9708_RAM_POINTS_MASK);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_ReadReg(AD9708_REG_RAM_ADDR, &readback);
    if ((status == AD9708_OK) &&
        ((readback & AD9708_RAM_ADDR_MASK) !=
         ((uint32_t)points & AD9708_RAM_ADDR_MASK)))
    {
      status = AD9708_ERROR_VERIFY;
    }
  }
  if (status == AD9708_OK)
  {
    status = AD9708_ReadReg(AD9708_REG_RAM_DATA, &readback);
    if ((status == AD9708_OK) &&
        ((readback & AD9708_CODE_MASK) != samples[points - 1U]))
    {
      status = AD9708_ERROR_VERIFY;
    }
  }
  status = AD9708_EndSession(status);

  if (status == AD9708_OK)
  {
    s_ad9708_data.ram_points = points;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_ConfigureFrequencySweep
 * Comment  :       配置FPGA硬件线性扫频的端点、步进和驻留周期
 * Parameter:       low_hz/high_hz: 端点; step_hz: 步进;
 *                  dwell_cycles: 驻留周期; mode: 模式;
 *                  initial_direction: 手动模式初始方向
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_ConfigureFrequencySweep(
    float low_hz,
    float high_hz,
    float step_hz,
    uint32_t dwell_cycles,
    AD9708_SweepModeTypeDef mode,
    AD9708_SweepDirectionTypeDef initial_direction)
{
  AD9708_StatusTypeDef status;
  uint32_t low_word;
  uint32_t high_word;
  uint32_t step_word;
  uint32_t control;
  AD9708_SweepModeTypeDef old_mode;
  AD9708_SweepDirectionTypeDef old_direction;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if ((AD9708_FrequencyValid(low_hz) == 0U) ||
      (AD9708_FrequencyValid(high_hz) == 0U) ||
      (high_hz <= low_hz) ||
      (step_hz <= 0.0f) ||
      (step_hz > (high_hz - low_hz)) ||
      (dwell_cycles == 0U) ||
      (mode > AD9708_SWEEP_MANUAL) ||
      (initial_direction > AD9708_SWEEP_UP) ||
      ((mode != AD9708_SWEEP_MANUAL) &&
       (initial_direction != AD9708_SWEEP_UP)))
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  low_word = AD9708_HzToFreqWord(low_hz);
  high_word = AD9708_HzToFreqWord(high_hz);
  step_word = AD9708_HzToFreqWord(step_hz);
  if ((low_word == 0U) || (high_word <= low_word) ||
      (step_word == 0U) || (step_word > (high_word - low_word)))
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  status = AD9708_StopSweep();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  old_mode = s_ad9708_data.sweep_mode;
  old_direction = s_ad9708_data.sweep_direction;
  s_ad9708_data.sweep_mode = mode;
  s_ad9708_data.sweep_direction = initial_direction;
  control = AD9708_MakeSweepControl(0U, 0U, 0U);

  status = AD9708_FromLink(FPGA_Link_Begin(&s_ad9708_link));
  if (status != AD9708_OK)
  {
    s_ad9708_data.sweep_mode = old_mode;
    s_ad9708_data.sweep_direction = old_direction;
    return AD9708_Return(status);
  }
  status = AD9708_WriteVerify(AD9708_REG_FREQ_WORD,
                              low_word,
                              0xFFFFFFFFUL);
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_SWEEP_LOW,
                                low_word,
                                0xFFFFFFFFUL);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_SWEEP_HIGH,
                                high_word,
                                0xFFFFFFFFUL);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_SWEEP_STEP,
                                step_word,
                                0xFFFFFFFFUL);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_SWEEP_DWELL,
                                dwell_cycles,
                                0xFFFFFFFFUL);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_SWEEP_CONTROL,
                                control,
                                AD9708_SWEEP_CONTROL_MASK);
  }
  status = AD9708_EndSession(status);

  if (status == AD9708_OK)
  {
    s_ad9708_data.sweep_enabled = 0U;
    s_ad9708_data.sweep_running = 0U;
    s_ad9708_data.sweep_hold = 0U;
    s_ad9708_data.sweep_done = 0U;
    s_ad9708_data.freq_word = low_word;
    s_ad9708_data.output_hz = AD9708_FreqWordToHz(low_word);
    s_ad9708_data.current_hz = s_ad9708_data.output_hz;
    s_ad9708_data.sweep_low_hz = s_ad9708_data.output_hz;
    s_ad9708_data.sweep_high_hz = AD9708_FreqWordToHz(high_word);
    s_ad9708_data.sweep_step_hz = AD9708_FreqWordToHz(step_word);
    s_ad9708_data.sweep_dwell_cycles = dwell_cycles;
  }
  else
  {
    s_ad9708_data.sweep_mode = old_mode;
    s_ad9708_data.sweep_direction = old_direction;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_StartSweep
 * Comment  :       从已配置端点启动FPGA硬件扫频
 * Parameter:       null
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_StartSweep(void)
{
  AD9708_StatusTypeDef status;
  uint32_t control;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if (s_ad9708_data.sweep_dwell_cycles == 0U)
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  control = AD9708_MakeSweepControl(1U, 0U, 1U);
  status = AD9708_WriteVerify(AD9708_REG_SWEEP_CONTROL,
                              control,
                              AD9708_SWEEP_CONTROL_MASK &
                              ~AD9708_SWEEP_CONTROL_START);
  if (status == AD9708_OK)
  {
    s_ad9708_data.sweep_enabled = 1U;
    s_ad9708_data.sweep_running = 1U;
    s_ad9708_data.sweep_hold = 0U;
    s_ad9708_data.sweep_done = 0U;
    s_ad9708_data.current_hz =
        (s_ad9708_data.sweep_direction == AD9708_SWEEP_UP) ?
        s_ad9708_data.sweep_low_hz : s_ad9708_data.sweep_high_hz;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_HoldSweep
 * Comment  :       暂停或继续硬件扫频并保持波形输出
 * Parameter:       hold: 0继续，非0暂停
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_HoldSweep(uint8_t hold)
{
  AD9708_StatusTypeDef status;
  uint32_t control;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if (s_ad9708_data.sweep_enabled == 0U)
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  control = AD9708_MakeSweepControl(1U, hold, 0U);
  status = AD9708_WriteVerify(AD9708_REG_SWEEP_CONTROL,
                              control,
                              AD9708_SWEEP_CONTROL_MASK);
  if (status == AD9708_OK)
  {
    s_ad9708_data.sweep_hold = (hold != 0U) ? 1U : 0U;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_SetSweepDirection
 * Comment  :       手动扫频模式下切换FPGA扫频方向
 * Parameter:       direction: 向上或向下
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_SetSweepDirection(
    AD9708_SweepDirectionTypeDef direction)
{
  AD9708_StatusTypeDef status;
  uint32_t control;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if ((s_ad9708_data.sweep_mode != AD9708_SWEEP_MANUAL) ||
      (s_ad9708_data.sweep_enabled == 0U) ||
      (direction > AD9708_SWEEP_UP))
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  s_ad9708_data.sweep_direction = direction;
  control = AD9708_MakeSweepControl(1U,
                                    s_ad9708_data.sweep_hold,
                                    0U);
  status = AD9708_WriteVerify(AD9708_REG_SWEEP_CONTROL,
                              control,
                              AD9708_SWEEP_CONTROL_MASK);
  if (status != AD9708_OK)
  {
    s_ad9708_data.sweep_direction =
        (direction == AD9708_SWEEP_UP) ?
        AD9708_SWEEP_DOWN : AD9708_SWEEP_UP;
  }
  else
  {
    s_ad9708_data.sweep_done = 0U;
    s_ad9708_data.sweep_running = 1U;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_StopSweep
 * Comment  :       停止扫频并把停止瞬间频点保存为基础频率
 * Parameter:       null
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_StopSweep(void)
{
  AD9708_StatusTypeDef status;
  uint32_t current_word;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }

  if (s_ad9708_data.sweep_enabled != 0U)
  {
    status = AD9708_HoldSweep(1U);
    if (status == AD9708_OK)
    {
      status = AD9708_ReadReg(AD9708_REG_CURRENT_FREQ, &current_word);
    }
    if (status == AD9708_OK)
    {
      status = AD9708_WriteVerify(AD9708_REG_FREQ_WORD,
                                  current_word,
                                  0xFFFFFFFFUL);
    }
    if (status == AD9708_OK)
    {
      s_ad9708_data.freq_word = current_word;
      s_ad9708_data.output_hz = AD9708_FreqWordToHz(current_word);
      s_ad9708_data.current_hz = s_ad9708_data.output_hz;
    }
  }
  else
  {
    status = AD9708_OK;
  }

  if (status == AD9708_OK)
  {
    status = AD9708_WriteVerify(AD9708_REG_SWEEP_CONTROL,
                                0U,
                                AD9708_SWEEP_CONTROL_MASK);
  }
  if (status == AD9708_OK)
  {
    s_ad9708_data.sweep_enabled = 0U;
    s_ad9708_data.sweep_running = 0U;
    s_ad9708_data.sweep_hold = 0U;
    s_ad9708_data.sweep_done = 0U;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_IsSweepDone
 * Comment  :       查询单向或手动扫频是否到达端点
 * Parameter:       done: 端点状态输出指针
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_IsSweepDone(uint8_t *done)
{
  AD9708_StatusTypeDef status;

  if (done == NULL)
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  status = AD9708_PollStatus();
  if (status == AD9708_OK)
  {
    *done = s_ad9708_data.sweep_done;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_GetCurrentFrequency
 * Comment  :       读取FPGA当前实际使用的DDS频率字并转换为Hz
 * Parameter:       current_hz: 当前频率输出指针
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_GetCurrentFrequency(float *current_hz)
{
  AD9708_StatusTypeDef status;
  uint32_t current_word;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  if (current_hz == NULL)
  {
    return AD9708_Return(AD9708_ERROR_PARAM);
  }

  status = AD9708_ReadReg(AD9708_REG_CURRENT_FREQ, &current_word);
  if (status == AD9708_OK)
  {
    s_ad9708_data.current_hz = AD9708_FreqWordToHz(current_word);
    *current_hz = s_ad9708_data.current_hz;
  }
  return AD9708_Return(status);
}

/************************************************************
 * Function :       AD9708_GetData
 * Comment  :       返回AD9708底层驱动的只读状态
 * Parameter:       null
 * Return   :       状态结构体只读指针
************************************************************/
const AD9708_DataTypeDef *AD9708_GetData(void)
{
  return &s_ad9708_data;
}

/************************************************************
 * Function :       AD9708_HzToFreqWord
 * Comment  :       将Hz转换为125MHz、32位DDS频率字
 * Parameter:       output_hz: 目标频率
 * Return   :       32位DDS频率字
************************************************************/
uint32_t AD9708_HzToFreqWord(float output_hz)
{
  double word;

  if (!(output_hz > 0.0f))
  {
    return 0U;
  }

  word = ((double)output_hz * 4294967296.0) /
         (double)AD9708_DAC_CLK_HZ;
  if (word >= 4294967295.0)
  {
    return 0xFFFFFFFFUL;
  }
  return (uint32_t)(word + 0.5);
}

/************************************************************
 * Function :       AD9708_FreqWordToHz
 * Comment  :       将32位DDS频率字转换为实际Hz
 * Parameter:       freq_word: 32位DDS频率字
 * Return   :       实际频率，单位Hz
************************************************************/
float AD9708_FreqWordToHz(uint32_t freq_word)
{
  return (float)(((double)freq_word * (double)AD9708_DAC_CLK_HZ) /
                 4294967296.0);
}

/************************************************************
 * Function :       AD9708_RequireInit
 * Comment  :       检查AD9708底层驱动是否已经初始化
 * Parameter:       null
 * Return   :       AD9708状态
************************************************************/
static AD9708_StatusTypeDef AD9708_RequireInit(void)
{
  return (s_ad9708_data.initialized != 0U) ?
         AD9708_OK : AD9708_ERROR_NOT_INIT;
}

/************************************************************
 * Function :       AD9708_FromLink
 * Comment  :       将通用FPGA链路状态转换为AD9708状态
 * Parameter:       status: FPGA链路状态
 * Return   :       AD9708状态
************************************************************/
static AD9708_StatusTypeDef AD9708_FromLink(FPGA_LinkStatusTypeDef status)
{
  if (status == FPGA_LINK_OK)
  {
    return AD9708_OK;
  }
  if (status == FPGA_LINK_ERROR_PARAM)
  {
    return AD9708_ERROR_PARAM;
  }
  return AD9708_ERROR_LINK;
}

/************************************************************
 * Function :       AD9708_Return
 * Comment  :       保存并返回最近一次AD9708状态
 * Parameter:       status: 待保存状态
 * Return   :       原状态
************************************************************/
static AD9708_StatusTypeDef AD9708_Return(AD9708_StatusTypeDef status)
{
  s_ad9708_data.status = status;
  return status;
}

/************************************************************
 * Function :       AD9708_WriteReg
 * Comment  :       写一个FPGA寄存器并转换链路状态
 * Parameter:       addr: 寄存器地址; value: 32位数据
 * Return   :       AD9708状态
************************************************************/
static AD9708_StatusTypeDef AD9708_WriteReg(uint8_t addr, uint32_t value)
{
  return AD9708_FromLink(FPGA_Link_WriteReg(&s_ad9708_link, addr, value));
}

/************************************************************
 * Function :       AD9708_ReadReg
 * Comment  :       读一个FPGA寄存器并转换链路状态
 * Parameter:       addr: 寄存器地址; value: 读数指针
 * Return   :       AD9708状态
************************************************************/
static AD9708_StatusTypeDef AD9708_ReadReg(uint8_t addr, uint32_t *value)
{
  return AD9708_FromLink(FPGA_Link_ReadReg(&s_ad9708_link, addr, value));
}

/************************************************************
 * Function :       AD9708_WriteVerify
 * Comment  :       写寄存器并按掩码核验读回值
 * Parameter:       addr: 地址; value: 写值; mask: 核验位
 * Return   :       AD9708状态
************************************************************/
static AD9708_StatusTypeDef AD9708_WriteVerify(uint8_t addr,
                                               uint32_t value,
                                               uint32_t mask)
{
  AD9708_StatusTypeDef status;
  uint32_t readback;

  status = AD9708_WriteReg(addr, value);
  if (status != AD9708_OK)
  {
    return status;
  }
  status = AD9708_ReadReg(addr, &readback);
  if ((status == AD9708_OK) &&
      ((readback & mask) != (value & mask)))
  {
    status = AD9708_ERROR_VERIFY;
  }
  return status;
}

/************************************************************
 * Function :       AD9708_EndSession
 * Comment  :       结束连续SPI事务并保留首个错误
 * Parameter:       status: 事务主体状态
 * Return   :       AD9708状态
************************************************************/
static AD9708_StatusTypeDef AD9708_EndSession(AD9708_StatusTypeDef status)
{
  AD9708_StatusTypeDef end_status;

  end_status = AD9708_FromLink(FPGA_Link_End(&s_ad9708_link));
  return (status == AD9708_OK) ? end_status : status;
}

/************************************************************
 * Function :       AD9708_FrequencyValid
 * Comment  :       检查模块配置频率上限和非零DDS频率字
 * Parameter:       output_hz: 目标频率
 * Return   :       1有效，0无效
************************************************************/
static uint8_t AD9708_FrequencyValid(float output_hz)
{
  if (!(output_hz > 0.0f) || (output_hz > AD9708_MAX_OUTPUT_HZ))
  {
    return 0U;
  }
  return (AD9708_HzToFreqWord(output_hz) != 0U) ? 1U : 0U;
}

/************************************************************
 * Function :       AD9708_MakeSweepControl
 * Comment  :       组合FPGA硬件扫频控制寄存器
 * Parameter:       enable: 使能; hold: 暂停; start: 启动脉冲
 * Return   :       扫频控制寄存器值
************************************************************/
static uint32_t AD9708_MakeSweepControl(uint8_t enable,
                                       uint8_t hold,
                                       uint8_t start)
{
  uint32_t control;

  control = ((uint32_t)s_ad9708_data.sweep_mode <<
             AD9708_SWEEP_CONTROL_MODE_POS);
  if (enable != 0U)
  {
    control |= AD9708_SWEEP_CONTROL_ENABLE;
  }
  if (hold != 0U)
  {
    control |= AD9708_SWEEP_CONTROL_HOLD;
  }
  if (start != 0U)
  {
    control |= AD9708_SWEEP_CONTROL_START;
  }
  if (s_ad9708_data.sweep_direction == AD9708_SWEEP_UP)
  {
    control |= AD9708_SWEEP_CONTROL_DIRECTION;
  }
  return control;
}
/************************************************************
 * Function :       AD9708_InternalVoltageCalibration
 * Comment  :       返回底层持有的两点电压校准数据
 * Parameter:       null
 * Return   :       可由应用层更新的校准数据指针
************************************************************/
AD9708_VoltageCalibrationTypeDef *
AD9708_InternalVoltageCalibration(void)
{
  return &s_ad9708_voltage_calibration;
}

/************************************************************
 * Function :       AD9708_InternalVoltageToCode
 * Comment  :       将校准范围内的电压换算为8位DAC码
 * Parameter:       voltage_v: 目标电压; code: DAC码输出指针
 * Return   :       AD9708状态
 ************************************************************/
AD9708_StatusTypeDef AD9708_InternalVoltageToCode(float voltage_v,
                                                   uint8_t *code)
{
  float code_value;

  if (s_ad9708_voltage_calibration.valid == 0U)
  {
    return AD9708_ERROR_CALIBRATION;
  }
  if ((code == NULL) ||
      !(voltage_v >= s_ad9708_voltage_calibration.code0_voltage_v) ||
      !(voltage_v <= s_ad9708_voltage_calibration.code255_voltage_v))
  {
    return AD9708_ERROR_PARAM;
  }

  code_value =
      (voltage_v - s_ad9708_voltage_calibration.code0_voltage_v) /
      s_ad9708_voltage_calibration.volts_per_code;
  if (code_value <= 0.0f)
  {
    *code = 0U;
  }
  else if (code_value >= (float)AD9708_MAX_CODE)
  {
    *code = AD9708_MAX_CODE;
  }
  else
  {
    *code = (uint8_t)(code_value + 0.5f);
  }
  return AD9708_OK;
}

/************************************************************
 * Function :       AD9708_InternalVoltageLevelToCode
 * Comment  :       将Vpp和中心电压换算为不削顶的Q8.8幅度与中心码
 * Parameter:       amplitude_vpp: 峰峰值; offset_v: 中心电压;
 *                  amplitude_q8/offset_q8: 码值输出指针
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_InternalVoltageLevelToCode(
    float amplitude_vpp,
    float offset_v,
    uint16_t *amplitude_q8,
    uint16_t *offset_q8)
{
  float span_v;
  float half_vpp;
  float amplitude_value;
  float offset_value;
  uint32_t negative_room;
  uint32_t positive_room;
  uint32_t max_amplitude;

  if (s_ad9708_voltage_calibration.valid == 0U)
  {
    return AD9708_ERROR_CALIBRATION;
  }
  if ((amplitude_q8 == NULL) || (offset_q8 == NULL))
  {
    return AD9708_ERROR_PARAM;
  }

  span_v = s_ad9708_voltage_calibration.code255_voltage_v -
           s_ad9708_voltage_calibration.code0_voltage_v;
  half_vpp = amplitude_vpp * 0.5f;
  if (!(amplitude_vpp >= 0.0f) ||
      !(amplitude_vpp <= span_v) ||
      !(offset_v - half_vpp >=
        s_ad9708_voltage_calibration.code0_voltage_v) ||
      !(offset_v + half_vpp <=
        s_ad9708_voltage_calibration.code255_voltage_v))
  {
    return AD9708_ERROR_PARAM;
  }

  amplitude_value = amplitude_vpp /
                    s_ad9708_voltage_calibration.volts_per_code;
  if (amplitude_value <= 0.0f)
  {
    *amplitude_q8 = 0U;
  }
  else if (amplitude_value >= (float)AD9708_MAX_CODE)
  {
    *amplitude_q8 = AD9708_MAX_AMPLITUDE_Q8;
  }
  else
  {
    *amplitude_q8 = (uint16_t)((((amplitude_value + 1.0f) * 0.5f) *
                                256.0f) +
                               0.5f);
  }

  offset_value =
      (offset_v - s_ad9708_voltage_calibration.code0_voltage_v) /
      s_ad9708_voltage_calibration.volts_per_code;
  /* 非零波形的数字中心位于Q8.8偏置码减0.5码。 */
  if (*amplitude_q8 > 0U)
  {
    offset_value += 0.5f;
  }
  if (offset_value <= 0.0f)
  {
    *offset_q8 = 0U;
  }
  else if (offset_value >= (float)AD9708_MAX_CODE)
  {
    *offset_q8 = (uint16_t)AD9708_MAX_CODE << 8;
  }
  else
  {
    *offset_q8 = (uint16_t)((offset_value * 256.0f) + 0.5f);
  }

  negative_room = *offset_q8;
  positive_room = ((uint32_t)AD9708_MAX_CODE << 8) + 256U -
                  (uint32_t)(*offset_q8);
  max_amplitude = negative_room;
  if (positive_room < max_amplitude)
  {
    max_amplitude = positive_room;
  }
  if (max_amplitude > AD9708_MAX_AMPLITUDE_Q8)
  {
    max_amplitude = AD9708_MAX_AMPLITUDE_Q8;
  }
  if ((uint32_t)(*amplitude_q8) > max_amplitude)
  {
    *amplitude_q8 = (uint16_t)max_amplitude;
  }
  if (*amplitude_q8 == 0U)
  {
    offset_value =
        (offset_v - s_ad9708_voltage_calibration.code0_voltage_v) /
        s_ad9708_voltage_calibration.volts_per_code;
    *offset_q8 = (uint16_t)((offset_value * 256.0f) + 0.5f);
  }
  return AD9708_OK;
}

/************************************************************
 * Function :       AD9708_InternalLevelValid
 * Comment  :       检查幅度和偏置组合是否会发生数字削顶
 * Parameter:       amplitude_q8: Q8.8幅度码; offset_q8: Q8.8中心码
 * Return   :       1有效，0无效
************************************************************/
static uint8_t AD9708_InternalLevelValid(uint16_t amplitude_q8,
                                    uint16_t offset_q8)
{
  uint32_t lower_limit;
  uint32_t upper_limit;

  if ((amplitude_q8 > AD9708_MAX_AMPLITUDE_Q8) ||
      ((amplitude_q8 > 0U) &&
       (amplitude_q8 < AD9708_MIN_AMPLITUDE_Q8)))
  {
    return 0U;
  }
  if (amplitude_q8 == 0U)
  {
    return (offset_q8 <= ((uint16_t)AD9708_MAX_CODE << 8)) ? 1U : 0U;
  }

  lower_limit = amplitude_q8;
  upper_limit = ((uint32_t)AD9708_MAX_CODE << 8) + 256U - amplitude_q8;
  if (((uint32_t)offset_q8 < lower_limit) ||
      ((uint32_t)offset_q8 > upper_limit))
  {
    return 0U;
  }
  return 1U;
}

/************************************************************
 * Function :       AD9708_InternalClampSample
 * Comment  :       将浮点波形值四舍五入并限制为8位DAC码
 * Parameter:       value: 浮点DAC码
 * Return   :       0~255采样值
************************************************************/
static uint8_t AD9708_InternalClampSample(float value)
{
  if (value <= 0.0f)
  {
    return 0U;
  }
  if (value >= (float)AD9708_MAX_CODE)
  {
    return AD9708_MAX_CODE;
  }
  return (uint8_t)(value + 0.5f);
}

/************************************************************
 * Function :       AD9708_InternalGenerateWave
 * Comment  :       生成满幅正弦或SINC波表供FPGA RAM播放
 * Parameter:       wave: 波形类型; points: 波形点数
 * Return   :       AD9708状态
************************************************************/
static AD9708_StatusTypeDef AD9708_InternalGenerateWave(
    AD9708_WaveformTypeDef wave,
    uint16_t points)
{
  uint32_t index;
  float x;
  float y;
  float value;

  if ((points < AD9708_RAM_MIN_POINTS) ||
      (points > AD9708_RAM_MAX_POINTS) ||
      ((wave != AD9708_WAVE_SINE) && (wave != AD9708_WAVE_SINC)))
  {
    return AD9708_ERROR_PARAM;
  }

  for (index = 0U; index < points; index++)
  {
    if (wave == AD9708_WAVE_SINE)
    {
      x = AD9708_INTERNAL_TWO_PI * ((float)index / (float)points);
      value = 127.5f + (127.5f * arm_sin_f32(x));
    }
    else
    {
      x = (((float)index - (((float)points - 1.0f) * 0.5f)) /
           (((float)points - 1.0f) * 0.5f)) *
          (4.0f * AD9708_INTERNAL_TWO_PI);
      y = (x > -1.0e-6f) && (x < 1.0e-6f) ?
          1.0f : arm_sin_f32(x) / x;
      y = (y - AD9708_INTERNAL_SINC_MIN) /
          (1.0f - AD9708_INTERNAL_SINC_MIN);
      value = y * 255.0f;
    }
    s_ad9708_wave_table[index] = AD9708_InternalClampSample(value);
  }
  return AD9708_OK;
}

/************************************************************
 * Function :       AD9708_LoadGeneratedWave
 * Comment  :       Generate and load a RAM waveform during boot preparation.
 *                  Output remains disabled after AD9708_LoadWave().
 * Parameter:       wave: sine or SINC; points: 2..1024.
 * Return   :       AD9708 status
************************************************************/
AD9708_StatusTypeDef AD9708_LoadGeneratedWave(AD9708_WaveformTypeDef wave,
                                              uint16_t points)
{
  AD9708_StatusTypeDef status;

  status = AD9708_RequireInit();
  if (status != AD9708_OK)
  {
    return AD9708_Return(status);
  }
  status = AD9708_InternalGenerateWave(wave, points);
  if (status == AD9708_OK)
  {
    status = AD9708_LoadWave(s_ad9708_wave_table, points);
  }
  return AD9708_Return(status);
}
/************************************************************
 * Function :       AD9708_InternalPhaseIndexToWord
 * Comment  :       将波表采样点索引转换为32位DDS相位字
 * Parameter:       points: 波形点数; phase_index: 相位索引;
 *                  phase_word: 相位字输出指针
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_InternalPhaseIndexToWord(
    uint16_t points,
    uint16_t phase_index,
    uint32_t *phase_word)
{
  if ((phase_word == NULL) ||
      (points < AD9708_RAM_MIN_POINTS) ||
      (points > AD9708_RAM_MAX_POINTS) ||
      (phase_index >= points))
  {
    return AD9708_ERROR_PARAM;
  }

  *phase_word = (uint32_t)(((uint64_t)phase_index << 32) /
                           (uint64_t)points);
  return AD9708_OK;
}

/************************************************************
 * Function :       AD9708_InternalStartMode
 * Comment  :       原子化停止、配置并启动一种FPGA DDS波形模式
 * Parameter:       mode: 模式; output_hz: 频率; amplitude_q8: Q8.8幅度码;
 *                  offset_q8: Q8.8中心码; phase_word: 起始相位字
 * Return   :       AD9708状态
************************************************************/
static AD9708_StatusTypeDef AD9708_InternalStartMode(
    AD9708_ModeTypeDef mode,
    float output_hz,
    uint16_t amplitude_q8,
    uint16_t offset_q8,
    uint32_t phase_word)
{
  AD9708_StatusTypeDef status;

  if (AD9708_InternalLevelValid(amplitude_q8, offset_q8) == 0U)
  {
    return AD9708_ERROR_PARAM;
  }
  if (AD9708_FrequencyValid(output_hz) == 0U)
  {
    return AD9708_ERROR_PARAM;
  }

  status = AD9708_StopSweep();
  if (status == AD9708_OK)
  {
    status = AD9708_SetEnable(0U);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_SetMode(mode);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_SetLevelFine(amplitude_q8, offset_q8);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_SetFrequencyHz(output_hz);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_SetPhaseOffsetWord(phase_word);
  }
  if (status == AD9708_OK)
  {
    status = AD9708_ResetPhase();
  }
  if (status == AD9708_OK)
  {
    status = AD9708_SetEnable(1U);
  }
  return status;
}

/************************************************************
 * Function :       AD9708_InternalOutputGenerated
 * Comment  :       输出内置RAM波形或FPGA实时生成波形
 * Parameter:       wave: 波形; output_hz: 频率; points: 点数;
 *                  amplitude_q8: Q8.8幅度码; offset_q8: Q8.8中心码;
 *                  phase_word: 相位
 * Return   :       AD9708状态
************************************************************/
static AD9708_StatusTypeDef AD9708_InternalOutputGenerated(
    AD9708_WaveformTypeDef wave,
    float output_hz,
    uint16_t points,
    uint16_t amplitude_q8,
    uint16_t offset_q8,
    uint32_t phase_word)
{
  AD9708_StatusTypeDef status;
  AD9708_ModeTypeDef mode;

  if (AD9708_GetData()->initialized == 0U)
  {
    return AD9708_ERROR_NOT_INIT;
  }
  if ((AD9708_FrequencyValid(output_hz) == 0U) ||
      (AD9708_InternalLevelValid(amplitude_q8, offset_q8) == 0U))
  {
    return AD9708_ERROR_PARAM;
  }

  if ((wave == AD9708_WAVE_SINE) || (wave == AD9708_WAVE_SINC))
  {
    status = AD9708_InternalGenerateWave(wave, points);
    if (status == AD9708_OK)
    {
      status = AD9708_LoadWave(s_ad9708_wave_table, points);
    }
    if (status != AD9708_OK)
    {
      return status;
    }
    mode = AD9708_MODE_RAM_DDS;
  }
  else if (wave == AD9708_WAVE_TRIANGLE)
  {
    mode = AD9708_MODE_TRIANGLE;
  }
  else if (wave == AD9708_WAVE_SQUARE)
  {
    mode = AD9708_MODE_SQUARE;
  }
  else if (wave == AD9708_WAVE_SAWTOOTH)
  {
    mode = AD9708_MODE_SAWTOOTH;
  }
  else
  {
    return AD9708_ERROR_PARAM;
  }

  return AD9708_InternalStartMode(mode,
                              output_hz,
                              amplitude_q8,
                              offset_q8,
                              phase_word);
}

/************************************************************
 * Function :       AD9708_InternalOutputGeneratedVoltage
 * Comment  :       将电压参数换算为码值后输出内置波形
 * Parameter:       波形、频率、点数、Vpp、中心电压和相位字
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_InternalOutputGeneratedVoltage(
    AD9708_WaveformTypeDef wave,
    float output_hz,
    uint16_t points,
    float amplitude_vpp,
    float offset_v,
    uint32_t phase_word)
{
  AD9708_StatusTypeDef status;
  uint16_t amplitude_q8;
  uint16_t offset_q8;

  if (AD9708_GetData()->initialized == 0U)
  {
    return AD9708_ERROR_NOT_INIT;
  }
  status = AD9708_InternalVoltageLevelToCode(amplitude_vpp,
                                         offset_v,
                                         &amplitude_q8,
                                         &offset_q8);
  if (status == AD9708_OK)
  {
    status = AD9708_InternalOutputGenerated(wave,
                                        output_hz,
                                        points,
                                        amplitude_q8,
                                        offset_q8,
                                        phase_word);
  }
  return status;
}

/************************************************************
 * Function :       AD9708_InternalOutputArbitraryCode
 * Comment  :       按原始幅度码和Q8.8中心码输出用户任意波
 * Parameter:       波表、点数、频率、幅度码、Q8.8中心码和相位字
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_InternalOutputArbitraryCode(
    const uint8_t *wave,
    uint16_t points,
    float output_hz,
    uint16_t amplitude_q8,
    uint16_t offset_q8,
    uint32_t phase_word)
{
  AD9708_StatusTypeDef status;

  if (AD9708_GetData()->initialized == 0U)
  {
    return AD9708_ERROR_NOT_INIT;
  }
  if ((wave == NULL) ||
      (AD9708_FrequencyValid(output_hz) == 0U) ||
      (AD9708_InternalLevelValid(amplitude_q8, offset_q8) == 0U))
  {
    return AD9708_ERROR_PARAM;
  }

  status = AD9708_LoadWave(wave, points);
  if (status == AD9708_OK)
  {
    status = AD9708_InternalStartMode(AD9708_MODE_RAM_DDS,
                                  output_hz,
                                  amplitude_q8,
                                  offset_q8,
                                  phase_word);
  }
  return status;
}

/************************************************************
 * Function :       AD9708_InternalStartConfiguredSweep
 * Comment  :       配置并启动默认向上的FPGA硬件扫频
 * Parameter:       low_hz/high_hz: 端点; step_hz: 步进;
 *                  dwell_us: 驻留时间; mode: 扫频模式
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_InternalStartConfiguredSweep(
    float low_hz,
    float high_hz,
    float step_hz,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode)
{
  AD9708_StatusTypeDef status;
  uint64_t dwell_cycles;

  if ((dwell_us == 0U) || (dwell_us > AD9708_MAX_SWEEP_DWELL_US))
  {
    return AD9708_ERROR_PARAM;
  }

  dwell_cycles = (uint64_t)dwell_us *
                 ((uint64_t)AD9708_DAC_CLK_HZ / 1000000ULL);
  status = AD9708_ConfigureFrequencySweep(low_hz,
                                           high_hz,
                                           step_hz,
                                           (uint32_t)dwell_cycles,
                                           mode,
                                           AD9708_SWEEP_UP);
  if (status == AD9708_OK)
  {
    status = AD9708_StartSweep();
  }
  return status;
}
