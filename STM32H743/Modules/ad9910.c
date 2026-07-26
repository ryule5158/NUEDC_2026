#include "ad9910.h"
#include <math.h>
#include <stddef.h>

#if defined(__clang__)
#pragma clang diagnostic ignored "-Wunsafe-buffer-usage"
#pragma clang diagnostic ignored "-Wswitch-default"
#endif

#define AD9910_SPI_TIMEOUT_MS      100U  /* 硬件SPI传输超时，单位ms。 */
#define AD9910_REG_CFR1            0x00U /* 控制功能寄存器1地址。 */
#define AD9910_REG_CFR2            0x01U /* 控制功能寄存器2地址。 */
#define AD9910_REG_CFR3            0x02U /* 控制功能寄存器3地址。 */
#define AD9910_REG_FTW             0x07U /* 频率调谐字寄存器地址。 */
#define AD9910_REG_DR_LIMIT        0x0BU /* 数字斜坡上下限寄存器地址。 */
#define AD9910_REG_DR_STEP         0x0CU /* 数字斜坡步进寄存器地址。 */
#define AD9910_REG_DR_RATE         0x0DU /* 数字斜坡速率寄存器地址。 */
#define AD9910_REG_PROFILE0        0x0EU /* Profile 0寄存器首地址。 */
#define AD9910_PROFILE_COUNT       8U    /* Profile寄存器数量。 */
#define AD9910_REG_RAM             0x16U /* RAM数据寄存器地址。 */

/*
 * 硬件通信方式选择。
 * 1: 使用GPIO模拟软件SPI，沿用原F103工程时序。
 * 0: 使用HAL硬件SPI外设。
 */
#define AD9910_USE_SOFT_SPI        1U    /* 1使用GPIO软件SPI。 */
#define AD9910_SOFT_SPI_DELAY_CYCLES 120U /* 软件SPI边沿延时周期数。 */
#define AD9910_CTRL_DELAY_CYCLES   6000U /* 控制引脚保持延时周期数。 */

#define AD9910_SCK_Pin             GPIO_PIN_2  /* 串行时钟引脚。 */
#define AD9910_SCK_GPIO_Port       GPIOE       /* 串行时钟端口。 */
#define AD9910_SDIO_Pin            GPIO_PIN_6  /* 串行数据引脚。 */
#define AD9910_SDIO_GPIO_Port      GPIOE       /* 串行数据端口。 */
#define AD9910_PWR_Pin             GPIO_PIN_13 /* 外部掉电引脚。 */
#define AD9910_PWR_GPIO_Port       GPIOC       /* 外部掉电端口。 */
#define AD9910_DROVER_Pin          GPIO_PIN_0  /* 扫频到端点指示引脚。 */
#define AD9910_DROVER_GPIO_Port    GPIOA       /* 扫频到端点指示端口。 */
#define AD9910_DRCTL_Pin           GPIO_PIN_8  /* 扫频方向控制引脚。 */
#define AD9910_DRCTL_GPIO_Port     GPIOC       /* 扫频方向控制端口。 */
#define AD9910_DRHOLD_Pin          GPIO_PIN_1  /* 扫频保持控制引脚。 */
#define AD9910_DRHOLD_GPIO_Port    GPIOC       /* 扫频保持控制端口。 */
#define AD9910_RESET_Pin           GPIO_PIN_6  /* 硬件复位引脚。 */
#define AD9910_RESET_GPIO_Port     GPIOA       /* 硬件复位端口。 */
#define AD9910_PF1_Pin             GPIO_PIN_6  /* Profile选择位1引脚。 */
#define AD9910_PF1_GPIO_Port       GPIOC       /* Profile选择位1端口。 */
#define AD9910_PF2_Pin             GPIO_PIN_7  /* Profile选择位2引脚。 */
#define AD9910_PF2_GPIO_Port       GPIOC       /* Profile选择位2端口。 */
#define AD9910_PF0_Pin             GPIO_PIN_5  /* Profile选择位0引脚。 */
#define AD9910_PF0_GPIO_Port       GPIOB       /* Profile选择位0端口。 */
#define AD9910_OSK_Pin             GPIO_PIN_7  /* 输出幅度键控引脚。 */
#define AD9910_OSK_GPIO_Port       GPIOB       /* 输出幅度键控端口。 */
#define AD9910_IO_UPDATE_Pin       GPIO_PIN_8  /* 寄存器更新引脚。 */
#define AD9910_IO_UPDATE_GPIO_Port GPIOB       /* 寄存器更新端口。 */
#define AD9910_CS_Pin              GPIO_PIN_9  /* 串行片选引脚。 */
#define AD9910_CS_GPIO_Port        GPIOB       /* 串行片选端口。 */

/* 写入指定AD9910控制引脚。 */
#define AD9910_PIN_WRITE(name, state) \
  HAL_GPIO_WritePin(AD9910_##name##_GPIO_Port, AD9910_##name##_Pin, (state))

/* RAM波形数据的写入目标。 */
typedef enum
{
  AD9910_RAM_DEST_AMPLITUDE = 0, /* 仅控制幅度。 */
  AD9910_RAM_DEST_POLAR           /* 同时控制幅度和极性。 */
} AD9910_RamDestination;

#if (AD9910_USE_SOFT_SPI == 0U)
static SPI_HandleTypeDef *s_hspi; /* AD9910硬件SPI句柄。 */
#endif

/* 单频Profile寄存器缓存。 */
static uint8_t s_profile0[8] =
{
  0x3FU, 0xFFU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U
};

/* RAM波形幅度缩放缓存。 */
static uint16_t s_ram_scaled_samples[AD9910_RAM_POINTS];

/* 单频模式CFR1默认配置。 */
static uint8_t s_cfr1_single_tone[4] = {0x00U, 0x40U, 0x00U, 0x00U};

/* 单频模式CFR2默认配置。 */
static uint8_t s_cfr2_single_tone[4] = {0x01U, 0x00U, 0x00U, 0x00U};

/* 1GHz系统时钟对应的CFR3配置。 */
static uint8_t s_cfr3_pll_1ghz[4] = {0x05U, 0x0FU, 0x41U, 0x32U};

/* 将14位幅度字写入单频Profile的Bit61~Bit48。 */
static void AD9910_StoreProfileAmplitude(uint16_t amplitude)
{
  s_profile0[0] = (uint8_t)((amplitude >> 8) & 0x3FU);
  s_profile0[1] = (uint8_t)amplitude;
}

/************************************************************
 * Function :       AD9910_DelayCycles
 * Comment  :       提供GPIO时序使用的短阻塞延时
 * Parameter:       cycles: 循环计数
 * Return   :       null
 * Date     :       2026-06-11 V1
************************************************************/
static void AD9910_DelayCycles(uint32_t cycles)
{
  volatile uint32_t i;

  for (i = 0U; i < cycles; i++)
  {
    __NOP();
  }
}

/************************************************************
 * Function :       AD9910_ControlDelay
 * Comment  :       AD9910控制引脚或片选状态变化后的保持延时
 * Parameter:       null
 * Return   :       null
 * Date     :       2026-06-11 V1
************************************************************/
static void AD9910_ControlDelay(void)
{
  AD9910_DelayCycles(AD9910_CTRL_DELAY_CYCLES);
}

#if (AD9910_USE_SOFT_SPI == 1U)
/************************************************************
 * Function :       AD9910_SoftSpiDelay
 * Comment  :       软件SPI时钟边沿之间的延时
 * Parameter:       null
 * Return   :       null
 * Date     :       2026-06-11 V1
************************************************************/
static void AD9910_SoftSpiDelay(void)
{
  AD9910_DelayCycles(AD9910_SOFT_SPI_DELAY_CYCLES);
}

/************************************************************
 * Function :       AD9910_SoftSpiWriteByte
 * Comment  :       通过GPIO软件SPI向AD9910发送1字节数据，高位先发
 * Parameter:       data: 待发送字节
 * Return   :       null
 * Date     :       2026-06-11 V1
************************************************************/
static void AD9910_SoftSpiWriteByte(uint8_t data)
{
  uint8_t mask;

  for (mask = 0x80U; mask != 0U; mask >>= 1)
  {
    HAL_GPIO_WritePin(AD9910_SDIO_GPIO_Port,
                      AD9910_SDIO_Pin,
                      ((data & mask) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    AD9910_SoftSpiDelay();
    HAL_GPIO_WritePin(AD9910_SCK_GPIO_Port, AD9910_SCK_Pin, GPIO_PIN_SET);
    AD9910_SoftSpiDelay();
    HAL_GPIO_WritePin(AD9910_SCK_GPIO_Port, AD9910_SCK_Pin, GPIO_PIN_RESET);
    AD9910_SoftSpiDelay();
  }
}
#endif

/************************************************************
 * Function :       AD9910_CsLow
 * Comment  :       拉低片选，选中AD9910 SPI接口
 * Parameter:       null
 * Return   :       null
 * Date     :       2026-06-11 V1
************************************************************/
static void AD9910_CsLow(void)
{
  AD9910_PIN_WRITE(CS, GPIO_PIN_RESET);
}

/************************************************************
 * Function :       AD9910_CsHigh
 * Comment  :       拉高片选，释放AD9910 SPI接口
 * Parameter:       null
 * Return   :       null
 * Date     :       2026-06-11 V1
************************************************************/
static void AD9910_CsHigh(void)
{
  AD9910_PIN_WRITE(CS, GPIO_PIN_SET);
}

/************************************************************
 * Function :       AD9910_PulseIoUpdate
 * Comment  :       产生IO_UPDATE脉冲，使AD9910寄存器写入生效
 * Parameter:       null
 * Return   :       null
 * Date     :       2026-06-11 V1
************************************************************/
static void AD9910_PulseIoUpdate(void)
{
  AD9910_PIN_WRITE(IO_UPDATE, GPIO_PIN_SET);
  AD9910_ControlDelay();
  AD9910_PIN_WRITE(IO_UPDATE, GPIO_PIN_RESET);
  AD9910_ControlDelay();
}

/************************************************************
 * Function :       AD9910_Transmit
 * Comment  :       通过当前SPI方式向AD9910发送原始字节流
 * Parameter:       data: 字节缓冲区; length: 字节数
 * Return   :       AD9910_OK表示成功，其他值表示参数或SPI错误
 * Date     :       2026-06-11 V1
************************************************************/
static AD9910_Status AD9910_Transmit(uint8_t *data, uint16_t length)
{
#if (AD9910_USE_SOFT_SPI == 1U)
  uint16_t i;

  if ((data == NULL) || (length == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  for (i = 0U; i < length; i++)
  {
    AD9910_SoftSpiWriteByte(data[i]);
  }

  return AD9910_OK;
#else
  if ((s_hspi == NULL) || (data == NULL) || (length == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  if (HAL_SPI_Transmit(s_hspi, data, length, AD9910_SPI_TIMEOUT_MS) != HAL_OK)
  {
    return AD9910_ERROR;
  }

  return AD9910_OK;
#endif
}

/************************************************************
 * Function :       AD9910_WriteRegister
 * Comment  :       写入一个AD9910寄存器，并统一处理片选时序
 * Parameter:       reg: 寄存器地址; data: 寄存器数据; length: 数据字节数
 * Return   :       AD9910_OK表示成功，其他值表示参数或SPI错误
 * Date     :       2026-06-11 V1
************************************************************/
static AD9910_Status AD9910_WriteRegister(uint8_t reg, uint8_t *data, uint16_t length)
{
  AD9910_Status status;

  if ((data == NULL) || (length == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  AD9910_CsLow();

  status = AD9910_Transmit(&reg, 1U);
  if (status == AD9910_OK)
  {
    status = AD9910_Transmit(data, length);
  }

  AD9910_CsHigh();
  AD9910_ControlDelay();

  return status;
}

/************************************************************
 * Function :       AD9910_FrequencyToTuningWord
 * Comment  :       将输出频率转换为AD9910的32位频率调谐字FTW
 * Parameter:       freq_hz: 输出频率
 * Return   :       32位频率调谐字
 * Date     :       2026-06-11 V1
************************************************************/
static uint32_t AD9910_FrequencyToTuningWord(uint32_t freq_hz)
{
  uint64_t numerator;

  numerator = ((uint64_t)freq_hz << 32);
  return (uint32_t)(numerator / AD9910_SYSCLK_HZ);
}

/************************************************************
 * Function :       AD9910_FrequencyFineToTuningWord
 * Comment  :       将小数Hz输出频率转换为AD9910的32位频率调谐字FTW
 * Parameter:       freq_hz: 输出频率，单位Hz，允许带小数
 * Return   :       32位频率调谐字
 * Date     :       2026-06-26 V1
************************************************************/
static uint32_t AD9910_FrequencyFineToTuningWord(double freq_hz)
{
  double ftw_value; /* AD9910频率字浮点计算值 */

  if (freq_hz <= 0.0f)
  {
    return 0U;
  }

  ftw_value = (freq_hz * 4294967296.0) / (double)AD9910_SYSCLK_HZ;

  if (ftw_value > 4294967295.0)
  {
    return 0xFFFFFFFFU;
  }

  return (uint32_t)(ftw_value + 0.5);
}

/************************************************************
 * Function :       AD9910_PutU32Be
 * Comment  :       按大端格式存放32位数据，用于AD9910寄存器写入
 * Parameter:       dst: 目标缓冲区; value: 32位数据
 * Return   :       null
 * Date     :       2026-06-11 V1
************************************************************/
static void AD9910_PutU32Be(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value >> 24);
  dst[1] = (uint8_t)(value >> 16);
  dst[2] = (uint8_t)(value >> 8);
  dst[3] = (uint8_t)value;
}

/************************************************************
 * Function :       AD9910_WriteFtwRegister
 * Comment  :       写入AD9910频率调谐字寄存器
 * Parameter:       freq_hz: 目标输出频率
 * Return   :       AD9910_OK表示成功，其他值表示参数或通信错误
 * Date     :       2026-06-11 V1
************************************************************/
static AD9910_Status AD9910_WriteFtwRegister(uint32_t freq_hz)
{
  uint8_t ftw_data[4];

  if (freq_hz > AD9910_MAX_OUTPUT_HZ)
  {
    return AD9910_BAD_PARAM;
  }

  AD9910_PutU32Be(ftw_data, AD9910_FrequencyToTuningWord(freq_hz));
  return AD9910_WriteRegister(AD9910_REG_FTW, ftw_data, sizeof(ftw_data));
}

/************************************************************
 * Function :       AD9910_GetWaveSample
 * Comment  :       生成内置RAM波形的一个14位采样点
 * Parameter:       wave: 波形类型; index: 采样索引; points: 采样点数
 * Return   :       14位采样值
 * Date     :       2026-06-11 V1
************************************************************/
static uint16_t AD9910_GetWaveSample(AD9910_Waveform wave, uint16_t index, uint16_t points)
{
  uint32_t value;
  uint16_t half_points;
  double x;
  double y;
  const double pi = 3.14159265358979323846;
  const double sinc_min = -0.21723362821122166;

  half_points = (uint16_t)(points / 2U);

  switch (wave)
  {
    case AD9910_WAVE_TRIANGLE:
      if (index < half_points)
      {
        value = ((uint32_t)index * AD9910_MAX_AMPLITUDE) / ((uint32_t)half_points - 1U);
      }
      else
      {
        value = ((uint32_t)(points - 1U - index) * AD9910_MAX_AMPLITUDE) /
                ((uint32_t)half_points - 1U);
      }
      if (value > AD9910_MAX_AMPLITUDE)
      {
        value = AD9910_MAX_AMPLITUDE;
      }
      return (uint16_t)value;

    case AD9910_WAVE_SQUARE:
      return (index < half_points) ? 0U : AD9910_MAX_AMPLITUDE;

    case AD9910_WAVE_SAWTOOTH:
      value = ((uint32_t)index * AD9910_MAX_AMPLITUDE) /
              ((uint32_t)points - 1U);
      return (uint16_t)value;

    case AD9910_WAVE_SINC:
      /* Sinc范围为+/-8*pi，归一化后主瓣满幅，第一负旁瓣约为0。 */
      x = (((double)index - (((double)points - 1.0) * 0.5)) /
           (((double)points - 1.0) * 0.5)) * 8.0 * pi;
      if (fabs(x) < 1.0e-9)
      {
        y = 1.0;
      }
      else
      {
        y = sin(x) / x;
      }
      y = (y - sinc_min) / (1.0 - sinc_min);
      if (y < 0.0)
      {
        y = 0.0;
      }
      if (y > 1.0)
      {
        y = 1.0;
      }
      return (uint16_t)((y * (double)AD9910_MAX_AMPLITUDE) + 0.5);
  }

  return 0U;
}

/************************************************************
 * Function :       AD9910_RamSampleToWord
 * Comment  :       将14位采样值转换为AD9910 RAM使用的32位数据格式
 * Parameter:       sample: 14位波形采样值; destination: 幅度RAM或极性RAM模式
 * Return   :       AD9910 RAM数据字
 * Date     :       2026-06-11 V1
************************************************************/
static uint32_t AD9910_RamSampleToWord(uint16_t sample,
                                       AD9910_RamDestination destination)
{
  int32_t signed_sample;
  uint16_t phase;
  uint16_t magnitude;

  sample &= 0x3FFFU;

  if (destination == AD9910_RAM_DEST_AMPLITUDE)
  {
    return ((uint32_t)sample) << 18;
  }

  /* 极性模式用于双极性基带波形：相位表示正负号，幅度表示绝对值。 */
  signed_sample = (((int32_t)sample - 8192) * 2);
  if (signed_sample >= 0)
  {
    phase = 0x0000U;
    magnitude = (signed_sample > (int32_t)AD9910_MAX_AMPLITUDE) ?
                AD9910_MAX_AMPLITUDE :
                (uint16_t)signed_sample;
  }
  else
  {
    phase = 0x8000U;
    signed_sample = -signed_sample;
    magnitude = (signed_sample > (int32_t)AD9910_MAX_AMPLITUDE) ?
                AD9910_MAX_AMPLITUDE :
                (uint16_t)signed_sample;
  }

  return (((uint32_t)phase) << 16) | (((uint32_t)magnitude) << 2);
}

/************************************************************
 * Function :       AD9910_GetRamWord
 * Comment  :       为内置波形的一个采样点生成AD9910 RAM数据字
 * Parameter:       wave: 波形类型; index: 采样索引; points: 采样点数; destination: RAM目标模式
 * Return   :       AD9910 RAM数据字
 * Date     :       2026-06-11 V1
************************************************************/
static uint32_t AD9910_GetRamWord(AD9910_Waveform wave,
                                  uint16_t index,
                                  uint16_t points,
                                  AD9910_RamDestination destination)
{
  uint16_t sample;

  sample = AD9910_GetWaveSample(wave, index, points);
  return AD9910_RamSampleToWord(sample, destination);
}

/************************************************************
 * Function :       AD9910_WriteRamSamples
 * Comment  :       将生成的内置波形采样点连续写入AD9910 RAM
 * Parameter:       wave: 波形类型; points: 采样点数; destination: RAM目标模式
 * Return   :       AD9910_OK表示成功，其他值表示通信错误
 * Date     :       2026-06-11 V1
************************************************************/
static AD9910_Status AD9910_WriteRamSamples(AD9910_Waveform wave,
                                            uint16_t points,
                                            AD9910_RamDestination destination)
{
  uint8_t reg;
  uint8_t buffer[64];
  uint16_t index;
  uint16_t used;
  uint32_t word;
  AD9910_Status status;

  reg = AD9910_REG_RAM;
  used = 0U;

  AD9910_CsLow();

  status = AD9910_Transmit(&reg, 1U);
  if (status != AD9910_OK)
  {
    AD9910_CsHigh();
    return status;
  }

  for (index = 0U; index < points; index++)
  {
    word = AD9910_GetRamWord(wave, index, points, destination);
    AD9910_PutU32Be(&buffer[used], word);
    used = (uint16_t)(used + 4U);

    if (used == sizeof(buffer))
    {
      status = AD9910_Transmit(buffer, used);
      if (status != AD9910_OK)
      {
        AD9910_CsHigh();
        return status;
      }
      used = 0U;
    }
  }

  if (used > 0U)
  {
    status = AD9910_Transmit(buffer, used);
  }

  AD9910_CsHigh();
  AD9910_ControlDelay();

  return status;
}

/************************************************************
 * Function :       AD9910_WriteRamCustomSamples
 * Comment  :       将用户提供的波形采样表连续写入AD9910 RAM
 * Parameter:       samples: 14位采样表; points: 采样点数; destination: RAM目标模式
 * Return   :       AD9910_OK表示成功，其他值表示参数或通信错误
 * Date     :       2026-06-11 V1
************************************************************/
static AD9910_Status AD9910_WriteRamCustomSamples(const uint16_t *samples,
                                                  uint16_t points,
                                                  AD9910_RamDestination destination)
{
  uint8_t reg;
  uint8_t buffer[64];
  uint16_t index;
  uint16_t src_index;
  uint16_t used;
  uint32_t word;
  AD9910_Status status;

  if (samples == NULL)
  {
    return AD9910_BAD_PARAM;
  }

  reg = AD9910_REG_RAM;
  used = 0U;

  AD9910_CsLow();

  status = AD9910_Transmit(&reg, 1U);
  if (status != AD9910_OK)
  {
    AD9910_CsHigh();
    return status;
  }

  for (index = 0U; index < points; index++)
  {
    src_index = (uint16_t)(points - 1U - index);  /* 反向写入RAM，修正非对称任意波的左右镜像 */
    word = AD9910_RamSampleToWord(samples[src_index], destination);
    AD9910_PutU32Be(&buffer[used], word);
    used = (uint16_t)(used + 4U);

    if (used == sizeof(buffer))
    {
      status = AD9910_Transmit(buffer, used);
      if (status != AD9910_OK)
      {
        AD9910_CsHigh();
        return status;
      }
      used = 0U;
    }
  }

  if (used > 0U)
  {
    status = AD9910_Transmit(buffer, used);
  }

  AD9910_CsHigh();
  AD9910_ControlDelay();

  return status;
}

/************************************************************
 * Function :       AD9910_WriteAllProfiles
 * Comment  :       将相同的单频Profile数据写入AD9910全部8个Profile
 * Parameter:       null
 * Return   :       AD9910_OK表示成功，其他值表示通信错误
 * Date     :       2026-06-11 V1
************************************************************/
static AD9910_Status AD9910_WriteAllProfiles(void)
{
  AD9910_Status status;
  uint8_t reg;

  /* 写入全部Profile，避免PF引脚状态选到空Profile。 */
  for (reg = AD9910_REG_PROFILE0; reg < (AD9910_REG_PROFILE0 + AD9910_PROFILE_COUNT); reg++)
  {
    status = AD9910_WriteRegister(reg, s_profile0, sizeof(s_profile0));
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  AD9910_PulseIoUpdate();
  return AD9910_OK;
}

/************************************************************
 * Function :       AD9910_WriteAllRamProfiles
 * Comment  :       将相同的RAM播放Profile写入AD9910全部8个Profile
 * Parameter:       profile: Profile数据; length: Profile字节数
 * Return   :       AD9910_OK表示成功，其他值表示通信错误
 * Date     :       2026-06-11 V1
************************************************************/
static AD9910_Status AD9910_WriteAllRamProfiles(uint8_t *profile, uint16_t length)
{
  AD9910_Status status;
  uint8_t reg;

  /* RAM播放也依赖Profile寄存器，因此保持8个Profile一致。 */
  for (reg = AD9910_REG_PROFILE0; reg < (AD9910_REG_PROFILE0 + AD9910_PROFILE_COUNT); reg++)
  {
    status = AD9910_WriteRegister(reg, profile, length);
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  AD9910_PulseIoUpdate();
  return AD9910_OK;
}

/************************************************************
 * Function :       AD9910_SetRamWaveformPointsEx
 * Comment  :       配置AD9910 RAM播放并写入内置波形表
 * Parameter:       wave: 波形类型; playback_step: RAM播放步进; points: 采样点数; destination: RAM目标模式
 * Return   :       AD9910_OK表示成功，其他值表示参数或通信错误
 * Date     :       2026-06-11 V1
************************************************************/
static AD9910_Status AD9910_SetRamWaveformPointsEx(AD9910_Waveform wave,
                                                   uint16_t playback_step,
                                                   uint16_t points,
                                                   AD9910_RamDestination destination)
{
  uint8_t cfr1_ram[4];
  uint8_t ram_profile0[8];
  uint16_t end_field;
  AD9910_Status status;

  if ((points < 16U) || (points > AD9910_RAM_POINTS))
  {
    return AD9910_BAD_PARAM;
  }

  if (playback_step == 0U)
  {
    playback_step = 1U;
  }

  end_field = (uint16_t)((points - 1U) << 6);

  cfr1_ram[0] = (destination == AD9910_RAM_DEST_POLAR) ? 0x60U : 0x40U;
  cfr1_ram[1] = 0x40U;
  cfr1_ram[2] = 0x20U;  /* IO_UPDATE时清DDS相位累加器，保证RAM输出起点稳定 */
  cfr1_ram[3] = 0x00U;

  ram_profile0[0] = 0x00U;
  ram_profile0[1] = (uint8_t)(playback_step >> 8);
  ram_profile0[2] = (uint8_t)playback_step;
  ram_profile0[3] = (uint8_t)(end_field >> 8);
  ram_profile0[4] = (uint8_t)end_field;
  ram_profile0[5] = 0x00U;
  ram_profile0[6] = 0x00U;
  ram_profile0[7] = 0x04U;

  status = AD9910_WriteRegister(AD9910_REG_CFR1, cfr1_ram, sizeof(cfr1_ram));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteAllRamProfiles(ram_profile0, sizeof(ram_profile0));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteRamSamples(wave, points, destination);
  if (status != AD9910_OK)
  {
    return status;
  }

  cfr1_ram[0] |= 0x80U;
  status = AD9910_WriteRegister(AD9910_REG_CFR1, cfr1_ram, sizeof(cfr1_ram));
  if (status == AD9910_OK)
  {
    AD9910_PulseIoUpdate();
  }

  return status;
}

/************************************************************
 * Function :       AD9910_SetRamCustomWaveformPointsEx
 * Comment  :       配置AD9910 RAM播放并写入用户自定义波形表
 * Parameter:       samples: 采样表; points: 采样点数; playback_step: RAM播放步进; destination: RAM目标模式
 * Return   :       AD9910_OK表示成功，其他值表示参数或通信错误
 * Date     :       2026-06-11 V1
************************************************************/
static AD9910_Status AD9910_SetRamCustomWaveformPointsEx(const uint16_t *samples,
                                                         uint16_t points,
                                                         uint16_t playback_step,
                                                         AD9910_RamDestination destination)
{
  uint8_t cfr1_ram[4];
  uint8_t ram_profile0[8];
  uint16_t end_field;
  AD9910_Status status;

  if ((samples == NULL) || (points < 16U) || (points > AD9910_RAM_POINTS))
  {
    return AD9910_BAD_PARAM;
  }

  if (playback_step == 0U)
  {
    playback_step = 1U;
  }

  end_field = (uint16_t)((points - 1U) << 6);

  cfr1_ram[0] = (destination == AD9910_RAM_DEST_POLAR) ? 0x60U : 0x40U;
  cfr1_ram[1] = 0x40U;
  cfr1_ram[2] = 0x20U;  /* IO_UPDATE时清DDS相位累加器，保证RAM输出起点稳定 */
  cfr1_ram[3] = 0x00U;

  ram_profile0[0] = 0x00U;
  ram_profile0[1] = (uint8_t)(playback_step >> 8);
  ram_profile0[2] = (uint8_t)playback_step;
  ram_profile0[3] = (uint8_t)(end_field >> 8);
  ram_profile0[4] = (uint8_t)end_field;
  ram_profile0[5] = 0x00U;
  ram_profile0[6] = 0x00U;
  ram_profile0[7] = 0x04U;

  status = AD9910_WriteRegister(AD9910_REG_CFR1, cfr1_ram, sizeof(cfr1_ram));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteAllRamProfiles(ram_profile0, sizeof(ram_profile0));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteRamCustomSamples(samples, points, destination);
  if (status != AD9910_OK)
  {
    return status;
  }

  cfr1_ram[0] |= 0x80U;
  status = AD9910_WriteRegister(AD9910_REG_CFR1, cfr1_ram, sizeof(cfr1_ram));
  if (status == AD9910_OK)
  {
    AD9910_PulseIoUpdate();
  }

  return status;
}

/************************************************************
 * Function :       AD9910_Reset
 * Comment  :       复位AD9910并将控制引脚恢复到默认空闲状态
 * Parameter:       null
 * Return   :       AD9910_OK表示成功
 * Date     :       2026-06-10 V1
************************************************************/
AD9910_Status AD9910_Reset(void)
{
  AD9910_PowerDown(GPIO_PIN_RESET);
  AD9910_CsHigh();
  AD9910_PIN_WRITE(IO_UPDATE, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(DRCTL, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(DRHOLD, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(OSK, GPIO_PIN_RESET);
  AD9910_SetProfile(0U);

  AD9910_PIN_WRITE(RESET, GPIO_PIN_SET);
  HAL_Delay(5U);
  AD9910_PIN_WRITE(RESET, GPIO_PIN_RESET);
  HAL_Delay(1U);

  return AD9910_OK;
}

/************************************************************
 * Function :       AD9910_Init
 * Comment  :       初始化AD9910并写入默认CFR配置
 * Parameter:       hspi: 硬件SPI句柄, 软件SPI模式传NULL
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
AD9910_Status AD9910_Init(SPI_HandleTypeDef *hspi)
{
  AD9910_Status status;

#if (AD9910_USE_SOFT_SPI == 0U)
  if (hspi == NULL)
  {
    return AD9910_BAD_PARAM;
  }

  s_hspi = hspi;
#else
  (void)hspi;
#endif

  AD9910_Reset();

  status = AD9910_WriteRegister(AD9910_REG_CFR1, s_cfr1_single_tone, sizeof(s_cfr1_single_tone));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteRegister(AD9910_REG_CFR2, s_cfr2_single_tone, sizeof(s_cfr2_single_tone));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteRegister(AD9910_REG_CFR3, s_cfr3_pll_1ghz, sizeof(s_cfr3_pll_1ghz));
  if (status != AD9910_OK)
  {
    return status;
  }

  AD9910_PulseIoUpdate();
  HAL_Delay(100U);

  return AD9910_OK;
}

/************************************************************
 * Function :       AD9910_SetFrequencyHz
 * Comment  :       设置AD9910当前单频输出频率
 * Parameter:       freq_hz: 输出频率(Hz)
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
AD9910_Status AD9910_SetFrequencyHz(uint32_t freq_hz)
{
  AD9910_Status status;
  uint32_t ftw;

  if (freq_hz > AD9910_MAX_OUTPUT_HZ)
  {
    return AD9910_BAD_PARAM;
  }

  /* 任何正弦频率设置都先恢复单频模式，避免上一次RAM/扫频状态残留。 */
  AD9910_PIN_WRITE(DRCTL, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(DRHOLD, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(OSK, GPIO_PIN_RESET);
  AD9910_SetProfile(0U);

  status = AD9910_WriteRegister(AD9910_REG_CFR1, s_cfr1_single_tone, sizeof(s_cfr1_single_tone));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteRegister(AD9910_REG_CFR2, s_cfr2_single_tone, sizeof(s_cfr2_single_tone));
  if (status != AD9910_OK)
  {
    return status;
  }

  ftw = AD9910_FrequencyToTuningWord(freq_hz);
  s_profile0[4] = (uint8_t)(ftw >> 24);
  s_profile0[5] = (uint8_t)(ftw >> 16);
  s_profile0[6] = (uint8_t)(ftw >> 8);
  s_profile0[7] = (uint8_t)ftw;

  return AD9910_WriteAllProfiles();
}

/************************************************************
 * Function :       AD9910_SetFrequencyFineHz
 * Comment  :       设置AD9910当前单频输出频率，支持小数Hz微调
 * Parameter:       freq_hz: 输出频率，单位Hz，允许带小数
 * Return   :       AD9910_OK表示成功，其他值表示参数或通信错误
 * Date     :       2026-06-26 V1
************************************************************/
AD9910_Status AD9910_SetFrequencyFineHz(float freq_hz)
{
  AD9910_Status status; /* AD9910通信状态 */
  uint32_t ftw;         /* AD9910频率调谐字 */

  if ((freq_hz < 0.0f) || (freq_hz > (float)AD9910_MAX_OUTPUT_HZ))
  {
    return AD9910_BAD_PARAM;
  }

  AD9910_PIN_WRITE(DRCTL, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(DRHOLD, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(OSK, GPIO_PIN_RESET);
  AD9910_SetProfile(0U);

  status = AD9910_WriteRegister(AD9910_REG_CFR1, s_cfr1_single_tone, sizeof(s_cfr1_single_tone));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteRegister(AD9910_REG_CFR2, s_cfr2_single_tone, sizeof(s_cfr2_single_tone));
  if (status != AD9910_OK)
  {
    return status;
  }

  ftw = AD9910_FrequencyFineToTuningWord(freq_hz);
  s_profile0[4] = (uint8_t)(ftw >> 24);
  s_profile0[5] = (uint8_t)(ftw >> 16);
  s_profile0[6] = (uint8_t)(ftw >> 8);
  s_profile0[7] = (uint8_t)ftw;

  return AD9910_WriteAllProfiles();
}

/************************************************************
 * Function :       AD9910_SetPhaseOffsetWord
 * Comment  :       设置AD9910当前单频输出相位偏移字
 * Parameter:       phase_word: 16位相位字，0~65535对应0~360度
 * Return   :       AD9910_OK表示成功，其他值表示通信错误
 * Date     :       2026-06-26 V1
************************************************************/
AD9910_Status AD9910_SetPhaseOffsetWord(uint16_t phase_word)
{
  s_profile0[2] = (uint8_t)(phase_word >> 8);
  s_profile0[3] = (uint8_t)phase_word;

  return AD9910_WriteAllProfiles();
}

/************************************************************
 * Function :       AD9910_UpdateSingleToneFast
 * Comment  :       PLL快速通道，一次性更新Profile0中的ASF、POW和FTW
 * Parameter:       freq_hz: 频率Hz; phase_word: 16位POW;
 *                  amplitude: 14位ASF
 * Return   :       AD9910_OK表示成功，其他值表示参数或通信错误
 * Note     :       调用前须完成AD9910_Init且保持单频模式。与分别调用
 *                  频率/相位/幅度接口相比，本函数只写一个Profile并
 *                  产生一次IO_UPDATE，避免闭环周期被8个Profile拖慢。
************************************************************/
AD9910_Status AD9910_UpdateSingleToneFast(double freq_hz,
                                           uint16_t phase_word,
                                           uint16_t amplitude)
{
  AD9910_Status status;
  uint32_t ftw;

  if ((!isfinite(freq_hz)) || (freq_hz < 0.0f) ||
      (freq_hz > (double)AD9910_MAX_OUTPUT_HZ) ||
      (amplitude > AD9910_MAX_AMPLITUDE))
  {
    return AD9910_BAD_PARAM;
  }

  ftw = AD9910_FrequencyFineToTuningWord(freq_hz);
  AD9910_StoreProfileAmplitude(amplitude);
  s_profile0[2] = (uint8_t)(phase_word >> 8);
  s_profile0[3] = (uint8_t)phase_word;
  s_profile0[4] = (uint8_t)(ftw >> 24);
  s_profile0[5] = (uint8_t)(ftw >> 16);
  s_profile0[6] = (uint8_t)(ftw >> 8);
  s_profile0[7] = (uint8_t)ftw;

  /* PLL固定使用Profile0。这里只改变选通脚，不重写CFR，输出连续。 */
  AD9910_PIN_WRITE(DRCTL, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(DRHOLD, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(OSK, GPIO_PIN_RESET);
  AD9910_SetProfile(0U);

  status = AD9910_WriteRegister(AD9910_REG_PROFILE0,
                                s_profile0,
                                sizeof(s_profile0));
  if (status == AD9910_OK)
  {
    AD9910_PulseIoUpdate();
  }
  return status;
}

/************************************************************
 * Function :       AD9910_SetAmplitude
 * Comment  :       设置AD9910当前单频输出幅度
 * Parameter:       amplitude: 14位幅度控制值
 * Return   :       AD9910_OK表示成功, 其他值表示通信错误
 * Date     :       2026-06-10 V1
************************************************************/
AD9910_Status AD9910_SetAmplitude(uint16_t amplitude)
{
  if (amplitude > AD9910_MAX_AMPLITUDE)
  {
    return AD9910_BAD_PARAM;
  }

  AD9910_StoreProfileAmplitude(amplitude);

  return AD9910_WriteAllProfiles();
}

/************************************************************
 * Function :       AD9910_SetRamWaveform
 * Comment  :       使用默认1024点RAM输出内置波形
 * Parameter:       wave: 内置波形类型; playback_step: RAM播放步进
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
AD9910_Status AD9910_SetRamWaveform(AD9910_Waveform wave, uint16_t playback_step)
{
  return AD9910_SetRamWaveformPoints(wave, playback_step, AD9910_RAM_POINTS);
}

/************************************************************
 * Function :       AD9910_SetRamWaveformCarrier
 * Comment  :       设置AD9910以指定载波和RAM表输出内置波形
 * Parameter:       wave: 内置波形类型; carrier_hz: 载波频率(Hz); playback_step: RAM播放步进; points: RAM点数
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
AD9910_Status AD9910_SetRamWaveformCarrier(AD9910_Waveform wave,
                                           uint32_t carrier_hz,
                                           uint16_t playback_step,
                                           uint16_t points)
{
  AD9910_Status status;
  AD9910_RamDestination destination;

  /*
   * 模块输出路径使用RAM幅度播放模式。
   * 该配置是前期调试中已验证能输出三角波的寄存器流程。
   */
  status = AD9910_SetAmplitude(AD9910_MAX_AMPLITUDE);
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_SetFrequencyHz(carrier_hz);
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteFtwRegister(carrier_hz);
  if (status != AD9910_OK)
  {
    return status;
  }

  AD9910_PulseIoUpdate();
  destination = AD9910_RAM_DEST_POLAR;
  return AD9910_SetRamWaveformPointsEx(wave, playback_step, points, destination);
}

/************************************************************
 * Function :       AD9910_SetRamCustomWaveformCarrier
 * Comment  :       设置AD9910以指定载波和用户RAM表输出自定义波形
 * Parameter:       samples: 波形采样表; points: 采样点数; carrier_hz: 载波频率(Hz); playback_step: RAM播放步进
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
AD9910_Status AD9910_SetRamCustomWaveformCarrier(const uint16_t *samples,
                                                 uint16_t points,
                                                 uint32_t carrier_hz,
                                                 uint16_t playback_step)
{
  AD9910_RamDestination destination = AD9910_RAM_DEST_POLAR;
  AD9910_Status status;
  uint32_t ftw;

  /* Reset pins to known state */
  AD9910_PIN_WRITE(DRCTL, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(DRHOLD, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(OSK, GPIO_PIN_RESET);
  AD9910_SetProfile(0U);

  /* Step 1: switch to single-tone mode to reset the RAM state machine.
   * Without this, going directly from RAM→RAM leaves stale internal state
   * that causes reversed playback and amplitude drift. */
  status = AD9910_WriteRegister(AD9910_REG_CFR1, s_cfr1_single_tone,
                                sizeof(s_cfr1_single_tone));
  if (status != AD9910_OK) return status;

  status = AD9910_WriteRegister(AD9910_REG_CFR2, s_cfr2_single_tone,
                                sizeof(s_cfr2_single_tone));
  if (status != AD9910_OK) return status;

  /* ASF = full scale, FTW = carrier_hz (usually 0) */
  AD9910_StoreProfileAmplitude(AD9910_MAX_AMPLITUDE);
  ftw = AD9910_FrequencyToTuningWord(carrier_hz);
  s_profile0[4] = (uint8_t)(ftw >> 24);
  s_profile0[5] = (uint8_t)(ftw >> 16);
  s_profile0[6] = (uint8_t)(ftw >> 8);
  s_profile0[7] = (uint8_t)ftw;

  /* IO_UPDATE 1: latch single-tone mode → resets RAM state machine */
  status = AD9910_WriteAllProfiles();
  if (status != AD9910_OK) return status;

  /* Step 2: switch to RAM mode and write waveform data.
   * IO_UPDATE 2: latch RAM mode with new waveform */
  return AD9910_SetRamCustomWaveformPointsEx(samples,
                                             points,
                                             playback_step,
                                              destination);
}

/************************************************************
 * Function :       AD9910_SetRamCustomWaveformCarrierEx
 * Comment  :       设置AD9910以指定RAM目标模式输出用户RAM表
 * Parameter:       samples: 波形采样表; points: 采样点数; carrier_hz: 载波频率; playback_step: RAM播放步进; destination: RAM目标模式
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-26 V1
************************************************************/
static AD9910_Status AD9910_SetRamCustomWaveformCarrierEx(const uint16_t *samples,
                                                          uint16_t points,
                                                          uint32_t carrier_hz,
                                                          uint16_t playback_step,
                                                          AD9910_RamDestination destination)
{
  AD9910_Status status; /* AD9910通信状态 */
  uint32_t ftw;         /* AD9910频率调谐字 */

  AD9910_PIN_WRITE(DRCTL, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(DRHOLD, GPIO_PIN_RESET);
  AD9910_PIN_WRITE(OSK, GPIO_PIN_RESET);
  AD9910_SetProfile(0U);

  status = AD9910_WriteRegister(AD9910_REG_CFR1, s_cfr1_single_tone,
                                sizeof(s_cfr1_single_tone));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteRegister(AD9910_REG_CFR2, s_cfr2_single_tone,
                                sizeof(s_cfr2_single_tone));
  if (status != AD9910_OK)
  {
    return status;
  }

  AD9910_StoreProfileAmplitude(AD9910_MAX_AMPLITUDE);
  ftw = AD9910_FrequencyToTuningWord(carrier_hz);
  s_profile0[4] = (uint8_t)(ftw >> 24);
  s_profile0[5] = (uint8_t)(ftw >> 16);
  s_profile0[6] = (uint8_t)(ftw >> 8);
  s_profile0[7] = (uint8_t)ftw;

  status = AD9910_WriteAllProfiles();
  if (status != AD9910_OK)
  {
    return status;
  }

  return AD9910_SetRamCustomWaveformPointsEx(samples,
                                             points,
                                             playback_step,
                                             destination);
}

/************************************************************
 * Function :       AD9910_SetRamWaveformCarrierAmplitude
 * Comment  :       输出带14位幅度控制的AD9910内置RAM波形
 * Parameter:       wave: 内置波形; carrier_hz: 载波频率; playback_step: RAM播放步进; points: 波形点数; amplitude: 14位幅度; retry_delay_ms: 重写等待时间
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-12 V1
************************************************************/
AD9910_Status AD9910_SetRamWaveformCarrierAmplitude(AD9910_Waveform wave,
                                                    uint32_t carrier_hz,
                                                    uint16_t playback_step,
                                                    uint16_t points,
                                                    uint16_t amplitude,
                                                    uint32_t retry_delay_ms)
{
  AD9910_Status status;
  uint16_t i;
  uint32_t sample;
  int32_t centered;
  int32_t scaled;

  if (((wave != AD9910_WAVE_TRIANGLE) &&
       (wave != AD9910_WAVE_SQUARE) &&
       (wave != AD9910_WAVE_SAWTOOTH) &&
       (wave != AD9910_WAVE_SINC)) ||
      (points < 16U) ||
      (points > AD9910_RAM_POINTS) ||
      (amplitude > AD9910_MAX_AMPLITUDE))
  {
    return AD9910_BAD_PARAM;
  }

  for (i = 0U; i < points; i++)
  {
    sample = AD9910_GetWaveSample(wave, i, points);
    centered = (int32_t)sample - (int32_t)(AD9910_MAX_AMPLITUDE / 2U);
    scaled = (int32_t)(AD9910_MAX_AMPLITUDE / 2U) +
             (centered * (int32_t)amplitude) / (int32_t)AD9910_MAX_AMPLITUDE;

    if (scaled < 0)
    {
      scaled = 0;
    }
    else if (scaled > (int32_t)AD9910_MAX_AMPLITUDE)
    {
      scaled = (int32_t)AD9910_MAX_AMPLITUDE;
    }

    s_ram_scaled_samples[i] = (uint16_t)scaled;
  }

  status = AD9910_SetRamCustomWaveformCarrierEx(s_ram_scaled_samples,
                                                points,
                                                carrier_hz,
                                                playback_step,
                                                AD9910_RAM_DEST_POLAR);
  if (status == AD9910_OK)
  {
    return AD9910_OK;
  }

  /* Retry only on failure */
  if (retry_delay_ms > 0U)
  {
    HAL_Delay(retry_delay_ms);
    status = AD9910_SetRamCustomWaveformCarrierEx(s_ram_scaled_samples,
                                                  points,
                                                  carrier_hz,
                                                  playback_step,
                                                  AD9910_RAM_DEST_POLAR);
  }
  return status;
}

/************************************************************
 * Function :       AD9910_SetRamCustomWaveformCarrierAmplitude
 * Comment  :       输出带14位幅度控制的AD9910用户RAM波形
 * Parameter:       samples: 用户波形表; points: 波形点数; carrier_hz: 载波频率; playback_step: RAM播放步进; amplitude: 14位幅度; retry_delay_ms: 重写等待时间
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-12 V1
************************************************************/
AD9910_Status AD9910_SetRamCustomWaveformCarrierAmplitude(const uint16_t *samples,
                                                          uint16_t points,
                                                          uint32_t carrier_hz,
                                                          uint16_t playback_step,
                                                          uint16_t amplitude,
                                                          uint32_t retry_delay_ms)
{
  AD9910_Status status;
  uint16_t i;
  uint32_t sample;
  int32_t centered;
  int32_t scaled;

  if ((samples == NULL) ||
      (points < 16U) ||
      (points > AD9910_RAM_POINTS) ||
      (amplitude > AD9910_MAX_AMPLITUDE))
  {
    return AD9910_BAD_PARAM;
  }

  for (i = 0U; i < points; i++)
  {
    sample = samples[i];
    if (sample > AD9910_MAX_AMPLITUDE)
    {
      sample = AD9910_MAX_AMPLITUDE;
    }

    centered = (int32_t)sample - (int32_t)(AD9910_MAX_AMPLITUDE / 2U);
    scaled = (int32_t)(AD9910_MAX_AMPLITUDE / 2U) +
             (centered * (int32_t)amplitude) / (int32_t)AD9910_MAX_AMPLITUDE;

    if (scaled < 0)
    {
      scaled = 0;
    }
    else if (scaled > (int32_t)AD9910_MAX_AMPLITUDE)
    {
      scaled = (int32_t)AD9910_MAX_AMPLITUDE;
    }

    s_ram_scaled_samples[i] = (uint16_t)scaled;
  }

  status = AD9910_SetRamCustomWaveformCarrierEx(s_ram_scaled_samples,
                                                points,
                                                carrier_hz,
                                                playback_step,
                                                AD9910_RAM_DEST_POLAR);
  if (status == AD9910_OK)
  {
    return AD9910_OK;
  }

  /* Retry only on failure */
  if (retry_delay_ms > 0U)
  {
    HAL_Delay(retry_delay_ms);
    status = AD9910_SetRamCustomWaveformCarrierEx(s_ram_scaled_samples,
                                                  points,
                                                  carrier_hz,
                                                  playback_step,
                                                  AD9910_RAM_DEST_POLAR);
  }
  return status;
}

/************************************************************
 * Function :       AD9910_SetRamCustomWaveformCarrierPolarAmplitude
 * Comment  :       输出带幅度控制的AD9910用户RAM双极性波形，改善低频交流波形输出幅度
 * Parameter:       samples: 用户波形表; points: 波形点数; carrier_hz: 载波频率; playback_step: RAM播放步进; amplitude: 14位幅度; retry_delay_ms: 重写等待时间
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-26 V1
************************************************************/
AD9910_Status AD9910_SetRamCustomWaveformCarrierPolarAmplitude(const uint16_t *samples,
                                                               uint16_t points,
                                                               uint32_t carrier_hz,
                                                               uint16_t playback_step,
                                                               uint16_t amplitude,
                                                               uint32_t retry_delay_ms)
{
  AD9910_Status status; /* AD9910通信状态 */
  uint16_t i;           /* 采样点循环索引 */
  uint32_t sample;      /* 原始采样值 */
  int32_t centered;     /* 去中心后的有符号采样值 */
  int32_t scaled;       /* 幅度缩放后的采样值 */

  if ((samples == NULL) ||
      (points < 16U) ||
      (points > AD9910_RAM_POINTS) ||
      (amplitude > AD9910_MAX_AMPLITUDE))
  {
    return AD9910_BAD_PARAM;
  }

  for (i = 0U; i < points; i++)
  {
    sample = samples[i];
    if (sample > AD9910_MAX_AMPLITUDE)
    {
      sample = AD9910_MAX_AMPLITUDE;
    }

    centered = (int32_t)sample - (int32_t)(AD9910_MAX_AMPLITUDE / 2U);
    scaled = (int32_t)(AD9910_MAX_AMPLITUDE / 2U) +
             (centered * (int32_t)amplitude) / (int32_t)AD9910_MAX_AMPLITUDE;

    if (scaled < 0)
    {
      scaled = 0;
    }
    else if (scaled > (int32_t)AD9910_MAX_AMPLITUDE)
    {
      scaled = (int32_t)AD9910_MAX_AMPLITUDE;
    }

    s_ram_scaled_samples[i] = (uint16_t)scaled;
  }

  status = AD9910_SetRamCustomWaveformCarrierEx(s_ram_scaled_samples,
                                                points,
                                                carrier_hz,
                                                playback_step,
                                                AD9910_RAM_DEST_POLAR);
  if (status == AD9910_OK)
  {
    return AD9910_OK;
  }

  if (retry_delay_ms > 0U)
  {
    HAL_Delay(retry_delay_ms);
    status = AD9910_SetRamCustomWaveformCarrierEx(s_ram_scaled_samples,
                                                  points,
                                                  carrier_hz,
                                                  playback_step,
                                                  AD9910_RAM_DEST_POLAR);
  }

  return status;
}

/************************************************************
 * Function :       AD9910_SetRamWaveformPoints
 * Comment  :       使用指定点数RAM输出AD9910内置波形
 * Parameter:       wave: 内置波形类型; playback_step: RAM播放步进; points: RAM点数
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
AD9910_Status AD9910_SetRamWaveformPoints(AD9910_Waveform wave,
                                          uint16_t playback_step,
                                          uint16_t points)
{
  return AD9910_SetRamWaveformPointsEx(wave,
                                       playback_step,
                                       points,
                                       AD9910_RAM_DEST_POLAR);
}

/************************************************************
 * Function :       AD9910_SetRamPlaybackStep
 * Comment  :       不重写RAM表, 仅更新AD9910 RAM播放步进
 * Parameter:       playback_step: RAM播放步进; points: 当前RAM点数
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
AD9910_Status AD9910_SetRamPlaybackStep(uint16_t playback_step, uint16_t points)
{
  uint8_t ram_profile0[8];
  uint16_t end_field;

  if ((points < 16U) || (points > AD9910_RAM_POINTS))
  {
    return AD9910_BAD_PARAM;
  }

  if (playback_step == 0U)
  {
    playback_step = 1U;
  }

  end_field = (uint16_t)((points - 1U) << 6);

  ram_profile0[0] = 0x00U;
  ram_profile0[1] = (uint8_t)(playback_step >> 8);
  ram_profile0[2] = (uint8_t)playback_step;
  ram_profile0[3] = (uint8_t)(end_field >> 8);
  ram_profile0[4] = (uint8_t)end_field;
  ram_profile0[5] = 0x00U;
  ram_profile0[6] = 0x00U;
  ram_profile0[7] = 0x04U;

  return AD9910_WriteAllRamProfiles(ram_profile0, sizeof(ram_profile0));
}

/************************************************************
 * Function :       AD9910_CalcRamOutputHz
 * Comment  :       按默认1024点RAM计算波形输出频率
 * Parameter:       playback_step: RAM播放步进
 * Return   :       估算输出频率(Hz)
 * Date     :       2026-06-10 V1
************************************************************/
uint32_t AD9910_CalcRamOutputHz(uint16_t playback_step)
{
  return AD9910_CalcRamOutputHzPoints(playback_step, AD9910_RAM_POINTS);
}

/************************************************************
 * Function :       AD9910_CalcRamOutputHzPoints
 * Comment  :       按指定RAM点数计算波形输出频率
 * Parameter:       playback_step: RAM播放步进; points: RAM点数
 * Return   :       估算输出频率(Hz)
 * Date     :       2026-06-10 V1
************************************************************/
uint32_t AD9910_CalcRamOutputHzPoints(uint16_t playback_step, uint16_t points)
{
  uint64_t denom;

  if (points == 0U)
  {
    return 0U;
  }

  if (playback_step == 0U)
  {
    playback_step = 1U;
  }

  denom = 4ULL * (uint64_t)playback_step * (uint64_t)points;
  return (uint32_t)(AD9910_SYSCLK_HZ / denom);
}

/************************************************************
 * Function :       AD9910_CalcRamPlaybackStep
 * Comment  :       根据目标波形频率计算AD9910 RAM播放步进
 * Parameter:       output_hz: 目标频率, 单位Hz; points: RAM波形点数
 * Return   :       RAM播放步进1~65535, 返回0表示频率超出可实现范围
 * Date     :       2026-06-12 V1
************************************************************/
uint16_t AD9910_CalcRamPlaybackStep(uint32_t output_hz, uint16_t points)
{
  uint64_t denom;
  uint64_t step;

  if ((output_hz == 0U) ||
      (points < 16U) ||
      (points > AD9910_RAM_POINTS) ||
      (output_hz > AD9910_CalcRamOutputHzPoints(1U, points)))
  {
    return 0U;
  }

  denom = 4ULL * (uint64_t)points * (uint64_t)output_hz;
  step = (AD9910_SYSCLK_HZ + (denom / 2ULL)) / denom;
  if ((step == 0ULL) || (step > 65535ULL))
  {
    return 0U;
  }

  return (uint16_t)step;
}

/************************************************************
 * Function :       AD9910_ConfigureFrequencySweep
 * Comment  :       配置AD9910数字斜坡频率扫频
 * Parameter:       low_hz: 起始频率; high_hz: 终止频率; up_step_hz: 上扫步进; down_step_hz: 下扫步进; up_rate: 上扫速率; down_rate: 下扫速率; mode: 扫频模式
 * Return   :       AD9910_OK表示成功, 其他值表示参数或通信错误
 * Date     :       2026-06-10 V1
************************************************************/
AD9910_Status AD9910_ConfigureFrequencySweep(uint32_t low_hz,
                                             uint32_t high_hz,
                                             uint32_t up_step_hz,
                                             uint32_t down_step_hz,
                                             uint16_t up_rate,
                                             uint16_t down_rate,
                                             AD9910_SweepMode mode)
{
  uint8_t cfr1[4] = {0x00U, 0x40U, 0x00U, 0x00U};
  uint8_t cfr2[4] = {0x00U, 0x48U, 0x08U, 0x20U};
  uint8_t dr_limit[8];
  uint8_t dr_step[8];
  uint8_t dr_rate[4];
  uint32_t lower;
  uint32_t upper;
  uint32_t inc;
  uint32_t dec;
  AD9910_Status status;

  if ((low_hz >= high_hz) ||
      (high_hz > AD9910_MAX_OUTPUT_HZ) ||
      (up_step_hz == 0U) ||
      (down_step_hz == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  if (mode == AD9910_SWEEP_AUTO)
  {
    cfr2[1] |= 0x06U;
  }

  lower = AD9910_FrequencyToTuningWord(low_hz);
  upper = AD9910_FrequencyToTuningWord(high_hz);
  inc = AD9910_FrequencyToTuningWord(up_step_hz);
  dec = AD9910_FrequencyToTuningWord(down_step_hz);

  AD9910_PutU32Be(&dr_limit[0], upper);
  AD9910_PutU32Be(&dr_limit[4], lower);
  AD9910_PutU32Be(&dr_step[0], dec);
  AD9910_PutU32Be(&dr_step[4], inc);

  dr_rate[0] = (uint8_t)(down_rate >> 8);
  dr_rate[1] = (uint8_t)down_rate;
  dr_rate[2] = (uint8_t)(up_rate >> 8);
  dr_rate[3] = (uint8_t)up_rate;

  status = AD9910_WriteRegister(AD9910_REG_CFR1, cfr1, sizeof(cfr1));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteRegister(AD9910_REG_CFR2, cfr2, sizeof(cfr2));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteRegister(AD9910_REG_DR_LIMIT, dr_limit, sizeof(dr_limit));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteRegister(AD9910_REG_DR_STEP, dr_step, sizeof(dr_step));
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_WriteRegister(AD9910_REG_DR_RATE, dr_rate, sizeof(dr_rate));
  if (status == AD9910_OK)
  {
    AD9910_PulseIoUpdate();
  }

  return status;
}

/************************************************************
 * Function :       AD9910_SetSweepDirection
 * Comment  :       设置AD9910手动扫频方向
 * Parameter:       direction: 扫频方向
 * Return   :       null
 * Date     :       2026-06-10 V1
************************************************************/
void AD9910_SetSweepDirection(AD9910_SweepDirection direction)
{
  AD9910_PIN_WRITE(DRCTL, (direction == AD9910_SWEEP_UP) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/************************************************************
 * Function :       AD9910_HoldSweep
 * Comment  :       控制AD9910扫频保持或释放
 * Parameter:       hold: GPIO_PIN_SET保持, GPIO_PIN_RESET释放
 * Return   :       null
 * Date     :       2026-06-10 V1
************************************************************/
void AD9910_HoldSweep(GPIO_PinState hold)
{
  AD9910_PIN_WRITE(DRHOLD, hold);
}

/************************************************************
 * Function :       AD9910_IsSweepDone
 * Comment  :       读取AD9910扫频完成状态引脚
 * Parameter:       null
 * Return   :       1表示完成, 0表示未完成
 * Date     :       2026-06-10 V1
************************************************************/
uint8_t AD9910_IsSweepDone(void)
{
  return (HAL_GPIO_ReadPin(AD9910_DROVER_GPIO_Port, AD9910_DROVER_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

/************************************************************
 * Function :       AD9910_SetProfile
 * Comment  :       设置AD9910 Profile选择引脚PF0~PF2
 * Parameter:       profile: Profile编号(0~7)
 * Return   :       null
 * Date     :       2026-06-10 V1
************************************************************/
void AD9910_SetProfile(uint8_t profile)
{
  AD9910_PIN_WRITE(PF0, ((profile & 0x01U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  AD9910_PIN_WRITE(PF1, ((profile & 0x02U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  AD9910_PIN_WRITE(PF2, ((profile & 0x04U) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/************************************************************
 * Function :       AD9910_PowerDown
 * Comment  :       控制AD9910外部掉电引脚
 * Parameter:       power_down: GPIO_PIN_SET掉电, GPIO_PIN_RESET运行
 * Return   :       null
 * Date     :       2026-06-10 V1
************************************************************/
void AD9910_PowerDown(GPIO_PinState power_down)
{
  AD9910_PIN_WRITE(PWR, power_down);
}
