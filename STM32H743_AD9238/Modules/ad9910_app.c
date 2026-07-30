#include "ad9910_app.h"
#include <math.h>

/* AD9910应用层后台任务类型。 */
typedef enum
{
  AD9910_APP_PROCESS_NONE = 0,       /* 无后台任务。 */
  AD9910_APP_PROCESS_RAM_SWEEP,      /* RAM波形软件扫频。 */
  AD9910_APP_PROCESS_MANUAL_SWEEP,   /* 数字斜坡手动换向。 */
  AD9910_APP_PROCESS_SINE_SWEEP      /* 正弦波软件扫频。 */
} AD9910_AppProcessTypeDef;

/* AD9910 RAM波形扫频状态。 */
typedef struct
{
  uint16_t points;             /* RAM波形点数。 */
  uint32_t low_hz;             /* 扫频下限，单位Hz。 */
  uint32_t high_hz;            /* 扫频上限，单位Hz。 */
  uint32_t current_hz;         /* 当前目标频率，单位Hz。 */
  uint32_t frequency_step_hz;  /* 频率步进，单位Hz。 */
  int8_t direction;            /* 扫频方向，1向上、-1向下。 */
  uint32_t dwell_ms;           /* 频点驻留时间，单位ms。 */
  uint32_t last_tick;          /* 上次换频系统节拍。 */
} AD9910_AppRamSweepTypeDef;

/* AD9910正弦波软件扫频状态。 */
typedef struct
{
  uint32_t low_hz;             /* 扫频下限，单位Hz。 */
  uint32_t high_hz;            /* 扫频上限，单位Hz。 */
  uint32_t current_hz;         /* 当前频率，单位Hz。 */
  uint32_t frequency_step_hz;  /* 频率步进，单位Hz。 */
  int8_t direction;            /* 扫频方向，1向上、-1向下。 */
  uint32_t dwell_ms;           /* 频点驻留时间，单位ms。 */
  uint32_t last_tick;          /* 上次换频系统节拍。 */
  uint16_t amplitude;          /* 14位输出幅度。 */
  AD9910_SweepDirMode dir_mode; /* 终点处理方式。 */
} AD9910_AppSineSweepTypeDef;

static uint8_t s_ad9910_initialized; /* 应用层初始化标志。 */
static AD9910_AppProcessTypeDef s_process_type = AD9910_APP_PROCESS_NONE; /* 当前后台任务。 */
static AD9910_AppRamSweepTypeDef s_ram_sweep; /* RAM波形扫频状态。 */
static AD9910_AppSineSweepTypeDef s_sine_sweep; /* 正弦波软件扫频状态。 */
static AD9910_SweepDirection s_manual_direction = AD9910_SWEEP_UP; /* 手动扫频方向。 */
static uint32_t s_manual_toggle_ms; /* 手动换向间隔，单位ms。 */
static uint32_t s_manual_last_tick; /* 上次手动换向系统节拍。 */

/* AD9910应用层RAM相位偏移表，用于通过改变RAM索引实现通用波形相位控制 */
static uint16_t s_ram_phase_table[AD9910_RAM_POINTS];

/* AD9910用户可编辑任意波表。 */
uint16_t AD9910_AppArbitraryTable[AD9910_APP_WAVE_TABLE_SIZE] =
{
      0U,  2048U,  4096U,  6144U,  8192U, 10240U, 12288U, 14336U,
  16383U, 14336U, 12288U, 10240U,  8192U,  6144U,  4096U,  2048U,
      0U,     0U, 16383U, 16383U,     0U,     0U, 16383U, 16383U,
   8192U,  8192U,  4096U, 12288U,  4096U, 12288U,  8192U,     0U
};

/*
 * RAM波形频率说明:
 * 三角波、方波、锯齿波和任意波的实际频率由points和playback_step共同决定。
 * actual_hz = AD9910_SYSCLK_HZ / (4 * points * playback_step), playback_step只能取整数。
 * 因此output_hz是目标频率, 若points选择不合适会出现量化误差; 100kHz测试建议points=125。
 */

/************************************************************
 * Function :       AD9910_AppGetRamWaveSample
 * Comment  :       根据波形类型和采样索引生成一个14位RAM波形采样点
 * Parameter:       wave: RAM波形类型; index: 当前采样索引; points: 一个周期采样点数
 * Return   :       14位采样值，范围0~16383
 * Date     :       2026-06-26 V1
************************************************************/
static uint16_t AD9910_AppGetRamWaveSample(AD9910_Waveform wave, uint16_t index, uint16_t points)
{
  uint32_t value;       /* 计算得到的RAM波形采样值 */
  uint16_t half_points; /* 半周期采样点数 */
  double x;             /* SINC波形归一化横坐标 */
  double y;             /* SINC波形归一化纵坐标 */
  const double pi = 3.14159265358979323846;       /* 圆周率常量 */
  const double sinc_min = -0.21723362821122166;   /* SINC第一负旁瓣近似最小值 */

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
      value = ((uint32_t)index * AD9910_MAX_AMPLITUDE) / ((uint32_t)points - 1U);
      return (uint16_t)value;

    case AD9910_WAVE_SINC:
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
      else if (y > 1.0)
      {
        y = 1.0;
      }
      return (uint16_t)((y * (double)AD9910_MAX_AMPLITUDE) + 0.5);
  }

  return 0U;
}

/************************************************************
 * Function :       AD9910_AppInit
 * Comment  :       初始化AD9910应用层，主函数先调用一次
 * Parameter:       null
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-12 V3
************************************************************/
AD9910_Status AD9910_AppInit(void)
{
  AD9910_Status status;

  status = AD9910_Init(NULL);
  if (status == AD9910_OK)
  {
    s_ad9910_initialized = 1U;
    AD9910_AppStop();
  }

  return status;
}

/************************************************************
 * Function :       AD9910_AppOutputSine
 * Comment  :       输出AD9910单频正弦波
 * Parameter:       freq_hz: 输出频率，范围0~450000000Hz; amplitude: 14位幅度，范围0~16383
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-12 V3
************************************************************/
AD9910_Status AD9910_AppOutputSine(uint32_t freq_hz, uint16_t amplitude)
{
  AD9910_Status status;

  if (amplitude > AD9910_MAX_AMPLITUDE)
  {
    return AD9910_BAD_PARAM;
  }

  if (s_ad9910_initialized == 0U)
  {
    status = AD9910_AppInit();
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  AD9910_AppStop();

  /* 应用层正弦输出只使用公开库接口配置频率和幅度，不直接走底层 SetSingleTone。 */
  status = AD9910_SetAmplitude(amplitude);
  if (status != AD9910_OK)
  {
    return status;
  }

  return AD9910_SetFrequencyHz(freq_hz);
}

/************************************************************
 * Function :       AD9910_AppSetSinePhaseOffsetDeg
 * Comment  :       应用层设置AD9910当前正弦输出相位偏移
 * Parameter:       phase_deg: 相位偏移角度，单位度
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-26 V1
************************************************************/
AD9910_Status AD9910_AppSetSinePhaseOffsetDeg(float phase_deg)
{
  AD9910_Status status; /* AD9910应用层初始化状态 */
  uint16_t phase_word; /* AD9910 16位相位偏移字 */

  if (s_ad9910_initialized == 0U)
  {
    status = AD9910_AppInit();
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  while (phase_deg >= 360.0f)
  {
    phase_deg -= 360.0f;
  }

  while (phase_deg < 0.0f)
  {
    phase_deg += 360.0f;
  }

  phase_word = (uint16_t)((phase_deg * 65536.0f) / 360.0f);
  return AD9910_SetPhaseOffsetWord(phase_word);
}

/************************************************************
 * Function :       AD9910_AppOutputRamWavePhaseIndex
 * Comment  :       按RAM相位索引输出AD9910内置RAM波形，用改变采样起点实现相位偏移
 * Parameter:       wave: RAM波形类型; output_hz: 目标波形频率; points: 点数，范围16~1024; amplitude: 14位幅度; phase_index: 相位索引，0~points-1对应0~360度; retry_delay_ms: 重写等待时间
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-26 V1
************************************************************/
AD9910_Status AD9910_AppOutputRamWavePhaseIndex(AD9910_Waveform wave,
                                                uint32_t output_hz,
                                                uint16_t points,
                                                uint16_t amplitude,
                                                uint16_t phase_index,
                                                uint32_t retry_delay_ms)
{
  AD9910_Status status;     /* AD9910应用层状态 */
  uint16_t playback_step;   /* AD9910 RAM播放步进 */
  uint16_t sample_index;    /* 加入相位偏移后的采样索引 */

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

  playback_step = AD9910_CalcRamPlaybackStep(output_hz, points);
  if (playback_step == 0U)
  {
    return AD9910_BAD_PARAM;
  }

  if (s_ad9910_initialized == 0U)
  {
    status = AD9910_AppInit();
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  phase_index %= points;
  for (uint16_t i = 0U; i < points; i++)
  {
    sample_index = (uint16_t)(i + phase_index);
    if (sample_index >= points)
    {
      sample_index = (uint16_t)(sample_index - points);
    }

    s_ram_phase_table[i] = AD9910_AppGetRamWaveSample(wave, sample_index, points);
  }

  AD9910_AppStop();
  return AD9910_SetRamCustomWaveformCarrierPolarAmplitude(s_ram_phase_table,
                                                          points,
                                                          0U,
                                                          playback_step,
                                                          amplitude,
                                                          retry_delay_ms);
}

/************************************************************
 * Function :       AD9910_AppOutputArbitraryPhaseIndex
 * Comment  :       按RAM相位索引输出AD9910用户任意波，用改变采样起点实现相位偏移
 * Parameter:       wave_table: 用户波形表; points: 点数，范围16~1024; output_hz: 目标波形频率; amplitude: 14位幅度; phase_index: 相位索引，0~points-1对应0~360度; retry_delay_ms: 重写等待时间
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-26 V1
************************************************************/
AD9910_Status AD9910_AppOutputArbitraryPhaseIndex(const uint16_t *wave_table,
                                                  uint16_t points,
                                                  uint32_t output_hz,
                                                  uint16_t amplitude,
                                                  uint16_t phase_index,
                                                  uint32_t retry_delay_ms)
{
  AD9910_Status status;     /* AD9910应用层状态 */
  uint16_t playback_step;   /* AD9910 RAM播放步进 */
  uint16_t sample_index;    /* 加入相位偏移后的采样索引 */

  if ((wave_table == NULL) ||
      (points < 16U) ||
      (points > AD9910_RAM_POINTS) ||
      (amplitude > AD9910_MAX_AMPLITUDE))
  {
    return AD9910_BAD_PARAM;
  }

  playback_step = AD9910_CalcRamPlaybackStep(output_hz, points);
  if (playback_step == 0U)
  {
    return AD9910_BAD_PARAM;
  }

  if (s_ad9910_initialized == 0U)
  {
    status = AD9910_AppInit();
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  phase_index %= points;
  for (uint16_t i = 0U; i < points; i++)
  {
    sample_index = (uint16_t)(i + phase_index);
    if (sample_index >= points)
    {
      sample_index = (uint16_t)(sample_index - points);
    }

    s_ram_phase_table[i] = wave_table[sample_index];
  }

  AD9910_AppStop();
  return AD9910_SetRamCustomWaveformCarrierPolarAmplitude(s_ram_phase_table,
                                                          points,
                                                          0U,
                                                          playback_step,
                                                          amplitude,
                                                          retry_delay_ms);
}

/************************************************************
 * Function :       AD9910_AppOutputTrianglePhaseIndex
 * Comment  :       按RAM相位索引输出AD9910三角波，用通用RAM调相接口实现
 * Parameter:       output_hz: 目标波形频率; points: 点数，范围16~1024; amplitude: 14位幅度; phase_index: 相位索引，0~points-1对应0~360度; retry_delay_ms: 重写等待时间
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-26 V2
************************************************************/
AD9910_Status AD9910_AppOutputTrianglePhaseIndex(uint32_t output_hz,
                                                 uint16_t points,
                                                 uint16_t amplitude,
                                                 uint16_t phase_index,
                                                 uint32_t retry_delay_ms)
{
  return AD9910_AppOutputRamWavePhaseIndex(AD9910_WAVE_TRIANGLE,
                                           output_hz,
                                           points,
                                           amplitude,
                                           phase_index,
                                           retry_delay_ms);
}

/************************************************************
 * Function :       AD9910_AppOutputSquarePhaseIndex
 * Comment  :       按RAM相位索引输出AD9910方波，用改变采样起点实现相位偏移
 * Parameter:       output_hz: 目标波形频率; points: RAM点数，范围16~1024; amplitude: 14位幅度; phase_index: 相位索引，0~points-1对应0~360度; retry_delay_ms: 重写等待时间
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-28 V1
************************************************************/
AD9910_Status AD9910_AppOutputSquarePhaseIndex(uint32_t output_hz,
                                               uint16_t points,
                                               uint16_t amplitude,
                                               uint16_t phase_index,
                                               uint32_t retry_delay_ms)
{
  return AD9910_AppOutputRamWavePhaseIndex(AD9910_WAVE_SQUARE,
                                           output_hz,
                                           points,
                                           amplitude,
                                           phase_index,
                                           retry_delay_ms);
}

/************************************************************
 * Function :       AD9910_AppOutputSquare
 * Comment  :       输出AD9910方波
 * Parameter:       output_hz: 目标波形频率, 实际频率受points和整数step影响; points: 点数，范围16~1024; amplitude: 14位幅度，范围0~16383; retry_delay_ms: 重写等待时间，0为不重写
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-12 V3
************************************************************/
AD9910_Status AD9910_AppOutputSquare(uint32_t output_hz,
                                     uint16_t points,
                                     uint16_t amplitude,
                                     uint32_t retry_delay_ms)
{
  return AD9910_AppOutputSquarePhaseIndex(output_hz,
                                          points,
                                          amplitude,
                                          0U,
                                          retry_delay_ms);
}

/************************************************************
 * Function :       AD9910_AppOutputSawtoothPhaseIndex
 * Comment  :       按RAM相位索引输出AD9910锯齿波，用改变采样起点实现相位偏移
 * Parameter:       output_hz: 目标波形频率; points: RAM点数，范围16~1024; amplitude: 14位幅度; phase_index: 相位索引，0~points-1对应0~360度; retry_delay_ms: 重写等待时间
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-28 V1
************************************************************/
AD9910_Status AD9910_AppOutputSawtoothPhaseIndex(uint32_t output_hz,
                                                 uint16_t points,
                                                 uint16_t amplitude,
                                                 uint16_t phase_index,
                                                 uint32_t retry_delay_ms)
{
  return AD9910_AppOutputRamWavePhaseIndex(AD9910_WAVE_SAWTOOTH,
                                           output_hz,
                                           points,
                                           amplitude,
                                           phase_index,
                                           retry_delay_ms);
}

/************************************************************
 * Function :       AD9910_AppOutputSawtooth
 * Comment  :       输出AD9910锯齿波
 * Parameter:       output_hz: 目标波形频率, 实际频率受points和整数step影响; points: 点数，范围16~1024; amplitude: 14位幅度，范围0~16383; retry_delay_ms: 重写等待时间，0为不重写
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-12 V3
************************************************************/
AD9910_Status AD9910_AppOutputSawtooth(uint32_t output_hz,
                                       uint16_t points,
                                       uint16_t amplitude,
                                       uint32_t retry_delay_ms)
{
  return AD9910_AppOutputSawtoothPhaseIndex(output_hz,
                                            points,
                                            amplitude,
                                            0U,
                                            retry_delay_ms);
}

/************************************************************
 * Function :       AD9910_AppOutputArbitraryHz
 * Comment  :       按目标波形频率输出AD9910用户任意波形
 * Parameter:       wave_table: 用户波形表，每点范围0~16383; points: 点数，范围16~1024; output_hz: 目标波形频率, 实际频率受points和整数step影响; amplitude: 14位幅度，范围0~16383; retry_delay_ms: 重写等待时间，0为不重写
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-12 V3
************************************************************/
AD9910_Status AD9910_AppOutputArbitraryHz(const uint16_t *wave_table,
                                          uint16_t points,
                                          uint32_t output_hz,
                                          uint16_t amplitude,
                                          uint32_t retry_delay_ms)
{
  return AD9910_AppOutputArbitraryPhaseIndex(wave_table,
                                             points,
                                             output_hz,
                                             amplitude,
                                             0U,
                                             retry_delay_ms);
}

/************************************************************
 * Function :       AD9910_AppSweepSine
 * Comment  :       启动AD9910正弦扫频，使用AD9910数字扫频功能
 * Parameter:       low_hz: 扫频下限Hz; high_hz: 扫频上限Hz; up_step_hz: 上扫频率步进Hz; down_step_hz: 下扫频率步进Hz; up_rate: 上扫驻留控制字; down_rate: 下扫驻留控制字; amplitude: 14位幅度，范围0~16383; mode: 自动或手动扫频; manual_direction: 手动初始方向; manual_toggle_ms: 手动方向切换间隔，0为不自动切换
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-12 V3
************************************************************/
AD9910_Status AD9910_AppSweepSine(uint32_t low_hz,
                                  uint32_t high_hz,
                                  uint32_t up_step_hz,
                                  uint32_t down_step_hz,
                                  uint16_t up_rate,
                                  uint16_t down_rate,
                                  uint16_t amplitude,
                                  AD9910_SweepMode mode,
                                  AD9910_SweepDirection manual_direction,
                                  uint32_t manual_toggle_ms)
{
  AD9910_Status status;

  if (amplitude > AD9910_MAX_AMPLITUDE)
  {
    return AD9910_BAD_PARAM;
  }

  if (s_ad9910_initialized == 0U)
  {
    status = AD9910_AppInit();
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  AD9910_AppStop();
  status = AD9910_ConfigureFrequencySweep(low_hz,
                                          high_hz,
                                          up_step_hz,
                                          down_step_hz,
                                          up_rate,
                                          down_rate,
                                          mode);
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_SetAmplitude(amplitude);
  if (status != AD9910_OK)
  {
    return status;
  }

  if (mode == AD9910_SWEEP_MANUAL)
  {
    s_manual_direction = manual_direction;
    s_manual_toggle_ms = manual_toggle_ms;
    s_manual_last_tick = HAL_GetTick();
    AD9910_SetSweepDirection(s_manual_direction);
    if (manual_toggle_ms != 0U)
    {
      s_process_type = AD9910_APP_PROCESS_MANUAL_SWEEP;
    }
  }

  return AD9910_OK;
}

/************************************************************
 * Function :       AD9910_AppSweepSineMs
 * Comment  :       启动AD9910正弦软件扫频，通过改写频率字实现ms~s级驻留
 * Parameter:       low_hz/high_hz: 扫频范围, 单位Hz, low_hz<high_hz且≤450MHz
 *                  frequency_step_hz: 频率步进, 单位Hz, 最小值约0.23Hz
 *                  amplitude: 14位幅度, 范围0~16383
 *                  dwell_ms: 每个频点驻留时间, 单位ms, 无上限
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Note     :       需要在while(1)中周期调用AD9910_AppProcess()驱动状态机
 * Date     :       2026-06-15 V1
************************************************************/
AD9910_Status AD9910_AppSweepSineMs(uint32_t low_hz,
                                     uint32_t high_hz,
                                     uint32_t frequency_step_hz,
                                     uint16_t amplitude,
                                     uint32_t dwell_ms,
                                     AD9910_SweepDirMode dir_mode)
{
  AD9910_Status status;

  if ((low_hz == 0U) ||
      (high_hz <= low_hz) ||
      (frequency_step_hz == 0U) ||
      (amplitude > AD9910_MAX_AMPLITUDE) ||
      (dwell_ms == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  if (s_ad9910_initialized == 0U)
  {
    status = AD9910_AppInit();
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  AD9910_AppStop();

  status = AD9910_SetAmplitude(amplitude);
  if (status != AD9910_OK)
  {
    return status;
  }

  status = AD9910_SetFrequencyHz(low_hz);
  if (status != AD9910_OK)
  {
    return status;
  }

  s_sine_sweep.low_hz            = low_hz;
  s_sine_sweep.high_hz           = high_hz;
  s_sine_sweep.current_hz        = low_hz;
  s_sine_sweep.frequency_step_hz = frequency_step_hz;
  s_sine_sweep.amplitude         = amplitude;
  s_sine_sweep.direction         = 1;
  s_sine_sweep.dwell_ms          = dwell_ms;
  s_sine_sweep.dir_mode          = dir_mode;
  s_sine_sweep.last_tick         = HAL_GetTick();
  s_process_type = AD9910_APP_PROCESS_SINE_SWEEP;

  return AD9910_OK;
}

/************************************************************
 * Function :       AD9910_AppSweepTriangle
 * Comment  :       启动AD9910三角波扫频，通过改变RAM播放步进实现
 * Parameter:       low_hz/high_hz: 目标扫频范围Hz, 实际频点受points和整数step影响; frequency_step_hz: 目标频率步进Hz; points: 点数，范围16~1024; amplitude: 14位幅度，范围0~16383; dwell_ms: 每步驻留时间ms; retry_delay_ms: 重写等待时间，0为不重写
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-12 V3
************************************************************/
AD9910_Status AD9910_AppSweepTriangle(uint32_t low_hz,
                                      uint32_t high_hz,
                                      uint32_t frequency_step_hz,
                                      uint16_t points,
                                      uint16_t amplitude,
                                      uint32_t dwell_ms,
                                      uint32_t retry_delay_ms)
{
  AD9910_Status status;
  uint16_t playback_step;

  if ((points < 16U) ||
      (points > AD9910_RAM_POINTS) ||
      (low_hz == 0U) ||
      (high_hz <= low_hz) ||
      (frequency_step_hz == 0U) ||
      (amplitude > AD9910_MAX_AMPLITUDE) ||
      (dwell_ms == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  playback_step = AD9910_CalcRamPlaybackStep(low_hz, points);
  if ((playback_step == 0U) ||
      (AD9910_CalcRamPlaybackStep(high_hz, points) == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  if (s_ad9910_initialized == 0U)
  {
    status = AD9910_AppInit();
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  status = AD9910_AppOutputTrianglePhaseIndex(low_hz,
                                              points,
                                              amplitude,
                                              0U,
                                              retry_delay_ms);
  if (status != AD9910_OK)
  {
    return status;
  }

  s_ram_sweep.points = points;
  s_ram_sweep.low_hz = low_hz;
  s_ram_sweep.high_hz = high_hz;
  s_ram_sweep.current_hz = low_hz;
  s_ram_sweep.frequency_step_hz = frequency_step_hz;
  s_ram_sweep.direction = 1;
  s_ram_sweep.dwell_ms = dwell_ms;
  s_ram_sweep.last_tick = HAL_GetTick();
  s_process_type = AD9910_APP_PROCESS_RAM_SWEEP;

  return AD9910_OK;
}

/************************************************************
 * Function :       AD9910_AppSweepSquare
 * Comment  :       启动AD9910方波扫频，通过改变RAM播放步进实现
 * Parameter:       low_hz/high_hz: 目标扫频范围Hz, 实际频点受points和整数step影响; frequency_step_hz: 目标频率步进Hz; points: 点数，范围16~1024; amplitude: 14位幅度，范围0~16383; dwell_ms: 每步驻留时间ms; retry_delay_ms: 重写等待时间，0为不重写
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-12 V3
************************************************************/
AD9910_Status AD9910_AppSweepSquare(uint32_t low_hz,
                                    uint32_t high_hz,
                                    uint32_t frequency_step_hz,
                                    uint16_t points,
                                    uint16_t amplitude,
                                    uint32_t dwell_ms,
                                    uint32_t retry_delay_ms)
{
  AD9910_Status status;
  uint16_t playback_step;

  if ((points < 16U) ||
      (points > AD9910_RAM_POINTS) ||
      (low_hz == 0U) ||
      (high_hz <= low_hz) ||
      (frequency_step_hz == 0U) ||
      (amplitude > AD9910_MAX_AMPLITUDE) ||
      (dwell_ms == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  playback_step = AD9910_CalcRamPlaybackStep(low_hz, points);
  if ((playback_step == 0U) ||
      (AD9910_CalcRamPlaybackStep(high_hz, points) == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  if (s_ad9910_initialized == 0U)
  {
    status = AD9910_AppInit();
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  status = AD9910_AppOutputSquarePhaseIndex(low_hz,
                                            points,
                                            amplitude,
                                            0U,
                                            retry_delay_ms);
  if (status != AD9910_OK)
  {
    return status;
  }

  s_ram_sweep.points = points;
  s_ram_sweep.low_hz = low_hz;
  s_ram_sweep.high_hz = high_hz;
  s_ram_sweep.current_hz = low_hz;
  s_ram_sweep.frequency_step_hz = frequency_step_hz;
  s_ram_sweep.direction = 1;
  s_ram_sweep.dwell_ms = dwell_ms;
  s_ram_sweep.last_tick = HAL_GetTick();
  s_process_type = AD9910_APP_PROCESS_RAM_SWEEP;

  return AD9910_OK;
}

/************************************************************
 * Function :       AD9910_AppSweepSawtooth
 * Comment  :       启动AD9910锯齿波扫频，通过改变RAM播放步进实现
 * Parameter:       low_hz/high_hz: 目标扫频范围Hz, 实际频点受points和整数step影响; frequency_step_hz: 目标频率步进Hz; points: 点数，范围16~1024; amplitude: 14位幅度，范围0~16383; dwell_ms: 每步驻留时间ms; retry_delay_ms: 重写等待时间，0为不重写
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-12 V3
************************************************************/
AD9910_Status AD9910_AppSweepSawtooth(uint32_t low_hz,
                                      uint32_t high_hz,
                                      uint32_t frequency_step_hz,
                                      uint16_t points,
                                      uint16_t amplitude,
                                      uint32_t dwell_ms,
                                      uint32_t retry_delay_ms)
{
  AD9910_Status status;
  uint16_t playback_step;

  if ((points < 16U) ||
      (points > AD9910_RAM_POINTS) ||
      (low_hz == 0U) ||
      (high_hz <= low_hz) ||
      (frequency_step_hz == 0U) ||
      (amplitude > AD9910_MAX_AMPLITUDE) ||
      (dwell_ms == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  playback_step = AD9910_CalcRamPlaybackStep(low_hz, points);
  if ((playback_step == 0U) ||
      (AD9910_CalcRamPlaybackStep(high_hz, points) == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  if (s_ad9910_initialized == 0U)
  {
    status = AD9910_AppInit();
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  status = AD9910_AppOutputSawtoothPhaseIndex(low_hz,
                                              points,
                                              amplitude,
                                              0U,
                                              retry_delay_ms);
  if (status != AD9910_OK)
  {
    return status;
  }

  s_ram_sweep.points = points;
  s_ram_sweep.low_hz = low_hz;
  s_ram_sweep.high_hz = high_hz;
  s_ram_sweep.current_hz = low_hz;
  s_ram_sweep.frequency_step_hz = frequency_step_hz;
  s_ram_sweep.direction = 1;
  s_ram_sweep.dwell_ms = dwell_ms;
  s_ram_sweep.last_tick = HAL_GetTick();
  s_process_type = AD9910_APP_PROCESS_RAM_SWEEP;

  return AD9910_OK;
}

/************************************************************
 * Function :       AD9910_AppSweepArbitrary
 * Comment  :       启动AD9910用户任意波形扫频，通过改变RAM播放步进实现
 * Parameter:       wave_table: 用户波形表，每点范围0~16383; points: 点数，范围16~1024; low_hz/high_hz: 目标扫频范围Hz, 实际频点受points和整数step影响; frequency_step_hz: 目标频率步进Hz; amplitude: 14位幅度，范围0~16383; dwell_ms: 每步驻留时间ms; retry_delay_ms: 重写等待时间，0为不重写
 * Return   :       AD9910_OK表示成功，其他值表示失败
 * Date     :       2026-06-12 V3
************************************************************/
AD9910_Status AD9910_AppSweepArbitrary(const uint16_t *wave_table,
                                       uint16_t points,
                                       uint32_t low_hz,
                                       uint32_t high_hz,
                                       uint32_t frequency_step_hz,
                                       uint16_t amplitude,
                                       uint32_t dwell_ms,
                                       uint32_t retry_delay_ms)
{
  AD9910_Status status;
  uint16_t playback_step;

  if ((wave_table == NULL) ||
      (points < 16U) ||
      (points > AD9910_RAM_POINTS) ||
      (low_hz == 0U) ||
      (high_hz <= low_hz) ||
      (frequency_step_hz == 0U) ||
      (amplitude > AD9910_MAX_AMPLITUDE) ||
      (dwell_ms == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  playback_step = AD9910_CalcRamPlaybackStep(low_hz, points);
  if ((playback_step == 0U) ||
      (AD9910_CalcRamPlaybackStep(high_hz, points) == 0U))
  {
    return AD9910_BAD_PARAM;
  }

  if (s_ad9910_initialized == 0U)
  {
    status = AD9910_AppInit();
    if (status != AD9910_OK)
    {
      return status;
    }
  }

  status = AD9910_AppOutputArbitraryPhaseIndex(wave_table,
                                               points,
                                               low_hz,
                                               amplitude,
                                               0U,
                                               retry_delay_ms);
  if (status != AD9910_OK)
  {
    return status;
  }

  s_ram_sweep.points = points;
  s_ram_sweep.low_hz = low_hz;
  s_ram_sweep.high_hz = high_hz;
  s_ram_sweep.current_hz = low_hz;
  s_ram_sweep.frequency_step_hz = frequency_step_hz;
  s_ram_sweep.direction = 1;
  s_ram_sweep.dwell_ms = dwell_ms;
  s_ram_sweep.last_tick = HAL_GetTick();
  s_process_type = AD9910_APP_PROCESS_RAM_SWEEP;

  return AD9910_OK;
}

/************************************************************
 * Function :       AD9910_AppStop
 * Comment  :       停止AD9910应用层后台任务，当前硬件输出保持最后状态
 * Parameter:       null
 * Return   :       null
 * Date     :       2026-06-12 V3
************************************************************/
void AD9910_AppStop(void)
{
  s_process_type = AD9910_APP_PROCESS_NONE;
  s_manual_toggle_ms = 0U;
  s_ram_sweep.points = 0U;
  s_ram_sweep.low_hz = 0U;
  s_ram_sweep.high_hz = 0U;
  s_ram_sweep.current_hz = 0U;
  s_ram_sweep.frequency_step_hz = 0U;
  s_ram_sweep.direction = 1;
  s_ram_sweep.dwell_ms = 0U;
  s_ram_sweep.last_tick = 0U;
  s_sine_sweep.low_hz = 0U;
  s_sine_sweep.high_hz = 0U;
  s_sine_sweep.current_hz = 0U;
  s_sine_sweep.frequency_step_hz = 0U;
  s_sine_sweep.amplitude = 0U;
  s_sine_sweep.direction = 1;
  s_sine_sweep.dwell_ms = 0U;
  s_sine_sweep.last_tick = 0U;
}

/************************************************************
 * Function :       AD9910_AppProcess
 * Comment  :       AD9910应用层后台处理函数，放在while(1)中周期调用
 * Parameter:       null
 * Return   :       null
 * Date     :       2026-06-12 V3
************************************************************/
void AD9910_AppProcess(void)
{
  uint32_t now_tick;
  uint32_t next_hz;
  uint16_t playback_step;

  if (s_process_type == AD9910_APP_PROCESS_RAM_SWEEP)
  {
    now_tick = HAL_GetTick();
    if ((now_tick - s_ram_sweep.last_tick) < s_ram_sweep.dwell_ms)
    {
      return;
    }
    s_ram_sweep.last_tick = now_tick;

    next_hz = s_ram_sweep.current_hz;
    if (s_ram_sweep.direction > 0)
    {
      if ((next_hz >= s_ram_sweep.high_hz) ||
          (s_ram_sweep.frequency_step_hz >=
           (s_ram_sweep.high_hz - next_hz)))
      {
        next_hz = s_ram_sweep.high_hz;
        s_ram_sweep.direction = -1;
      }
      else
      {
        next_hz += s_ram_sweep.frequency_step_hz;
      }
    }
    else
    {
      if ((next_hz <= s_ram_sweep.low_hz) ||
          (s_ram_sweep.frequency_step_hz >=
           (next_hz - s_ram_sweep.low_hz)))
      {
        next_hz = s_ram_sweep.low_hz;
        s_ram_sweep.direction = 1;
      }
      else
      {
        next_hz -= s_ram_sweep.frequency_step_hz;
      }
    }

    playback_step = AD9910_CalcRamPlaybackStep(next_hz,
                                               s_ram_sweep.points);
    if ((playback_step != 0U) &&
        (AD9910_SetRamPlaybackStep(playback_step,
                                   s_ram_sweep.points) == AD9910_OK))
    {
      s_ram_sweep.current_hz = next_hz;
    }
    else
    {
      AD9910_AppStop();
    }
    return;
  }

  if (s_process_type == AD9910_APP_PROCESS_SINE_SWEEP)
  {
    now_tick = HAL_GetTick();
    if ((now_tick - s_sine_sweep.last_tick) < s_sine_sweep.dwell_ms)
    {
      return;
    }
    s_sine_sweep.last_tick = now_tick;

    next_hz = s_sine_sweep.current_hz;
    if (s_sine_sweep.direction > 0)
    {
      if ((next_hz >= s_sine_sweep.high_hz) ||
          (s_sine_sweep.frequency_step_hz >=
           (s_sine_sweep.high_hz - next_hz)))
      {
        next_hz = s_sine_sweep.high_hz;
        if (s_sine_sweep.dir_mode == AD9910_SWEEP_BIDIR)
        {
          s_sine_sweep.direction = -1;
        }
        else if (s_sine_sweep.dir_mode == AD9910_SWEEP_UNIDIR_LOOP)
        {
          next_hz = s_sine_sweep.low_hz;
        }
        else
        {
          AD9910_AppStop();
          return;
        }
      }
      else
      {
        next_hz += s_sine_sweep.frequency_step_hz;
      }
    }
    else
    {
      if ((next_hz <= s_sine_sweep.low_hz) ||
          (s_sine_sweep.frequency_step_hz >=
           (next_hz - s_sine_sweep.low_hz)))
      {
        next_hz = s_sine_sweep.low_hz;
        s_sine_sweep.direction = 1;
      }
      else
      {
        next_hz -= s_sine_sweep.frequency_step_hz;
      }
    }

    if (AD9910_SetFrequencyHz(next_hz) == AD9910_OK)
    {
      s_sine_sweep.current_hz = next_hz;
    }
    else
    {
      AD9910_AppStop();
    }
    return;
  }

  if (s_process_type == AD9910_APP_PROCESS_MANUAL_SWEEP)
  {
    if (s_manual_toggle_ms == 0U)
    {
      return;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - s_manual_last_tick) < s_manual_toggle_ms)
    {
      return;
    }

    s_manual_last_tick = now_tick;
    s_manual_direction = (s_manual_direction == AD9910_SWEEP_UP) ?
                         AD9910_SWEEP_DOWN :
                         AD9910_SWEEP_UP;
    AD9910_SetSweepDirection(s_manual_direction);
  }
}

/* 丢弃首尾数据后降采样并转换为14位RAM采样表。 */
uint16_t AD9910_AppDownSample(const float *src,
                              uint32_t src_len,
                              uint16_t discard_front,
                              uint16_t discard_back,
                              uint16_t dst_max,
                              uint16_t *dst)
{
    uint32_t valid_len;
    uint32_t dst_len;
    float    step;
    float    v;

    if (src == NULL || dst == NULL || src_len == 0U || dst_max == 0U) {
        return 0U;
    }

    if ((uint32_t)discard_front + (uint32_t)discard_back >= src_len) {
        return 0U;
    }

    valid_len = src_len - discard_front - discard_back;

    if (valid_len <= dst_max) {
        for (uint32_t i = 0U; i < valid_len; i++) {
            v = src[discard_front + i];
            if (v < 0.0f) v = 0.0f;
            if (v > 3.3f) v = 3.3f;
            dst[i] = (uint16_t)((v / 3.3f) * (float)AD9910_MAX_AMPLITUDE + 0.5f);
        }
        return (uint16_t)valid_len;
    }

    dst_len = dst_max;
    step = (float)(valid_len - 1U) / (float)(dst_len - 1U);

    for (uint32_t j = 0U; j < dst_len; j++) {
        float    pos = (float)j * step;
        uint32_t idx = (uint32_t)pos;
        float    frac = pos - (float)idx;

        if (idx + 1U >= valid_len) {
            v = src[discard_front + valid_len - 1U];
        } else {
            v = src[discard_front + idx] * (1.0f - frac)
              + src[discard_front + idx + 1U] * frac;
        }

        if (v < 0.0f) v = 0.0f;
        if (v > 3.3f) v = 3.3f;
        dst[j] = (uint16_t)((v / 3.3f) * (float)AD9910_MAX_AMPLITUDE + 0.5f);
    }

    return (uint16_t)dst_len;
}

/* 从输入波形中提取一个周期并转换为14位RAM采样表。 */
uint16_t AD9910_AppExtractOneCycle(const float *src,
                                   uint32_t      len,
                                   uint16_t     *dst)
{
    float    dc;
    float    amp;
    float    sum;
    uint32_t i;
    int32_t  cross_idx1;
    int32_t  cross_idx2;
    int32_t  cycle_len;
    uint32_t edge;
    uint32_t mid_start;
    uint32_t mid_end;
    float    v_min;
    float    v_max;

    if ((src == NULL) || (dst == NULL) || (len < 4U)) {
        return 0U;
    }

    /* ---- 1. 算 DC 和幅度 ---- */
    sum   = 0.0f;
    v_min = 1e9f;
    v_max = -1e9f;
    for (i = 0U; i < len; i++) {
        sum += src[i];
        if (src[i] < v_min) v_min = src[i];
        if (src[i] > v_max) v_max = src[i];
    }
    dc  = sum / (float)len;
    amp = (v_max - v_min) * 0.05f;
    if (amp < 1e-6f) {
        return 0U;
    }

    /* ---- 2. 状态机找上升过零点（带滞回，不要求相邻点跃过）---- */
    mid_start = len / 4U;
    mid_end   = len - len / 4U;

    {
        uint8_t was_below = (src[mid_start] <= dc - amp) ? 1U : 0U;

        cross_idx1 = -1;
        for (i = mid_start + 1U; i < mid_end; i++) {
            if (was_below && (src[i] >= dc + amp)) {
                cross_idx1 = (int32_t)i;
                break;
            }
            if (!was_below && (src[i] <= dc - amp)) {
                was_below = 1U;
            }
        }
    }
    if (cross_idx1 < 0) {
        return 0U;
    }

    /* ---- 3. 在过零点 1 之后，找下一个上升过零点 ---- */
    {
        uint8_t was_below = 1U;
        i = (uint32_t)cross_idx1 + 1U;

        for (; i < len; i++) {
            if (src[i] <= dc - amp) {
                was_below = 1U;
                i++;
                break;
            }
        }

        cross_idx2 = -1;
        for (; i < len; i++) {
            if (was_below && (src[i] >= dc + amp)) {
                if ((int32_t)i - cross_idx1 > (int32_t)len / 20) {
                    cross_idx2 = (int32_t)i;
                    break;
                }
            }
            if (!was_below && (src[i] <= dc - amp)) {
                was_below = 1U;
            }
        }
    }
    if (cross_idx2 < 0) {
        return 0U;
    }

    /* ---- 4. 截取一个整周期 ---- */
    cycle_len = cross_idx2 - cross_idx1;
    if (cycle_len < 32) {
        return 0U;
    }
    edge = (uint32_t)((float)cycle_len * 0.025f);
    if (edge == 0U) edge = 1U;

    /* ---- 5. 降采样到 1024 ---- */
    return AD9910_AppDownSample(src, (uint32_t)cross_idx2 - edge,
                                (uint32_t)cross_idx1 + edge, 0U,
                                AD9910_RAM_POINTS, dst);
}
