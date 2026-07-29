#include "ad9708_app.h"
#include "spi.h"

#if defined(NUEDC_TARGET_MSPM0G3507)
#define AD9708_APP_SPI_HANDLE hspi1 /* TI板使用独占SPI0控制FPGA。 */
#else
#define AD9708_APP_SPI_HANDLE hspi2 /* STM32板使用SPI2控制FPGA。 */
#endif
#include <float.h>

/* 32点满量程正弦波表，供不依赖电压校准的链路测试使用。 */
static const uint8_t s_ad9708_app_sine_code[32] =
{
  128U, 152U, 176U, 198U, 218U, 234U, 246U, 253U,
  255U, 253U, 246U, 234U, 218U, 198U, 176U, 152U,
  128U, 103U,  79U,  57U,  37U,  21U,   9U,   2U,
    0U,   2U,   9U,  21U,  37U,  57U,  79U, 103U
};

/************************************************************
 * Function :       AD9708_AppInit
 * Comment  :       初始化高速DAC应用层并核验FPGA固件
 * Parameter:       null
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppInit(void)
{
  return AD9708_Init(&AD9708_APP_SPI_HANDLE);
}

/************************************************************
 * Function :       AD9708_AppSetVoltageCalibration
 * Comment  :       保存DAC码0和255对应的两点实测电压
 * Parameter:       code0_voltage_v: 码0电压; code255_voltage_v: 码255电压
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppSetVoltageCalibration(
    float code0_voltage_v,
    float code255_voltage_v)
{
  float span_v;
  AD9708_VoltageCalibrationTypeDef *calibration;

  if (!(code0_voltage_v >= -FLT_MAX) ||
      !(code0_voltage_v <= FLT_MAX) ||
      !(code255_voltage_v >= -FLT_MAX) ||
      !(code255_voltage_v <= FLT_MAX) ||
      !(code255_voltage_v > code0_voltage_v))
  {
    return AD9708_ERROR_PARAM;
  }

  span_v = code255_voltage_v - code0_voltage_v;
  if (!(span_v > 0.0f) || !(span_v <= FLT_MAX))
  {
    return AD9708_ERROR_PARAM;
  }

  calibration = AD9708_InternalVoltageCalibration();
  calibration->code0_voltage_v = code0_voltage_v;
  calibration->code255_voltage_v = code255_voltage_v;
  calibration->volts_per_code =
      span_v / (float)AD9708_MAX_CODE;
  calibration->valid = 1U;
  return AD9708_OK;
}

/************************************************************
 * Function :       AD9708_AppGetVoltageCalibration
 * Comment  :       返回当前两点电压校准数据的只读指针
 * Parameter:       null
 * Return   :       电压校准数据只读指针
************************************************************/
const AD9708_VoltageCalibrationTypeDef *
AD9708_AppGetVoltageCalibration(void)
{
  return AD9708_InternalVoltageCalibration();
}

/************************************************************
 * Function :       AD9708_AppPollStatus
 * Comment  :       读取FPGA和高速DAC当前状态
 * Parameter:       null
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppPollStatus(void)
{
  return AD9708_PollStatus();
}

/************************************************************
 * Function :       AD9708_AppSetPhase
 * Comment  :       设置0~360度相位偏移并立即重置DDS相位
 * Parameter:       phase_deg: 相位角，范围[0, 360)
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppSetPhase(float phase_deg)
{
  AD9708_StatusTypeDef status;
  uint32_t phase_word;

  if (!(phase_deg >= 0.0f) || !(phase_deg < 360.0f))
  {
    return AD9708_ERROR_PARAM;
  }

  phase_word = (uint32_t)(((double)phase_deg * 4294967296.0) / 360.0);
  status = AD9708_SetPhaseOffsetWord(phase_word);
  if (status == AD9708_OK)
  {
    status = AD9708_ResetPhase();
  }
  return status;
}

/************************************************************
 * Function :       AD9708_AppOutputConstant
 * Comment  :       输出校准范围内的恒定电压
 * Parameter:       voltage_v: 目标电压，单位V
 * Return   :       AD9708状态
 ************************************************************/
AD9708_StatusTypeDef AD9708_AppOutputConstant(float voltage_v)
{
  AD9708_StatusTypeDef status;
  uint8_t code;

  if (AD9708_GetData()->initialized == 0U)
  {
    return AD9708_ERROR_NOT_INIT;
  }
  status = AD9708_InternalVoltageToCode(voltage_v, &code);
  if (status == AD9708_OK)
  {
    status = AD9708_OutputConstant(code);
  }
  return status;
}

/************************************************************
 * Function :       AD9708_AppOutputCalibrationCode
 * Comment  :       输出恒定原始码供两点电压校准
 * Parameter:       code: 0~255的DAC原始码
 * Return   :       AD9708状态
 ************************************************************/
AD9708_StatusTypeDef AD9708_AppOutputCalibrationCode(uint8_t code)
{
  return AD9708_OutputConstant(code);
}

/************************************************************
 * Function :       AD9708_AppOutputSineCode
 * Comment  :       按原始峰值码和中心码输出正弦波，不改变校准参数
 * Parameter:       output_hz: 频率; amplitude_code: 峰值码;
 *                  offset_code: 中心码
 * Return   :       AD9708状态
 ************************************************************/
AD9708_StatusTypeDef AD9708_AppOutputSineCode(float output_hz,
                                              uint8_t amplitude_code,
                                              uint8_t offset_code)
{
  return AD9708_InternalOutputArbitraryCode(
      s_ad9708_app_sine_code,
      (uint16_t)(sizeof(s_ad9708_app_sine_code) /
                 sizeof(s_ad9708_app_sine_code[0])),
      output_hz,
      (uint16_t)amplitude_code << 8,
      (uint16_t)offset_code << 8,
      0U);
}

/************************************************************
 * Function :       AD9708_AppOutputWave
 * Comment  :       按电压参数输出用户提供的2~1024点任意波
 * Parameter:       wave: 采样表; points: 点数; output_hz: 频率;
 *                  amplitude_vpp: 峰峰值; offset_v: 中心电压，单位V
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppOutputWave(const uint8_t *wave,
                                          uint16_t points,
                                          float output_hz,
                                          float amplitude_vpp,
                                          float offset_v)
{
  return AD9708_AppOutputArbitraryPhaseIndex(wave,
                                              points,
                                              output_hz,
                                              amplitude_vpp,
                                              offset_v,
                                              0U);
}

/************************************************************
 * Function :       AD9708_AppOutputSine
 * Comment  :       按电压参数输出默认1024点正弦波
 * Parameter:       output_hz: 频率; amplitude_vpp: 峰峰值;
 *                  offset_v: 中心电压，单位V
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppOutputSine(float output_hz,
                                          float amplitude_vpp,
                                          float offset_v)
{
  return AD9708_InternalOutputGeneratedVoltage(
      AD9708_WAVE_SINE,
      output_hz,
      AD9708_APP_DEFAULT_RAM_POINTS,
      amplitude_vpp,
      offset_v,
      0U);
}

/************************************************************
 * Function :       AD9708_AppOutputTriangle
 * Comment  :       按电压参数输出FPGA内置三角波
 * Parameter:       output_hz: 频率; amplitude_vpp: 峰峰值;
 *                  offset_v: 中心电压，单位V
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppOutputTriangle(float output_hz,
                                              float amplitude_vpp,
                                              float offset_v)
{
  return AD9708_InternalOutputGeneratedVoltage(
      AD9708_WAVE_TRIANGLE,
      output_hz,
      AD9708_APP_DEFAULT_RAM_POINTS,
      amplitude_vpp,
      offset_v,
      0U);
}

/************************************************************
 * Function :       AD9708_AppOutputSquare
 * Comment  :       按电压参数输出FPGA内置50%占空比方波
 * Parameter:       output_hz: 频率; amplitude_vpp: 峰峰值;
 *                  offset_v: 中心电压，单位V
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppOutputSquare(float output_hz,
                                            float amplitude_vpp,
                                            float offset_v)
{
  return AD9708_InternalOutputGeneratedVoltage(
      AD9708_WAVE_SQUARE,
      output_hz,
      AD9708_APP_DEFAULT_RAM_POINTS,
      amplitude_vpp,
      offset_v,
      0U);
}

/************************************************************
 * Function :       AD9708_AppOutputSawtooth
 * Comment  :       按电压参数输出FPGA内置上升锯齿波
 * Parameter:       output_hz: 频率; amplitude_vpp: 峰峰值;
 *                  offset_v: 中心电压，单位V
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppOutputSawtooth(float output_hz,
                                              float amplitude_vpp,
                                              float offset_v)
{
  return AD9708_InternalOutputGeneratedVoltage(
      AD9708_WAVE_SAWTOOTH,
      output_hz,
      AD9708_APP_DEFAULT_RAM_POINTS,
      amplitude_vpp,
      offset_v,
      0U);
}

/************************************************************
 * Function :       AD9708_AppOutputSinc
 * Comment  :       按电压参数输出默认1024点SINC波
 * Parameter:       output_hz: 频率; amplitude_vpp: 峰峰值;
 *                  offset_v: 中心电压，单位V
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppOutputSinc(float output_hz,
                                          float amplitude_vpp,
                                          float offset_v)
{
  return AD9708_InternalOutputGeneratedVoltage(
      AD9708_WAVE_SINC,
      output_hz,
      AD9708_APP_DEFAULT_RAM_POINTS,
      amplitude_vpp,
      offset_v,
      0U);
}

/************************************************************
 * Function :       AD9708_AppOutputWavePhaseIndex
 * Comment  :       按采样点索引设置内置波形的起始相位
 * Parameter:       wave: 波形; output_hz: 频率; points: 相位点数;
 *                  amplitude_vpp: 峰峰值; offset_v: 中心电压，单位V;
 *                  phase_index: 相位索引
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppOutputWavePhaseIndex(
    AD9708_WaveformTypeDef wave,
    float output_hz,
    uint16_t points,
    float amplitude_vpp,
    float offset_v,
    uint16_t phase_index)
{
  AD9708_StatusTypeDef status;
  uint32_t phase_word;

  if (AD9708_GetData()->initialized == 0U)
  {
    return AD9708_ERROR_NOT_INIT;
  }
  status = AD9708_InternalPhaseIndexToWord(points, phase_index, &phase_word);
  if (status != AD9708_OK)
  {
    return status;
  }
  return AD9708_InternalOutputGeneratedVoltage(wave,
                                           output_hz,
                                           points,
                                           amplitude_vpp,
                                           offset_v,
                                           phase_word);
}

/************************************************************
 * Function :       AD9708_AppOutputArbitraryPhaseIndex
 * Comment  :       按采样点索引设置用户任意波的起始相位
 * Parameter:       wave: 波表; points: 点数; output_hz: 频率;
 *                  amplitude_vpp: 峰峰值; offset_v: 中心电压，单位V;
 *                  phase_index: 相位索引
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppOutputArbitraryPhaseIndex(
    const uint8_t *wave,
    uint16_t points,
    float output_hz,
    float amplitude_vpp,
    float offset_v,
    uint16_t phase_index)
{
  AD9708_StatusTypeDef status;
  uint32_t phase_word;
  uint16_t amplitude_q8;
  uint16_t offset_q8;

  if (AD9708_GetData()->initialized == 0U)
  {
    return AD9708_ERROR_NOT_INIT;
  }
  status = AD9708_InternalPhaseIndexToWord(points, phase_index, &phase_word);
  if (status != AD9708_OK)
  {
    return status;
  }
  status = AD9708_InternalVoltageLevelToCode(amplitude_vpp,
                                         offset_v,
                                         &amplitude_q8,
                                         &offset_q8);
  if (status != AD9708_OK)
  {
    return status;
  }
  return AD9708_InternalOutputArbitraryCode(wave,
                                        points,
                                        output_hz,
                                        amplitude_q8,
                                        offset_q8,
                                        phase_word);
}

/************************************************************
 * Function :       AD9708_AppHoldSweep
 * Comment  :       暂停或继续FPGA硬件扫频
 * Parameter:       hold: 0继续，非0暂停
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppHoldSweep(uint8_t hold)
{
  return AD9708_HoldSweep(hold);
}

/************************************************************
 * Function :       AD9708_AppSetSweepDirection
 * Comment  :       手动扫频模式下切换方向
 * Parameter:       direction: 向上或向下
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppSetSweepDirection(
    AD9708_SweepDirectionTypeDef direction)
{
  return AD9708_SetSweepDirection(direction);
}

/************************************************************
 * Function :       AD9708_AppStopSweep
 * Comment  :       停止扫频并保持停止瞬间频率
 * Parameter:       null
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppStopSweep(void)
{
  return AD9708_StopSweep();
}

/************************************************************
 * Function :       AD9708_AppIsSweepDone
 * Comment  :       查询单向或手动扫频是否到达端点
 * Parameter:       done: 端点状态输出指针
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppIsSweepDone(uint8_t *done)
{
  return AD9708_IsSweepDone(done);
}

/************************************************************
 * Function :       AD9708_AppSweepSine
 * Comment  :       启动正弦波FPGA硬件扫频
 * Parameter:       端点、步进、Vpp、中心电压、驻留时间和扫频模式
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppSweepSine(
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode)
{
  AD9708_StatusTypeDef status;

  status = AD9708_AppOutputSine(low_hz, amplitude_vpp, offset_v);
  if (status == AD9708_OK)
  {
    status = AD9708_InternalStartConfiguredSweep(low_hz,
                                             high_hz,
                                             step_hz,
                                             dwell_us,
                                             mode);
  }
  return status;
}

/************************************************************
 * Function :       AD9708_AppSweepTriangle
 * Comment  :       启动三角波FPGA硬件扫频
 * Parameter:       端点、步进、Vpp、中心电压、驻留时间和扫频模式
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppSweepTriangle(
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode)
{
  AD9708_StatusTypeDef status;

  status = AD9708_AppOutputTriangle(low_hz, amplitude_vpp, offset_v);
  if (status == AD9708_OK)
  {
    status = AD9708_InternalStartConfiguredSweep(low_hz,
                                             high_hz,
                                             step_hz,
                                             dwell_us,
                                             mode);
  }
  return status;
}

/************************************************************
 * Function :       AD9708_AppSweepSquare
 * Comment  :       启动方波FPGA硬件扫频
 * Parameter:       端点、步进、Vpp、中心电压、驻留时间和扫频模式
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppSweepSquare(
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode)
{
  AD9708_StatusTypeDef status;

  status = AD9708_AppOutputSquare(low_hz, amplitude_vpp, offset_v);
  if (status == AD9708_OK)
  {
    status = AD9708_InternalStartConfiguredSweep(low_hz,
                                             high_hz,
                                             step_hz,
                                             dwell_us,
                                             mode);
  }
  return status;
}

/************************************************************
 * Function :       AD9708_AppSweepSawtooth
 * Comment  :       启动锯齿波FPGA硬件扫频
 * Parameter:       端点、步进、Vpp、中心电压、驻留时间和扫频模式
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppSweepSawtooth(
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode)
{
  AD9708_StatusTypeDef status;

  status = AD9708_AppOutputSawtooth(low_hz, amplitude_vpp, offset_v);
  if (status == AD9708_OK)
  {
    status = AD9708_InternalStartConfiguredSweep(low_hz,
                                             high_hz,
                                             step_hz,
                                             dwell_us,
                                             mode);
  }
  return status;
}

/************************************************************
 * Function :       AD9708_AppSweepSinc
 * Comment  :       启动SINC波FPGA硬件扫频
 * Parameter:       端点、步进、Vpp、中心电压、驻留时间和扫频模式
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppSweepSinc(
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode)
{
  AD9708_StatusTypeDef status;

  status = AD9708_AppOutputSinc(low_hz, amplitude_vpp, offset_v);
  if (status == AD9708_OK)
  {
    status = AD9708_InternalStartConfiguredSweep(low_hz,
                                             high_hz,
                                             step_hz,
                                             dwell_us,
                                             mode);
  }
  return status;
}

/************************************************************
 * Function :       AD9708_AppSweepArbitrary
 * Comment  :       启动用户任意波FPGA硬件扫频
 * Parameter:       波表、点数、端点、步进、Vpp、中心电压、驻留时间和模式
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppSweepArbitrary(
    const uint8_t *wave,
    uint16_t points,
    float low_hz,
    float high_hz,
    float step_hz,
    float amplitude_vpp,
    float offset_v,
    uint32_t dwell_us,
    AD9708_SweepModeTypeDef mode)
{
  AD9708_StatusTypeDef status;

  status = AD9708_AppOutputWave(wave,
                                 points,
                                 low_hz,
                                 amplitude_vpp,
                                 offset_v);
  if (status == AD9708_OK)
  {
    status = AD9708_InternalStartConfiguredSweep(low_hz,
                                             high_hz,
                                             step_hz,
                                             dwell_us,
                                             mode);
  }
  return status;
}

/************************************************************
 * Function :       AD9708_AppStop
 * Comment  :       停止扫频并关闭DAC波形输出
 * Parameter:       null
 * Return   :       AD9708状态
************************************************************/
AD9708_StatusTypeDef AD9708_AppStop(void)
{
  AD9708_StatusTypeDef status;

  status = AD9708_StopSweep();
  if (status == AD9708_OK)
  {
    status = AD9708_SetEnable(0U);
  }
  return status;
}

/************************************************************
 * Function :       AD9708_AppGetData
 * Comment  :       返回高速DAC底层状态的只读指针
 * Parameter:       null
 * Return   :       状态结构体只读指针
************************************************************/
const AD9708_DataTypeDef *AD9708_AppGetData(void)
{
  return AD9708_GetData();
}
