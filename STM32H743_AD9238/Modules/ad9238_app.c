#include "ad9238_app.h"
#include "spi.h"

#if defined(NUEDC_TARGET_MSPM0G3507)
#define AD9238_APP_SPI_HANDLE hspi1 /* TI板使用独占SPI0控制FPGA。 */
#else
#define AD9238_APP_SPI_HANDLE hspi2 /* STM32板使用SPI2控制FPGA。 */
#endif
#include <float.h>

static AD9238_SampleTypeDef
    s_ad9238_samples[AD9238_BUFFER_MAX_SAMPLES]; /* FPGA signed Q8.8 samples. */
static AD9238_AppDataTypeDef s_ad9238_app_data; /* 应用层最近一次运行数据。 */
static AD9238_VoltageCalibrationTypeDef
    s_ad9238_calibration; /* 模块输入电压两点校准数据。 */

/* 执行立即或阈值触发的公共阻塞采集流程。 */
static AD9238_StatusTypeDef AD9238_AppCaptureInternal(
    const AD9238_CaptureConfigTypeDef *config,
    uint32_t timeout_ms);

/* 将校准范围内的输入电压换算为12位触发码。 */
static AD9238_StatusTypeDef AD9238_AppVoltageToCode(float voltage_v,
                                                    uint16_t *code);

/* 检查浮点数是否为有限值。 */
static uint8_t AD9238_AppFloatValid(float value);

/************************************************************
 * Function :       AD9238_AppInit
 * Comment  :       初始化高速ADC并装入模块电路的理想电压换算
 * Parameter:       null
 * Return   :       AD9238状态
 ************************************************************/
AD9238_StatusTypeDef AD9238_AppInit(void)
{
  AD9238_StatusTypeDef status;
  float code4095_voltage_v;

  s_ad9238_app_data = (AD9238_AppDataTypeDef){0};
  s_ad9238_calibration = (AD9238_VoltageCalibrationTypeDef){0};
  code4095_voltage_v = AD9238_MODULE_INPUT_MIN_V +
                      4095.0f * AD9238_IDEAL_VOLTS_PER_CODE;
  status = AD9238_AppSetVoltageCalibration(0.0f,
                                           AD9238_MODULE_INPUT_MIN_V,
                                           4095.0f,
                                           code4095_voltage_v);
  if (status == AD9238_OK)
  {
    status = AD9238_Init(&AD9238_APP_SPI_HANDLE);
  }

  s_ad9238_app_data.status = status;
  s_ad9238_app_data.initialized = (status == AD9238_OK) ? 1U : 0U;
  return status;
}

/************************************************************
 * Function :       AD9238_AppCapture
 * Comment  :       立即触发并阻塞读取一块高速采样数据
 * Parameter:       sample_count: 点数; decimation: 抽取倍数;
 *                  timeout_ms: 超时时间
 * Return   :       AD9238状态
 ************************************************************/
AD9238_StatusTypeDef AD9238_AppCapture(uint16_t sample_count,
                                       uint16_t decimation,
                                       uint32_t timeout_ms)
{
  AD9238_CaptureConfigTypeDef config;

  config.sample_count = sample_count;
  config.decimation = decimation;
  config.trigger_mode = AD9238_TRIGGER_IMMEDIATE;
  config.trigger_threshold = 2048U;
  return AD9238_AppCaptureInternal(&config, timeout_ms);
}

/************************************************************
 * Function :       AD9238_AppCaptureTriggered
 * Comment  :       按校准后的输入电压阈值触发并阻塞读取采样块
 * Parameter:       点数、抽取倍数、边沿、触发电压和超时时间
 * Return   :       AD9238状态
 ************************************************************/
AD9238_StatusTypeDef AD9238_AppCaptureTriggered(
    uint16_t sample_count,
    uint16_t decimation,
    AD9238_TriggerTypeDef trigger_mode,
    float trigger_voltage_v,
    uint32_t timeout_ms)
{
  AD9238_CaptureConfigTypeDef config;
  AD9238_StatusTypeDef status;

  if (trigger_mode == AD9238_TRIGGER_IMMEDIATE)
  {
    return AD9238_AppCapture(sample_count, decimation, timeout_ms);
  }

  config.sample_count = sample_count;
  config.decimation = decimation;
  config.trigger_mode = trigger_mode;
  status = AD9238_AppVoltageToCode(trigger_voltage_v,
                                   &config.trigger_threshold);
  if (status != AD9238_OK)
  {
    s_ad9238_app_data.status = status;
    return status;
  }
  return AD9238_AppCaptureInternal(&config, timeout_ms);
}

/************************************************************
 * Function :       AD9238_AppSetVoltageCalibration
 * Comment  :       保存两个输入电压及其实测平均码的线性校准
 * Parameter:       低码、低电压、高码和高电压
 * Return   :       AD9238状态
 ************************************************************/
AD9238_StatusTypeDef AD9238_AppSetVoltageCalibration(
    float low_code,
    float low_voltage_v,
    float high_code,
    float high_voltage_v)
{
  float code_span;
  float voltage_span;

  if ((AD9238_AppFloatValid(low_code) == 0U) ||
      (AD9238_AppFloatValid(high_code) == 0U) ||
      (low_code < 0.0f) || (high_code > 4095.0f) ||
      !(high_code > low_code) ||
      (AD9238_AppFloatValid(low_voltage_v) == 0U) ||
      (AD9238_AppFloatValid(high_voltage_v) == 0U) ||
      !(high_voltage_v > low_voltage_v))
  {
    return AD9238_ERROR_PARAM;
  }

  code_span = high_code - low_code;
  voltage_span = high_voltage_v - low_voltage_v;
  s_ad9238_calibration.low_code = low_code;
  s_ad9238_calibration.high_code = high_code;
  s_ad9238_calibration.low_voltage_v = low_voltage_v;
  s_ad9238_calibration.high_voltage_v = high_voltage_v;
  s_ad9238_calibration.volts_per_code = voltage_span / code_span;
  s_ad9238_calibration.intercept_v = low_voltage_v -
      (float)low_code * s_ad9238_calibration.volts_per_code;
  s_ad9238_calibration.valid = 1U;
  return AD9238_OK;
}

/************************************************************
 * Function :       AD9238_AppCodeToVoltage
 * Comment  :       将原始采样码换算为校准后的模块输入电压
 * Parameter:       code: 12位原始采样码
 * Return   :       输入电压，单位V
 ************************************************************/
float AD9238_AppCodeToVoltage(uint16_t code)
{
  if (s_ad9238_calibration.valid == 0U)
  {
    return 0.0f;
  }
  return (float)code * s_ad9238_calibration.volts_per_code +
         s_ad9238_calibration.intercept_v;
}

/************************************************************
 * Function :       AD9238_AppGetSamples
 * Comment  :       返回应用层原始采样缓存的只读指针
 * Parameter:       null
 * Return   :       原始采样缓存只读指针
 ************************************************************/
const AD9238_SampleTypeDef *AD9238_AppGetSamples(void)
{
  return s_ad9238_samples;
}

/************************************************************
 * Function :       AD9238_AppGetSampleCount
 * Comment  :       返回应用层缓存中的有效点数
 * Parameter:       null
 * Return   :       有效采样点数
 ************************************************************/
uint16_t AD9238_AppGetSampleCount(void)
{
  return s_ad9238_app_data.sample_count;
}

/************************************************************
 * Function :       AD9238_AppGetData
 * Comment  :       返回应用层最近一次采集结果的只读指针
 * Parameter:       null
 * Return   :       应用层数据只读指针
 ************************************************************/
const AD9238_AppDataTypeDef *AD9238_AppGetData(void)
{
  return &s_ad9238_app_data;
}

/************************************************************
 * Function :       AD9238_AppGetVoltageCalibration
 * Comment  :       返回当前输入电压校准数据的只读指针
 * Parameter:       null
 * Return   :       电压校准数据只读指针
 ************************************************************/
const AD9238_VoltageCalibrationTypeDef *
AD9238_AppGetVoltageCalibration(void)
{
  return &s_ad9238_calibration;
}

/************************************************************
 * Function :       AD9238_AppCaptureInternal
 * Comment  :       执行启动、等待、读取和状态更新的公共流程
 * Parameter:       config: 采集配置; timeout_ms: 超时时间
 * Return   :       AD9238状态
 ************************************************************/
static AD9238_StatusTypeDef AD9238_AppCaptureInternal(
    const AD9238_CaptureConfigTypeDef *config,
    uint32_t timeout_ms)
{
  AD9238_StatusTypeDef status;
  AD9238_StatusTypeDef info_status;
  uint16_t read_count;

  if (s_ad9238_app_data.initialized == 0U)
  {
    return AD9238_ERROR_NOT_INIT;
  }
  if (timeout_ms == 0U)
  {
    return AD9238_ERROR_PARAM;
  }

  s_ad9238_app_data.sample_count = 0U;
  s_ad9238_app_data.capture_stage = AD9238_APP_CAPTURE_START;
  status = AD9238_StartCapture(config);
  if (status == AD9238_OK)
  {
    s_ad9238_app_data.capture_stage = AD9238_APP_CAPTURE_WAIT;
    status = AD9238_WaitCapture(timeout_ms);
  }
  if (status == AD9238_OK)
  {
    s_ad9238_app_data.capture_stage = AD9238_APP_CAPTURE_READ;
    status = AD9238_ReadCapture(s_ad9238_samples,
                                AD9238_BUFFER_MAX_SAMPLES,
                                &read_count);
  }
  else if (status == AD9238_ERROR_TIMEOUT)
  {
    (void)AD9238_AbortCapture();
  }

  if (status == AD9238_OK)
  {
    s_ad9238_app_data.sample_count = read_count;
  }
  info_status = AD9238_GetCaptureInfo(&s_ad9238_app_data.capture);
  if (status == AD9238_OK)
  {
    if (info_status == AD9238_OK)
    {
      s_ad9238_app_data.capture_stage = AD9238_APP_CAPTURE_COMPLETE;
    }
    else
    {
      s_ad9238_app_data.capture_stage = AD9238_APP_CAPTURE_STATUS;
      status = info_status;
      s_ad9238_app_data.sample_count = 0U;
    }
  }
  s_ad9238_app_data.status = status;
  return status;
}

/************************************************************
 * Function :       AD9238_AppVoltageToCode
 * Comment  :       将校准范围内的输入电压换算为最近的12位码
 * Parameter:       voltage_v: 输入电压; code: 输出码指针
 * Return   :       AD9238状态
 ************************************************************/
static AD9238_StatusTypeDef AD9238_AppVoltageToCode(float voltage_v,
                                                    uint16_t *code)
{
  float code_f;

  if ((code == NULL) || (s_ad9238_calibration.valid == 0U) ||
      (AD9238_AppFloatValid(voltage_v) == 0U) ||
      (voltage_v < s_ad9238_calibration.low_voltage_v) ||
      (voltage_v > s_ad9238_calibration.high_voltage_v))
  {
    return AD9238_ERROR_PARAM;
  }

  code_f = (voltage_v - s_ad9238_calibration.intercept_v) /
           s_ad9238_calibration.volts_per_code;
  if (code_f < 0.0f)
  {
    code_f = 0.0f;
  }
  else if (code_f > 4095.0f)
  {
    code_f = 4095.0f;
  }
  *code = (uint16_t)(code_f + 0.5f);
  return AD9238_OK;
}

/************************************************************
 * Function :       AD9238_AppFloatValid
 * Comment  :       排除NaN和无穷大的浮点参数
 * Parameter:       value: 待检查浮点数
 * Return   :       1有效，0无效
 ************************************************************/
static uint8_t AD9238_AppFloatValid(float value)
{
  return ((value >= -FLT_MAX) && (value <= FLT_MAX)) ? 1U : 0U;
}
