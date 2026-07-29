#include "ad9280_app.h"
#include "spi.h"

#if defined(NUEDC_TARGET_MSPM0G3507)
#define AD9280_APP_SPI_HANDLE hspi1 /* TI板使用独占SPI0控制FPGA。 */
#else
#define AD9280_APP_SPI_HANDLE hspi2 /* STM32板使用SPI2控制FPGA。 */
#endif
#include <float.h>

static AD9280_SampleTypeDef
    s_ad9280_samples[AD9280_BUFFER_MAX_SAMPLES]; /* FPGA signed Q8.8 samples. */
static AD9280_AppDataTypeDef s_ad9280_app_data; /* 应用层最近一次运行数据。 */
static AD9280_VoltageCalibrationTypeDef
    s_ad9280_calibration; /* 模块输入电压两点校准数据。 */

/* 执行立即或阈值触发的公共阻塞采集流程。 */
static AD9280_StatusTypeDef AD9280_AppCaptureInternal(
    const AD9280_CaptureConfigTypeDef *config,
    uint32_t timeout_ms);

/* 将校准范围内的输入电压换算为8位触发码。 */
static AD9280_StatusTypeDef AD9280_AppVoltageToCode(float voltage_v,
                                                    uint8_t *code);

/* 检查浮点数是否为有限值。 */
static uint8_t AD9280_AppFloatValid(float value);

/************************************************************
 * Function :       AD9280_AppInit
 * Comment  :       初始化高速ADC并装入模块电路的理想电压换算
 * Parameter:       null
 * Return   :       AD9280状态
 ************************************************************/
AD9280_StatusTypeDef AD9280_AppInit(void)
{
  AD9280_StatusTypeDef status;
  float code255_voltage_v;

  s_ad9280_app_data = (AD9280_AppDataTypeDef){0};
  s_ad9280_calibration = (AD9280_VoltageCalibrationTypeDef){0};
  code255_voltage_v = AD9280_MODULE_INPUT_MIN_V +
                      255.0f * AD9280_IDEAL_VOLTS_PER_CODE;
  status = AD9280_AppSetVoltageCalibration(0.0f,
                                           AD9280_MODULE_INPUT_MIN_V,
                                           255.0f,
                                           code255_voltage_v);
  if (status == AD9280_OK)
  {
    status = AD9280_Init(&AD9280_APP_SPI_HANDLE);
  }

  s_ad9280_app_data.status = status;
  s_ad9280_app_data.initialized = (status == AD9280_OK) ? 1U : 0U;
  return status;
}

/************************************************************
 * Function :       AD9280_AppCapture
 * Comment  :       立即触发并阻塞读取一块高速采样数据
 * Parameter:       sample_count: 点数; decimation: 抽取倍数;
 *                  timeout_ms: 超时时间
 * Return   :       AD9280状态
 ************************************************************/
AD9280_StatusTypeDef AD9280_AppCapture(uint16_t sample_count,
                                       uint16_t decimation,
                                       uint32_t timeout_ms)
{
  AD9280_CaptureConfigTypeDef config;

  config.sample_count = sample_count;
  config.decimation = decimation;
  config.trigger_mode = AD9280_TRIGGER_IMMEDIATE;
  config.trigger_threshold = 128U;
  return AD9280_AppCaptureInternal(&config, timeout_ms);
}

/************************************************************
 * Function :       AD9280_AppCaptureTriggered
 * Comment  :       按校准后的输入电压阈值触发并阻塞读取采样块
 * Parameter:       点数、抽取倍数、边沿、触发电压和超时时间
 * Return   :       AD9280状态
 ************************************************************/
AD9280_StatusTypeDef AD9280_AppCaptureTriggered(
    uint16_t sample_count,
    uint16_t decimation,
    AD9280_TriggerTypeDef trigger_mode,
    float trigger_voltage_v,
    uint32_t timeout_ms)
{
  AD9280_CaptureConfigTypeDef config;
  AD9280_StatusTypeDef status;

  if (trigger_mode == AD9280_TRIGGER_IMMEDIATE)
  {
    return AD9280_AppCapture(sample_count, decimation, timeout_ms);
  }

  config.sample_count = sample_count;
  config.decimation = decimation;
  config.trigger_mode = trigger_mode;
  status = AD9280_AppVoltageToCode(trigger_voltage_v,
                                   &config.trigger_threshold);
  if (status != AD9280_OK)
  {
    s_ad9280_app_data.status = status;
    return status;
  }
  return AD9280_AppCaptureInternal(&config, timeout_ms);
}

/************************************************************
 * Function :       AD9280_AppSetVoltageCalibration
 * Comment  :       保存两个输入电压及其实测平均码的线性校准
 * Parameter:       低码、低电压、高码和高电压
 * Return   :       AD9280状态
 ************************************************************/
AD9280_StatusTypeDef AD9280_AppSetVoltageCalibration(
    float low_code,
    float low_voltage_v,
    float high_code,
    float high_voltage_v)
{
  float code_span;
  float voltage_span;

  if ((AD9280_AppFloatValid(low_code) == 0U) ||
      (AD9280_AppFloatValid(high_code) == 0U) ||
      (low_code < 0.0f) || (high_code > 255.0f) ||
      !(high_code > low_code) ||
      (AD9280_AppFloatValid(low_voltage_v) == 0U) ||
      (AD9280_AppFloatValid(high_voltage_v) == 0U) ||
      !(high_voltage_v > low_voltage_v))
  {
    return AD9280_ERROR_PARAM;
  }

  code_span = high_code - low_code;
  voltage_span = high_voltage_v - low_voltage_v;
  s_ad9280_calibration.low_code = low_code;
  s_ad9280_calibration.high_code = high_code;
  s_ad9280_calibration.low_voltage_v = low_voltage_v;
  s_ad9280_calibration.high_voltage_v = high_voltage_v;
  s_ad9280_calibration.volts_per_code = voltage_span / code_span;
  s_ad9280_calibration.intercept_v = low_voltage_v -
      (float)low_code * s_ad9280_calibration.volts_per_code;
  s_ad9280_calibration.valid = 1U;
  return AD9280_OK;
}

/************************************************************
 * Function :       AD9280_AppCodeToVoltage
 * Comment  :       将原始采样码换算为校准后的模块输入电压
 * Parameter:       code: 8位原始采样码
 * Return   :       输入电压，单位V
 ************************************************************/
float AD9280_AppCodeToVoltage(uint8_t code)
{
  if (s_ad9280_calibration.valid == 0U)
  {
    return 0.0f;
  }
  return (float)code * s_ad9280_calibration.volts_per_code +
         s_ad9280_calibration.intercept_v;
}

/************************************************************
 * Function :       AD9280_AppGetSamples
 * Comment  :       返回应用层原始采样缓存的只读指针
 * Parameter:       null
 * Return   :       原始采样缓存只读指针
 ************************************************************/
const AD9280_SampleTypeDef *AD9280_AppGetSamples(void)
{
  return s_ad9280_samples;
}

/************************************************************
 * Function :       AD9280_AppGetSampleCount
 * Comment  :       返回应用层缓存中的有效点数
 * Parameter:       null
 * Return   :       有效采样点数
 ************************************************************/
uint16_t AD9280_AppGetSampleCount(void)
{
  return s_ad9280_app_data.sample_count;
}

/************************************************************
 * Function :       AD9280_AppGetData
 * Comment  :       返回应用层最近一次采集结果的只读指针
 * Parameter:       null
 * Return   :       应用层数据只读指针
 ************************************************************/
const AD9280_AppDataTypeDef *AD9280_AppGetData(void)
{
  return &s_ad9280_app_data;
}

/************************************************************
 * Function :       AD9280_AppGetVoltageCalibration
 * Comment  :       返回当前输入电压校准数据的只读指针
 * Parameter:       null
 * Return   :       电压校准数据只读指针
 ************************************************************/
const AD9280_VoltageCalibrationTypeDef *
AD9280_AppGetVoltageCalibration(void)
{
  return &s_ad9280_calibration;
}

/************************************************************
 * Function :       AD9280_AppCaptureInternal
 * Comment  :       执行启动、等待、读取和状态更新的公共流程
 * Parameter:       config: 采集配置; timeout_ms: 超时时间
 * Return   :       AD9280状态
 ************************************************************/
static AD9280_StatusTypeDef AD9280_AppCaptureInternal(
    const AD9280_CaptureConfigTypeDef *config,
    uint32_t timeout_ms)
{
  AD9280_StatusTypeDef status;
  AD9280_StatusTypeDef info_status;
  uint16_t read_count;

  if (s_ad9280_app_data.initialized == 0U)
  {
    return AD9280_ERROR_NOT_INIT;
  }
  if (timeout_ms == 0U)
  {
    return AD9280_ERROR_PARAM;
  }

  s_ad9280_app_data.sample_count = 0U;
  s_ad9280_app_data.capture_stage = AD9280_APP_CAPTURE_START;
  status = AD9280_StartCapture(config);
  if (status == AD9280_OK)
  {
    s_ad9280_app_data.capture_stage = AD9280_APP_CAPTURE_WAIT;
    status = AD9280_WaitCapture(timeout_ms);
  }
  if (status == AD9280_OK)
  {
    s_ad9280_app_data.capture_stage = AD9280_APP_CAPTURE_READ;
    status = AD9280_ReadCapture(s_ad9280_samples,
                                AD9280_BUFFER_MAX_SAMPLES,
                                &read_count);
  }
  else if (status == AD9280_ERROR_TIMEOUT)
  {
    (void)AD9280_AbortCapture();
  }

  if (status == AD9280_OK)
  {
    s_ad9280_app_data.sample_count = read_count;
  }
  info_status = AD9280_GetCaptureInfo(&s_ad9280_app_data.capture);
  if (status == AD9280_OK)
  {
    if (info_status == AD9280_OK)
    {
      s_ad9280_app_data.capture_stage = AD9280_APP_CAPTURE_COMPLETE;
    }
    else
    {
      s_ad9280_app_data.capture_stage = AD9280_APP_CAPTURE_STATUS;
      status = info_status;
      s_ad9280_app_data.sample_count = 0U;
    }
  }
  s_ad9280_app_data.status = status;
  return status;
}

/************************************************************
 * Function :       AD9280_AppVoltageToCode
 * Comment  :       将校准范围内的输入电压换算为最近的8位码
 * Parameter:       voltage_v: 输入电压; code: 输出码指针
 * Return   :       AD9280状态
 ************************************************************/
static AD9280_StatusTypeDef AD9280_AppVoltageToCode(float voltage_v,
                                                    uint8_t *code)
{
  float code_f;

  if ((code == NULL) || (s_ad9280_calibration.valid == 0U) ||
      (AD9280_AppFloatValid(voltage_v) == 0U) ||
      (voltage_v < s_ad9280_calibration.low_voltage_v) ||
      (voltage_v > s_ad9280_calibration.high_voltage_v))
  {
    return AD9280_ERROR_PARAM;
  }

  code_f = (voltage_v - s_ad9280_calibration.intercept_v) /
           s_ad9280_calibration.volts_per_code;
  if (code_f < 0.0f)
  {
    code_f = 0.0f;
  }
  else if (code_f > 255.0f)
  {
    code_f = 255.0f;
  }
  *code = (uint8_t)(code_f + 0.5f);
  return AD9280_OK;
}

/************************************************************
 * Function :       AD9280_AppFloatValid
 * Comment  :       排除NaN和无穷大的浮点参数
 * Parameter:       value: 待检查浮点数
 * Return   :       1有效，0无效
 ************************************************************/
static uint8_t AD9280_AppFloatValid(float value)
{
  return ((value >= -FLT_MAX) && (value <= FLT_MAX)) ? 1U : 0U;
}
