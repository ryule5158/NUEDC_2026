#include "ltc2208_app.h"
#include "spi.h"

#if defined(NUEDC_TARGET_MSPM0G3507)
#define LTC2208_APP_SPI_HANDLE hspi1 /* TI板使用独占SPI0控制FPGA。 */
#else
#define LTC2208_APP_SPI_HANDLE hspi2 /* STM32板使用SPI2控制FPGA。 */
#endif
#include <float.h>

static LTC2208_SampleTypeDef
    s_ltc2208_samples[LTC2208_BUFFER_MAX_SAMPLES]; /* FPGA signed-20 Q3 samples. */
static LTC2208_AppDataTypeDef s_ltc2208_app_data; /* 应用层最近一次运行数据。 */
static LTC2208_VoltageCalibrationTypeDef
    s_ltc2208_calibration; /* 模块输入电压两点校准数据。 */

/* 执行立即或阈值触发的公共阻塞采集流程。 */
static LTC2208_StatusTypeDef LTC2208_AppCaptureInternal(
    const LTC2208_CaptureConfigTypeDef *config,
    uint32_t timeout_ms);

/* 将校准范围内的输入电压换算为16位触发码。 */
static LTC2208_StatusTypeDef LTC2208_AppVoltageToCode(float voltage_v,
                                                    uint16_t *code);

/* 检查浮点数是否为有限值。 */
static uint8_t LTC2208_AppFloatValid(float value);

/************************************************************
 * Function :       LTC2208_AppInit
 * Comment  :       初始化高速ADC并装入模块电路的理想电压换算
 * Parameter:       null
 * Return   :       LTC2208状态
 ************************************************************/
LTC2208_StatusTypeDef LTC2208_AppInit(void)
{
  const LTC2208_DataTypeDef *driver;
  LTC2208_StatusTypeDef status;
  float code65535_voltage_v;

  s_ltc2208_app_data = (LTC2208_AppDataTypeDef){0};
  s_ltc2208_calibration = (LTC2208_VoltageCalibrationTypeDef){0};
  code65535_voltage_v = LTC2208_MODULE_INPUT_MIN_V +
                        65535.0f * LTC2208_IDEAL_VOLTS_PER_CODE;
  status = LTC2208_AppSetVoltageCalibration(0.0f,
                                           LTC2208_MODULE_INPUT_MIN_V,
                                           65535.0f,
                                           code65535_voltage_v);
  if (status == LTC2208_OK)
  {
    status = LTC2208_Init(&LTC2208_APP_SPI_HANDLE);
  }

  /*
   * LTC2208_Init has already read the FPGA READY bit.  Mirror that first
   * driver snapshot into the application cache so diagnostics printed before
   * the first capture report ready=1 instead of the zero-initialized value.
   */
  if (status == LTC2208_OK)
  {
    driver = LTC2208_GetData();
    if (driver != NULL)
    {
      s_ltc2208_app_data.capture = driver->capture;
    }
  }

  s_ltc2208_app_data.status = status;
  s_ltc2208_app_data.initialized = (status == LTC2208_OK) ? 1U : 0U;
  return status;
}

/************************************************************
 * Function :       LTC2208_AppCapture
 * Comment  :       立即触发并阻塞读取一块高速采样数据
 * Parameter:       sample_count: 点数; decimation: 抽取倍数;
 *                  timeout_ms: 超时时间
 * Return   :       LTC2208状态
 ************************************************************/
LTC2208_StatusTypeDef LTC2208_AppCapture(uint16_t sample_count,
                                       uint16_t decimation,
                                       uint32_t timeout_ms)
{
  LTC2208_CaptureConfigTypeDef config;

  config.sample_count = sample_count;
  config.decimation = decimation;
  config.trigger_mode = LTC2208_TRIGGER_IMMEDIATE;
  config.trigger_threshold = 32768U;
  return LTC2208_AppCaptureInternal(&config, timeout_ms);
}

/************************************************************
 * Function :       LTC2208_AppCaptureTriggered
 * Comment  :       按校准后的输入电压阈值触发并阻塞读取采样块
 * Parameter:       点数、抽取倍数、边沿、触发电压和超时时间
 * Return   :       LTC2208状态
 ************************************************************/
LTC2208_StatusTypeDef LTC2208_AppCaptureTriggered(
    uint16_t sample_count,
    uint16_t decimation,
    LTC2208_TriggerTypeDef trigger_mode,
    float trigger_voltage_v,
    uint32_t timeout_ms)
{
  LTC2208_CaptureConfigTypeDef config;
  LTC2208_StatusTypeDef status;

  if (trigger_mode == LTC2208_TRIGGER_IMMEDIATE)
  {
    return LTC2208_AppCapture(sample_count, decimation, timeout_ms);
  }

  config.sample_count = sample_count;
  config.decimation = decimation;
  config.trigger_mode = trigger_mode;
  status = LTC2208_AppVoltageToCode(trigger_voltage_v,
                                   &config.trigger_threshold);
  if (status != LTC2208_OK)
  {
    s_ltc2208_app_data.status = status;
    return status;
  }
  return LTC2208_AppCaptureInternal(&config, timeout_ms);
}

/************************************************************
 * Function :       LTC2208_AppSetVoltageCalibration
 * Comment  :       保存两个输入电压及其实测平均码的线性校准
 * Parameter:       低码、低电压、高码和高电压
 * Return   :       LTC2208状态
 ************************************************************/
LTC2208_StatusTypeDef LTC2208_AppSetVoltageCalibration(
    float low_code,
    float low_voltage_v,
    float high_code,
    float high_voltage_v)
{
  float code_span;
  float voltage_span;

  if ((LTC2208_AppFloatValid(low_code) == 0U) ||
      (LTC2208_AppFloatValid(high_code) == 0U) ||
      (low_code < 0.0f) || (high_code > 65535.0f) ||
      !(high_code > low_code) ||
      (LTC2208_AppFloatValid(low_voltage_v) == 0U) ||
      (LTC2208_AppFloatValid(high_voltage_v) == 0U) ||
      !(high_voltage_v > low_voltage_v))
  {
    return LTC2208_ERROR_PARAM;
  }

  code_span = high_code - low_code;
  voltage_span = high_voltage_v - low_voltage_v;
  s_ltc2208_calibration.low_code = low_code;
  s_ltc2208_calibration.high_code = high_code;
  s_ltc2208_calibration.low_voltage_v = low_voltage_v;
  s_ltc2208_calibration.high_voltage_v = high_voltage_v;
  s_ltc2208_calibration.volts_per_code = voltage_span / code_span;
  s_ltc2208_calibration.intercept_v = low_voltage_v -
      (float)low_code * s_ltc2208_calibration.volts_per_code;
  s_ltc2208_calibration.valid = 1U;
  return LTC2208_OK;
}

/************************************************************
 * Function :       LTC2208_AppCodeToVoltage
 * Comment  :       将原始采样码换算为校准后的模块输入电压
 * Parameter:       code: 16位原始采样码
 * Return   :       输入电压，单位V
 ************************************************************/
float LTC2208_AppCodeToVoltage(uint16_t code)
{
  if (s_ltc2208_calibration.valid == 0U)
  {
    return 0.0f;
  }
  return (float)code * s_ltc2208_calibration.volts_per_code +
         s_ltc2208_calibration.intercept_v;
}

/************************************************************
 * Function :       LTC2208_AppGetSamples
 * Comment  :       返回应用层原始采样缓存的只读指针
 * Parameter:       null
 * Return   :       原始采样缓存只读指针
 ************************************************************/
const LTC2208_SampleTypeDef *LTC2208_AppGetSamples(void)
{
  return s_ltc2208_samples;
}

/************************************************************
 * Function :       LTC2208_AppGetSampleCount
 * Comment  :       返回应用层缓存中的有效点数
 * Parameter:       null
 * Return   :       有效采样点数
 ************************************************************/
uint16_t LTC2208_AppGetSampleCount(void)
{
  return s_ltc2208_app_data.sample_count;
}

/************************************************************
 * Function :       LTC2208_AppGetData
 * Comment  :       返回应用层最近一次采集结果的只读指针
 * Parameter:       null
 * Return   :       应用层数据只读指针
 ************************************************************/
const LTC2208_AppDataTypeDef *LTC2208_AppGetData(void)
{
  return &s_ltc2208_app_data;
}

/************************************************************
 * Function :       LTC2208_AppGetVoltageCalibration
 * Comment  :       返回当前输入电压校准数据的只读指针
 * Parameter:       null
 * Return   :       电压校准数据只读指针
 ************************************************************/
const LTC2208_VoltageCalibrationTypeDef *
LTC2208_AppGetVoltageCalibration(void)
{
  return &s_ltc2208_calibration;
}

/************************************************************
 * Function :       LTC2208_AppCaptureInternal
 * Comment  :       执行启动、等待、读取和状态更新的公共流程
 * Parameter:       config: 采集配置; timeout_ms: 超时时间
 * Return   :       LTC2208状态
 ************************************************************/
static LTC2208_StatusTypeDef LTC2208_AppCaptureInternal(
    const LTC2208_CaptureConfigTypeDef *config,
    uint32_t timeout_ms)
{
  LTC2208_StatusTypeDef status;
  LTC2208_StatusTypeDef info_status;
  uint16_t read_count;

  if (s_ltc2208_app_data.initialized == 0U)
  {
    return LTC2208_ERROR_NOT_INIT;
  }
  if (timeout_ms == 0U)
  {
    return LTC2208_ERROR_PARAM;
  }

  s_ltc2208_app_data.sample_count = 0U;
  s_ltc2208_app_data.capture_stage = LTC2208_APP_CAPTURE_START;
  status = LTC2208_StartCapture(config);
  if (status == LTC2208_OK)
  {
    s_ltc2208_app_data.capture_stage = LTC2208_APP_CAPTURE_WAIT;
    status = LTC2208_WaitCapture(timeout_ms);
  }
  if (status == LTC2208_OK)
  {
    s_ltc2208_app_data.capture_stage = LTC2208_APP_CAPTURE_READ;
    status = LTC2208_ReadCapture(s_ltc2208_samples,
                                LTC2208_BUFFER_MAX_SAMPLES,
                                &read_count);
  }
  else if (status == LTC2208_ERROR_TIMEOUT)
  {
    (void)LTC2208_AbortCapture();
  }

  if (status == LTC2208_OK)
  {
    s_ltc2208_app_data.sample_count = read_count;
  }
  info_status = LTC2208_GetCaptureInfo(&s_ltc2208_app_data.capture);
  if (status == LTC2208_OK)
  {
    if (info_status == LTC2208_OK)
    {
      s_ltc2208_app_data.capture_stage = LTC2208_APP_CAPTURE_COMPLETE;
    }
    else
    {
      s_ltc2208_app_data.capture_stage = LTC2208_APP_CAPTURE_STATUS;
      status = info_status;
      s_ltc2208_app_data.sample_count = 0U;
    }
  }
  s_ltc2208_app_data.status = status;
  return status;
}

/************************************************************
 * Function :       LTC2208_AppVoltageToCode
 * Comment  :       将校准范围内的输入电压换算为最近的16位码
 * Parameter:       voltage_v: 输入电压; code: 输出码指针
 * Return   :       LTC2208状态
 ************************************************************/
static LTC2208_StatusTypeDef LTC2208_AppVoltageToCode(float voltage_v,
                                                    uint16_t *code)
{
  float code_f;

  if ((code == NULL) || (s_ltc2208_calibration.valid == 0U) ||
      (LTC2208_AppFloatValid(voltage_v) == 0U) ||
      (voltage_v < s_ltc2208_calibration.low_voltage_v) ||
      (voltage_v > s_ltc2208_calibration.high_voltage_v))
  {
    return LTC2208_ERROR_PARAM;
  }

  code_f = (voltage_v - s_ltc2208_calibration.intercept_v) /
           s_ltc2208_calibration.volts_per_code;
  if (code_f < 0.0f)
  {
    code_f = 0.0f;
  }
  else if (code_f > 65535.0f)
  {
    code_f = 65535.0f;
  }
  *code = (uint16_t)(code_f + 0.5f);
  return LTC2208_OK;
}

/************************************************************
 * Function :       LTC2208_AppFloatValid
 * Comment  :       排除NaN和无穷大的浮点参数
 * Parameter:       value: 待检查浮点数
 * Return   :       1有效，0无效
 ************************************************************/
static uint8_t LTC2208_AppFloatValid(float value)
{
  return ((value >= -FLT_MAX) && (value <= FLT_MAX)) ? 1U : 0U;
}
