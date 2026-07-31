#include "ad603.h"

/* 检查AD603两点校准配置是否有效。 */
static uint8_t AD603_IsConfigValid(const AD603_ConfigTypeDef *config);

/* 填充用户手册给出的默认增益校准参数。 */
void AD603_GetDefaultConfig(AD603_ConfigTypeDef *config)
{
  if (config == NULL)
  {
    return;
  }

  config->dac_output = AD5687_OUTPUT_CV1;
  config->min_gain_mdb = AD603_DEFAULT_MIN_GAIN_MDB;
  config->max_gain_mdb = AD603_DEFAULT_MAX_GAIN_MDB;
  config->min_control_mv = AD603_DEFAULT_MIN_CONTROL_MV;
  config->max_control_mv = AD603_DEFAULT_MAX_CONTROL_MV;
}

/* 初始化AD603并把所选CV口置为最小增益电压。 */
AD603_StatusTypeDef AD603_Init(AD603_HandleTypeDef *handle,
                               const AD603_ConfigTypeDef *config)
{
  AD603_StatusTypeDef status;

  if ((handle == NULL) || (AD603_IsConfigValid(config) == 0U))
  {
    return AD603_ERROR_PARAM;
  }

  handle->config = *config;
  handle->gain_mdb = config->min_gain_mdb;
  handle->control_mv = config->min_control_mv;
  handle->initialized = 1U;

  status = AD603_SetGainMdb(handle, config->min_gain_mdb);
  if (status != AD603_OK)
  {
    handle->initialized = 0U;
  }

  return status;
}

/* 按mdB设置模块增益，例如10000表示10dB。 */
AD603_StatusTypeDef AD603_SetGainMdb(AD603_HandleTypeDef *handle,
                                     int32_t gain_mdb)
{
  AD603_StatusTypeDef status;
  uint16_t voltage_mv;

  if ((handle == NULL) || (handle->initialized == 0U))
  {
    return AD603_ERROR_PARAM;
  }

  status = AD603_GainToVoltageMv(&handle->config, gain_mdb, &voltage_mv);
  if (status != AD603_OK)
  {
    return status;
  }

  if (AD5687_AppOutputVoltageMv(handle->config.dac_output, voltage_mv) != HAL_OK)
  {
    return AD603_ERROR_DAC;
  }

  handle->gain_mdb = gain_mdb;
  handle->control_mv = voltage_mv;
  return AD603_OK;
}

/* 根据两点校准参数计算目标控制电压。 */
AD603_StatusTypeDef AD603_GainToVoltageMv(const AD603_ConfigTypeDef *config,
                                          int32_t gain_mdb,
                                          uint16_t *voltage_mv)
{
  int64_t gain_offset;
  int64_t gain_span;
  int64_t voltage_span;

  if ((voltage_mv == NULL) || (AD603_IsConfigValid(config) == 0U))
  {
    return AD603_ERROR_PARAM;
  }

  if ((gain_mdb < config->min_gain_mdb) ||
      (gain_mdb > config->max_gain_mdb))
  {
    return AD603_ERROR_RANGE;
  }

  gain_offset = (int64_t)gain_mdb - config->min_gain_mdb;
  gain_span = (int64_t)config->max_gain_mdb - config->min_gain_mdb;
  voltage_span = (int64_t)config->max_control_mv - config->min_control_mv;
  *voltage_mv = (uint16_t)(config->min_control_mv +
    ((gain_offset * voltage_span + (gain_span / 2)) / gain_span));

  return AD603_OK;
}

/* 检查AD603两点校准配置是否有效。 */
static uint8_t AD603_IsConfigValid(const AD603_ConfigTypeDef *config)
{
  if (config == NULL)
  {
    return 0U;
  }

  return (uint8_t)(
    (config->dac_output >= AD5687_OUTPUT_CV1) &&
    (config->dac_output <= AD5687_OUTPUT_CV4) &&
    (config->min_gain_mdb < config->max_gain_mdb) &&
    (config->min_control_mv < config->max_control_mv) &&
    (config->max_control_mv <= AD5687_APP_VREF_MV));
}
