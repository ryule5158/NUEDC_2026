#include "dac8831.h"
#include <stddef.h>

/* 检查DAC8831配置是否完整。 */
static bool DAC8831_ConfigIsValid(const DAC8831_ConfigTypeDef *cfg);
/* 提供软件SPI短延时。 */
static void DAC8831_DelayCycles(uint16_t cycles);
/* 发送一位软件SPI数据。 */
static void DAC8831_WriteBit(const DAC8831_HandleTypeDef *dev,
                             GPIO_PinState bit);
/* 拉低DAC8831片选。 */
static void DAC8831_Select(const DAC8831_HandleTypeDef *dev);
/* 拉高DAC8831片选。 */
static void DAC8831_Deselect(const DAC8831_HandleTypeDef *dev);

/************************************************************
 * Function :       DAC8831_GetDefaultConfig
 * Comment  :       获取DAC8831默认驱动配置参数
 * Parameter:       cfg: DAC8831配置结构体指针
 * Return   :       null
 * Date     :       2026-06-10 V1
************************************************************/
void DAC8831_GetDefaultConfig(DAC8831_ConfigTypeDef *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  cfg->sck_port = NULL;
  cfg->sck_pin = 0U;
  cfg->mosi_port = NULL;
  cfg->mosi_pin = 0U;
  cfg->cs_port = NULL;
  cfg->cs_pin = 0U;
  cfg->ldac_port = NULL;
  cfg->ldac_pin = 0U;
  cfg->ldac_mode = DAC8831_LDAC_TIED_LOW;
  cfg->spi_delay_cycles = DAC8831_SOFT_SPI_DELAY_CYCLES;
  cfg->ldac_pulse_delay_cycles = 16U;
  cfg->vref_mv = DAC8831_DEFAULT_VREF_MV;
  cfg->output_mode = DAC8831_OUTPUT_UNIPOLAR;
}

/************************************************************
 * Function :       DAC8831_Init
 * Comment  :       初始化DAC8831驱动句柄并设置片选空闲状态
 * Parameter:       dev: DAC8831驱动句柄; cfg: DAC8831配置参数
 * Return   :       DAC8831_OK表示成功, 其他值表示参数错误
 * Date     :       2026-06-10 V1
************************************************************/
DAC8831_StatusTypeDef DAC8831_Init(DAC8831_HandleTypeDef *dev,
                                   const DAC8831_ConfigTypeDef *cfg)
{
  if ((dev == NULL) || !DAC8831_ConfigIsValid(cfg))
  {
    return DAC8831_ERROR_PARAM;
  }

  dev->cfg = *cfg;
  dev->last_code = 0U;

  HAL_GPIO_WritePin(dev->cfg.sck_port, dev->cfg.sck_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(dev->cfg.mosi_port, dev->cfg.mosi_pin, GPIO_PIN_RESET);
  DAC8831_Deselect(dev);

  if (dev->cfg.ldac_mode == DAC8831_LDAC_GPIO_PULSE)
  {
    HAL_GPIO_WritePin(dev->cfg.ldac_port, dev->cfg.ldac_pin, GPIO_PIN_SET);
  }

  return DAC8831_OK;
}

/************************************************************
 * Function :       DAC8831_WriteRaw
 * Comment  :       向DAC8831写入16位原始DAC码值
 * Parameter:       dev: DAC8831驱动句柄; code: 16位DAC码值
 * Return   :       DAC8831_OK表示成功, 其他值表示参数或SPI错误
 * Date     :       2026-06-10 V1
************************************************************/
DAC8831_StatusTypeDef DAC8831_WriteRaw(DAC8831_HandleTypeDef *dev,
                                       uint16_t code)
{
  int8_t bit;

  if ((dev == NULL) || !DAC8831_ConfigIsValid(&dev->cfg))
  {
    return DAC8831_ERROR_PARAM;
  }

  DAC8831_Select(dev);
  for (bit = 15; bit >= 0; bit--)
  {
    DAC8831_WriteBit(dev,
                     ((code >> (uint8_t)bit) & 0x01U) != 0U ?
                     GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  DAC8831_Deselect(dev);
  HAL_GPIO_WritePin(dev->cfg.mosi_port, dev->cfg.mosi_pin, GPIO_PIN_RESET);

  dev->last_code = code;

  if (dev->cfg.ldac_mode == DAC8831_LDAC_GPIO_PULSE)
  {
    return DAC8831_PulseLdac(dev);
  }

  return DAC8831_OK;
}

/************************************************************
 * Function :       DAC8831_WriteUnipolarMv
 * Comment  :       按单极性毫伏值设置DAC8831输出
 * Parameter:       dev: DAC8831驱动句柄; millivolts: 目标电压(mV)
 * Return   :       DAC8831_OK表示成功, 其他值表示参数或SPI错误
 * Date     :       2026-06-10 V1
************************************************************/
DAC8831_StatusTypeDef DAC8831_WriteUnipolarMv(DAC8831_HandleTypeDef *dev,
                                              float millivolts)
{
  if (dev == NULL)
  {
    return DAC8831_ERROR_PARAM;
  }

  return DAC8831_WriteRaw(dev,
                          DAC8831_UnipolarMvToCode(millivolts,
                                                   dev->cfg.vref_mv));
}

/************************************************************
 * Function :       DAC8831_WriteBipolarMv
 * Comment  :       按双极性毫伏值设置DAC8831输出
 * Parameter:       dev: DAC8831驱动句柄; millivolts: 目标电压(mV)
 * Return   :       DAC8831_OK表示成功, 其他值表示参数或SPI错误
 * Date     :       2026-06-10 V1
************************************************************/
DAC8831_StatusTypeDef DAC8831_WriteBipolarMv(DAC8831_HandleTypeDef *dev,
                                             float millivolts)
{
  if (dev == NULL)
  {
    return DAC8831_ERROR_PARAM;
  }

  return DAC8831_WriteRaw(dev,
                          DAC8831_BipolarMvToCode(millivolts,
                                                  dev->cfg.vref_mv));
}

/************************************************************
 * Function :       DAC8831_PulseLdac
 * Comment  :       在GPIO控制LDAC模式下输出一次LDAC锁存脉冲
 * Parameter:       dev: DAC8831驱动句柄
 * Return   :       DAC8831_OK表示成功, 其他值表示参数错误
 * Date     :       2026-06-10 V1
************************************************************/
DAC8831_StatusTypeDef DAC8831_PulseLdac(DAC8831_HandleTypeDef *dev)
{
  if ((dev == NULL) ||
      (dev->cfg.ldac_mode != DAC8831_LDAC_GPIO_PULSE) ||
      (dev->cfg.ldac_port == NULL))
  {
    return DAC8831_ERROR_PARAM;
  }

  HAL_GPIO_WritePin(dev->cfg.ldac_port, dev->cfg.ldac_pin, GPIO_PIN_RESET);
  DAC8831_DelayCycles(dev->cfg.ldac_pulse_delay_cycles);
  HAL_GPIO_WritePin(dev->cfg.ldac_port, dev->cfg.ldac_pin, GPIO_PIN_SET);

  return DAC8831_OK;
}

/************************************************************
 * Function :       DAC8831_UnipolarMvToCode
 * Comment  :       将单极性输出电压毫伏值转换为DAC8831 16位码值
 * Parameter:       millivolts: 目标电压(mV); vref_mv: 参考电压(mV)
 * Return   :       16位DAC码值
 * Date     :       2026-06-10 V1
************************************************************/
uint16_t DAC8831_UnipolarMvToCode(float millivolts, float vref_mv)
{
  float code;

  if (vref_mv <= 0.0f)
  {
    return 0U;
  }

  if (millivolts <= 0.0f)
  {
    return 0U;
  }

  if (millivolts >= vref_mv)
  {
    return DAC8831_MAX_CODE;
  }

  code = (millivolts * 65536.0f / vref_mv) + 0.5f;
  if (code >= 65535.0f)
  {
    return DAC8831_MAX_CODE;
  }

  return (uint16_t)code;
}

/************************************************************
 * Function :       DAC8831_BipolarMvToCode
 * Comment  :       将双极性输出电压毫伏值转换为DAC8831 16位码值
 * Parameter:       millivolts: 目标电压(mV); vref_mv: 参考电压(mV)
 * Return   :       16位DAC码值
 * Date     :       2026-06-10 V1
************************************************************/
uint16_t DAC8831_BipolarMvToCode(float millivolts, float vref_mv)
{
  float code;

  if (vref_mv <= 0.0f)
  {
    return DAC8831_MIDSCALE_CODE;
  }

  if (millivolts <= -vref_mv)
  {
    return 0U;
  }

  if (millivolts >= vref_mv)
  {
    return DAC8831_MAX_CODE;
  }

  code = ((millivolts * 32768.0f) / vref_mv) + 32768.0f + 0.5f;
  if (code <= 0.0f)
  {
    return 0U;
  }
  if (code >= 65535.0f)
  {
    return DAC8831_MAX_CODE;
  }

  return (uint16_t)code;
}

/************************************************************
 * Function :       DAC8831_CodeToUnipolarMv
 * Comment  :       将DAC8831 16位码值换算为单极性输出电压毫伏值
 * Parameter:       code: 16位DAC码值; vref_mv: 参考电压(mV)
 * Return   :       换算后的电压(mV)
 * Date     :       2026-06-10 V1
************************************************************/
float DAC8831_CodeToUnipolarMv(uint16_t code, float vref_mv)
{
  if (vref_mv <= 0.0f)
  {
    return 0.0f;
  }

  return ((float)code * vref_mv) / 65536.0f;
}

/************************************************************
 * Function :       DAC8831_CodeToBipolarMv
 * Comment  :       将DAC8831 16位码值换算为双极性输出电压毫伏值
 * Parameter:       code: 16位DAC码值; vref_mv: 参考电压(mV)
 * Return   :       换算后的电压(mV)
 * Date     :       2026-06-10 V1
************************************************************/
float DAC8831_CodeToBipolarMv(uint16_t code, float vref_mv)
{
  if (vref_mv <= 0.0f)
  {
    return 0.0f;
  }

  return (((float)code - 32768.0f) * vref_mv) / 32768.0f;
}

/* 检查DAC8831配置是否完整。 */
static bool DAC8831_ConfigIsValid(const DAC8831_ConfigTypeDef *cfg)
{
  if ((cfg == NULL) ||
      (cfg->sck_port == NULL) ||
      (cfg->sck_pin == 0U) ||
      (cfg->mosi_port == NULL) ||
      (cfg->mosi_pin == 0U) ||
      (cfg->cs_port == NULL) ||
      (cfg->cs_pin == 0U))
  {
    return false;
  }

  if ((cfg->ldac_mode == DAC8831_LDAC_GPIO_PULSE) &&
      ((cfg->ldac_port == NULL) || (cfg->ldac_pin == 0U)))
  {
    return false;
  }

  return true;
}

/* 发送一位软件SPI数据。 */
static void DAC8831_WriteBit(const DAC8831_HandleTypeDef *dev,
                             GPIO_PinState bit)
{
  HAL_GPIO_WritePin(dev->cfg.mosi_port, dev->cfg.mosi_pin, bit);
  DAC8831_DelayCycles(dev->cfg.spi_delay_cycles);
  HAL_GPIO_WritePin(dev->cfg.sck_port, dev->cfg.sck_pin, GPIO_PIN_SET);
  DAC8831_DelayCycles(dev->cfg.spi_delay_cycles);
  HAL_GPIO_WritePin(dev->cfg.sck_port, dev->cfg.sck_pin, GPIO_PIN_RESET);
  DAC8831_DelayCycles(dev->cfg.spi_delay_cycles);
}

/* 提供软件SPI短延时。 */
static void DAC8831_DelayCycles(uint16_t cycles)
{
  while (cycles-- > 0U)
  {
    __NOP();
  }
}

/* 拉低DAC8831片选。 */
static void DAC8831_Select(const DAC8831_HandleTypeDef *dev)
{
  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_RESET);
}

/* 拉高DAC8831片选。 */
static void DAC8831_Deselect(const DAC8831_HandleTypeDef *dev)
{
  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);
}
