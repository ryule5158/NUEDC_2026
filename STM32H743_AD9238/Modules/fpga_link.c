#include "fpga_link.h"

#define FPGA_LINK_FRAME_LEN        6U                       /* SPI命令帧字节数。 */
#define FPGA_LINK_WRITE_FLAG       0x80U                    /* 地址字节的写操作标志。 */
#define FPGA_LINK_ADDR_MASK        0x7FU                    /* FPGA寄存器地址有效位。 */
#define FPGA_LINK_ACK0             0xA5U                    /* FPGA响应帧首字节。 */
#define FPGA_LINK_ACK1             0x5AU                    /* FPGA响应帧次字节。 */
#define FPGA_LINK_SPI_PRESCALER    SPI_BAUDRATEPRESCALER_256 /* FPGA链路SPI分频系数。 */
#define FPGA_LINK_CS_GUARD_CYCLES  1024U                    /* 片选边沿保护延时周期数。 */
#define FPGA_LINK_READ_ATTEMPTS    3U                       /* 只读事务最大尝试次数。 */

static FPGA_LinkStatusTypeDef FPGA_Link_CheckConfig(FPGA_LinkHandleTypeDef *dev);
static FPGA_LinkStatusTypeDef FPGA_Link_Transfer(FPGA_LinkHandleTypeDef *dev,
                                                 uint8_t addr,
                                                 uint8_t is_write,
                                                 uint32_t tx_value,
                                                 uint32_t *rx_value);
static FPGA_LinkStatusTypeDef FPGA_Link_TransferFrame(FPGA_LinkHandleTypeDef *dev,
                                                      uint8_t addr,
                                                      uint8_t is_write,
                                                      uint32_t tx_value,
                                                      uint32_t *rx_value);
static HAL_StatusTypeDef FPGA_Link_UseSpi8(FPGA_LinkHandleTypeDef *dev,
                                           SPI_InitTypeDef *saved,
                                           uint8_t *changed);
static HAL_StatusTypeDef FPGA_Link_RestoreSpi(FPGA_LinkHandleTypeDef *dev,
                                               const SPI_InitTypeDef *saved,
                                               uint8_t changed);
static void FPGA_Link_DelayCycles(uint32_t cycles);

/************************************************************
 * Function :       FPGA_Link_GetDefaultConfig
 * Comment  :       填充STM32到FPGA链路的默认SPI和片选配置
 * Parameter:       cfg: 配置结构体指针
 * Return   :       null
************************************************************/
void FPGA_Link_GetDefaultConfig(FPGA_LinkConfigTypeDef *cfg)
{
  if (cfg == NULL)
  {
    return;
  }

  cfg->hspi = NULL;
  cfg->cs_port = FPGA_LINK_CS_PORT;
  cfg->cs_pin = FPGA_LINK_CS_PIN;
  cfg->timeout_ms = FPGA_LINK_TIMEOUT_MS;
}

/************************************************************
 * Function :       FPGA_Link_Init
 * Comment  :       绑定SPI和片选引脚，并将链路置为空闲状态
 * Parameter:       dev: 链路句柄; cfg: 链路配置
 * Return   :       链路状态
************************************************************/
FPGA_LinkStatusTypeDef FPGA_Link_Init(FPGA_LinkHandleTypeDef *dev,
                                      const FPGA_LinkConfigTypeDef *cfg)
{
  if ((dev == NULL) || (cfg == NULL))
  {
    return FPGA_LINK_ERROR_PARAM;
  }

  dev->cfg = *cfg;
  dev->last_rx = 0U;
  dev->session_active = 0U;
  dev->spi_changed = 0U;
  dev->last_status = FPGA_Link_CheckConfig(dev);
  if (dev->last_status != FPGA_LINK_OK)
  {
    return dev->last_status;
  }

  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);
  return FPGA_LINK_OK;
}

/************************************************************
 * Function :       FPGA_Link_Begin
 * Comment  :       开始连续事务并仅执行一次SPI模式切换
 * Parameter:       dev: 链路句柄
 * Return   :       链路状态
************************************************************/
FPGA_LinkStatusTypeDef FPGA_Link_Begin(FPGA_LinkHandleTypeDef *dev)
{
  HAL_StatusTypeDef hal_status;

  if (dev == NULL)
  {
    return FPGA_LINK_ERROR_PARAM;
  }
  if (dev->session_active != 0U)
  {
    dev->last_status = FPGA_LINK_ERROR_BUSY;
    return dev->last_status;
  }

  dev->last_status = FPGA_Link_CheckConfig(dev);
  if (dev->last_status != FPGA_LINK_OK)
  {
    return dev->last_status;
  }

  hal_status = FPGA_Link_UseSpi8(dev, &dev->saved_spi, &dev->spi_changed);
  if (hal_status != HAL_OK)
  {
    dev->last_status = (hal_status == HAL_BUSY) ?
                       FPGA_LINK_ERROR_BUSY : FPGA_LINK_ERROR_SPI;
    return dev->last_status;
  }

  dev->session_active = 1U;
  dev->last_status = FPGA_LINK_OK;
  return dev->last_status;
}

/************************************************************
 * Function :       FPGA_Link_End
 * Comment  :       结束连续事务并恢复事务前的SPI配置
 * Parameter:       dev: 链路句柄
 * Return   :       链路状态
************************************************************/
FPGA_LinkStatusTypeDef FPGA_Link_End(FPGA_LinkHandleTypeDef *dev)
{
  HAL_StatusTypeDef hal_status;

  if ((dev == NULL) || (dev->session_active == 0U))
  {
    return FPGA_LINK_ERROR_PARAM;
  }

  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);
  hal_status = FPGA_Link_RestoreSpi(dev,
                                    &dev->saved_spi,
                                    dev->spi_changed);
  dev->session_active = 0U;
  dev->spi_changed = 0U;
  dev->last_status = (hal_status == HAL_OK) ?
                     FPGA_LINK_OK : FPGA_LINK_ERROR_SPI;
  return dev->last_status;
}

/************************************************************
 * Function :       FPGA_Link_WriteReg
 * Comment  :       写一个32位FPGA寄存器
 * Parameter:       dev: 链路句柄; addr: 7位地址; value: 数据
 * Return   :       链路状态
************************************************************/
FPGA_LinkStatusTypeDef FPGA_Link_WriteReg(FPGA_LinkHandleTypeDef *dev,
                                          uint8_t addr,
                                          uint32_t value)
{
  return FPGA_Link_Transfer(dev, addr, 1U, value, NULL);
}

/************************************************************
 * Function :       FPGA_Link_ReadReg
 * Comment  :       读一个32位FPGA寄存器
 * Parameter:       dev: 链路句柄; addr: 7位地址; value: 读数指针
 * Return   :       链路状态
************************************************************/
FPGA_LinkStatusTypeDef FPGA_Link_ReadReg(FPGA_LinkHandleTypeDef *dev,
                                         uint8_t addr,
                                         uint32_t *value)
{
  FPGA_LinkStatusTypeDef status;
  uint32_t attempt;

  status = FPGA_LINK_ERROR_ACK;
  for (attempt = 0U; attempt < FPGA_LINK_READ_ATTEMPTS; attempt++)
  {
    status = FPGA_Link_Transfer(dev, addr, 0U, 0U, value);
    if ((status == FPGA_LINK_OK) ||
        ((status != FPGA_LINK_ERROR_ACK) &&
         (status != FPGA_LINK_ERROR_SPI)))
    {
      break;
    }
    FPGA_Link_DelayCycles(FPGA_LINK_CS_GUARD_CYCLES);
  }
  return status;
}

/************************************************************
 * Function :       FPGA_Link_CheckConfig
 * Comment  :       检查链路句柄、SPI、片选和超时配置
 * Parameter:       dev: 链路句柄
 * Return   :       链路状态
************************************************************/
static FPGA_LinkStatusTypeDef FPGA_Link_CheckConfig(FPGA_LinkHandleTypeDef *dev)
{
  if ((dev == NULL) ||
      (dev->cfg.hspi == NULL) ||
      (dev->cfg.cs_port == NULL) ||
      (dev->cfg.cs_pin == 0U) ||
      (dev->cfg.timeout_ms == 0U))
  {
    return FPGA_LINK_ERROR_PARAM;
  }

  return FPGA_LINK_OK;
}

/************************************************************
 * Function :       FPGA_Link_Transfer
 * Comment  :       自动管理单帧事务，连续事务中直接复用当前SPI配置
 * Parameter:       dev: 链路句柄; addr: 地址; is_write: 写标志;
 *                  tx_value: 写数据; rx_value: 可选读数指针
 * Return   :       链路状态
************************************************************/
static FPGA_LinkStatusTypeDef FPGA_Link_Transfer(FPGA_LinkHandleTypeDef *dev,
                                                 uint8_t addr,
                                                 uint8_t is_write,
                                                 uint32_t tx_value,
                                                 uint32_t *rx_value)
{
  FPGA_LinkStatusTypeDef status;
  FPGA_LinkStatusTypeDef end_status;
  uint8_t manage_session;

  if ((dev == NULL) ||
      ((addr & (uint8_t)(~FPGA_LINK_ADDR_MASK)) != 0U) ||
      ((is_write == 0U) && (rx_value == NULL)))
  {
    return FPGA_LINK_ERROR_PARAM;
  }

  manage_session = (dev->session_active == 0U) ? 1U : 0U;
  if (manage_session != 0U)
  {
    status = FPGA_Link_Begin(dev);
    if (status != FPGA_LINK_OK)
    {
      return status;
    }
  }

  status = FPGA_Link_TransferFrame(dev,
                                   addr,
                                   is_write,
                                   tx_value,
                                   rx_value);

  if (manage_session != 0U)
  {
    end_status = FPGA_Link_End(dev);
    if (status == FPGA_LINK_OK)
    {
      status = end_status;
    }
  }

  dev->last_status = status;
  return status;
}

/************************************************************
 * Function :       FPGA_Link_TransferFrame
 * Comment  :       在已配置的SPI上执行一帧6字节寄存器事务
 * Parameter:       dev: 链路句柄; addr: 地址; is_write: 写标志;
 *                  tx_value: 写数据; rx_value: 可选读数指针
 * Return   :       链路状态
************************************************************/
static FPGA_LinkStatusTypeDef FPGA_Link_TransferFrame(FPGA_LinkHandleTypeDef *dev,
                                                      uint8_t addr,
                                                      uint8_t is_write,
                                                      uint32_t tx_value,
                                                      uint32_t *rx_value)
{
  uint8_t tx[FPGA_LINK_FRAME_LEN];
  uint8_t rx[FPGA_LINK_FRAME_LEN] = {0U};
  HAL_StatusTypeDef hal_status;
  tx[0] = (uint8_t)(((is_write != 0U) ? FPGA_LINK_WRITE_FLAG : 0U) |
                    (addr & FPGA_LINK_ADDR_MASK));
  tx[1] = (uint8_t)(tx_value >> 24);
  tx[2] = (uint8_t)(tx_value >> 16);
  tx[3] = (uint8_t)(tx_value >> 8);
  tx[4] = (uint8_t)tx_value;
  tx[5] = 0U;

  FPGA_Link_DelayCycles(FPGA_LINK_CS_GUARD_CYCLES);
  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_RESET);
  FPGA_Link_DelayCycles(FPGA_LINK_CS_GUARD_CYCLES);
  hal_status = HAL_SPI_TransmitReceive(dev->cfg.hspi,
                                       tx,
                                       rx,
                                       FPGA_LINK_FRAME_LEN,
                                       dev->cfg.timeout_ms);
  HAL_GPIO_WritePin(dev->cfg.cs_port, dev->cfg.cs_pin, GPIO_PIN_SET);
  FPGA_Link_DelayCycles(FPGA_LINK_CS_GUARD_CYCLES);

  if (hal_status != HAL_OK)
  {
    return FPGA_LINK_ERROR_SPI;
  }
  if ((rx[0] != FPGA_LINK_ACK0) || (rx[1] != FPGA_LINK_ACK1))
  {
    return FPGA_LINK_ERROR_ACK;
  }

  dev->last_rx = ((uint32_t)rx[2] << 24) |
                 ((uint32_t)rx[3] << 16) |
                 ((uint32_t)rx[4] << 8) |
                 (uint32_t)rx[5];
  if (rx_value != NULL)
  {
    *rx_value = dev->last_rx;
  }
  return FPGA_LINK_OK;
}

/************************************************************
 * Function :       FPGA_Link_UseSpi8
 * Comment  :       将共享SPI临时切换到FPGA所需的8位模式0
 * Parameter:       dev: 链路句柄; saved: 原配置; changed: 切换标志
 * Return   :       HAL状态
************************************************************/
static HAL_StatusTypeDef FPGA_Link_UseSpi8(FPGA_LinkHandleTypeDef *dev,
                                           SPI_InitTypeDef *saved,
                                           uint8_t *changed)
{
  SPI_HandleTypeDef *hspi;

  if ((dev == NULL) || (saved == NULL) || (changed == NULL))
  {
    return HAL_ERROR;
  }

  hspi = dev->cfg.hspi;
  *saved = hspi->Init;
  *changed = 0U;

  if (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY)
  {
    return HAL_BUSY;
  }

  if ((hspi->Init.Mode == SPI_MODE_MASTER) &&
      (hspi->Init.Direction == SPI_DIRECTION_2LINES) &&
      (hspi->Init.DataSize == SPI_DATASIZE_8BIT) &&
      (hspi->Init.CLKPolarity == SPI_POLARITY_LOW) &&
      (hspi->Init.CLKPhase == SPI_PHASE_1EDGE) &&
      (hspi->Init.BaudRatePrescaler == FPGA_LINK_SPI_PRESCALER) &&
      (hspi->Init.FirstBit == SPI_FIRSTBIT_MSB) &&
      (hspi->Init.NSS == SPI_NSS_SOFT))
  {
    return HAL_OK;
  }

  if (HAL_SPI_DeInit(hspi) != HAL_OK)
  {
    return HAL_ERROR;
  }

  hspi->Init.Mode = SPI_MODE_MASTER;
  hspi->Init.Direction = SPI_DIRECTION_2LINES;
  hspi->Init.DataSize = SPI_DATASIZE_8BIT;
  hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi->Init.BaudRatePrescaler = FPGA_LINK_SPI_PRESCALER;
  hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi->Init.NSS = SPI_NSS_SOFT;
  hspi->Init.TIMode = SPI_TIMODE_DISABLE;
  hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi->Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;

  if (HAL_SPI_Init(hspi) != HAL_OK)
  {
    hspi->Init = *saved;
    (void)HAL_SPI_Init(hspi);
    return HAL_ERROR;
  }

  *changed = 1U;
  return HAL_OK;
}

/************************************************************
 * Function :       FPGA_Link_RestoreSpi
 * Comment  :       恢复访问FPGA前保存的共享SPI配置
 * Parameter:       dev: 链路句柄; saved: 原配置; changed: 恢复标志
 * Return   :       HAL状态
************************************************************/
static HAL_StatusTypeDef FPGA_Link_RestoreSpi(FPGA_LinkHandleTypeDef *dev,
                                               const SPI_InitTypeDef *saved,
                                               uint8_t changed)
{
  if ((dev == NULL) || (saved == NULL))
  {
    return HAL_ERROR;
  }
  if (changed == 0U)
  {
    return HAL_OK;
  }
  if (HAL_SPI_DeInit(dev->cfg.hspi) != HAL_OK)
  {
    return HAL_ERROR;
  }

  dev->cfg.hspi->Init = *saved;
  return HAL_SPI_Init(dev->cfg.hspi);
}

/************************************************************
 * Function :       FPGA_Link_DelayCycles
 * Comment  :       保证FPGA 50MHz同步器能识别片选建立和帧间高电平
 * Parameter:       cycles: 阻塞循环次数
 * Return   :       null
************************************************************/
static void FPGA_Link_DelayCycles(uint32_t cycles)
{
  volatile uint32_t index;

  for (index = 0U; index < cycles; index++)
  {
    __NOP();
  }
}
