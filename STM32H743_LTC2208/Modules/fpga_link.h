#ifndef __FPGA_LINK_H
#define __FPGA_LINK_H /* STM32到FPGA通信层头文件包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#if defined(NUEDC_TARGET_MSPM0G3507)
#define FPGA_LINK_CS_PORT          (&NUEDC_STM32_GPIOA) /* TI板连接FPGA的虚拟片选端口。 */
#define FPGA_LINK_CS_PIN           GPIO_PIN_15  /* TI板连接FPGA的片选引脚。 */
#else
#define FPGA_LINK_CS_PORT          GPIOE        /* STM32连接FPGA的片选端口。 */
#define FPGA_LINK_CS_PIN           GPIO_PIN_8   /* STM32连接FPGA的片选引脚。 */
#endif
#define FPGA_LINK_TIMEOUT_MS       10U         /* 单帧SPI超时时间，单位ms。 */

/* STM32到FPGA通信层返回状态。 */
typedef enum
{
  FPGA_LINK_OK = 0,       /* 通信成功。 */
  FPGA_LINK_ERROR_PARAM,  /* 链路配置或输入参数无效。 */
  FPGA_LINK_ERROR_BUSY,   /* SPI或连续事务正在占用。 */
  FPGA_LINK_ERROR_SPI,    /* STM32硬件SPI操作失败。 */
  FPGA_LINK_ERROR_ACK     /* FPGA响应帧头校验失败。 */
} FPGA_LinkStatusTypeDef;

/* STM32到FPGA的硬件通信配置。 */
typedef struct
{
  SPI_HandleTypeDef *hspi;         /* STM32硬件SPI句柄。 */
  GPIO_TypeDef *cs_port;           /* FPGA片选GPIO端口。 */
  uint16_t cs_pin;                 /* FPGA片选GPIO引脚。 */
  uint32_t timeout_ms;             /* 单帧通信超时时间，单位ms。 */
} FPGA_LinkConfigTypeDef;

/* STM32到FPGA通信链路的运行句柄。 */
typedef struct
{
  FPGA_LinkConfigTypeDef cfg;      /* 当前链路配置。 */
  SPI_InitTypeDef saved_spi;       /* 连续事务前保存的SPI配置。 */
  uint32_t last_rx;                /* 最近一次读取的32位数据。 */
  FPGA_LinkStatusTypeDef last_status; /* 最近一次链路状态。 */
  uint8_t session_active;          /* 连续事务是否已开始。 */
  uint8_t spi_changed;             /* SPI配置是否需要恢复。 */
} FPGA_LinkHandleTypeDef;

/* 填充STM32到FPGA链路的默认配置。 */
void FPGA_Link_GetDefaultConfig(FPGA_LinkConfigTypeDef *cfg);

/* 初始化链路句柄并将片选置为高电平。 */
FPGA_LinkStatusTypeDef FPGA_Link_Init(FPGA_LinkHandleTypeDef *dev,
                                      const FPGA_LinkConfigTypeDef *cfg);

/* 开始连续事务，期间只切换一次SPI工作模式。 */
FPGA_LinkStatusTypeDef FPGA_Link_Begin(FPGA_LinkHandleTypeDef *dev);

/* 结束连续事务并恢复原SPI配置。 */
FPGA_LinkStatusTypeDef FPGA_Link_End(FPGA_LinkHandleTypeDef *dev);

/* 写一个32位FPGA寄存器。 */
FPGA_LinkStatusTypeDef FPGA_Link_WriteReg(FPGA_LinkHandleTypeDef *dev,
                                          uint8_t addr,
                                          uint32_t value);

/* 读一个32位FPGA寄存器。 */
FPGA_LinkStatusTypeDef FPGA_Link_ReadReg(FPGA_LinkHandleTypeDef *dev,
                                         uint8_t addr,
                                         uint32_t *value);

#ifdef __cplusplus
}
#endif

#endif /* __FPGA_LINK_H */
