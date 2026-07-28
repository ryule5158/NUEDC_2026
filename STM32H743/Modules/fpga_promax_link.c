#include "fpga_promax_link.h"
#include "fpga_promax_link_map.h"
#include "fpga_link.h"
#include "spi.h"

#define FPGA_PROMAX_LINK_TRANSPORT_PARAM    (-1) /* 本地传输参数错误。 */
#define FPGA_PROMAX_LINK_TRANSPORT_FRAME    (-2) /* 本地逻辑请求帧头错误。 */
#define FPGA_PROMAX_LINK_TRANSPORT_PHYSICAL (-3) /* 6字节物理链路事务失败。 */
#define FPGA_PROMAX_LINK_TRANSPORT_CLEANUP  (-4) /* SPI配置恢复失败。 */
#define FPGA_PROMAX_LINK_TRANSPORT_VERIFY   (-5) /* 稳定配置寄存器读回不匹配。 */

static FPGA_LinkHandleTypeDef s_promax_physical_link; /* ProMax独占的逻辑链路句柄。 */

/* 从大端逻辑帧读取32位数据。 */
static uint32_t FPGA_ProMaxLoadBe32(const uint8_t *src)
{
  return ((uint32_t)src[0] << 24) |
         ((uint32_t)src[1] << 16) |
         ((uint32_t)src[2] << 8) |
         (uint32_t)src[3];
}

/* 将32位数据按大端序写入逻辑帧。 */
static void FPGA_ProMaxStoreBe32(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value >> 24);
  dst[1] = (uint8_t)(value >> 16);
  dst[2] = (uint8_t)(value >> 8);
  dst[3] = (uint8_t)value;
}

/* 组装带序号和CRC的本地9字节逻辑响应。 */
static void FPGA_ProMaxBuildResponse(
  const uint8_t request[FPGA_PROMAX_FRAME_SIZE],
  uint8_t protocol_status,
  uint32_t value,
  uint8_t response[FPGA_PROMAX_FRAME_SIZE])
{
  response[0] = FPGA_PROMAX_RESPONSE_SOF;
  response[1] = request[1];
  response[2] = protocol_status;
  response[3] = request[3];
  FPGA_ProMaxStoreBe32(&response[4], value);
  response[8] = FpgaPromax_Crc8(response, 8U);
}

/* 将一次合法逻辑读写映射为现有6字节FPGA寄存器事务。 */
static FPGA_LinkStatusTypeDef FPGA_ProMaxAccessPhysical(
  FPGA_LinkHandleTypeDef *link,
  uint8_t command,
  uint8_t physical_address,
  uint32_t tx_value,
  uint32_t *rx_value)
{
  if (command == FPGA_PROMAX_CMD_WRITE)
  {
    return FPGA_Link_WriteReg(link, physical_address, tx_value);
  }
  return FPGA_Link_ReadReg(link, physical_address, rx_value);
}

/* 在本地校验9字节协议，并通过FPGA_Link完成真实物理访问。 */
static int32_t FPGA_ProMaxTransport(
  void *user,
  const uint8_t request[FPGA_PROMAX_FRAME_SIZE],
  uint8_t response[FPGA_PROMAX_FRAME_SIZE],
  uint32_t timeout_ms)
{
  FPGA_LinkHandleTypeDef *link;
  FPGA_LinkStatusTypeDef begin_status;
  FPGA_LinkStatusTypeDef end_status;
  FPGA_LinkStatusTypeDef link_status;
  uint32_t rx_value;
  uint32_t tx_value;
  uint32_t verify_value;
  uint8_t command;
  uint8_t logical_address;
  uint8_t physical_address;
  uint8_t protocol_status;

  link = (FPGA_LinkHandleTypeDef *)user;
  if ((link == NULL) || (request == NULL) ||
      (response == NULL) || (timeout_ms == 0U))
  {
    return FPGA_PROMAX_LINK_TRANSPORT_PARAM;
  }
  if (request[0] != FPGA_PROMAX_REQUEST_SOF)
  {
    return FPGA_PROMAX_LINK_TRANSPORT_FRAME;
  }
  if (FpgaPromax_Crc8(request, 8U) != request[8])
  {
    FPGA_ProMaxBuildResponse(request,
                             FPGA_PROMAX_STATUS_CRC,
                             0U,
                             response);
    return 0;
  }

  command = request[2];
  logical_address = request[3];
  tx_value = FPGA_ProMaxLoadBe32(&request[4]);
  rx_value = 0U;
  verify_value = 0U;
  protocol_status = FPGA_PROMAX_STATUS_OK;

  if (command == FPGA_PROMAX_CMD_PING)
  {
    (void)FPGA_ProMaxMapAddress(FPGA_PROMAX_REG_ID, &physical_address);
    command = FPGA_PROMAX_CMD_READ;
  }
  else if ((command != FPGA_PROMAX_CMD_READ) &&
           (command != FPGA_PROMAX_CMD_WRITE))
  {
    protocol_status = FPGA_PROMAX_STATUS_BAD_CMD;
  }
  else if (FPGA_ProMaxMapAddress(logical_address,
                                 &physical_address) == 0U)
  {
    protocol_status = FPGA_PROMAX_STATUS_BAD_ADDR;
  }
  else if ((command == FPGA_PROMAX_CMD_WRITE) &&
           (FPGA_ProMaxIsWritable(logical_address) == 0U))
  {
    protocol_status = FPGA_PROMAX_STATUS_READ_ONLY;
  }
  else
  {
    /* 地址和属性均合法，继续执行物理事务。 */
  }

  if (protocol_status != FPGA_PROMAX_STATUS_OK)
  {
    FPGA_ProMaxBuildResponse(request,
                             protocol_status,
                             0U,
                             response);
    return 0;
  }

  link->cfg.timeout_ms = timeout_ms;
  begin_status = FPGA_Link_Begin(link);
  if (begin_status == FPGA_LINK_ERROR_BUSY)
  {
    FPGA_ProMaxBuildResponse(request,
                             FPGA_PROMAX_STATUS_BUSY,
                             0U,
                             response);
    return 0;
  }
  if (begin_status != FPGA_LINK_OK)
  {
    return FPGA_PROMAX_LINK_TRANSPORT_PHYSICAL;
  }

  link_status = FPGA_ProMaxAccessPhysical(link,
                                          command,
                                          physical_address,
                                          tx_value,
                                          &rx_value);
  if ((link_status == FPGA_LINK_OK) &&
      (command == FPGA_PROMAX_CMD_WRITE))
  {
    link_status = FPGA_Link_ReadReg(link,
                                   physical_address,
                                   &verify_value);
    if ((link_status == FPGA_LINK_OK) &&
        (FPGA_ProMaxVerifyWrite(logical_address,
                                tx_value,
                                verify_value) == 0U))
    {
      end_status = FPGA_Link_End(link);
      return (end_status == FPGA_LINK_OK) ?
             FPGA_PROMAX_LINK_TRANSPORT_VERIFY :
             FPGA_PROMAX_LINK_TRANSPORT_CLEANUP;
    }
  }
  end_status = FPGA_Link_End(link);
  if (end_status != FPGA_LINK_OK)
  {
    return FPGA_PROMAX_LINK_TRANSPORT_CLEANUP;
  }
  if (link_status == FPGA_LINK_ERROR_BUSY)
  {
    FPGA_ProMaxBuildResponse(request,
                             FPGA_PROMAX_STATUS_BUSY,
                             0U,
                             response);
    return 0;
  }
  if (link_status != FPGA_LINK_OK)
  {
    return FPGA_PROMAX_LINK_TRANSPORT_PHYSICAL;
  }

  FPGA_ProMaxBuildResponse(request,
                           FPGA_PROMAX_STATUS_OK,
                           rx_value,
                           response);
  return 0;
}

/* 返回HAL单调毫秒时基。 */
static uint32_t FPGA_ProMaxTimeMs(void *user)
{
  (void)user;
  return HAL_GetTick();
}

/* 使用HAL执行低频结果轮询延时。 */
static void FPGA_ProMaxDelayMs(void *user, uint32_t delay_ms)
{
  (void)user;
  HAL_Delay(delay_ms);
}

/* 绑定现有SPI2链路，随后读取并校验ProMax设备信息。 */
FpgaPromax_Result FPGA_ProMax_Init(FpgaPromax *dev)
{
  FPGA_LinkConfigTypeDef config;
  FPGA_LinkStatusTypeDef link_status;
  FpgaPromax_Result result;

  if (dev == NULL)
  {
    return FPGA_PROMAX_E_ARGUMENT;
  }

  FPGA_Link_GetDefaultConfig(&config);
  config.hspi = &hspi2;
  link_status = FPGA_Link_Init(&s_promax_physical_link, &config);
  if (link_status != FPGA_LINK_OK)
  {
    return FPGA_PROMAX_E_TRANSPORT;
  }

  result = FpgaPromax_Init(dev,
                           FPGA_ProMaxTransport,
                           &s_promax_physical_link,
                           config.timeout_ms);
  FpgaPromax_SetTimebase(dev,
                         FPGA_ProMaxTimeMs,
                         FPGA_ProMaxDelayMs,
                         NULL);
  return result;
}
/*
 * Perform one raw 7-bit physical-register read for link/image diagnostics.
 * Keep the Begin/End pair here so callers cannot leave SPI2 in the FPGA link
 * configuration after an error.
 */
FpgaPromax_Result FPGA_ProMax_ReadPhysicalRegister(uint8_t physical_address,
                                                   uint32_t *value)
{
  FPGA_LinkStatusTypeDef begin_status;
  FPGA_LinkStatusTypeDef read_status;
  FPGA_LinkStatusTypeDef end_status;

  if ((value == NULL) || (physical_address > 0x7FU) ||
      (s_promax_physical_link.cfg.hspi == NULL))
  {
    return FPGA_PROMAX_E_ARGUMENT;
  }

  *value = 0U;
  begin_status = FPGA_Link_Begin(&s_promax_physical_link);
  if (begin_status != FPGA_LINK_OK)
  {
    return FPGA_PROMAX_E_TRANSPORT;
  }

  read_status = FPGA_Link_ReadReg(&s_promax_physical_link,
                                  physical_address,
                                  value);
  end_status = FPGA_Link_End(&s_promax_physical_link);
  if ((read_status != FPGA_LINK_OK) || (end_status != FPGA_LINK_OK))
  {
    return FPGA_PROMAX_E_TRANSPORT;
  }
  return FPGA_PROMAX_OK;
}
