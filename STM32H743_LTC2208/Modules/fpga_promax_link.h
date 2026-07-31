#ifndef __FPGA_PROMAX_LINK_H
#define __FPGA_PROMAX_LINK_H /* 主控板ProMax链路封装接口包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "fpga_promax.h"

/* 使用当前主控板的FPGA链路初始化一个ProMax设备句柄。 */
FpgaPromax_Result FPGA_ProMax_Init(FpgaPromax *dev);

/*
 * Read one physical FPGA register without the ProMax logical-address mapper.
 * This is intentionally a diagnostic API: normal application code should use
 * the typed FpgaPromax_* functions instead.
 *
 * FPGA_ProMax_Init() must be called first, even when it returns
 * FPGA_PROMAX_E_WRONG_DEVICE, because that call prepares the shared SPI link.
 */
FpgaPromax_Result FPGA_ProMax_ReadPhysicalRegister(uint8_t physical_address,
                                                   uint32_t *value);
#ifdef __cplusplus
}
#endif

#endif /* __FPGA_PROMAX_LINK_H */
