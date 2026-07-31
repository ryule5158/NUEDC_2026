#ifndef __FPGA_PROMAX_LINK_MAP_H
#define __FPGA_PROMAX_LINK_MAP_H /* ProMax逻辑地址映射私有头文件包含保护。 */

#include <stddef.h>
#include <stdint.h>
#include "fpga_promax.h"

/* 将ProMax逻辑地址映射到现有6字节FPGA总线的空闲物理地址。 */
static inline uint8_t FPGA_ProMaxMapAddress(uint8_t logical,
                                           uint8_t *physical)
{
  if (physical == NULL)
  {
    return 0U;
  }

  if (logical <= 0x05U)
  {
    *physical = (uint8_t)(0x30U + logical);
  }
  else if ((logical >= 0x10U) && (logical <= 0x17U))
  {
    *physical = (uint8_t)(0x38U + logical - 0x10U);
  }
  else if ((logical >= 0x20U) && (logical <= 0x27U))
  {
    *physical = (uint8_t)(0x40U + logical - 0x20U);
  }
  else if ((logical >= 0x40U) && (logical <= 0x41U))
  {
    *physical = (uint8_t)(0x48U + logical - 0x40U);
  }
  else if ((logical >= 0x50U) && (logical <= 0x67U))
  {
    *physical = logical;
  }
  else if ((logical >= FPGA_PROMAX_REG_WAVE_CONTROL) &&
           (logical <= FPGA_PROMAX_REG_WAVE_SAMPLES))
  {
    *physical = (uint8_t)(0x68U + logical - FPGA_PROMAX_REG_WAVE_CONTROL);
  }
  else
  {
    return 0U;
  }
  return 1U;
}

/* 判断一个已映射逻辑寄存器是否允许写入。 */
static inline uint8_t FPGA_ProMaxIsWritable(uint8_t logical)
{
  if ((logical == 0x03U) || (logical == 0x05U) ||
      ((logical >= 0x10U) && (logical <= 0x17U)) ||
      ((logical >= 0x20U) && (logical <= 0x27U)) ||
      ((logical >= 0x40U) && (logical <= 0x41U)) ||
      (logical == FPGA_PROMAX_REG_WAVE_CONTROL))
  {
    return 1U;
  }
  return 0U;
}

/* 按逻辑寄存器有效位校验稳定配置的写后读回值。 */
static inline uint8_t FPGA_ProMaxVerifyWrite(uint8_t logical,
                                            uint32_t expected,
                                            uint32_t actual)
{
  uint32_t mask;

  if ((logical == FPGA_PROMAX_REG_CONTROL) ||
      (logical == FPGA_PROMAX_REG_SNAPSHOT))
  {
    mask = 0x00000001U;
  }
  else if (((logical >= FPGA_PROMAX_REG_PHASE_INC0) &&
            (logical <
             (FPGA_PROMAX_REG_PHASE_INC0 + FPGA_PROMAX_DDC_CHANNELS))) ||
           ((logical >= FPGA_PROMAX_REG_PHASE_OFFSET0) &&
            (logical <
             (FPGA_PROMAX_REG_PHASE_OFFSET0 + FPGA_PROMAX_DDC_CHANNELS))))
  {
    mask = 0xFFFFFFFFU;
  }
  else if (logical == FPGA_PROMAX_REG_MF_SELECTOR)
  {
    mask = 0x0000031FU;
  }
  else if (logical == FPGA_PROMAX_REG_MF_COEFF)
  {
    mask = 0x0003FFFFU;
  }
  else if (logical == FPGA_PROMAX_REG_WAVE_CONTROL)
  {
    mask = 0x0000001FU;
  }
  else
  {
    return 0U;
  }
  return ((expected & mask) == (actual & mask)) ? 1U : 0U;
}

#endif /* __FPGA_PROMAX_LINK_MAP_H */
