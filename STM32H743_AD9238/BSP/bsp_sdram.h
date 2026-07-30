#ifndef BSP_SDRAM_H
#define BSP_SDRAM_H

#include "main.h"
#include "bsp_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_SDRAM_BASE_ADDRESS  ((uint32_t)0xC0000000U) /* SDRAM 映射基地址。 */
#define BSP_SDRAM_SIZE_BYTES    ((uint32_t)(32U * 1024U * 1024U)) /* SDRAM 容量。 */

/* SDRAM 破坏性测试的首个失败位置。 */
typedef struct
{
    uint32_t address;  /* 失败地址。 */
    uint32_t expected; /* 期望读回值。 */
    uint32_t actual;   /* 实际读回值。 */
} BSP_SDRAM_TestFailure;

extern SDRAM_HandleTypeDef hsdram1; /* FMC SDRAM HAL 句柄。 */

/* 初始化 W9825G6KH 及 FMC SDRAM Bank1。 */
BSP_StorageStatus BSP_SDRAM_Init(void);

/* 从指定字节偏移读取数据。 */
BSP_StorageStatus BSP_SDRAM_Read(uint32_t offset, void *buffer, uint32_t length);

/* 向指定字节偏移写入数据。 */
BSP_StorageStatus BSP_SDRAM_Write(uint32_t offset, const void *buffer, uint32_t length);

/* 破坏性测试指定区间，长度按 32 位字向下对齐。 */
BSP_StorageStatus BSP_SDRAM_Test(uint32_t offset,
                                 uint32_t length,
                                 BSP_SDRAM_TestFailure *failure);

/* 为启用 D-Cache 的工程配置可选 MPU 区域，区域号范围 0～15。 */
BSP_StorageStatus BSP_SDRAM_ConfigMPU(uint32_t region_number);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SDRAM_H */
