#ifndef BSP_NAND_H
#define BSP_NAND_H

#include "main.h"
#include "bsp_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_NAND_BASE_ADDRESS          ((uint32_t)0x80000000U) /* NAND 映射基地址。 */
#define BSP_NAND_MT29F4G08_ID          ((uint32_t)0xDC909556U) /* 512 MB 器件标识。 */
#define BSP_NAND_MT29F16G08_ID         ((uint32_t)0x48002689U) /* 2 GB 兼容器件标识。 */
#define BSP_NAND_ECC_SECTOR_SIZE       512U /* 单次硬件 ECC 数据长度。 */
#define BSP_NAND_ECC_SPARE_OFFSET      16U  /* ECC 在 spare 区内的保存偏移。 */

#ifndef BSP_NAND_CONFIGURE_MPU
#define BSP_NAND_CONFIGURE_MPU         1U /* 默认配置 NAND 的 MPU 属性。 */
#endif

#ifndef BSP_NAND_MPU_REGION_NUMBER
#define BSP_NAND_MPU_REGION_NUMBER     MPU_REGION_NUMBER3 /* 默认 MPU 区域号。 */
#endif

/* NAND 几何参数和识别结果。 */
typedef struct
{
    uint8_t raw_id[5];          /* 器件原始五字节标识。 */
    uint32_t legacy_id;         /* 兼容旧工程的四字节标识。 */
    uint32_t page_main_size;    /* 单页主数据区字节数。 */
    uint32_t page_spare_size;   /* 单页 spare 区字节数。 */
    uint32_t pages_per_block;   /* 每块页数。 */
    uint32_t block_count;       /* 总块数。 */
    uint32_t plane_count;       /* 内部 plane 数量。 */
} BSP_NAND_Info;

extern NAND_HandleTypeDef hnand3; /* FMC NAND Bank3 HAL 句柄。 */

/* 初始化 NAND 与 FMC Bank3。 */
BSP_StorageStatus BSP_NAND_Init(void);
/* 复位 NAND 器件。 */
BSP_StorageStatus BSP_NAND_Reset(void);
/* 读取器件标识和几何参数。 */
BSP_StorageStatus BSP_NAND_ReadID(BSP_NAND_Info *info);
/* 读取 NAND 状态寄存器。 */
BSP_StorageStatus BSP_NAND_ReadStatus(uint8_t *status);
/* 返回最近一次识别得到的几何参数。 */
const BSP_NAND_Info *BSP_NAND_GetInfo(void);

BSP_StorageStatus BSP_NAND_Read(uint32_t page,
                                uint32_t column,
                                void *buffer,
                                uint32_t length);
BSP_StorageStatus BSP_NAND_Program(uint32_t page,
                                   uint32_t column,
                                   const void *buffer,
                                   uint32_t length);
BSP_StorageStatus BSP_NAND_ReadSpare(uint32_t page,
                                     uint32_t offset,
                                     void *buffer,
                                     uint32_t length);
BSP_StorageStatus BSP_NAND_ProgramSpare(uint32_t page,
                                        uint32_t offset,
                                        const void *buffer,
                                        uint32_t length);
BSP_StorageStatus BSP_NAND_EraseBlock(uint32_t block);
BSP_StorageStatus BSP_NAND_IsBadBlock(uint32_t block, uint8_t *is_bad);

/* 使用 STM32 FMC 1 位 Hamming ECC 读整页，ECC 保存在 spare+16。 */
BSP_StorageStatus BSP_NAND_ReadPageECC(uint32_t page, void *buffer);
/* 使用 STM32 FMC 1 位 Hamming ECC 写整页。 */
BSP_StorageStatus BSP_NAND_ProgramPageECC(uint32_t page, const void *buffer);

#ifdef __cplusplus
}
#endif

#endif /* BSP_NAND_H */
