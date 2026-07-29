#ifndef BSP_QSPI_FLASH_H
#define BSP_QSPI_FLASH_H

#include "main.h"
#include "bsp_storage.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_QSPI_FLASH_MAPPED_ADDRESS  ((uint32_t)0x90000000U) /* 内存映射基地址。 */
#define BSP_QSPI_FLASH_PAGE_SIZE       256U  /* 单页编程字节数。 */
#define BSP_QSPI_FLASH_SECTOR_SIZE     4096U /* 最小擦除扇区字节数。 */
#define BSP_QSPI_FLASH_EXPECTED_SIZE   ((uint32_t)(32U * 1024U * 1024U)) /* 标称容量。 */

/* QSPI Flash 识别结果与当前工作状态。 */
typedef struct
{
    uint8_t manufacturer_id;     /* JEDEC 厂商标识。 */
    uint8_t memory_type;         /* JEDEC 存储类型。 */
    uint8_t capacity_code;       /* JEDEC 容量编码。 */
    uint8_t qpi_mode;            /* 当前是否进入 QPI 模式。 */
    uint8_t four_byte_addressing;/* 当前是否使用四字节地址。 */
    uint8_t memory_mapped;       /* 当前是否启用内存映射。 */
    uint32_t jedec_id;           /* 完整 JEDEC 标识。 */
    uint32_t capacity_bytes;     /* 识别得到的容量。 */
    uint32_t qspi_clock_hz;      /* 实际 QSPI 时钟。 */
} BSP_QSPI_FlashInfo;

extern QSPI_HandleTypeDef hqspi; /* QUADSPI HAL 句柄。 */

/* 初始化 W25Q256 和 QUADSPI。 */
BSP_StorageStatus BSP_QSPI_Flash_Init(void);
/* 读取三字节 JEDEC 标识。 */
BSP_StorageStatus BSP_QSPI_Flash_ReadJEDECID(uint32_t *jedec_id);
/* 读取指定状态寄存器。 */
BSP_StorageStatus BSP_QSPI_Flash_ReadStatus(uint32_t register_number, uint8_t *value);
/* 返回最近一次识别得到的器件信息。 */
const BSP_QSPI_FlashInfo *BSP_QSPI_Flash_GetInfo(void);

/* 从 Flash 线性地址读取数据。 */
BSP_StorageStatus BSP_QSPI_Flash_Read(uint32_t address, void *buffer, uint32_t length);

/* 只编程已擦除位（1→0），并自动跨页拆分。 */
BSP_StorageStatus BSP_QSPI_Flash_Program(uint32_t address,
                                         const void *buffer,
                                         uint32_t length);
/* 擦除包含指定地址的 4 KB 扇区。 */
BSP_StorageStatus BSP_QSPI_Flash_EraseSector(uint32_t address);
/* 擦除整颗 Flash。 */
BSP_StorageStatus BSP_QSPI_Flash_EraseChip(void);

/* 启用从 0x90000000 开始的只读内存映射。 */
BSP_StorageStatus BSP_QSPI_Flash_EnableMemoryMapped(void);
/* 退出内存映射，恢复命令模式。 */
BSP_StorageStatus BSP_QSPI_Flash_DisableMemoryMapped(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_QSPI_FLASH_H */
