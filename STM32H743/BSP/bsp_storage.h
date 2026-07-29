#ifndef BSP_STORAGE_H
#define BSP_STORAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 板载存储器统一返回状态。 */
typedef enum
{
    BSP_STORAGE_OK = 0,                 /* 操作成功。 */
    BSP_STORAGE_ECC_CORRECTED = 1,      /* ECC 已纠正单比特错误。 */
    BSP_STORAGE_ERROR = -1,             /* 外设或器件返回通用错误。 */
    BSP_STORAGE_TIMEOUT = -2,           /* 等待器件就绪超时。 */
    BSP_STORAGE_INVALID_ARGUMENT = -3,  /* 指针、地址或长度无效。 */
    BSP_STORAGE_UNSUPPORTED = -4,       /* 当前器件不支持请求功能。 */
    BSP_STORAGE_VERIFY_FAILED = -5,     /* 写入后的读回校验失败。 */
    BSP_STORAGE_ECC_UNCORRECTABLE = -6  /* ECC 检测到不可纠正错误。 */
} BSP_StorageStatus;

/* 三类板载存储器的初始化与器件标识结果。 */
typedef struct
{
    BSP_StorageStatus sdram_status; /* SDRAM 初始化状态。 */
    BSP_StorageStatus nand_status;  /* NAND 初始化状态。 */
    BSP_StorageStatus qspi_status;  /* QSPI Flash 初始化状态。 */
    uint32_t nand_id;               /* NAND 兼容格式器件标识。 */
    uint32_t qspi_jedec_id;         /* QSPI Flash JEDEC 标识。 */
} BSP_StorageProbeResult;

/* 初始化全部板载存储器；main.c 默认不调用。 */
BSP_StorageStatus BSP_Storage_InitAll(BSP_StorageProbeResult *result);

/* 返回便于串口打印的状态字符串。 */
const char *BSP_Storage_StatusString(BSP_StorageStatus status);

#ifdef __cplusplus
}
#endif

#endif /* BSP_STORAGE_H */
