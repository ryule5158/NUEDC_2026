#include "bsp_storage.h"

#include "bsp_nand.h"
#include "bsp_qspi_flash.h"
#include "bsp_sdram.h"

BSP_StorageStatus BSP_Storage_InitAll(BSP_StorageProbeResult *result)
{
    BSP_StorageProbeResult local_result;
    const BSP_NAND_Info *nand_info;
    const BSP_QSPI_FlashInfo *qspi_info;
    BSP_StorageStatus overall = BSP_STORAGE_OK;

    local_result.sdram_status = BSP_SDRAM_Init();
    local_result.nand_status = BSP_NAND_Init();
    local_result.qspi_status = BSP_QSPI_Flash_Init();

    nand_info = BSP_NAND_GetInfo();
    qspi_info = BSP_QSPI_Flash_GetInfo();
    local_result.nand_id = (nand_info != NULL) ? nand_info->legacy_id : 0U;
    local_result.qspi_jedec_id = (qspi_info != NULL) ? qspi_info->jedec_id : 0U;

    if (local_result.sdram_status < BSP_STORAGE_OK)
    {
        overall = local_result.sdram_status;
    }
    else if (local_result.nand_status < BSP_STORAGE_OK)
    {
        overall = local_result.nand_status;
    }
    else if (local_result.qspi_status < BSP_STORAGE_OK)
    {
        overall = local_result.qspi_status;
    }

    if (result != NULL)
    {
        *result = local_result;
    }
    return overall;
}

const char *BSP_Storage_StatusString(BSP_StorageStatus status)
{
    switch (status)
    {
        case BSP_STORAGE_OK:                return "ok";
        case BSP_STORAGE_ECC_CORRECTED:     return "ecc corrected";
        case BSP_STORAGE_ERROR:             return "device error";
        case BSP_STORAGE_TIMEOUT:           return "timeout";
        case BSP_STORAGE_INVALID_ARGUMENT:  return "invalid argument";
        case BSP_STORAGE_UNSUPPORTED:       return "unsupported device";
        case BSP_STORAGE_VERIFY_FAILED:     return "verify failed";
        case BSP_STORAGE_ECC_UNCORRECTABLE: return "uncorrectable ecc";
        default:                            return "unknown";
    }
}
