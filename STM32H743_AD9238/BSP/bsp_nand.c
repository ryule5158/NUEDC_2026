#include "bsp_nand.h"

#define NAND_COMMAND_AREA             (1UL << 16)
#define NAND_ADDRESS_AREA             (1UL << 17)

#define NAND_CMD_READ_1ST             0x00U
#define NAND_CMD_READ_2ND             0x30U
#define NAND_CMD_RANDOM_READ_1ST      0x05U
#define NAND_CMD_RANDOM_READ_2ND      0xE0U
#define NAND_CMD_PROGRAM_1ST          0x80U
#define NAND_CMD_RANDOM_PROGRAM       0x85U
#define NAND_CMD_PROGRAM_2ND          0x10U
#define NAND_CMD_ERASE_1ST            0x60U
#define NAND_CMD_ERASE_2ND            0xD0U
#define NAND_CMD_READ_ID              0x90U
#define NAND_CMD_READ_STATUS          0x70U
#define NAND_CMD_RESET_DEVICE         0xFFU
#define NAND_CMD_SET_FEATURE          0xEFU

#define NAND_STATUS_FAIL              0x01U
#define NAND_STATUS_READY             0x40U
#define NAND_READY_TIMEOUT_MS         1000U
#define NAND_ERASE_TIMEOUT_MS         5000U

NAND_HandleTypeDef hnand3;

static BSP_NAND_Info nand_info;
static uint8_t nand_initialized;

static void nand_write_command(uint8_t command)
{
    *(__IO uint8_t *)(BSP_NAND_BASE_ADDRESS | NAND_COMMAND_AREA) = command;
    __DSB();
}

static void nand_write_address(uint8_t address)
{
    *(__IO uint8_t *)(BSP_NAND_BASE_ADDRESS | NAND_ADDRESS_AREA) = address;
    __DSB();
}

static void nand_write_data(uint8_t data)
{
    *(__IO uint8_t *)BSP_NAND_BASE_ADDRESS = data;
    __DSB();
}

static uint8_t nand_read_data(void)
{
    uint8_t data = *(__IO uint8_t *)BSP_NAND_BASE_ADDRESS;
    __DSB();
    return data;
}

static void nand_delay_cycles(uint32_t cycles)
{
    while (cycles-- != 0U)
    {
        __NOP();
    }
}

static void nand_send_page_address(uint32_t page, uint32_t column)
{
    nand_write_address((uint8_t)column);
    nand_write_address((uint8_t)(column >> 8));
    nand_write_address((uint8_t)page);
    nand_write_address((uint8_t)(page >> 8));
    nand_write_address((uint8_t)(page >> 16));
}

static BSP_StorageStatus nand_wait_ready(uint32_t timeout_ms,
                                         uint8_t check_fail,
                                         uint8_t *final_status)
{
    uint32_t start = HAL_GetTick();
    uint8_t status;

    do
    {
        nand_write_command(NAND_CMD_READ_STATUS);
        status = nand_read_data();
        if ((status & NAND_STATUS_READY) != 0U)
        {
            if (final_status != NULL)
            {
                *final_status = status;
            }
            if ((check_fail != 0U) && ((status & NAND_STATUS_FAIL) != 0U))
            {
                return BSP_STORAGE_ERROR;
            }
            return BSP_STORAGE_OK;
        }
    } while ((HAL_GetTick() - start) <= timeout_ms);

    return BSP_STORAGE_TIMEOUT;
}

static BSP_StorageStatus nand_validate_range(uint32_t page,
                                              uint32_t column,
                                              uint32_t length)
{
    uint64_t total_pages = (uint64_t)nand_info.pages_per_block * nand_info.block_count;
    uint32_t total_page_size = nand_info.page_main_size + nand_info.page_spare_size;

    if ((nand_initialized == 0U) || (page >= total_pages) ||
        (column > total_page_size) || (length > (total_page_size - column)))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    return BSP_STORAGE_OK;
}

static BSP_StorageStatus nand_set_mode(uint8_t mode)
{
    BSP_StorageStatus status;

    nand_write_command(NAND_CMD_SET_FEATURE);
    nand_write_address(0x01U);
    nand_write_data(mode);
    nand_write_data(0U);
    nand_write_data(0U);
    nand_write_data(0U);
    status = nand_wait_ready(NAND_READY_TIMEOUT_MS, 1U, NULL);
    return status;
}

static uint16_t nand_ecc_extract_odd_even(uint8_t odd, uint32_t ecc)
{
    uint32_t bit;
    uint16_t result = 0U;

    for (bit = 0U; bit < 24U; ++bit)
    {
        if (((bit & 1U) == odd) && (((ecc >> bit) & 1U) != 0U))
        {
            result |= (uint16_t)(1U << (bit >> 1));
        }
    }
    return result;
}

static BSP_StorageStatus nand_ecc_correct(uint8_t *data,
                                           uint32_t stored_ecc,
                                           uint32_t calculated_ecc)
{
    uint16_t stored_odd = nand_ecc_extract_odd_even(1U, stored_ecc);
    uint16_t stored_even = nand_ecc_extract_odd_even(0U, stored_ecc);
    uint16_t calc_odd = nand_ecc_extract_odd_even(1U, calculated_ecc);
    uint16_t calc_even = nand_ecc_extract_odd_even(0U, calculated_ecc);
    uint16_t syndrome = stored_odd ^ stored_even ^ calc_odd ^ calc_even;
    uint16_t bit_position;

    if (syndrome != 0x0FFFU)
    {
        return BSP_STORAGE_ECC_UNCORRECTABLE;
    }

    bit_position = stored_odd ^ calc_odd;
    if (bit_position >= (BSP_NAND_ECC_SECTOR_SIZE * 8U))
    {
        return BSP_STORAGE_ECC_UNCORRECTABLE;
    }
    data[bit_position >> 3] ^= (uint8_t)(1U << (bit_position & 7U));
    return BSP_STORAGE_ECC_CORRECTED;
}

static BSP_StorageStatus nand_get_ecc(uint32_t *ecc)
{
    BSP_StorageStatus result = BSP_STORAGE_OK;

    if ((HAL_NAND_GetECC(&hnand3, ecc, NAND_READY_TIMEOUT_MS) != HAL_OK))
    {
        result = BSP_STORAGE_TIMEOUT;
    }
    (void)HAL_NAND_ECC_Disable(&hnand3);
    return result;
}

static void nand_configure_mpu(void)
{
#if (BSP_NAND_CONFIGURE_MPU != 0U)
    MPU_Region_InitTypeDef region = {0};

    HAL_MPU_Disable();
    region.Enable = MPU_REGION_ENABLE;
    region.Number = BSP_NAND_MPU_REGION_NUMBER;
    region.BaseAddress = BSP_NAND_BASE_ADDRESS;
    region.Size = MPU_REGION_SIZE_256MB;
    region.SubRegionDisable = 0x00U;
    region.TypeExtField = MPU_TEX_LEVEL0;
    region.AccessPermission = MPU_REGION_FULL_ACCESS;
    region.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    region.IsShareable = MPU_ACCESS_SHAREABLE;
    region.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    region.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&region);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
#endif
}

BSP_StorageStatus BSP_NAND_Init(void)
{
    FMC_NAND_PCC_TimingTypeDef common_timing = {0};
    FMC_NAND_PCC_TimingTypeDef attribute_timing = {0};
    BSP_StorageStatus status;

    nand_initialized = 0U;
    nand_configure_mpu();

    hnand3.Instance = FMC_NAND_DEVICE;
    hnand3.Init.NandBank = FMC_NAND_BANK3;
    hnand3.Init.Waitfeature = FMC_NAND_PCC_WAIT_FEATURE_DISABLE;
    hnand3.Init.MemoryDataWidth = FMC_NAND_PCC_MEM_BUS_WIDTH_8;
    hnand3.Init.EccComputation = FMC_NAND_ECC_DISABLE;
    hnand3.Init.ECCPageSize = FMC_NAND_ECC_PAGE_SIZE_512BYTE;
    hnand3.Init.TCLRSetupTime = 10U;
    hnand3.Init.TARSetupTime = 10U;

    common_timing.SetupTime = 10U;
    common_timing.WaitSetupTime = 10U;
    common_timing.HoldSetupTime = 10U;
    common_timing.HiZSetupTime = 10U;
    attribute_timing = common_timing;

    if (HAL_NAND_Init(&hnand3, &common_timing, &attribute_timing) != HAL_OK)
    {
        return BSP_STORAGE_ERROR;
    }
    status = BSP_NAND_Reset();
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    status = BSP_NAND_ReadID(&nand_info);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }

    if (nand_info.legacy_id == BSP_NAND_MT29F4G08_ID)
    {
        nand_info.page_main_size = 2048U;
        nand_info.page_spare_size = 64U;
        nand_info.pages_per_block = 64U;
        nand_info.block_count = 4096U;
        nand_info.plane_count = 2U;
    }
    else if (nand_info.legacy_id == BSP_NAND_MT29F16G08_ID)
    {
        nand_info.page_main_size = 4096U;
        nand_info.page_spare_size = 224U;
        nand_info.pages_per_block = 128U;
        nand_info.block_count = 4096U;
        nand_info.plane_count = 2U;
    }
    else
    {
        return BSP_STORAGE_UNSUPPORTED;
    }

    hnand3.Config.PageSize = nand_info.page_main_size;
    hnand3.Config.SpareAreaSize = nand_info.page_spare_size;
    hnand3.Config.BlockSize = nand_info.pages_per_block;
    hnand3.Config.BlockNbr = nand_info.block_count;
    hnand3.Config.PlaneNbr = nand_info.plane_count;
    hnand3.Config.PlaneSize = nand_info.block_count / nand_info.plane_count;
    hnand3.Config.ExtraCommandEnable = ENABLE;

    status = nand_set_mode(4U);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }

    nand_initialized = 1U;
    return BSP_STORAGE_OK;
}

BSP_StorageStatus BSP_NAND_Reset(void)
{
    nand_write_command(NAND_CMD_RESET_DEVICE);
    return nand_wait_ready(NAND_READY_TIMEOUT_MS, 0U, NULL);
}

BSP_StorageStatus BSP_NAND_ReadID(BSP_NAND_Info *info)
{
    BSP_NAND_Info detected = {0};
    uint32_t index;

    if (info == NULL)
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }

    nand_write_command(NAND_CMD_READ_ID);
    nand_write_address(0x00U);
    nand_delay_cycles(100U);
    for (index = 0U; index < 5U; ++index)
    {
        detected.raw_id[index] = nand_read_data();
    }
    detected.legacy_id = ((uint32_t)detected.raw_id[1] << 24) |
                         ((uint32_t)detected.raw_id[2] << 16) |
                         ((uint32_t)detected.raw_id[3] << 8) |
                         detected.raw_id[4];
    *info = detected;
    return (detected.raw_id[0] == 0x2CU) ? BSP_STORAGE_OK : BSP_STORAGE_UNSUPPORTED;
}

BSP_StorageStatus BSP_NAND_ReadStatus(uint8_t *status)
{
    if (status == NULL)
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    nand_write_command(NAND_CMD_READ_STATUS);
    *status = nand_read_data();
    return BSP_STORAGE_OK;
}

const BSP_NAND_Info *BSP_NAND_GetInfo(void)
{
    return &nand_info;
}

BSP_StorageStatus BSP_NAND_Read(uint32_t page,
                                uint32_t column,
                                void *buffer,
                                uint32_t length)
{
    uint8_t *output = (uint8_t *)buffer;
    BSP_StorageStatus status;
    uint32_t index;

    if ((buffer == NULL) && (length != 0U))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    status = nand_validate_range(page, column, length);
    if ((status != BSP_STORAGE_OK) || (length == 0U))
    {
        return status;
    }

    nand_write_command(NAND_CMD_READ_1ST);
    nand_send_page_address(page, column);
    nand_write_command(NAND_CMD_READ_2ND);
    status = nand_wait_ready(NAND_READY_TIMEOUT_MS, 0U, NULL);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    nand_write_command(NAND_CMD_READ_1ST); /* Exit status-read mode. */
    nand_delay_cycles(20U);

    for (index = 0U; index < length; ++index)
    {
        output[index] = nand_read_data();
    }
    return BSP_STORAGE_OK;
}

BSP_StorageStatus BSP_NAND_Program(uint32_t page,
                                   uint32_t column,
                                   const void *buffer,
                                   uint32_t length)
{
    const uint8_t *input = (const uint8_t *)buffer;
    BSP_StorageStatus status;
    uint32_t index;

    if ((buffer == NULL) && (length != 0U))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    status = nand_validate_range(page, column, length);
    if ((status != BSP_STORAGE_OK) || (length == 0U))
    {
        return status;
    }

    nand_write_command(NAND_CMD_PROGRAM_1ST);
    nand_send_page_address(page, column);
    nand_delay_cycles(40U);
    for (index = 0U; index < length; ++index)
    {
        nand_write_data(input[index]);
    }
    nand_write_command(NAND_CMD_PROGRAM_2ND);
    return nand_wait_ready(NAND_READY_TIMEOUT_MS, 1U, NULL);
}

BSP_StorageStatus BSP_NAND_ReadSpare(uint32_t page,
                                     uint32_t offset,
                                     void *buffer,
                                     uint32_t length)
{
    if ((offset > nand_info.page_spare_size) ||
        (length > (nand_info.page_spare_size - offset)))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    return BSP_NAND_Read(page, nand_info.page_main_size + offset, buffer, length);
}

BSP_StorageStatus BSP_NAND_ProgramSpare(uint32_t page,
                                        uint32_t offset,
                                        const void *buffer,
                                        uint32_t length)
{
    if ((offset > nand_info.page_spare_size) ||
        (length > (nand_info.page_spare_size - offset)))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    return BSP_NAND_Program(page, nand_info.page_main_size + offset, buffer, length);
}

BSP_StorageStatus BSP_NAND_EraseBlock(uint32_t block)
{
    uint32_t page;

    if ((nand_initialized == 0U) || (block >= nand_info.block_count))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    page = block * nand_info.pages_per_block;
    nand_write_command(NAND_CMD_ERASE_1ST);
    nand_write_address((uint8_t)page);
    nand_write_address((uint8_t)(page >> 8));
    nand_write_address((uint8_t)(page >> 16));
    nand_write_command(NAND_CMD_ERASE_2ND);
    return nand_wait_ready(NAND_ERASE_TIMEOUT_MS, 1U, NULL);
}

BSP_StorageStatus BSP_NAND_IsBadBlock(uint32_t block, uint8_t *is_bad)
{
    uint8_t marker0;
    uint8_t marker1;
    uint32_t first_page;
    BSP_StorageStatus status;

    if ((is_bad == NULL) || (nand_initialized == 0U) ||
        (block >= nand_info.block_count))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    first_page = block * nand_info.pages_per_block;
    status = BSP_NAND_ReadSpare(first_page, 0U, &marker0, 1U);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    status = BSP_NAND_ReadSpare(first_page + 1U, 0U, &marker1, 1U);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    *is_bad = ((marker0 != 0xFFU) || (marker1 != 0xFFU)) ? 1U : 0U;
    return BSP_STORAGE_OK;
}

BSP_StorageStatus BSP_NAND_ReadPageECC(uint32_t page, void *buffer)
{
    uint8_t *output = (uint8_t *)buffer;
    uint32_t calculated_ecc[8] = {0};
    uint32_t stored_ecc[8] = {0};
    uint32_t sectors;
    uint32_t sector;
    uint32_t index;
    uint32_t ecc_column;
    uint8_t *stored_bytes = (uint8_t *)stored_ecc;
    BSP_StorageStatus status;
    BSP_StorageStatus final_status = BSP_STORAGE_OK;

    if ((buffer == NULL) || (nand_info.page_main_size > 4096U))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    status = nand_validate_range(page, 0U, nand_info.page_main_size);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }

    nand_write_command(NAND_CMD_READ_1ST);
    nand_send_page_address(page, 0U);
    nand_write_command(NAND_CMD_READ_2ND);
    status = nand_wait_ready(NAND_READY_TIMEOUT_MS, 0U, NULL);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    nand_write_command(NAND_CMD_READ_1ST);
    nand_delay_cycles(20U);

    sectors = nand_info.page_main_size / BSP_NAND_ECC_SECTOR_SIZE;
    for (sector = 0U; sector < sectors; ++sector)
    {
        if (HAL_NAND_ECC_Enable(&hnand3) != HAL_OK)
        {
            return BSP_STORAGE_ERROR;
        }
        for (index = 0U; index < BSP_NAND_ECC_SECTOR_SIZE; ++index)
        {
            output[sector * BSP_NAND_ECC_SECTOR_SIZE + index] = nand_read_data();
        }
        status = nand_get_ecc(&calculated_ecc[sector]);
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
    }

    ecc_column = nand_info.page_main_size + BSP_NAND_ECC_SPARE_OFFSET;
    nand_write_command(NAND_CMD_RANDOM_READ_1ST);
    nand_write_address((uint8_t)ecc_column);
    nand_write_address((uint8_t)(ecc_column >> 8));
    nand_write_command(NAND_CMD_RANDOM_READ_2ND);
    nand_delay_cycles(40U);
    for (index = 0U; index < sectors * sizeof(uint32_t); ++index)
    {
        stored_bytes[index] = nand_read_data();
    }

    for (sector = 0U; sector < sectors; ++sector)
    {
        if ((stored_ecc[sector] == 0xFFFFFFFFU) ||
            (stored_ecc[sector] == calculated_ecc[sector]))
        {
            continue;
        }
        status = nand_ecc_correct(&output[sector * BSP_NAND_ECC_SECTOR_SIZE],
                                  stored_ecc[sector], calculated_ecc[sector]);
        if (status == BSP_STORAGE_ECC_UNCORRECTABLE)
        {
            return status;
        }
        final_status = BSP_STORAGE_ECC_CORRECTED;
    }
    return final_status;
}

BSP_StorageStatus BSP_NAND_ProgramPageECC(uint32_t page, const void *buffer)
{
    const uint8_t *input = (const uint8_t *)buffer;
    uint32_t calculated_ecc[8] = {0};
    uint32_t sectors;
    uint32_t sector;
    uint32_t index;
    uint32_t ecc_column;
    const uint8_t *ecc_bytes = (const uint8_t *)calculated_ecc;
    BSP_StorageStatus status;

    if ((buffer == NULL) || (nand_info.page_main_size > 4096U))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    status = nand_validate_range(page, 0U, nand_info.page_main_size);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }

    nand_write_command(NAND_CMD_PROGRAM_1ST);
    nand_send_page_address(page, 0U);
    nand_delay_cycles(40U);
    sectors = nand_info.page_main_size / BSP_NAND_ECC_SECTOR_SIZE;

    for (sector = 0U; sector < sectors; ++sector)
    {
        if (HAL_NAND_ECC_Enable(&hnand3) != HAL_OK)
        {
            return BSP_STORAGE_ERROR;
        }
        for (index = 0U; index < BSP_NAND_ECC_SECTOR_SIZE; ++index)
        {
            nand_write_data(input[sector * BSP_NAND_ECC_SECTOR_SIZE + index]);
        }
        status = nand_get_ecc(&calculated_ecc[sector]);
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
    }

    ecc_column = nand_info.page_main_size + BSP_NAND_ECC_SPARE_OFFSET;
    nand_write_command(NAND_CMD_RANDOM_PROGRAM);
    nand_write_address((uint8_t)ecc_column);
    nand_write_address((uint8_t)(ecc_column >> 8));
    nand_delay_cycles(40U);
    for (index = 0U; index < sectors * sizeof(uint32_t); ++index)
    {
        nand_write_data(ecc_bytes[index]);
    }
    nand_write_command(NAND_CMD_PROGRAM_2ND);
    return nand_wait_ready(NAND_READY_TIMEOUT_MS, 1U, NULL);
}

void HAL_NAND_MspInit(NAND_HandleTypeDef *hnand)
{
    GPIO_InitTypeDef gpio = {0};

    if ((hnand == NULL) || (hnand->Instance != FMC_NAND_DEVICE))
    {
        return;
    }

    __HAL_RCC_FMC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOD, &gpio);

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF12_FMC;

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 |
               GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio);

    gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOG, &gpio);
}
