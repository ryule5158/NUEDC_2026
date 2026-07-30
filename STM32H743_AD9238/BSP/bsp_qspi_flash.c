#include "bsp_qspi_flash.h"

#define QSPI_CMD_WRITE_ENABLE          0x06U
#define QSPI_CMD_READ_SR1              0x05U
#define QSPI_CMD_READ_SR2              0x35U
#define QSPI_CMD_READ_SR3              0x15U
#define QSPI_CMD_WRITE_SR2             0x31U
#define QSPI_CMD_FAST_READ             0x0BU
#define QSPI_CMD_PAGE_PROGRAM          0x02U
#define QSPI_CMD_SECTOR_ERASE          0x20U
#define QSPI_CMD_CHIP_ERASE            0xC7U
#define QSPI_CMD_READ_JEDEC_ID         0x9FU
#define QSPI_CMD_ENABLE_4BYTE_ADDR     0xB7U
#define QSPI_CMD_SET_READ_PARAMETERS   0xC0U
#define QSPI_CMD_ENTER_QPI             0x38U
#define QSPI_CMD_EXIT_QPI              0xFFU
#define QSPI_CMD_ENABLE_RESET          0x66U
#define QSPI_CMD_RESET_DEVICE          0x99U

#define QSPI_SR1_BUSY                  0x01U
#define QSPI_SR1_WEL                   0x02U
#define QSPI_SR2_QE                    0x02U
#define QSPI_COMMAND_TIMEOUT_MS        5000U
#define QSPI_PROGRAM_TIMEOUT_MS        1000U
#define QSPI_SECTOR_ERASE_TIMEOUT_MS   5000U
#define QSPI_CHIP_ERASE_TIMEOUT_MS     300000U
#define QSPI_MAX_CLOCK_HZ              100000000U

QSPI_HandleTypeDef hqspi;

static BSP_QSPI_FlashInfo qspi_info;
static uint8_t qspi_initialized;

static uint32_t qspi_instruction_mode(void)
{
    return (qspi_info.qpi_mode != 0U) ? QSPI_INSTRUCTION_4_LINES : QSPI_INSTRUCTION_1_LINE;
}

static uint32_t qspi_data_mode(void)
{
    return (qspi_info.qpi_mode != 0U) ? QSPI_DATA_4_LINES : QSPI_DATA_1_LINE;
}

static uint32_t qspi_address_mode(void)
{
    return (qspi_info.qpi_mode != 0U) ? QSPI_ADDRESS_4_LINES : QSPI_ADDRESS_1_LINE;
}

static uint32_t qspi_address_size(void)
{
    return (qspi_info.four_byte_addressing != 0U)
             ? QSPI_ADDRESS_32_BITS : QSPI_ADDRESS_24_BITS;
}

static BSP_StorageStatus qspi_send_command(uint32_t instruction,
                                            uint32_t address,
                                            uint32_t instruction_mode,
                                            uint32_t address_mode,
                                            uint32_t address_size,
                                            uint32_t data_mode,
                                            uint32_t dummy_cycles,
                                            uint32_t data_length)
{
    QSPI_CommandTypeDef command = {0};

    command.Instruction = instruction;
    command.Address = address;
    command.AlternateBytes = 0U;
    command.AddressSize = address_size;
    command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    command.DummyCycles = dummy_cycles;
    command.InstructionMode = instruction_mode;
    command.AddressMode = address_mode;
    command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    command.DataMode = data_mode;
    command.NbData = data_length;
    command.DdrMode = QSPI_DDR_MODE_DISABLE;
    command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    return (HAL_QSPI_Command(&hqspi, &command, QSPI_COMMAND_TIMEOUT_MS) == HAL_OK)
             ? BSP_STORAGE_OK : BSP_STORAGE_ERROR;
}

static BSP_StorageStatus qspi_receive(uint8_t *buffer, uint32_t length)
{
    return (HAL_QSPI_Receive(&hqspi, buffer, QSPI_COMMAND_TIMEOUT_MS) == HAL_OK)
             ? BSP_STORAGE_OK : BSP_STORAGE_ERROR;
}

static BSP_StorageStatus qspi_transmit(const uint8_t *buffer, uint32_t length)
{
    (void)length;
    return (HAL_QSPI_Transmit(&hqspi, (uint8_t *)buffer, QSPI_COMMAND_TIMEOUT_MS) == HAL_OK)
             ? BSP_STORAGE_OK : BSP_STORAGE_ERROR;
}

static BSP_StorageStatus qspi_read_status_internal(uint32_t register_number, uint8_t *value)
{
    uint32_t instruction;
    BSP_StorageStatus status;

    if (register_number == 1U)
    {
        instruction = QSPI_CMD_READ_SR1;
    }
    else if (register_number == 2U)
    {
        instruction = QSPI_CMD_READ_SR2;
    }
    else if (register_number == 3U)
    {
        instruction = QSPI_CMD_READ_SR3;
    }
    else
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }

    status = qspi_send_command(instruction, 0U,
                               qspi_instruction_mode(), QSPI_ADDRESS_NONE,
                               QSPI_ADDRESS_8_BITS, qspi_data_mode(), 0U, 1U);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    return qspi_receive(value, 1U);
}

static BSP_StorageStatus qspi_wait_busy(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t status_register;
    BSP_StorageStatus status;

    do
    {
        status = qspi_read_status_internal(1U, &status_register);
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
        if ((status_register & QSPI_SR1_BUSY) == 0U)
        {
            return BSP_STORAGE_OK;
        }
    } while ((HAL_GetTick() - start) <= timeout_ms);

    return BSP_STORAGE_TIMEOUT;
}

static BSP_StorageStatus qspi_write_enable(void)
{
    uint8_t status_register;
    BSP_StorageStatus status;

    status = qspi_send_command(QSPI_CMD_WRITE_ENABLE, 0U,
                               qspi_instruction_mode(), QSPI_ADDRESS_NONE,
                               QSPI_ADDRESS_8_BITS, QSPI_DATA_NONE, 0U, 0U);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    status = qspi_read_status_internal(1U, &status_register);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    return ((status_register & QSPI_SR1_WEL) != 0U)
             ? BSP_STORAGE_OK : BSP_STORAGE_ERROR;
}

static BSP_StorageStatus qspi_reset_flash(void)
{
    /* An MCU-only reset can leave the flash in QPI mode. Exit it first. */
    (void)qspi_send_command(QSPI_CMD_EXIT_QPI, 0U,
                            QSPI_INSTRUCTION_4_LINES, QSPI_ADDRESS_NONE,
                            QSPI_ADDRESS_8_BITS, QSPI_DATA_NONE, 0U, 0U);
    qspi_info.qpi_mode = 0U;
    qspi_info.four_byte_addressing = 0U;

    if (qspi_send_command(QSPI_CMD_ENABLE_RESET, 0U,
                          QSPI_INSTRUCTION_1_LINE, QSPI_ADDRESS_NONE,
                          QSPI_ADDRESS_8_BITS, QSPI_DATA_NONE, 0U, 0U) != BSP_STORAGE_OK)
    {
        return BSP_STORAGE_ERROR;
    }
    if (qspi_send_command(QSPI_CMD_RESET_DEVICE, 0U,
                          QSPI_INSTRUCTION_1_LINE, QSPI_ADDRESS_NONE,
                          QSPI_ADDRESS_8_BITS, QSPI_DATA_NONE, 0U, 0U) != BSP_STORAGE_OK)
    {
        return BSP_STORAGE_ERROR;
    }
    HAL_Delay(1U);
    return BSP_STORAGE_OK;
}

static BSP_StorageStatus qspi_enable_quad_and_qpi(void)
{
    uint8_t status_register2;
    uint8_t read_parameters = 0x30U; /* 8 dummy clocks at high frequency. */
    BSP_StorageStatus status;

    status = qspi_read_status_internal(2U, &status_register2);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    if ((status_register2 & QSPI_SR2_QE) == 0U)
    {
        status = qspi_write_enable();
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
        status_register2 |= QSPI_SR2_QE;
        status = qspi_send_command(QSPI_CMD_WRITE_SR2, 0U,
                                   QSPI_INSTRUCTION_1_LINE, QSPI_ADDRESS_NONE,
                                   QSPI_ADDRESS_8_BITS, QSPI_DATA_1_LINE, 0U, 1U);
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
        status = qspi_transmit(&status_register2, 1U);
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
        status = qspi_wait_busy(QSPI_PROGRAM_TIMEOUT_MS);
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
    }

    status = qspi_send_command(QSPI_CMD_ENTER_QPI, 0U,
                               QSPI_INSTRUCTION_1_LINE, QSPI_ADDRESS_NONE,
                               QSPI_ADDRESS_8_BITS, QSPI_DATA_NONE, 0U, 0U);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    qspi_info.qpi_mode = 1U;

    if (qspi_info.capacity_bytes > 0x01000000U)
    {
        status = qspi_write_enable();
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
        status = qspi_send_command(QSPI_CMD_ENABLE_4BYTE_ADDR, 0U,
                                   QSPI_INSTRUCTION_4_LINES, QSPI_ADDRESS_NONE,
                                   QSPI_ADDRESS_8_BITS, QSPI_DATA_NONE, 0U, 0U);
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
        qspi_info.four_byte_addressing = 1U;
    }

    status = qspi_write_enable();
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    status = qspi_send_command(QSPI_CMD_SET_READ_PARAMETERS, 0U,
                               QSPI_INSTRUCTION_4_LINES, QSPI_ADDRESS_NONE,
                               QSPI_ADDRESS_8_BITS, QSPI_DATA_4_LINES, 0U, 1U);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    return qspi_transmit(&read_parameters, 1U);
}

static BSP_StorageStatus qspi_validate_range(uint32_t address, uint32_t length)
{
    if ((qspi_initialized == 0U) || (address > qspi_info.capacity_bytes) ||
        (length > (qspi_info.capacity_bytes - address)))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    return BSP_STORAGE_OK;
}

BSP_StorageStatus BSP_QSPI_Flash_Init(void)
{
    uint32_t kernel_clock_hz;
    uint32_t divider;
    uint32_t jedec_id;
    BSP_StorageStatus status;

    qspi_initialized = 0U;
    qspi_info.manufacturer_id = 0U;
    qspi_info.memory_type = 0U;
    qspi_info.capacity_code = 0U;
    qspi_info.qpi_mode = 0U;
    qspi_info.four_byte_addressing = 0U;
    qspi_info.memory_mapped = 0U;
    qspi_info.jedec_id = 0U;
    qspi_info.capacity_bytes = 0U;

    kernel_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_QSPI);
    if (kernel_clock_hz == 0U)
    {
        kernel_clock_hz = HAL_RCC_GetHCLKFreq();
    }
    divider = (kernel_clock_hz + QSPI_MAX_CLOCK_HZ - 1U) / QSPI_MAX_CLOCK_HZ;
    if (divider < 1U)
    {
        divider = 1U;
    }
    if (divider > 256U)
    {
        return BSP_STORAGE_ERROR;
    }

    hqspi.Instance = QUADSPI;
    hqspi.Init.ClockPrescaler = divider - 1U;
    hqspi.Init.FifoThreshold = 4U;
    hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
    hqspi.Init.FlashSize = 24U; /* W25Q256: 25 address bits, HAL value is N-1. */
    hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_5_CYCLE;
    hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
    hqspi.Init.FlashID = QSPI_FLASH_ID_1;
    hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;

    if (HAL_QSPI_Init(&hqspi) != HAL_OK)
    {
        return BSP_STORAGE_ERROR;
    }
    qspi_info.qspi_clock_hz = kernel_clock_hz / divider;

    status = qspi_reset_flash();
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    status = BSP_QSPI_Flash_ReadJEDECID(&jedec_id);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }

    qspi_info.jedec_id = jedec_id;
    qspi_info.manufacturer_id = (uint8_t)(jedec_id >> 16);
    qspi_info.memory_type = (uint8_t)(jedec_id >> 8);
    qspi_info.capacity_code = (uint8_t)jedec_id;
    if ((qspi_info.manufacturer_id != 0xEFU) ||
        (qspi_info.capacity_code < 0x14U) ||
        (qspi_info.capacity_code > 0x1FU))
    {
        return BSP_STORAGE_UNSUPPORTED;
    }
    qspi_info.capacity_bytes = 1UL << qspi_info.capacity_code;

    status = qspi_enable_quad_and_qpi();
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    qspi_initialized = 1U;
    return BSP_STORAGE_OK;
}

BSP_StorageStatus BSP_QSPI_Flash_ReadJEDECID(uint32_t *jedec_id)
{
    uint8_t id[3];
    BSP_StorageStatus status;

    if (jedec_id == NULL)
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    status = qspi_send_command(QSPI_CMD_READ_JEDEC_ID, 0U,
                               qspi_instruction_mode(), QSPI_ADDRESS_NONE,
                               QSPI_ADDRESS_8_BITS, qspi_data_mode(), 0U, 3U);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    status = qspi_receive(id, sizeof(id));
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    *jedec_id = ((uint32_t)id[0] << 16) | ((uint32_t)id[1] << 8) | id[2];
    return BSP_STORAGE_OK;
}

BSP_StorageStatus BSP_QSPI_Flash_ReadStatus(uint32_t register_number, uint8_t *value)
{
    if (value == NULL)
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    if (qspi_info.memory_mapped != 0U)
    {
        return BSP_STORAGE_ERROR;
    }
    return qspi_read_status_internal(register_number, value);
}

const BSP_QSPI_FlashInfo *BSP_QSPI_Flash_GetInfo(void)
{
    return &qspi_info;
}

BSP_StorageStatus BSP_QSPI_Flash_Read(uint32_t address, void *buffer, uint32_t length)
{
    uint8_t *output = (uint8_t *)buffer;
    BSP_StorageStatus status;
    uint32_t index;

    if ((buffer == NULL) && (length != 0U))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    status = qspi_validate_range(address, length);
    if ((status != BSP_STORAGE_OK) || (length == 0U))
    {
        return status;
    }

    if (qspi_info.memory_mapped != 0U)
    {
        volatile const uint8_t *source =
            (volatile const uint8_t *)(BSP_QSPI_FLASH_MAPPED_ADDRESS + address);
        for (index = 0U; index < length; ++index)
        {
            output[index] = source[index];
        }
        return BSP_STORAGE_OK;
    }

    status = qspi_send_command(QSPI_CMD_FAST_READ, address,
                               qspi_instruction_mode(), qspi_address_mode(),
                               qspi_address_size(), qspi_data_mode(), 8U, length);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    return qspi_receive(output, length);
}

BSP_StorageStatus BSP_QSPI_Flash_Program(uint32_t address,
                                         const void *buffer,
                                         uint32_t length)
{
    const uint8_t *input = (const uint8_t *)buffer;
    BSP_StorageStatus status;
    uint32_t page_length;

    if ((buffer == NULL) && (length != 0U))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    status = qspi_validate_range(address, length);
    if ((status != BSP_STORAGE_OK) || (length == 0U))
    {
        return status;
    }
    if (qspi_info.memory_mapped != 0U)
    {
        return BSP_STORAGE_ERROR;
    }

    while (length != 0U)
    {
        page_length = BSP_QSPI_FLASH_PAGE_SIZE - (address % BSP_QSPI_FLASH_PAGE_SIZE);
        if (page_length > length)
        {
            page_length = length;
        }
        status = qspi_write_enable();
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
        status = qspi_send_command(QSPI_CMD_PAGE_PROGRAM, address,
                                   qspi_instruction_mode(), qspi_address_mode(),
                                   qspi_address_size(), qspi_data_mode(), 0U, page_length);
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
        status = qspi_transmit(input, page_length);
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
        status = qspi_wait_busy(QSPI_PROGRAM_TIMEOUT_MS);
        if (status != BSP_STORAGE_OK)
        {
            return status;
        }
        address += page_length;
        input += page_length;
        length -= page_length;
    }
    return BSP_STORAGE_OK;
}

BSP_StorageStatus BSP_QSPI_Flash_EraseSector(uint32_t address)
{
    BSP_StorageStatus status;

    if ((qspi_validate_range(address, BSP_QSPI_FLASH_SECTOR_SIZE) != BSP_STORAGE_OK) ||
        ((address % BSP_QSPI_FLASH_SECTOR_SIZE) != 0U) ||
        (qspi_info.memory_mapped != 0U))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }

    status = qspi_write_enable();
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    status = qspi_send_command(QSPI_CMD_SECTOR_ERASE, address,
                               qspi_instruction_mode(), qspi_address_mode(),
                               qspi_address_size(), QSPI_DATA_NONE, 0U, 0U);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    return qspi_wait_busy(QSPI_SECTOR_ERASE_TIMEOUT_MS);
}

BSP_StorageStatus BSP_QSPI_Flash_EraseChip(void)
{
    BSP_StorageStatus status;

    if ((qspi_initialized == 0U) || (qspi_info.memory_mapped != 0U))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    status = qspi_write_enable();
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    status = qspi_send_command(QSPI_CMD_CHIP_ERASE, 0U,
                               qspi_instruction_mode(), QSPI_ADDRESS_NONE,
                               QSPI_ADDRESS_8_BITS, QSPI_DATA_NONE, 0U, 0U);
    if (status != BSP_STORAGE_OK)
    {
        return status;
    }
    return qspi_wait_busy(QSPI_CHIP_ERASE_TIMEOUT_MS);
}

BSP_StorageStatus BSP_QSPI_Flash_EnableMemoryMapped(void)
{
    QSPI_CommandTypeDef command = {0};
    QSPI_MemoryMappedTypeDef memory_mapped = {0};

    if (qspi_initialized == 0U)
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    if (qspi_info.memory_mapped != 0U)
    {
        return BSP_STORAGE_OK;
    }

    command.Instruction = QSPI_CMD_FAST_READ;
    command.Address = 0U;
    command.AlternateBytes = 0U;
    command.AddressSize = qspi_address_size();
    command.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    command.DummyCycles = 8U;
    command.InstructionMode = qspi_instruction_mode();
    command.AddressMode = qspi_address_mode();
    command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    command.DataMode = qspi_data_mode();
    command.NbData = 0U;
    command.DdrMode = QSPI_DDR_MODE_DISABLE;
    command.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    command.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    memory_mapped.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
    memory_mapped.TimeOutPeriod = 0U;
    if (HAL_QSPI_MemoryMapped(&hqspi, &command, &memory_mapped) != HAL_OK)
    {
        return BSP_STORAGE_ERROR;
    }
    qspi_info.memory_mapped = 1U;
    return BSP_STORAGE_OK;
}

BSP_StorageStatus BSP_QSPI_Flash_DisableMemoryMapped(void)
{
    if (qspi_info.memory_mapped == 0U)
    {
        return BSP_STORAGE_OK;
    }
    if (HAL_QSPI_Abort(&hqspi) != HAL_OK)
    {
        return BSP_STORAGE_ERROR;
    }
    qspi_info.memory_mapped = 0U;
    return BSP_STORAGE_OK;
}

void HAL_QSPI_MspInit(QSPI_HandleTypeDef *qspi_handle)
{
    GPIO_InitTypeDef gpio = {0};

    if ((qspi_handle == NULL) || (qspi_handle->Instance != QUADSPI))
    {
        return;
    }

    __HAL_RCC_QSPI_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF10_QUADSPI;
    gpio.Pin = GPIO_PIN_6; /* BK1_NCS */
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pull = GPIO_NOPULL;
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9; /* BK1_IO0, BK1_IO1 */
    HAL_GPIO_Init(GPIOF, &gpio);

    gpio.Alternate = GPIO_AF9_QUADSPI;
    gpio.Pin = GPIO_PIN_2; /* CLK */
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7; /* BK1_IO3, BK1_IO2 */
    HAL_GPIO_Init(GPIOF, &gpio);
}
