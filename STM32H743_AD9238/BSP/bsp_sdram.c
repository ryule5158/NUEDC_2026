#include "bsp_sdram.h"

/* W9825G6KH mode register fields. */
#define SDRAM_MODE_BURST_LENGTH_1       0x0000U
#define SDRAM_MODE_BURST_SEQUENTIAL     0x0000U
#define SDRAM_MODE_CAS_LATENCY_2        0x0020U
#define SDRAM_MODE_STANDARD             0x0000U
#define SDRAM_MODE_WRITE_BURST_SINGLE   0x0200U

#define SDRAM_ROW_COUNT                 8192U
#define SDRAM_REFRESH_PERIOD_MS         64U
#define SDRAM_REFRESH_MARGIN_CYCLES     20U

SDRAM_HandleTypeDef hsdram1;

static uint32_t sdram_ns_to_cycles(uint32_t time_ns, uint32_t clock_hz)
{
    uint64_t cycles = ((uint64_t)time_ns * clock_hz + 999999999ULL) / 1000000000ULL;

    if (cycles < 1ULL)
    {
        cycles = 1ULL;
    }
    if (cycles > 16ULL)
    {
        cycles = 16ULL;
    }
    return (uint32_t)cycles;
}

static BSP_StorageStatus sdram_send_command(uint32_t mode,
                                             uint32_t auto_refresh_count,
                                             uint32_t mode_register)
{
    FMC_SDRAM_CommandTypeDef command = {0};

    command.CommandMode = mode;
    command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
    command.AutoRefreshNumber = auto_refresh_count;
    command.ModeRegisterDefinition = mode_register;

    return (HAL_SDRAM_SendCommand(&hsdram1, &command, 1000U) == HAL_OK)
             ? BSP_STORAGE_OK : BSP_STORAGE_ERROR;
}

BSP_StorageStatus BSP_SDRAM_Init(void)
{
    FMC_SDRAM_TimingTypeDef timing = {0};
    uint32_t fmc_clock_hz;
    uint32_t sdram_clock_hz;
    uint32_t refresh_count;
    uint32_t mode_register;

    fmc_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FMC);
    if (fmc_clock_hz == 0U)
    {
        fmc_clock_hz = HAL_RCC_GetHCLKFreq();
    }
    sdram_clock_hz = fmc_clock_hz / 2U;
    if (sdram_clock_hz == 0U)
    {
        return BSP_STORAGE_ERROR;
    }

    hsdram1.Instance = FMC_SDRAM_DEVICE;
    hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
    hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
    hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
    hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
    hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
    hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
    hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
    hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
    hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_ENABLE;
    hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;

    /* W9825G6KH limits, converted for the active SDRAM clock. */
    timing.LoadToActiveDelay = 2U;                                  /* tMRD */
    timing.ExitSelfRefreshDelay = sdram_ns_to_cycles(72U, sdram_clock_hz);
    timing.SelfRefreshTime = sdram_ns_to_cycles(42U, sdram_clock_hz); /* tRAS */
    timing.RowCycleDelay = sdram_ns_to_cycles(60U, sdram_clock_hz);   /* tRC */
    timing.WriteRecoveryTime = sdram_ns_to_cycles(15U, sdram_clock_hz);
    timing.RPDelay = sdram_ns_to_cycles(18U, sdram_clock_hz);         /* tRP */
    timing.RCDDelay = sdram_ns_to_cycles(18U, sdram_clock_hz);        /* tRCD */

    if (HAL_SDRAM_Init(&hsdram1, &timing) != HAL_OK)
    {
        return BSP_STORAGE_ERROR;
    }

    if (sdram_send_command(FMC_SDRAM_CMD_CLK_ENABLE, 1U, 0U) != BSP_STORAGE_OK)
    {
        return BSP_STORAGE_ERROR;
    }
    HAL_Delay(1U); /* The device requires at least 200 us after clock enable. */

    if (sdram_send_command(FMC_SDRAM_CMD_PALL, 1U, 0U) != BSP_STORAGE_OK)
    {
        return BSP_STORAGE_ERROR;
    }
    if (sdram_send_command(FMC_SDRAM_CMD_AUTOREFRESH_MODE, 8U, 0U) != BSP_STORAGE_OK)
    {
        return BSP_STORAGE_ERROR;
    }

    mode_register = SDRAM_MODE_BURST_LENGTH_1 |
                    SDRAM_MODE_BURST_SEQUENTIAL |
                    SDRAM_MODE_CAS_LATENCY_2 |
                    SDRAM_MODE_STANDARD |
                    SDRAM_MODE_WRITE_BURST_SINGLE;
    if (sdram_send_command(FMC_SDRAM_CMD_LOAD_MODE, 1U, mode_register) != BSP_STORAGE_OK)
    {
        return BSP_STORAGE_ERROR;
    }

    refresh_count = (uint32_t)(((uint64_t)sdram_clock_hz * SDRAM_REFRESH_PERIOD_MS) /
                               (1000ULL * SDRAM_ROW_COUNT));
    if (refresh_count <= SDRAM_REFRESH_MARGIN_CYCLES)
    {
        return BSP_STORAGE_ERROR;
    }
    refresh_count -= SDRAM_REFRESH_MARGIN_CYCLES;

    return (HAL_SDRAM_ProgramRefreshRate(&hsdram1, refresh_count) == HAL_OK)
             ? BSP_STORAGE_OK : BSP_STORAGE_ERROR;
}

BSP_StorageStatus BSP_SDRAM_Read(uint32_t offset, void *buffer, uint32_t length)
{
    uint8_t *destination = (uint8_t *)buffer;
    volatile const uint8_t *source;
    uint32_t index;

    if ((buffer == NULL) && (length != 0U))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    if ((offset > BSP_SDRAM_SIZE_BYTES) ||
        (length > (BSP_SDRAM_SIZE_BYTES - offset)))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }

    source = (volatile const uint8_t *)(BSP_SDRAM_BASE_ADDRESS + offset);
    for (index = 0U; index < length; ++index)
    {
        destination[index] = source[index];
    }
    return BSP_STORAGE_OK;
}

BSP_StorageStatus BSP_SDRAM_Write(uint32_t offset, const void *buffer, uint32_t length)
{
    const uint8_t *source = (const uint8_t *)buffer;
    volatile uint8_t *destination;
    uint32_t index;

    if ((buffer == NULL) && (length != 0U))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    if ((offset > BSP_SDRAM_SIZE_BYTES) ||
        (length > (BSP_SDRAM_SIZE_BYTES - offset)))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }

    destination = (volatile uint8_t *)(BSP_SDRAM_BASE_ADDRESS + offset);
    for (index = 0U; index < length; ++index)
    {
        destination[index] = source[index];
    }
    __DSB();
    return BSP_STORAGE_OK;
}

BSP_StorageStatus BSP_SDRAM_Test(uint32_t offset,
                                 uint32_t length,
                                 BSP_SDRAM_TestFailure *failure)
{
    volatile uint32_t *memory;
    uint32_t words;
    uint32_t index;
    uint32_t expected;
    uint32_t actual;

    if (((offset & 3U) != 0U) || (length < 4U) ||
        (offset > BSP_SDRAM_SIZE_BYTES) ||
        (length > (BSP_SDRAM_SIZE_BYTES - offset)))
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }

    words = length / 4U;
    memory = (volatile uint32_t *)(BSP_SDRAM_BASE_ADDRESS + offset);

    for (index = 0U; index < words; ++index)
    {
        memory[index] = 0xA5A50000U ^ (offset + index * 4U);
    }
    __DSB();
    for (index = 0U; index < words; ++index)
    {
        expected = 0xA5A50000U ^ (offset + index * 4U);
        actual = memory[index];
        if (actual != expected)
        {
            if (failure != NULL)
            {
                failure->address = BSP_SDRAM_BASE_ADDRESS + offset + index * 4U;
                failure->expected = expected;
                failure->actual = actual;
            }
            return BSP_STORAGE_VERIFY_FAILED;
        }
        memory[index] = ~expected;
    }
    __DSB();
    for (index = 0U; index < words; ++index)
    {
        expected = ~(0xA5A50000U ^ (offset + index * 4U));
        actual = memory[index];
        if (actual != expected)
        {
            if (failure != NULL)
            {
                failure->address = BSP_SDRAM_BASE_ADDRESS + offset + index * 4U;
                failure->expected = expected;
                failure->actual = actual;
            }
            return BSP_STORAGE_VERIFY_FAILED;
        }
    }
    return BSP_STORAGE_OK;
}

BSP_StorageStatus BSP_SDRAM_ConfigMPU(uint32_t region_number)
{
    MPU_Region_InitTypeDef region = {0};

    if (region_number > 15U)
    {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }

    HAL_MPU_Disable();
    region.Enable = MPU_REGION_ENABLE;
    region.Number = (uint8_t)region_number;
    region.BaseAddress = BSP_SDRAM_BASE_ADDRESS;
    region.Size = MPU_REGION_SIZE_32MB;
    region.SubRegionDisable = 0x00U;
    region.TypeExtField = MPU_TEX_LEVEL0;
    region.AccessPermission = MPU_REGION_FULL_ACCESS;
    region.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    region.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    region.IsCacheable = MPU_ACCESS_CACHEABLE;
    region.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&region);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
    return BSP_STORAGE_OK;
}

void HAL_SDRAM_MspInit(SDRAM_HandleTypeDef *hsdram)
{
    GPIO_InitTypeDef gpio = {0};

    if ((hsdram == NULL) || (hsdram->Instance != FMC_SDRAM_DEVICE))
    {
        return;
    }

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_FMC_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF12_FMC;

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 |
               GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_7 | GPIO_PIN_8 |
               GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 |
               GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
               GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_11 | GPIO_PIN_12 |
               GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOF, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_4 |
               GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOG, &gpio);
}
