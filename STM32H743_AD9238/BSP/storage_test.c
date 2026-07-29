#include "storage_test.h"

#include "bsp_nand.h"
#include "bsp_qspi_flash.h"
#include "bsp_sdram.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

#define TEST_QSPI_ADDRESS       0x00100000U
#define TEST_NAND_FIRST_BLOCK   100U
#define TEST_NAND_BUFFER_SIZE   4096U

static uint8_t nand_tx[TEST_NAND_BUFFER_SIZE];
static uint8_t nand_rx[TEST_NAND_BUFFER_SIZE];
static uint8_t qspi_tx[BSP_QSPI_FLASH_PAGE_SIZE];
static uint8_t qspi_rx[BSP_QSPI_FLASH_PAGE_SIZE];
static uint8_t sdram_ready;
static uint8_t nand_ready;
static uint8_t qspi_ready;

static uint8_t read_key(void)
{
    uint8_t key;

    for (;;)
    {
        if ((HAL_UART_Receive(&huart1, &key, 1U, HAL_MAX_DELAY) == HAL_OK) &&
            (key != '\r') && (key != '\n'))
        {
            printf("%c\r\n", key);
            return key;
        }
    }
}

static uint8_t confirm(const char *message)
{
    uint8_t key;

    printf("%s\r\n", message);
    printf("Type Y to continue, any other key to cancel: ");
    key = read_key();
    if ((key != 'Y') && (key != 'y'))
    {
        printf("Cancelled.\r\n");
        return 0U;
    }
    return 1U;
}

static uint32_t first_mismatch(const uint8_t *expected,
                               const uint8_t *actual,
                               uint32_t length)
{
    uint32_t index;

    for (index = 0U; index < length; ++index)
    {
        if (expected[index] != actual[index])
        {
            return index;
        }
    }
    return length;
}

static uint32_t first_non_ff(const uint8_t *data, uint32_t length)
{
    uint32_t index;

    for (index = 0U; index < length; ++index)
    {
        if (data[index] != 0xFFU)
        {
            return index;
        }
    }
    return length;
}

static BSP_StorageStatus ensure_sdram(void)
{
    BSP_StorageStatus status;

    if (sdram_ready != 0U)
    {
        return BSP_STORAGE_OK;
    }
    status = BSP_SDRAM_Init();
    printf("[SDRAM] init: %s\r\n", BSP_Storage_StatusString(status));
    if (status == BSP_STORAGE_OK)
    {
        sdram_ready = 1U;
    }
    return status;
}

static BSP_StorageStatus ensure_qspi(void)
{
    BSP_StorageStatus status;
    const BSP_QSPI_FlashInfo *info;

    if (qspi_ready != 0U)
    {
        return BSP_STORAGE_OK;
    }
    status = BSP_QSPI_Flash_Init();
    info = BSP_QSPI_Flash_GetInfo();
    printf("[QSPI] init: %s, JEDEC=0x%06lX, size=%lu, clock=%lu Hz\r\n",
           BSP_Storage_StatusString(status),
           (unsigned long)info->jedec_id,
           (unsigned long)info->capacity_bytes,
           (unsigned long)info->qspi_clock_hz);
    if (status == BSP_STORAGE_OK)
    {
        qspi_ready = 1U;
    }
    return status;
}

static BSP_StorageStatus ensure_nand(void)
{
    BSP_StorageStatus status;
    const BSP_NAND_Info *info;

    if (nand_ready != 0U)
    {
        return BSP_STORAGE_OK;
    }
    status = BSP_NAND_Init();
    info = BSP_NAND_GetInfo();
    printf("[NAND] init: %s, raw ID=%02X %02X %02X %02X %02X, legacy=0x%08lX\r\n",
           BSP_Storage_StatusString(status),
           info->raw_id[0], info->raw_id[1], info->raw_id[2],
           info->raw_id[3], info->raw_id[4],
           (unsigned long)info->legacy_id);
    if (status == BSP_STORAGE_OK)
    {
        nand_ready = 1U;
        printf("[NAND] page=%lu, spare=%lu, pages/block=%lu, blocks=%lu\r\n",
               (unsigned long)info->page_main_size,
               (unsigned long)info->page_spare_size,
               (unsigned long)info->pages_per_block,
               (unsigned long)info->block_count);
    }
    return status;
}

static void test_sdram(uint32_t length)
{
    BSP_SDRAM_TestFailure failure = {0};
    BSP_StorageStatus status;

    printf("[SDRAM] destructive range: 0xC0000000..0x%08lX (%lu bytes)\r\n",
           (unsigned long)(BSP_SDRAM_BASE_ADDRESS + length - 1U),
           (unsigned long)length);
    if (confirm("This SDRAM range will be overwritten.") == 0U)
    {
        return;
    }
    if (ensure_sdram() != BSP_STORAGE_OK)
    {
        return;
    }
    printf("[SDRAM] testing...\r\n");
    status = BSP_SDRAM_Test(0U, length, &failure);
    if (status == BSP_STORAGE_OK)
    {
        printf("[SDRAM] PASS (%lu bytes)\r\n", (unsigned long)length);
    }
    else
    {
        printf("[SDRAM] FAIL: %s, address=0x%08lX, expected=0x%08lX, actual=0x%08lX\r\n",
               BSP_Storage_StatusString(status),
               (unsigned long)failure.address,
               (unsigned long)failure.expected,
               (unsigned long)failure.actual);
    }
}

static void test_qspi(void)
{
    BSP_StorageStatus status;
    BSP_StorageStatus mapped_status;
    uint32_t index;

    if (ensure_qspi() != BSP_STORAGE_OK)
    {
        return;
    }
    printf("[QSPI] destructive sector: 0x%08lX..0x%08lX\r\n",
           (unsigned long)TEST_QSPI_ADDRESS,
           (unsigned long)(TEST_QSPI_ADDRESS + BSP_QSPI_FLASH_SECTOR_SIZE - 1U));
    if (confirm("The complete 4 KB QSPI sector will be erased.") == 0U)
    {
        return;
    }
    for (index = 0U; index < sizeof(qspi_tx); ++index)
    {
        qspi_tx[index] = (uint8_t)(0xA5U ^ index);
    }

    status = BSP_QSPI_Flash_EraseSector(TEST_QSPI_ADDRESS);
    printf("[QSPI] erase: %s\r\n", BSP_Storage_StatusString(status));
    if (status != BSP_STORAGE_OK)
    {
        return;
    }
    status = BSP_QSPI_Flash_Read(TEST_QSPI_ADDRESS, qspi_rx, sizeof(qspi_rx));
    if (status != BSP_STORAGE_OK)
    {
        printf("[QSPI] blank read failed: %s\r\n", BSP_Storage_StatusString(status));
        return;
    }
    index = first_non_ff(qspi_rx, sizeof(qspi_rx));
    if (index != sizeof(qspi_rx))
    {
        printf("[QSPI] blank check FAIL at +0x%lX: 0x%02X\r\n",
               (unsigned long)index, qspi_rx[index]);
        return;
    }
    printf("[QSPI] blank check PASS\r\n");

    status = BSP_QSPI_Flash_Program(TEST_QSPI_ADDRESS, qspi_tx, sizeof(qspi_tx));
    printf("[QSPI] program: %s\r\n", BSP_Storage_StatusString(status));
    if (status != BSP_STORAGE_OK)
    {
        return;
    }
    memset(qspi_rx, 0, sizeof(qspi_rx));
    status = BSP_QSPI_Flash_Read(TEST_QSPI_ADDRESS, qspi_rx, sizeof(qspi_rx));
    index = first_mismatch(qspi_tx, qspi_rx, sizeof(qspi_tx));
    if ((status != BSP_STORAGE_OK) || (index != sizeof(qspi_tx)))
    {
        printf("[QSPI] indirect read FAIL: %s, index=%lu\r\n",
               BSP_Storage_StatusString(status), (unsigned long)index);
        return;
    }
    printf("[QSPI] indirect read PASS\r\n");

    status = BSP_QSPI_Flash_EnableMemoryMapped();
    if (status != BSP_STORAGE_OK)
    {
        printf("[QSPI] memory-map enable failed: %s\r\n",
               BSP_Storage_StatusString(status));
        return;
    }
    memset(qspi_rx, 0, sizeof(qspi_rx));
    mapped_status = BSP_QSPI_Flash_Read(TEST_QSPI_ADDRESS,
                                       qspi_rx, sizeof(qspi_rx));
    index = first_mismatch(qspi_tx, qspi_rx, sizeof(qspi_tx));
    status = BSP_QSPI_Flash_DisableMemoryMapped();
    if ((mapped_status == BSP_STORAGE_OK) && (status == BSP_STORAGE_OK) &&
        (index == sizeof(qspi_tx)))
    {
        printf("[QSPI] memory-mapped read PASS (MCU 0x90100000)\r\n");
        printf("[QSPI] TEST PASS\r\n");
    }
    else
    {
        printf("[QSPI] memory-mapped read FAIL: read=%s, abort=%s, index=%lu\r\n",
               BSP_Storage_StatusString(mapped_status),
               BSP_Storage_StatusString(status), (unsigned long)index);
    }
}

static void identify_nand(void)
{
    printf("[NAND] PE8 will be switched to FMC_D5.\r\n");
    (void)ensure_nand();
}

static void test_nand(void)
{
    const BSP_NAND_Info *info;
    BSP_StorageStatus status;
    uint32_t block;
    uint32_t page;
    uint32_t index;
    uint8_t is_bad;

    printf("[NAND] PE8 will be switched to FMC_D5.\r\n");
    if (ensure_nand() != BSP_STORAGE_OK)
    {
        return;
    }
    info = BSP_NAND_GetInfo();
    if (info->page_main_size > sizeof(nand_tx))
    {
        printf("[NAND] unsupported page size for test buffer.\r\n");
        return;
    }
    for (block = TEST_NAND_FIRST_BLOCK; block < info->block_count; ++block)
    {
        status = BSP_NAND_IsBadBlock(block, &is_bad);
        if (status != BSP_STORAGE_OK)
        {
            printf("[NAND] bad-block check failed at block %lu: %s\r\n",
                   (unsigned long)block, BSP_Storage_StatusString(status));
            return;
        }
        if (is_bad == 0U)
        {
            break;
        }
        printf("[NAND] factory bad block %lu skipped.\r\n", (unsigned long)block);
    }
    if (block >= info->block_count)
    {
        printf("[NAND] no usable test block found.\r\n");
        return;
    }
    page = block * info->pages_per_block;
    printf("[NAND] destructive block=%lu, first page=%lu, block bytes=%lu\r\n",
           (unsigned long)block, (unsigned long)page,
           (unsigned long)(info->page_main_size * info->pages_per_block));
    if (confirm("The complete NAND block above will be erased.") == 0U)
    {
        return;
    }

    status = BSP_NAND_EraseBlock(block);
    printf("[NAND] erase: %s\r\n", BSP_Storage_StatusString(status));
    if (status != BSP_STORAGE_OK)
    {
        return;
    }
    status = BSP_NAND_Read(page, 0U, nand_rx, info->page_main_size);
    if (status != BSP_STORAGE_OK)
    {
        printf("[NAND] blank read failed: %s\r\n", BSP_Storage_StatusString(status));
        return;
    }
    index = first_non_ff(nand_rx, info->page_main_size);
    if (index != info->page_main_size)
    {
        printf("[NAND] blank check FAIL at +0x%lX: 0x%02X\r\n",
               (unsigned long)index, nand_rx[index]);
        return;
    }
    printf("[NAND] blank check PASS\r\n");

    for (index = 0U; index < info->page_main_size; ++index)
    {
        nand_tx[index] = (uint8_t)(0x5AU ^ index ^ (index >> 8));
    }
    status = BSP_NAND_ProgramPageECC(page, nand_tx);
    printf("[NAND] program with ECC: %s\r\n", BSP_Storage_StatusString(status));
    if (status != BSP_STORAGE_OK)
    {
        return;
    }
    memset(nand_rx, 0, info->page_main_size);
    status = BSP_NAND_ReadPageECC(page, nand_rx);
    printf("[NAND] read with ECC: %s\r\n", BSP_Storage_StatusString(status));
    index = first_mismatch(nand_tx, nand_rx, info->page_main_size);
    if (((status == BSP_STORAGE_OK) || (status == BSP_STORAGE_ECC_CORRECTED)) &&
        (index == info->page_main_size))
    {
        printf("[NAND] compare PASS\r\n");
        if (status == BSP_STORAGE_ECC_CORRECTED)
        {
            printf("[NAND] WARNING: a fresh page required ECC correction.\r\n");
        }
        printf("[NAND] TEST PASS\r\n");
    }
    else
    {
        printf("[NAND] compare FAIL at index=%lu\r\n", (unsigned long)index);
    }
}

static void print_menu(void)
{
    printf("\r\nCommands:\r\n");
    printf("  I - identify QSPI (no erase)\r\n");
    printf("  Q - erase/program/read one QSPI sector\r\n");
    printf("  P - identify NAND (no erase, enables FMC/PE8)\r\n");
    printf("  N - erase/program/read one good NAND block\r\n");
    printf("  S - test first 1 MB SDRAM\r\n");
    printf("  F - test full 32 MB SDRAM\r\n");
    printf("  M - print this menu\r\n");
}

void Storage_Test_RunConsole(void)
{
    uint8_t key;

    printf("\r\n========================================\r\n");
    printf(" STM32H743 CORE-BOARD STORAGE TEST\r\n");
    printf(" USART1 PA9/PA10, 115200 8N1\r\n");
    printf(" No flash erase is performed at boot.\r\n");
    printf(" WARNING: SDRAM/NAND use PE8 as FMC_D5.\r\n");
    printf("========================================\r\n");
    print_menu();
    for (;;)
    {
        printf("\r\nstorage> ");
        key = read_key();
        switch (key)
        {
            case 'I':
            case 'i': (void)ensure_qspi(); break;
            case 'Q':
            case 'q': test_qspi(); break;
            case 'P':
            case 'p': identify_nand(); break;
            case 'N':
            case 'n': test_nand(); break;
            case 'S':
            case 's': test_sdram(1024U * 1024U); break;
            case 'F':
            case 'f': test_sdram(BSP_SDRAM_SIZE_BYTES); break;
            case 'M':
            case 'm': print_menu(); break;
            default: printf("Unknown command '%c'. Press M for menu.\r\n", key); break;
        }
    }
}
