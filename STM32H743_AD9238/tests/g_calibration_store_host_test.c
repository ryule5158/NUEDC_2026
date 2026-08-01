#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define G_CALIBRATION_STORE_H
#define BSP_QSPI_FLASH_H
#define BSP_STORAGE_H

#define BSP_QSPI_FLASH_SECTOR_SIZE 4096U

typedef enum
{
    BSP_STORAGE_OK = 0,
    BSP_STORAGE_ECC_CORRECTED = 1,
    BSP_STORAGE_ERROR = -1,
    BSP_STORAGE_TIMEOUT = -2,
    BSP_STORAGE_INVALID_ARGUMENT = -3,
    BSP_STORAGE_UNSUPPORTED = -4,
    BSP_STORAGE_VERIFY_FAILED = -5,
    BSP_STORAGE_ECC_UNCORRECTABLE = -6
} BSP_StorageStatus;

#define G_CALIBRATION_POINT_COUNT 8U
typedef struct
{
    uint32_t gain_milli;
    uint32_t coefficient_u1e6[G_CALIBRATION_POINT_COUNT];
} G_CalibrationData;

typedef struct
{
    uint8_t manufacturer_id;
    uint8_t memory_type;
    uint8_t capacity_code;
    uint8_t qpi_mode;
    uint8_t four_byte_addressing;
    uint8_t memory_mapped;
    uint32_t jedec_id;
    uint32_t capacity_bytes;
    uint32_t qspi_clock_hz;
} BSP_QSPI_FlashInfo;

static uint8_t s_flash[2U * BSP_QSPI_FLASH_SECTOR_SIZE];
static BSP_QSPI_FlashInfo s_info;
static int32_t s_program_bytes_before_failure = -1;
static uint32_t s_erase_failure_address = UINT32_MAX;

static BSP_StorageStatus BSP_QSPI_Flash_Init(void)
{
    s_info.capacity_bytes = sizeof(s_flash);
    return BSP_STORAGE_OK;
}

static const BSP_QSPI_FlashInfo *BSP_QSPI_Flash_GetInfo(void)
{
    return &s_info;
}

static BSP_StorageStatus BSP_QSPI_Flash_Read(uint32_t address,
                                             void *buffer,
                                             uint32_t length)
{
    if ((buffer == NULL) ||
        (address > sizeof(s_flash)) ||
        (length > sizeof(s_flash) - address)) {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    memcpy(buffer, &s_flash[address], length);
    return BSP_STORAGE_OK;
}

static BSP_StorageStatus BSP_QSPI_Flash_Program(uint32_t address,
                                                const void *buffer,
                                                uint32_t length)
{
    const uint8_t *input = (const uint8_t *)buffer;
    uint32_t written = length;

    if ((buffer == NULL) ||
        (address > sizeof(s_flash)) ||
        (length > sizeof(s_flash) - address)) {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    if ((s_program_bytes_before_failure >= 0) &&
        ((uint32_t)s_program_bytes_before_failure < written)) {
        written = (uint32_t)s_program_bytes_before_failure;
    }
    for (uint32_t i = 0U; i < written; ++i) {
        s_flash[address + i] &= input[i];
    }
    if (written != length) {
        s_program_bytes_before_failure = -1;
        return BSP_STORAGE_ERROR;
    }
    return BSP_STORAGE_OK;
}

static BSP_StorageStatus BSP_QSPI_Flash_EraseSector(uint32_t address)
{
    if ((address % BSP_QSPI_FLASH_SECTOR_SIZE != 0U) ||
        (address > sizeof(s_flash) - BSP_QSPI_FLASH_SECTOR_SIZE)) {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    if (address == s_erase_failure_address) {
        s_erase_failure_address = UINT32_MAX;
        return BSP_STORAGE_ERROR;
    }
    memset(&s_flash[address], 0xFF, BSP_QSPI_FLASH_SECTOR_SIZE);
    return BSP_STORAGE_OK;
}

#include "g_calibration_store.c"

static G_CalibrationData MakeData(uint32_t value)
{
    G_CalibrationData data;

    data.gain_milli = value;
    for (uint32_t i = 0U; i < G_CALIBRATION_POINT_COUNT; ++i) {
        data.coefficient_u1e6[i] = 900000U + value + i;
    }
    return data;
}

static int DataEqual(const G_CalibrationData *left,
                     const G_CalibrationData *right)
{
    return memcmp(left, right, sizeof(*left)) == 0;
}

#define CHECK(condition) do { \
    if (!(condition)) { \
        printf("FAIL line=%d: %s\n", __LINE__, #condition); \
        return 1; \
    } \
} while (0)

int main(void)
{
    G_CalibrationData first = MakeData(9550U);
    G_CalibrationData second = MakeData(9560U);
    G_CalibrationData loaded_data;
    uint8_t loaded;

    memset(s_flash, 0xFF, sizeof(s_flash));
    CHECK(G_CalibrationStore_Init(&loaded_data, &loaded) == BSP_STORAGE_OK);
    CHECK(loaded == 0U);
    CHECK(G_CalibrationStore_IsReady() != 0U);

    CHECK(G_CalibrationStore_Save(&first) == BSP_STORAGE_OK);
    CHECK(G_CalibrationStore_Init(&loaded_data, &loaded) == BSP_STORAGE_OK);
    CHECK((loaded != 0U) && DataEqual(&loaded_data, &first));

    CHECK(G_CalibrationStore_Save(&second) == BSP_STORAGE_OK);
    CHECK(G_CalibrationStore_Init(&loaded_data, &loaded) == BSP_STORAGE_OK);
    CHECK((loaded != 0U) && DataEqual(&loaded_data, &second));

    /* Corrupt the newest record: startup must fall back to the prior CRC. */
    s_flash[64U + 12U] ^= 0x01U;
    CHECK(G_CalibrationStore_Init(&loaded_data, &loaded) == BSP_STORAGE_OK);
    CHECK((loaded != 0U) && DataEqual(&loaded_data, &first));

    /* A reset during programming must not destroy the last valid record. */
    s_program_bytes_before_failure = 20;
    CHECK(G_CalibrationStore_Save(&second) == BSP_STORAGE_ERROR);
    CHECK(G_CalibrationStore_Init(&loaded_data, &loaded) == BSP_STORAGE_OK);
    CHECK((loaded != 0U) && DataEqual(&loaded_data, &first));

    /* Exercise both full sectors and the protected rollover erase. */
    memset(s_flash, 0xFF, sizeof(s_flash));
    CHECK(G_CalibrationStore_Init(&loaded_data, &loaded) == BSP_STORAGE_OK);
    for (uint32_t sequence = 1U; sequence <= 130U; ++sequence) {
        G_CalibrationData current = MakeData(9000U + sequence);
        CHECK(G_CalibrationStore_Save(&current) == BSP_STORAGE_OK);
    }
    second = MakeData(9130U);
    CHECK(G_CalibrationStore_Init(&loaded_data, &loaded) == BSP_STORAGE_OK);
    CHECK((loaded != 0U) && DataEqual(&loaded_data, &second));

    /* At rollover, a failed erase leaves the other full sector recoverable. */
    memset(s_flash, 0xFF, sizeof(s_flash));
    CHECK(G_CalibrationStore_Init(&loaded_data, &loaded) == BSP_STORAGE_OK);
    for (uint32_t sequence = 1U; sequence <= 128U; ++sequence) {
        G_CalibrationData current = MakeData(9000U + sequence);
        CHECK(G_CalibrationStore_Save(&current) == BSP_STORAGE_OK);
    }
    first = MakeData(9128U);
    s_erase_failure_address = 0U;
    second = MakeData(9129U);
    CHECK(G_CalibrationStore_Save(&second) == BSP_STORAGE_ERROR);
    CHECK(G_CalibrationStore_Init(&loaded_data, &loaded) == BSP_STORAGE_OK);
    CHECK((loaded != 0U) && DataEqual(&loaded_data, &first));

    printf("G_CALIBRATION_STORE_HOST_PASS\n");
    return 0;
}
