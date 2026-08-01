#include "g_calibration_store.h"

#include "bsp_qspi_flash.h"
#include <stddef.h>
#include <string.h>

#define G_CAL_STORE_MAGIC          0x4743414CUL
#define G_CAL_STORE_VERSION        1UL
#define G_CAL_STORE_SECTOR_COUNT   2U
#define G_CAL_STORE_RECORD_SIZE    64U
#define G_CAL_STORE_SLOT_COUNT     \
    (BSP_QSPI_FLASH_SECTOR_SIZE / G_CAL_STORE_RECORD_SIZE)

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t sequence;
    G_CalibrationData data;
    uint32_t reserved[3];
    uint32_t crc32;
} G_CalibrationRecord;

typedef char G_CalibrationRecordMustBe64Bytes[
    (sizeof(G_CalibrationRecord) == G_CAL_STORE_RECORD_SIZE) ? 1 : -1];

typedef struct
{
    uint8_t has_valid;
    uint8_t latest_sector;
    uint32_t latest_sequence;
    uint32_t next_slot[G_CAL_STORE_SECTOR_COUNT];
    G_CalibrationData latest_data;
} G_CalibrationScan;

static uint8_t s_ready;
static uint32_t s_sector_address[G_CAL_STORE_SECTOR_COUNT];
static uint8_t s_sector_buffer[BSP_QSPI_FLASH_SECTOR_SIZE];

static uint32_t G_CalibrationStore_Crc32(const void *data, uint32_t length)
{
    const uint8_t *bytes = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint32_t i = 0U; i < length; ++i) {
        crc ^= bytes[i];
        for (uint32_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc >> 1U) ^
                  ((0U - (crc & 1U)) & 0xEDB88320UL);
        }
    }
    return ~crc;
}

static uint8_t G_CalibrationStore_IsBlank(const uint8_t *data,
                                           uint32_t length)
{
    for (uint32_t i = 0U; i < length; ++i) {
        if (data[i] != 0xFFU) return 0U;
    }
    return 1U;
}

static uint8_t G_CalibrationStore_IsRecordValid(
    const G_CalibrationRecord *record)
{
    uint32_t crc;

    if ((record == NULL) ||
        (record->magic != G_CAL_STORE_MAGIC) ||
        (record->version != G_CAL_STORE_VERSION)) {
        return 0U;
    }
    crc = G_CalibrationStore_Crc32(
        record,
        (uint32_t)offsetof(G_CalibrationRecord, crc32));
    return (crc == record->crc32) ? 1U : 0U;
}

static uint8_t G_CalibrationStore_SequenceIsNewer(uint32_t candidate,
                                                   uint32_t current)
{
    return ((int32_t)(candidate - current) > 0) ? 1U : 0U;
}

static BSP_StorageStatus G_CalibrationStore_Scan(
    G_CalibrationScan *scan)
{
    if ((scan == NULL) || (s_ready == 0U)) {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    memset(scan, 0, sizeof(*scan));

    for (uint32_t sector = 0U;
         sector < G_CAL_STORE_SECTOR_COUNT;
         ++sector) {
        BSP_StorageStatus status = BSP_QSPI_Flash_Read(
            s_sector_address[sector],
            s_sector_buffer,
            sizeof(s_sector_buffer));
        if (status != BSP_STORAGE_OK) return status;

        scan->next_slot[sector] = 0U;
        for (uint32_t slot = 0U;
             slot < G_CAL_STORE_SLOT_COUNT;
             ++slot) {
            const uint8_t *raw =
                &s_sector_buffer[slot * G_CAL_STORE_RECORD_SIZE];
            G_CalibrationRecord record;

            if (G_CalibrationStore_IsBlank(
                    raw, G_CAL_STORE_RECORD_SIZE) == 0U) {
                scan->next_slot[sector] = slot + 1U;
            }
            memcpy(&record, raw, sizeof(record));
            if (G_CalibrationStore_IsRecordValid(&record) != 0U) {
                if ((scan->has_valid == 0U) ||
                    (G_CalibrationStore_SequenceIsNewer(
                         record.sequence,
                         scan->latest_sequence) != 0U)) {
                    scan->has_valid = 1U;
                    scan->latest_sector = (uint8_t)sector;
                    scan->latest_sequence = record.sequence;
                    scan->latest_data = record.data;
                }
            }
        }
    }
    return BSP_STORAGE_OK;
}

BSP_StorageStatus G_CalibrationStore_Init(G_CalibrationData *data,
                                           uint8_t *loaded)
{
    const BSP_QSPI_FlashInfo *info;
    G_CalibrationScan scan;
    BSP_StorageStatus status;

    if ((data == NULL) || (loaded == NULL)) {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    *loaded = 0U;
    s_ready = 0U;

    status = BSP_QSPI_Flash_Init();
    if (status != BSP_STORAGE_OK) return status;
    info = BSP_QSPI_Flash_GetInfo();
    if ((info == NULL) ||
        (info->capacity_bytes <
         G_CAL_STORE_SECTOR_COUNT * BSP_QSPI_FLASH_SECTOR_SIZE)) {
        return BSP_STORAGE_UNSUPPORTED;
    }

    s_sector_address[0] = info->capacity_bytes -
        G_CAL_STORE_SECTOR_COUNT * BSP_QSPI_FLASH_SECTOR_SIZE;
    s_sector_address[1] = info->capacity_bytes -
        BSP_QSPI_FLASH_SECTOR_SIZE;
    s_ready = 1U;

    status = G_CalibrationStore_Scan(&scan);
    if (status != BSP_STORAGE_OK) {
        s_ready = 0U;
        return status;
    }
    if (scan.has_valid != 0U) {
        *data = scan.latest_data;
        *loaded = 1U;
    }
    return BSP_STORAGE_OK;
}

BSP_StorageStatus G_CalibrationStore_Save(const G_CalibrationData *data)
{
    G_CalibrationScan scan;
    G_CalibrationRecord record;
    G_CalibrationRecord verify;
    BSP_StorageStatus status;
    uint32_t target_sector;
    uint32_t target_slot;
    uint32_t target_address;

    if ((data == NULL) || (s_ready == 0U)) {
        return BSP_STORAGE_INVALID_ARGUMENT;
    }
    status = G_CalibrationStore_Scan(&scan);
    if (status != BSP_STORAGE_OK) return status;

    if (scan.has_valid != 0U) {
        target_sector = scan.latest_sector;
        if (scan.next_slot[target_sector] >= G_CAL_STORE_SLOT_COUNT) {
            target_sector = 1U - target_sector;
        }
    } else {
        target_sector =
            (scan.next_slot[0] < G_CAL_STORE_SLOT_COUNT) ? 0U : 1U;
    }

    if (scan.next_slot[target_sector] >= G_CAL_STORE_SLOT_COUNT) {
        status = BSP_QSPI_Flash_EraseSector(
            s_sector_address[target_sector]);
        if (status != BSP_STORAGE_OK) return status;
        target_slot = 0U;
    } else {
        target_slot = scan.next_slot[target_sector];
    }

    memset(&record, 0, sizeof(record));
    record.magic = G_CAL_STORE_MAGIC;
    record.version = G_CAL_STORE_VERSION;
    record.sequence = (scan.has_valid != 0U) ?
                      (scan.latest_sequence + 1U) : 1U;
    record.data = *data;
    record.crc32 = G_CalibrationStore_Crc32(
        &record,
        (uint32_t)offsetof(G_CalibrationRecord, crc32));

    target_address = s_sector_address[target_sector] +
                     target_slot * G_CAL_STORE_RECORD_SIZE;
    status = BSP_QSPI_Flash_Program(
        target_address, &record, sizeof(record));
    if (status != BSP_STORAGE_OK) return status;
    status = BSP_QSPI_Flash_Read(
        target_address, &verify, sizeof(verify));
    if (status != BSP_STORAGE_OK) return status;
    if ((G_CalibrationStore_IsRecordValid(&verify) == 0U) ||
        (memcmp(&verify, &record, sizeof(record)) != 0)) {
        return BSP_STORAGE_VERIFY_FAILED;
    }
    return BSP_STORAGE_OK;
}

uint8_t G_CalibrationStore_IsReady(void)
{
    return s_ready;
}
