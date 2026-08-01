#ifndef G_CALIBRATION_STORE_H
#define G_CALIBRATION_STORE_H

#include "bsp_storage.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define G_CALIBRATION_POINT_COUNT 8U

typedef struct
{
    uint32_t gain_milli;
    uint32_t coefficient_u1e6[G_CALIBRATION_POINT_COUNT];
} G_CalibrationData;

/*
 * The last two 4-KB sectors of the detected W25Q device are reserved for
 * calibration records.  Records are append-only, CRC protected and written
 * with the previous valid sector left intact.
 */
BSP_StorageStatus G_CalibrationStore_Init(G_CalibrationData *data,
                                           uint8_t *loaded);
BSP_StorageStatus G_CalibrationStore_Save(const G_CalibrationData *data);
uint8_t G_CalibrationStore_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* G_CALIBRATION_STORE_H */
