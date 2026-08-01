#ifndef G_BODE_CALIBRATION_H
#define G_BODE_CALIBRATION_H

#include <stdint.h>

typedef enum {
    G_BODE_MODE_THRU = 0,
    G_BODE_MODE_FILTER = 1
} G_BodeMode;

void G_BodeCalibration_Init(void);
void G_BodeCalibration_Start(G_BodeMode mode);
void G_BodeCalibration_Stop(void);
void G_BodeCalibration_Process(void);
void G_BodeCalibration_CaptureOnce(G_BodeMode mode);
uint8_t G_BodeCalibration_IsActive(void);
const char *G_BodeCalibration_ModeName(G_BodeMode mode);

#endif /* G_BODE_CALIBRATION_H */
