#ifndef G_TJC_UI_H
#define G_TJC_UI_H

#include "main.h"
#include "g_calibration_store.h"
#include "periodic_analyzer.h"

#include <stdint.h>

typedef enum {
    G_TJC_MODE_WAVE_1T = 1,
    G_TJC_MODE_WAVE_3T = 3,
    G_TJC_MODE_SPECTRUM = 4
} G_TjcUiMode;

typedef enum {
    G_TJC_PAGE_UNKNOWN = 0,
    G_TJC_PAGE_INDEX,
    G_TJC_PAGE_MAIN,
    G_TJC_PAGE_SET
} G_TjcUiPage;

typedef enum {
    G_TJC_CALIBRATION_NONE = 0,
    G_TJC_CALIBRATION_WRITE,
    G_TJC_CALIBRATION_RESTORE_DEFAULT
} G_TjcUiCalibrationAction;

typedef struct {
    G_TjcUiCalibrationAction action;
    G_CalibrationData data;
} G_TjcUiCalibrationRequest;

HAL_StatusTypeDef G_TjcUi_Init(void);
uint8_t G_TjcUi_IsHealthy(void);
uint8_t G_TjcUi_CanReinitialize(void);
uint8_t G_TjcUi_TakeRequest(G_TjcUiMode *mode);
uint8_t G_TjcUi_TakePageEvent(G_TjcUiPage *page);
uint8_t G_TjcUi_TakeCalibrationRequest(
    G_TjcUiCalibrationRequest *request);
uint8_t G_TjcUi_IsMainPageActive(void);

HAL_StatusTypeDef G_TjcUi_ShowIdle(void);
HAL_StatusTypeDef G_TjcUi_ShowAcquiring(G_TjcUiMode mode);
HAL_StatusTypeDef G_TjcUi_ShowError(const char *status_text);
HAL_StatusTypeDef G_TjcUi_ShowCalibration(
    const G_CalibrationData *data,
    const char *status_text);
HAL_StatusTypeDef G_TjcUi_ShowCalibrationStatus(
    const char *status_text);
HAL_StatusTypeDef G_TjcUi_Publish(
    const PeriodicAnalyzer_Result *result,
    G_TjcUiMode mode);

/*
 * HAL UART callbacks are owned by g_contest_cli.c and dispatch USART2 events
 * to these functions. They are public only for that callback dispatcher.
 */
void G_TjcUi_UartRxComplete(UART_HandleTypeDef *huart);
void G_TjcUi_UartError(UART_HandleTypeDef *huart);

#endif /* G_TJC_UI_H */
