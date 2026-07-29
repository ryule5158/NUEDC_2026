#ifndef G_TJC_UI_H
#define G_TJC_UI_H

#include "main.h"
#include "periodic_analyzer.h"

#include <stdint.h>

typedef enum {
    G_TJC_MODE_WAVE_1T = 1,
    G_TJC_MODE_WAVE_3T = 3,
    G_TJC_MODE_SPECTRUM = 4
} G_TjcUiMode;

HAL_StatusTypeDef G_TjcUi_Init(void);
uint8_t G_TjcUi_TakeRequest(G_TjcUiMode *mode);

HAL_StatusTypeDef G_TjcUi_ShowAcquiring(G_TjcUiMode mode);
HAL_StatusTypeDef G_TjcUi_ShowError(const char *status_text);
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
