#ifndef G_CONTEST_CLI_H
#define G_CONTEST_CLI_H

#include "main.h"
#include <stdint.h>

/* Initialize USART1 command reception. */
HAL_StatusTypeDef GContestCli_Init(void);

/* Move received bytes from the interrupt ring into the line parser. */
void GContestCli_Poll(void);

/*
 * Return non-zero when one measurement is requested and write the number of
 * complete waveform periods that must be printed (1 or 3).
 */
uint8_t GContestCli_TakeMeasurementRequest(uint8_t *waveform_cycles);

#endif /* G_CONTEST_CLI_H */
