#ifndef G_CONTEST_APP_H
#define G_CONTEST_APP_H

#include "main.h"
#include "periodic_analyzer.h"
#include <stdint.h>

HAL_StatusTypeDef GContest_Init(void);
void GContest_Process(void);
const PeriodicAnalyzer_Result *GContest_GetLastResult(void);
const float *GContest_GetLastSamples(void);
uint32_t GContest_GetFrameNumber(void);
uint32_t GContest_GetLastLatencyMs(void);

#endif /* G_CONTEST_APP_H */
