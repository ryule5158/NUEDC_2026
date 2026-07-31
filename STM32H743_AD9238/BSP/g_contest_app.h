#ifndef G_CONTEST_APP_H
#define G_CONTEST_APP_H

#include "main.h"
#include "periodic_analyzer.h"
#include <stdint.h>

HAL_StatusTypeDef GContest_Init(void);
void GContest_Process(uint8_t waveform_cycles);
/*
 * 输出最近一次成功结果的完整串口助手报告。主循环在屏幕发布完成后
 * 调用，使大量 printf 格式化和USART1线速都不处于屏幕关键路径。
 */
void GContest_PrintLastReport(uint8_t waveform_cycles);
const PeriodicAnalyzer_Result *GContest_GetLastResult(void);
const float *GContest_GetLastSamples(void);
uint32_t GContest_GetFrameNumber(void);
uint32_t GContest_GetLastLatencyMs(void);

#endif /* G_CONTEST_APP_H */
