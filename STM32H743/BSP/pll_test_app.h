#ifndef PLL_TEST_APP_H
#define PLL_TEST_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

HAL_StatusTypeDef PllTest_Init(void);
void PllTest_Process(void);
void PllTest_Stop(void);

#ifdef __cplusplus
}
#endif
#endif /* PLL_TEST_APP_H */
