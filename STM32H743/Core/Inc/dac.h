#ifndef __DAC_H__
#define __DAC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern DAC_HandleTypeDef hdac1;
extern DMA_HandleTypeDef hdma_dac1_ch1;

void MX_DAC1_Init(void);

#ifdef __cplusplus
}
#endif
#endif /* __DAC_H__ */
