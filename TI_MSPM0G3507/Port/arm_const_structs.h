#ifndef ARM_CONST_STRUCTS_H
#define ARM_CONST_STRUCTS_H

#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CMSIS-DSP兼容的固定FFT长度描述符。 */
extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len64;
extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len128;
extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len256;
extern const arm_cfft_instance_f32 arm_cfft_sR_f32_len512;

#ifdef __cplusplus
}
#endif

#endif /* ARM_CONST_STRUCTS_H */
