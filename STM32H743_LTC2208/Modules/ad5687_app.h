#ifndef __AD5687_APP_H
#define __AD5687_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ad5687.h"

#define AD5687_APP_VREF_MV            2500U             /* 测试板AD5687参考电压，单位mV */
#define AD5687_APP_GAIN               1U                /* 测试板AD5687输出增益 */
#define AD5687_APP_WAVE_TABLE_SIZE    16U               /* 内置波形表点数 */
#define AD5687_APP_SAMPLE_RATE_HZ     16000U             /* 软件波形最大送点速率，单位点/秒 */
#define AD5687_APP_MAX_OUTPUT_HZ      (AD5687_APP_SAMPLE_RATE_HZ / AD5687_APP_WAVE_TABLE_SIZE) /* 默认16点波形最高频率，单位Hz */
#define AD5687_APP_DEFAULT_OUTPUT_HZ  10U               /* 默认波形频率，单位Hz */
#define AD5687_APP_DEFAULT_AMPLITUDE  AD5687_MAX_CODE   /* 默认幅度，4095为满幅 */

/*
 * AD5687波形说明:
 * 本APP通过主循环AD5687_AppProcess()按表送点生成波形, 不是DDS高速波形源。
 * 频率上限约等于AD5687_APP_SAMPLE_RATE_HZ / table_size，默认16点表为1000Hz。
 * amplitude是12位DAC原始幅度码0~4095; 需要直接输出电压时使用AD5687_AppOutputVoltageMv()。
 */

/* AD5687应用层CV输出编号。 */
typedef enum
{
  AD5687_OUTPUT_CV1 = 0U, /* 测试板CV1输出 */
  AD5687_OUTPUT_CV2,      /* 测试板CV2输出 */
  AD5687_OUTPUT_CV3,      /* 测试板CV3输出 */
  AD5687_OUTPUT_CV4       /* 测试板CV4输出 */
} AD5687_OutputTypeDef;

/* 内置波形表和用户任意波表，数值范围均为0~4095。 */
extern const uint16_t AD5687_AppSineTable[AD5687_APP_WAVE_TABLE_SIZE];
extern const uint16_t AD5687_AppTriangleTable[AD5687_APP_WAVE_TABLE_SIZE];
extern const uint16_t AD5687_AppSquareTable[AD5687_APP_WAVE_TABLE_SIZE];
extern const uint16_t AD5687_AppSawtoothTable[AD5687_APP_WAVE_TABLE_SIZE];
extern uint16_t AD5687_AppArbitraryTable[AD5687_APP_WAVE_TABLE_SIZE];

/* 初始化两片AD5687，主函数先调用一次。 */
HAL_StatusTypeDef AD5687_AppInit(void);

/* 指定CV口输出原始码值，code范围0~4095。 */
HAL_StatusTypeDef AD5687_AppOutputRaw(AD5687_OutputTypeDef output,
                                      uint16_t code);
/* 指定CV口输出电压，voltage_mv范围0~2500mV。 */
HAL_StatusTypeDef AD5687_AppOutputVoltageMv(AD5687_OutputTypeDef output,
                                            uint32_t voltage_mv);
/* 一次设置CV1~CV4电压，各参数范围0~2500mV。 */
HAL_StatusTypeDef AD5687_AppOutputFourVoltageMv(uint32_t cv1_mv,
                                                uint32_t cv2_mv,
                                                uint32_t cv3_mv,
                                                uint32_t cv4_mv);

/* 输出常见波形，output_hz范围1~1000Hz，amplitude范围0~4095。 */
HAL_StatusTypeDef AD5687_AppOutputSine(AD5687_OutputTypeDef output,
                                       uint32_t output_hz,
                                       uint16_t amplitude);
/* 输出软件送点三角波。 */
HAL_StatusTypeDef AD5687_AppOutputTriangle(AD5687_OutputTypeDef output,
                                           uint32_t output_hz,
                                           uint16_t amplitude);
/* 输出软件送点方波。 */
HAL_StatusTypeDef AD5687_AppOutputSquare(AD5687_OutputTypeDef output,
                                         uint32_t output_hz,
                                         uint16_t amplitude);
/* 输出软件送点锯齿波。 */
HAL_StatusTypeDef AD5687_AppOutputSawtooth(AD5687_OutputTypeDef output,
                                           uint32_t output_hz,
                                           uint16_t amplitude);

/* 输出任意波，最高频率为16000/table_size Hz，幅度范围0~4095。 */
HAL_StatusTypeDef AD5687_AppOutputArbitraryHz(AD5687_OutputTypeDef output,
                                              const uint16_t *wave_table,
                                              uint16_t table_size,
                                              uint32_t output_hz,
                                              uint16_t amplitude);

/* 常见波形扫频，频率上限为1000Hz，幅度范围0~4095。 */
HAL_StatusTypeDef AD5687_AppSweepSine(AD5687_OutputTypeDef output,
                                      uint32_t low_hz,
                                      uint32_t high_hz,
                                      uint32_t frequency_step_hz,
                                      uint16_t amplitude,
                                      uint32_t dwell_ms);
/* 启动三角波往返扫频。 */
HAL_StatusTypeDef AD5687_AppSweepTriangle(AD5687_OutputTypeDef output,
                                          uint32_t low_hz,
                                          uint32_t high_hz,
                                          uint32_t frequency_step_hz,
                                          uint16_t amplitude,
                                          uint32_t dwell_ms);
/* 启动方波往返扫频。 */
HAL_StatusTypeDef AD5687_AppSweepSquare(AD5687_OutputTypeDef output,
                                        uint32_t low_hz,
                                        uint32_t high_hz,
                                        uint32_t frequency_step_hz,
                                        uint16_t amplitude,
                                        uint32_t dwell_ms);
/* 启动锯齿波往返扫频。 */
HAL_StatusTypeDef AD5687_AppSweepSawtooth(AD5687_OutputTypeDef output,
                                          uint32_t low_hz,
                                          uint32_t high_hz,
                                          uint32_t frequency_step_hz,
                                          uint16_t amplitude,
                                          uint32_t dwell_ms);

/* 任意波扫频，最高频率为16000/table_size Hz，幅度范围0~4095。 */
HAL_StatusTypeDef AD5687_AppSweepArbitrary(AD5687_OutputTypeDef output,
                                           const uint16_t *wave_table,
                                           uint16_t table_size,
                                           uint32_t low_hz,
                                           uint32_t high_hz,
                                           uint32_t frequency_step_hz,
                                           uint16_t amplitude,
                                           uint32_t dwell_ms);

/* 停止后台波形或扫频，当前CV口保持最后一次码值。 */
void AD5687_AppStop(void);
/* 放在主循环while(1)中周期调用, 软件波形和扫频依赖它持续送点。 */
void AD5687_AppProcess(void);

#ifdef __cplusplus
}
#endif

#endif /* __AD5687_APP_H */
