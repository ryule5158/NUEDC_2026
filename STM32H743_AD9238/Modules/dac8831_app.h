#ifndef __DAC8831_APP_H
#define __DAC8831_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dac8831.h"

#define DAC8831_APP_DEFAULT_VREF_MV    2500.0f          /* DAC8831应用层参考电压, 单位mV */
#define DAC8831_APP_WAVE_TABLE_SIZE    16U              /* DAC8831内置波形表点数 */
#define DAC8831_APP_SAMPLE_RATE_HZ     16000U             /* DAC8831软件波形最大送点速率, 单位点/秒 */
#define DAC8831_APP_MAX_OUTPUT_HZ      (DAC8831_APP_SAMPLE_RATE_HZ / DAC8831_APP_WAVE_TABLE_SIZE) /* DAC8831默认波形表最高频率, 单位Hz */
#define DAC8831_APP_DEFAULT_OUTPUT_HZ  10U              /* DAC8831默认波形频率, 单位Hz */
#define DAC8831_APP_DEFAULT_AMPLITUDE  DAC8831_MAX_CODE /* DAC8831默认输出幅度, 65535为满幅 */

/*
 * DAC8831波形说明:
 * 本APP通过主循环DAC8831_AppProcess()按表送点生成波形, 不是DDS高速波形源。
 * 频率上限约等于DAC8831_APP_SAMPLE_RATE_HZ / table_size，默认16点表为1000Hz。
 * amplitude是16位DAC原始幅度码0~65535; 需要直接输出电压时使用毫伏输出函数。
 */

/* DAC8831应用层最近输出状态。 */
typedef struct
{
  uint16_t last_code;              /* 最近一次输出的16位DAC码值 */
  float last_target_mv;            /* 最近一次输出对应的目标电压, 单位mV */
  DAC8831_StatusTypeDef status;    /* 最近一次DAC8831操作状态 */
} DAC8831_AppDataTypeDef;

/* DAC8831内置正弦波表, 数值范围0~65535 */
extern const uint16_t DAC8831_AppSineTable[DAC8831_APP_WAVE_TABLE_SIZE];
/* DAC8831内置三角波表, 数值范围0~65535 */
extern const uint16_t DAC8831_AppTriangleTable[DAC8831_APP_WAVE_TABLE_SIZE];
/* DAC8831内置方波表, 数值范围0~65535 */
extern const uint16_t DAC8831_AppSquareTable[DAC8831_APP_WAVE_TABLE_SIZE];
/* DAC8831内置锯齿波表, 数值范围0~65535 */
extern const uint16_t DAC8831_AppSawtoothTable[DAC8831_APP_WAVE_TABLE_SIZE];
/* DAC8831用户任意波表, 用户可直接改表内数值, 范围0~65535 */
extern uint16_t DAC8831_AppArbitraryTable[DAC8831_APP_WAVE_TABLE_SIZE];

/* 初始化DAC8831应用层, 主函数先调用一次 */
DAC8831_StatusTypeDef DAC8831_AppInit(void);
/* 获取DAC8831应用层状态和最近一次输出值 */
const DAC8831_AppDataTypeDef *DAC8831_AppGetData(void);

/* 输出DAC8831原始码值, code范围0~65535 */
DAC8831_StatusTypeDef DAC8831_AppOutputRaw(uint16_t code);
/* 输出DAC8831单极性电压, millivolts单位mV */
DAC8831_StatusTypeDef DAC8831_AppOutputUnipolarMv(float millivolts);
/* 输出DAC8831双极性电压, millivolts单位mV */
DAC8831_StatusTypeDef DAC8831_AppOutputBipolarMv(float millivolts);

/* 启动DAC8831正弦波输出, output_hz范围1~1000Hz, amplitude范围0~65535 */
DAC8831_StatusTypeDef DAC8831_AppOutputSine(uint32_t output_hz,
                                            uint16_t amplitude);
/* 启动DAC8831三角波输出, output_hz范围1~1000Hz, amplitude范围0~65535 */
DAC8831_StatusTypeDef DAC8831_AppOutputTriangle(uint32_t output_hz,
                                                uint16_t amplitude);
/* 启动DAC8831方波输出, output_hz范围1~1000Hz, amplitude范围0~65535 */
DAC8831_StatusTypeDef DAC8831_AppOutputSquare(uint32_t output_hz,
                                              uint16_t amplitude);
/* 启动DAC8831锯齿波输出, output_hz范围1~1000Hz, amplitude范围0~65535 */
DAC8831_StatusTypeDef DAC8831_AppOutputSawtooth(uint32_t output_hz,
                                                uint16_t amplitude);
/* 启动DAC8831任意波输出, 最大频率约为16000/table_size Hz */
DAC8831_StatusTypeDef DAC8831_AppOutputArbitraryHz(const uint16_t *wave_table,
                                                   uint16_t table_size,
                                                   uint32_t output_hz,
                                                   uint16_t amplitude);

/* 启动DAC8831正弦波扫频, 频率范围1~1000Hz */
DAC8831_StatusTypeDef DAC8831_AppSweepSine(uint32_t low_hz,
                                           uint32_t high_hz,
                                           uint32_t frequency_step_hz,
                                           uint16_t amplitude,
                                           uint32_t dwell_ms);
/* 启动DAC8831三角波扫频, 频率范围1~1000Hz */
DAC8831_StatusTypeDef DAC8831_AppSweepTriangle(uint32_t low_hz,
                                               uint32_t high_hz,
                                               uint32_t frequency_step_hz,
                                               uint16_t amplitude,
                                               uint32_t dwell_ms);
/* 启动DAC8831方波扫频, 频率范围1~1000Hz */
DAC8831_StatusTypeDef DAC8831_AppSweepSquare(uint32_t low_hz,
                                             uint32_t high_hz,
                                             uint32_t frequency_step_hz,
                                             uint16_t amplitude,
                                             uint32_t dwell_ms);
/* 启动DAC8831锯齿波扫频, 频率范围1~1000Hz */
DAC8831_StatusTypeDef DAC8831_AppSweepSawtooth(uint32_t low_hz,
                                               uint32_t high_hz,
                                               uint32_t frequency_step_hz,
                                               uint16_t amplitude,
                                               uint32_t dwell_ms);
/* 启动DAC8831任意波扫频, 最大频率约为16000/table_size Hz */
DAC8831_StatusTypeDef DAC8831_AppSweepArbitrary(const uint16_t *wave_table,
                                                uint16_t table_size,
                                                uint32_t low_hz,
                                                uint32_t high_hz,
                                                uint32_t frequency_step_hz,
                                                uint16_t amplitude,
                                                uint32_t dwell_ms);

/* 停止DAC8831后台波形/扫频任务, 当前输出保持最后一次码值 */
void DAC8831_AppStop(void);
/* DAC8831后台处理函数, 主循环while(1)中周期调用, 软件波形和扫频依赖它持续送点 */
void DAC8831_AppProcess(void);

#ifdef __cplusplus
}
#endif

#endif /* __DAC8831_APP_H */
