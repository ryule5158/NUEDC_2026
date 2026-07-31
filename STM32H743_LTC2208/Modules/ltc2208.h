#ifndef __LTC2208_H
#define __LTC2208_H /* LTC2208底层驱动头文件包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "fpga_link.h"
#include <stdint.h>

#define LTC2208_SAMPLE_CLOCK_HZ       130000000UL /* 本工程实际ADC采样时钟，单位Hz。 */
#define LTC2208_BUFFER_MAX_SAMPLES    4096U      /* FPGA块RAM最多保存的采样点数。 */
#define LTC2208_DECIMATION_MIN        1U         /* FPGA最小抽取倍数。 */
#define LTC2208_DECIMATION_MAX        65535U     /* FPGA最大抽取倍数。 */
#define LTC2208_READY_TIMEOUT_MS      100U       /* 等待ADC采样时钟锁定的超时时间。 */
#define LTC2208_MODULE_INPUT_MIN_V    (-1.125f)  /* ADC差分负满量程标称值。 */
#define LTC2208_MODULE_INPUT_MAX_V    1.125f     /* ADC差分正满量程标称值。 */
#define LTC2208_IDEAL_VOLTS_PER_CODE  (2.25f / 65536.0f)
#define LTC2208_CAPTURE_PROTOCOL_Q3_VERSION   0x00030002UL
#define LTC2208_NATIVE_RESOLUTION_BITS 16U
#define LTC2208_NATIVE_MAX_CODE        65535U
#define LTC2208_SAMPLE_WORD_BITS       20U
#define LTC2208_SAMPLE_FRACTION_BITS   3U

/*
 * FPGA capture samples are signed two's-complement 20-bit Q3 values carried
 * in int32_t.  One physical LTC2208 code is eight transport counts, retaining
 * the fractional precision produced by the CIC5/FIR81 path.  The remaining
 * capture statistics registers intentionally stay in raw signed-S16 units.
 */
typedef int32_t LTC2208_SampleTypeDef;

/* LTC2208底层驱动返回状态。 */
typedef enum
{
  LTC2208_OK = 0,          /* 操作成功。 */
  LTC2208_ERROR_PARAM,     /* 输入参数无效。 */
  LTC2208_ERROR_NOT_INIT,  /* 驱动尚未初始化。 */
  LTC2208_ERROR_LINK,      /* STM32与FPGA通信失败。 */
  LTC2208_ERROR_DEVICE,    /* FPGA采集固件标识或版本不匹配。 */
  LTC2208_ERROR_FORMAT,    /* FPGA sample protocol is not signed-20 Q3. */
  LTC2208_ERROR_NOT_READY, /* FPGA的130MHz采样时钟未锁定。 */
  LTC2208_ERROR_BUSY,      /* FPGA正在执行上一次采集。 */
  LTC2208_ERROR_TIMEOUT,   /* 等待触发或采集完成超时。 */
  LTC2208_ERROR_NO_DATA    /* FPGA中没有可读取的完整采样块。 */
} LTC2208_StatusTypeDef;

/* FPGA采集触发模式。 */
typedef enum
{
  LTC2208_TRIGGER_IMMEDIATE = 0U, /* 启动后立即采集。 */
  LTC2208_TRIGGER_RISING,          /* 输入码跨过阈值的上升沿触发。 */
  LTC2208_TRIGGER_FALLING,         /* 输入码跨过阈值的下降沿触发。 */
  LTC2208_TRIGGER_EITHER           /* 输入码跨过阈值的任意边沿触发。 */
} LTC2208_TriggerTypeDef;

/* 一次FPGA高速采集的配置。 */
typedef struct
{
  uint16_t sample_count;              /* 保存点数，范围1~4096。 */
  uint16_t decimation;                /* 抽取倍数，范围1~65535。 */
  LTC2208_TriggerTypeDef trigger_mode; /* 触发模式。 */
  uint16_t trigger_threshold;          /* 16位offset-binary原始码触发阈值。 */
} LTC2208_CaptureConfigTypeDef;

/* FPGA中最近一次采集的状态与统计结果。 */
typedef struct
{
  uint8_t adc_ready;             /* 130MHz采样时钟锁定标志。 */
  uint8_t busy;                  /* 正在等待触发或采集标志。 */
  uint8_t done;                  /* 完整采样块可读取标志。 */
  uint8_t triggered;             /* 已发生触发标志。 */
  uint8_t waiting_trigger;       /* 正在等待阈值触发标志。 */
  uint8_t overrange;             /* 采集期间出现LTC2208越界标志。 */
  int16_t min_sample_s16;          /* Minimum raw signed-S16 statistic. */
  int16_t max_sample_s16;          /* Maximum raw signed-S16 statistic. */
  int16_t latest_sample_s16;       /* Latest centered raw ADC sample. */
  uint16_t captured_count;       /* 实际保存的样本点数。 */
  uint32_t overrange_count;      /* 采集窗口内OTR有效的物理时钟数。 */
  int32_t sample_sum_s16;        /* Signed sum in raw signed-S16 units. */
  float mean_sample_s16;         /* Mean in raw signed-S16 units. */
  float stored_sample_rate_hz;   /* 抽取后的缓存采样率，单位Hz。 */
} LTC2208_CaptureInfoTypeDef;

/* LTC2208底层驱动运行数据。 */
typedef struct
{
  uint8_t initialized;                 /* 底层驱动初始化标志。 */
  uint32_t device_id;                  /* FPGA采集单元设备标识。 */
  uint32_t firmware_version;           /* FPGA采集协议版本。 */
  uint32_t sample_clock_hz;            /* ADC物理采样时钟，单位Hz。 */
  uint8_t sample_word_bits;            /* SPI sample word width. */
  uint8_t sample_fraction_bits;        /* Signed fixed-point fraction width. */
  LTC2208_CaptureConfigTypeDef config;  /* 最近一次采集配置。 */
  LTC2208_CaptureInfoTypeDef capture;   /* 最近一次采集状态。 */
  LTC2208_StatusTypeDef status;         /* 最近一次底层操作状态。 */
  uint16_t read_progress; /* Samples copied before a read error. */
} LTC2208_DataTypeDef;

/* 填充一次立即触发采集的默认配置。 */
void LTC2208_GetDefaultCaptureConfig(LTC2208_CaptureConfigTypeDef *config);

/* 初始化并核验STM32、FPGA和LTC2208完整链路。 */
LTC2208_StatusTypeDef LTC2208_Init(SPI_HandleTypeDef *hspi);

/* 启动一次由FPGA完成的高速采集。 */
LTC2208_StatusTypeDef LTC2208_StartCapture(
    const LTC2208_CaptureConfigTypeDef *config);

/* 等待触发和采集完成，超时后保持当前状态供调用者决定是否终止。 */
LTC2208_StatusTypeDef LTC2208_WaitCapture(uint32_t timeout_ms);

/* 从FPGA块RAM读取最近一次完整采样块。 */
LTC2208_StatusTypeDef LTC2208_ReadCapture(LTC2208_SampleTypeDef *samples,
                                        uint16_t capacity,
                                        uint16_t *sample_count);

/* 读取FPGA采集状态和统计结果。 */
LTC2208_StatusTypeDef LTC2208_GetCaptureInfo(LTC2208_CaptureInfoTypeDef *info);

/* 终止正在等待触发或正在进行的采集。 */
LTC2208_StatusTypeDef LTC2208_AbortCapture(void);
/* Enable FPGA CIC5/R65 plus FIR81 anti-interference path. */
LTC2208_StatusTypeDef LTC2208_SetAntiInterferenceFilter(uint8_t enable);



/* 获取LTC2208底层运行数据，只读使用。 */
const LTC2208_DataTypeDef *LTC2208_GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* __LTC2208_H */
