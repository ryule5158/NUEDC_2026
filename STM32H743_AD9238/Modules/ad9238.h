#ifndef __AD9238_H
#define __AD9238_H /* AD9238底层驱动头文件包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "fpga_link.h"
#include <stdint.h>

#define AD9238_SAMPLE_CLOCK_HZ       50000000UL /* AD9238额定最高采样时钟，单位Hz。 */
#define AD9238_BUFFER_MAX_SAMPLES    4096U      /* FPGA块RAM最多保存的采样点数。 */
#define AD9238_DECIMATION_MIN        1U         /* FPGA最小抽取倍数。 */
#define AD9238_DECIMATION_MAX        65535U     /* FPGA最大抽取倍数。 */
#define AD9238_READY_TIMEOUT_MS      100U       /* 等待ADC采样时钟锁定的超时时间。 */
#define AD9238_MODULE_INPUT_MIN_V    (-5.0f)    /* 高速AD/DA模块标称最小输入电压。 */
#define AD9238_MODULE_INPUT_MAX_V    5.0f       /* 高速AD/DA模块标称最大输入电压。 */
#define AD9238_IDEAL_VOLTS_PER_CODE  (10.0f / 4096.0f) /* 模块理想每码输入电压。 */
#define AD9238_CAPTURE_PROTOCOL_Q8_8_VERSION 0x00020002UL
#define AD9238_NATIVE_RESOLUTION_BITS 12U
#define AD9238_NATIVE_MAX_CODE        4095U
#define AD9238_SAMPLE_WORD_BITS       16U
#define AD9238_SAMPLE_FRACTION_BITS   8U

/*
 * FPGA capture samples are signed two's-complement Q8.8 values.
 * One native 12-bit ADC code is represented by 0x0010.
 */
typedef int16_t AD9238_SampleTypeDef;

/* AD9238底层驱动返回状态。 */
typedef enum
{
  AD9238_OK = 0,          /* 操作成功。 */
  AD9238_ERROR_PARAM,     /* 输入参数无效。 */
  AD9238_ERROR_NOT_INIT,  /* 驱动尚未初始化。 */
  AD9238_ERROR_LINK,      /* STM32与FPGA通信失败。 */
  AD9238_ERROR_DEVICE,    /* FPGA采集固件标识或版本不匹配。 */
  AD9238_ERROR_FORMAT,    /* FPGA sample protocol is not signed 16-bit Q8.8. */
  AD9238_ERROR_NOT_READY, /* FPGA的50MHz采样时钟未锁定。 */
  AD9238_ERROR_BUSY,      /* FPGA正在执行上一次采集。 */
  AD9238_ERROR_TIMEOUT,   /* 等待触发或采集完成超时。 */
  AD9238_ERROR_NO_DATA    /* FPGA中没有可读取的完整采样块。 */
} AD9238_StatusTypeDef;

/* FPGA采集触发模式。 */
typedef enum
{
  AD9238_TRIGGER_IMMEDIATE = 0U, /* 启动后立即采集。 */
  AD9238_TRIGGER_RISING,          /* 输入码跨过阈值的上升沿触发。 */
  AD9238_TRIGGER_FALLING,         /* 输入码跨过阈值的下降沿触发。 */
  AD9238_TRIGGER_EITHER           /* 输入码跨过阈值的任意边沿触发。 */
} AD9238_TriggerTypeDef;

/* 一次FPGA高速采集的配置。 */
typedef struct
{
  uint16_t sample_count;              /* 保存点数，范围1~4096。 */
  uint16_t decimation;                /* 抽取倍数，范围1~65535。 */
  AD9238_TriggerTypeDef trigger_mode; /* 触发模式。 */
  uint16_t trigger_threshold;          /* 12位原始码触发阈值。 */
} AD9238_CaptureConfigTypeDef;

/* FPGA中最近一次采集的状态与统计结果。 */
typedef struct
{
  uint8_t adc_ready;             /* 50MHz采样时钟锁定标志。 */
  uint8_t busy;                  /* 正在等待触发或采集标志。 */
  uint8_t done;                  /* 完整采样块可读取标志。 */
  uint8_t triggered;             /* 已发生触发标志。 */
  uint8_t waiting_trigger;       /* 正在等待阈值触发标志。 */
  uint8_t overrange;             /* 采集期间出现AD9238越界标志。 */
  AD9238_SampleTypeDef min_sample_q8_8; /* Minimum stored signed Q8.8 sample. */
  AD9238_SampleTypeDef max_sample_q8_8; /* Maximum stored signed Q8.8 sample. */
  AD9238_SampleTypeDef latest_sample_q8_8; /* Latest centered ADC sample. */
  uint16_t captured_count;       /* 实际保存的样本点数。 */
  uint32_t overrange_count;      /* 采集窗口内OTR有效的物理时钟数。 */
  int32_t sample_sum_q8_8;       /* Signed sum of stored Q8.8 samples. */
  float mean_sample_q8_8;        /* Mean in Q8.8 integer-LSB units. */
  float stored_sample_rate_hz;   /* 抽取后的缓存采样率，单位Hz。 */
} AD9238_CaptureInfoTypeDef;

/* AD9238底层驱动运行数据。 */
typedef struct
{
  uint8_t initialized;                 /* 底层驱动初始化标志。 */
  uint32_t device_id;                  /* FPGA采集单元设备标识。 */
  uint32_t firmware_version;           /* FPGA采集协议版本。 */
  uint32_t sample_clock_hz;            /* ADC物理采样时钟，单位Hz。 */
  uint8_t sample_word_bits;            /* SPI sample word width. */
  uint8_t sample_fraction_bits;        /* Signed fixed-point fraction width. */
  AD9238_CaptureConfigTypeDef config;  /* 最近一次采集配置。 */
  AD9238_CaptureInfoTypeDef capture;   /* 最近一次采集状态。 */
  AD9238_StatusTypeDef status;         /* 最近一次底层操作状态。 */
  uint16_t read_progress; /* Samples copied before a read error. */
} AD9238_DataTypeDef;

/* 填充一次立即触发采集的默认配置。 */
void AD9238_GetDefaultCaptureConfig(AD9238_CaptureConfigTypeDef *config);

/* 初始化并核验STM32、FPGA和AD9238完整链路。 */
AD9238_StatusTypeDef AD9238_Init(SPI_HandleTypeDef *hspi);

/* 启动一次由FPGA完成的高速采集。 */
AD9238_StatusTypeDef AD9238_StartCapture(
    const AD9238_CaptureConfigTypeDef *config);

/* 等待触发和采集完成，超时后保持当前状态供调用者决定是否终止。 */
AD9238_StatusTypeDef AD9238_WaitCapture(uint32_t timeout_ms);

/* 从FPGA块RAM读取最近一次完整采样块。 */
AD9238_StatusTypeDef AD9238_ReadCapture(AD9238_SampleTypeDef *samples,
                                        uint16_t capacity,
                                        uint16_t *sample_count);

/* 读取FPGA采集状态和统计结果。 */
AD9238_StatusTypeDef AD9238_GetCaptureInfo(AD9238_CaptureInfoTypeDef *info);

/* 终止正在等待触发或正在进行的采集。 */
AD9238_StatusTypeDef AD9238_AbortCapture(void);
/* Enable FPGA FIR307 plus CIC3/R25 anti-aliasing (protocol v2.2+ only). */
AD9238_StatusTypeDef AD9238_SetCic3Filter(uint8_t enable);



/* 获取AD9238底层运行数据，只读使用。 */
const AD9238_DataTypeDef *AD9238_GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* __AD9238_H */
