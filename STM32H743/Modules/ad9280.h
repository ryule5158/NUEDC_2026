#ifndef __AD9280_H
#define __AD9280_H /* AD9280底层驱动头文件包含保护。 */

#ifdef __cplusplus
extern "C" {
#endif

#include "fpga_link.h"
#include <stdint.h>

#define AD9280_SAMPLE_CLOCK_HZ       32000000UL /* AD9280额定最高采样时钟，单位Hz。 */
#define AD9280_BUFFER_MAX_SAMPLES    4096U      /* FPGA块RAM最多保存的采样点数。 */
#define AD9280_DECIMATION_MIN        1U         /* FPGA最小抽取倍数。 */
#define AD9280_DECIMATION_MAX        65535U     /* FPGA最大抽取倍数。 */
#define AD9280_READY_TIMEOUT_MS      100U       /* 等待ADC采样时钟锁定的超时时间。 */
#define AD9280_MODULE_INPUT_MIN_V    (-5.0f)    /* 高速AD/DA模块标称最小输入电压。 */
#define AD9280_MODULE_INPUT_MAX_V    5.0f       /* 高速AD/DA模块标称最大输入电压。 */
#define AD9280_IDEAL_VOLTS_PER_CODE  (10.0f / 256.0f) /* 模块理想每码输入电压。 */

/* AD9280底层驱动返回状态。 */
typedef enum
{
  AD9280_OK = 0,          /* 操作成功。 */
  AD9280_ERROR_PARAM,     /* 输入参数无效。 */
  AD9280_ERROR_NOT_INIT,  /* 驱动尚未初始化。 */
  AD9280_ERROR_LINK,      /* STM32与FPGA通信失败。 */
  AD9280_ERROR_DEVICE,    /* FPGA采集固件标识或版本不匹配。 */
  AD9280_ERROR_NOT_READY, /* FPGA的32MHz采样时钟未锁定。 */
  AD9280_ERROR_BUSY,      /* FPGA正在执行上一次采集。 */
  AD9280_ERROR_TIMEOUT,   /* 等待触发或采集完成超时。 */
  AD9280_ERROR_NO_DATA    /* FPGA中没有可读取的完整采样块。 */
} AD9280_StatusTypeDef;

/* FPGA采集触发模式。 */
typedef enum
{
  AD9280_TRIGGER_IMMEDIATE = 0U, /* 启动后立即采集。 */
  AD9280_TRIGGER_RISING,          /* 输入码跨过阈值的上升沿触发。 */
  AD9280_TRIGGER_FALLING,         /* 输入码跨过阈值的下降沿触发。 */
  AD9280_TRIGGER_EITHER           /* 输入码跨过阈值的任意边沿触发。 */
} AD9280_TriggerTypeDef;

/* 一次FPGA高速采集的配置。 */
typedef struct
{
  uint16_t sample_count;              /* 保存点数，范围1~4096。 */
  uint16_t decimation;                /* 抽取倍数，范围1~65535。 */
  AD9280_TriggerTypeDef trigger_mode; /* 触发模式。 */
  uint8_t trigger_threshold;          /* 8位原始码触发阈值。 */
} AD9280_CaptureConfigTypeDef;

/* FPGA中最近一次采集的状态与统计结果。 */
typedef struct
{
  uint8_t adc_ready;             /* 32MHz采样时钟锁定标志。 */
  uint8_t busy;                  /* 正在等待触发或采集标志。 */
  uint8_t done;                  /* 完整采样块可读取标志。 */
  uint8_t triggered;             /* 已发生触发标志。 */
  uint8_t waiting_trigger;       /* 正在等待阈值触发标志。 */
  uint8_t overrange;             /* 采集期间出现AD9280越界标志。 */
  uint8_t min_code;              /* 已保存样本最小原始码。 */
  uint8_t max_code;              /* 已保存样本最大原始码。 */
  uint8_t latest_code;           /* 最近一个物理采样码。 */
  uint16_t captured_count;       /* 实际保存的样本点数。 */
  uint32_t overrange_count;      /* 采集窗口内OTR有效的物理时钟数。 */
  uint32_t sample_sum;           /* 已保存样本原始码累加和。 */
  float mean_code;               /* 已保存样本原始码平均值。 */
  float stored_sample_rate_hz;   /* 抽取后的缓存采样率，单位Hz。 */
} AD9280_CaptureInfoTypeDef;

/* AD9280底层驱动运行数据。 */
typedef struct
{
  uint8_t initialized;                 /* 底层驱动初始化标志。 */
  uint32_t device_id;                  /* FPGA采集单元设备标识。 */
  uint32_t firmware_version;           /* FPGA采集协议版本。 */
  uint32_t sample_clock_hz;            /* ADC物理采样时钟，单位Hz。 */
  AD9280_CaptureConfigTypeDef config;  /* 最近一次采集配置。 */
  AD9280_CaptureInfoTypeDef capture;   /* 最近一次采集状态。 */
  AD9280_StatusTypeDef status;         /* 最近一次底层操作状态。 */
} AD9280_DataTypeDef;

/* 填充一次立即触发采集的默认配置。 */
void AD9280_GetDefaultCaptureConfig(AD9280_CaptureConfigTypeDef *config);

/* 初始化并核验STM32、FPGA和AD9280完整链路。 */
AD9280_StatusTypeDef AD9280_Init(SPI_HandleTypeDef *hspi);

/* 启动一次由FPGA完成的高速采集。 */
AD9280_StatusTypeDef AD9280_StartCapture(
    const AD9280_CaptureConfigTypeDef *config);

/* 等待触发和采集完成，超时后保持当前状态供调用者决定是否终止。 */
AD9280_StatusTypeDef AD9280_WaitCapture(uint32_t timeout_ms);

/* 从FPGA块RAM读取最近一次完整采样块。 */
AD9280_StatusTypeDef AD9280_ReadCapture(uint8_t *samples,
                                        uint16_t capacity,
                                        uint16_t *sample_count);

/* 读取FPGA采集状态和统计结果。 */
AD9280_StatusTypeDef AD9280_GetCaptureInfo(AD9280_CaptureInfoTypeDef *info);

/* 终止正在等待触发或正在进行的采集。 */
AD9280_StatusTypeDef AD9280_AbortCapture(void);
/* Enable the FPGA CIC3/R16 anti-alias decimator (firmware 1.2+ only). */
AD9280_StatusTypeDef AD9280_SetCic3Filter(uint8_t enable);



/* 获取AD9280底层运行数据，只读使用。 */
const AD9280_DataTypeDef *AD9280_GetData(void);

#ifdef __cplusplus
}
#endif

#endif /* __AD9280_H */
