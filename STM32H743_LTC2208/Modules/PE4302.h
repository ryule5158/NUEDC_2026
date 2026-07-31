#ifndef PE4302_H
#define PE4302_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* PE4302衰减器参数。 */
#define PE4302_ATTEN_MIN_DB         0.0f  /* 最小衰减，单位dB。 */
#define PE4302_ATTEN_MAX_DB         31.5f /* 最大衰减，单位dB。 */
#define PE4302_ATTEN_STEP_DB        0.5f  /* 衰减分辨率，单位dB。 */
#define PE4302_CODE_MAX             63U   /* 最大6位衰减码。 */
#define PE4302_BIT_COUNT             6U   /* 串行控制字位数。 */

/* PE4302驱动状态。 */
typedef enum
{
    PE4302_OK = 0,       /* 操作成功。 */
    PE4302_ERROR,        /* 器件操作失败。 */
    PE4302_ERROR_PARAM   /* 输入参数无效。 */
} PE4302_StatusTypeDef;

/* PE4302三线串行接口配置。 */
typedef struct
{
    GPIO_TypeDef *le_port;   /* LE锁存使能端口。 */
    uint16_t le_pin;         /* LE锁存使能引脚。 */
    GPIO_TypeDef *clk_port;  /* CLK串行时钟端口。 */
    uint16_t clk_pin;        /* CLK串行时钟引脚。 */
    GPIO_TypeDef *data_port; /* DATA串行数据端口。 */
    uint16_t data_pin;       /* DATA串行数据引脚。 */
} PE4302_ConfigTypeDef;

/* PE4302驱动句柄。 */
typedef struct
{
    PE4302_ConfigTypeDef cfg; /* 当前接口配置。 */
    uint8_t last_code;       /* 上次写入的6位衰减码。 */
} PE4302_HandleTypeDef;

/* 获取空引脚的默认接口配置。 */
void PE4302_GetDefaultConfig(PE4302_ConfigTypeDef *cfg);

/* 初始化PE4302三线串行接口。 */
PE4302_StatusTypeDef PE4302_Init(PE4302_HandleTypeDef *dev,
                                 const PE4302_ConfigTypeDef *cfg);

/* 写入0～63的6位衰减码。 */
PE4302_StatusTypeDef PE4302_WriteCode(PE4302_HandleTypeDef *dev,
                                      uint8_t code);

/* 按dB设置0～31.5dB衰减。 */
PE4302_StatusTypeDef PE4302_SetAttenDb(PE4302_HandleTypeDef *dev,
                                       float db);

/* 按0.5dB档位设置衰减。 */
PE4302_StatusTypeDef PE4302_SetAttenStep(PE4302_HandleTypeDef *dev,
                                         uint8_t half_db_step);

/* 将6位衰减码转换为dB。 */
float PE4302_CodeToDb(uint8_t code);

/* 将dB转换为最接近的6位衰减码。 */
uint8_t PE4302_DbToCode(float db);

#ifdef __cplusplus
}
#endif

#endif /* PE4302_H */
