#include "PE4302.h"
#include <stddef.h>

/* 检查PE4302接口配置是否完整。 */
static bool PE4302_ConfigIsValid(const PE4302_ConfigTypeDef *cfg);

/* 产生满足PE4302时序的短延时。 */
static void PE4302_Delay(void);

/* 使能指定GPIO端口时钟。 */
static void PE4302_EnableGpioClk(GPIO_TypeDef *port);

/************************************************************
 * Function :       PE4302_GetDefaultConfig
 * Comment  :       获取PE4302默认驱动配置参数
 * Parameter:       cfg: PE4302配置结构体指针
 * Return   :       null
 * Date     :       2026-07-13 V1
************************************************************/
void PE4302_GetDefaultConfig(PE4302_ConfigTypeDef *cfg)
{
    if (cfg == NULL)
    {
        return;
    }

    cfg->le_port   = NULL;
    cfg->le_pin    = 0U;
    cfg->clk_port  = NULL;
    cfg->clk_pin   = 0U;
    cfg->data_port = NULL;
    cfg->data_pin  = 0U;
}

/************************************************************
 * Function :       PE4302_Init
 * Comment  :       初始化PE4302驱动句柄, 配置GPIO空闲电平
 * Parameter:       dev: PE4302驱动句柄; cfg: PE4302配置参数
 * Return   :       PE4302_OK表示成功, 其他值表示参数错误
 * Date     :       2026-07-13 V1
************************************************************/
PE4302_StatusTypeDef PE4302_Init(PE4302_HandleTypeDef *dev,
                                 const PE4302_ConfigTypeDef *cfg)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if ((dev == NULL) || !PE4302_ConfigIsValid(cfg))
    {
        return PE4302_ERROR_PARAM;
    }

    dev->cfg = *cfg;
    dev->last_code = 0U;

    /* 使能各端口时钟 */
    PE4302_EnableGpioClk(dev->cfg.le_port);
    PE4302_EnableGpioClk(dev->cfg.clk_port);
    PE4302_EnableGpioClk(dev->cfg.data_port);

    /* 配置LE引脚: 推挽输出 */
    GPIO_InitStruct.Pin   = dev->cfg.le_pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(dev->cfg.le_port, &GPIO_InitStruct);

    /* 配置CLK引脚: 推挽输出 */
    GPIO_InitStruct.Pin   = dev->cfg.clk_pin;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(dev->cfg.clk_port, &GPIO_InitStruct);

    /* 配置DATA引脚: 推挽输出 */
    GPIO_InitStruct.Pin   = dev->cfg.data_pin;
    HAL_GPIO_Init(dev->cfg.data_port, &GPIO_InitStruct);

    /* 串行模式空闲状态：LE=LOW、CLK=LOW、DATA=LOW。 */
    HAL_GPIO_WritePin(dev->cfg.le_port,   dev->cfg.le_pin,   GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dev->cfg.clk_port,  dev->cfg.clk_pin,  GPIO_PIN_RESET);
    HAL_GPIO_WritePin(dev->cfg.data_port, dev->cfg.data_pin, GPIO_PIN_RESET);

    return PE4302_OK;
}

/************************************************************
 * Function :       PE4302_WriteCode
 * Comment  :       向PE4302写入6位衰减控制码 (串行, MSB先发)
 *                   控制码范围: 0~63, 对应衰减: 0~31.5dB
 * Parameter:       dev: PE4302驱动句柄; code: 6位衰减码
 * Return   :       PE4302_OK表示成功
 * Date     :       2026-07-13 V1
 *
 *       时序：LE保持低电平移入C16至C0.5，最后给一个高脉冲完成锁存。
************************************************************/
PE4302_StatusTypeDef PE4302_WriteCode(PE4302_HandleTypeDef *dev,
                                      uint8_t code)
{
    uint8_t i;

    if (dev == NULL)
    {
        return PE4302_ERROR_PARAM;
    }

    code &= PE4302_CODE_MAX;  /* 只取低6位。 */

    /* LE保持低电平，避免移位中的中间码进入衰减网络。 */
    HAL_GPIO_WritePin(dev->cfg.le_port, dev->cfg.le_pin, GPIO_PIN_RESET);
    PE4302_Delay();

    /* 串行移入6位控制字, MSB优先 */
    for (i = 0; i < PE4302_BIT_COUNT; i++)
    {
        /* DATA = 当前最高位 (C16 → C8 → ... → C0.5) */
        if ((code >> (PE4302_BIT_COUNT - 1U - i)) & 0x01U)
        {
            HAL_GPIO_WritePin(dev->cfg.data_port, dev->cfg.data_pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(dev->cfg.data_port, dev->cfg.data_pin, GPIO_PIN_RESET);
        }
        PE4302_Delay();

        /* CLK上升沿 → 移位寄存器采样DATA */
        HAL_GPIO_WritePin(dev->cfg.clk_port, dev->cfg.clk_pin, GPIO_PIN_SET);
        PE4302_Delay();
        HAL_GPIO_WritePin(dev->cfg.clk_port, dev->cfg.clk_pin, GPIO_PIN_RESET);
        PE4302_Delay();
    }

    /* LE高脉冲将完整6位控制字一次性锁存到衰减网络。 */
    HAL_GPIO_WritePin(dev->cfg.le_port, dev->cfg.le_pin, GPIO_PIN_SET);
    PE4302_Delay();
    HAL_GPIO_WritePin(dev->cfg.le_port, dev->cfg.le_pin, GPIO_PIN_RESET);
    PE4302_Delay();

    /* 清理: DATA=LOW */
    HAL_GPIO_WritePin(dev->cfg.data_port, dev->cfg.data_pin, GPIO_PIN_RESET);

    dev->last_code = code;

    return PE4302_OK;
}

/************************************************************
 * Function :       PE4302_SetAttenDb
 * Comment  :       按dB值设置衰减量
 * Parameter:       dev: PE4302驱动句柄; db: 目标衰减 (0~31.5dB)
 * Return   :       PE4302_OK表示成功
 * Date     :       2026-07-13 V1
************************************************************/
PE4302_StatusTypeDef PE4302_SetAttenDb(PE4302_HandleTypeDef *dev,
                                       float db)
{
    uint8_t code;

    if (dev == NULL)
    {
        return PE4302_ERROR_PARAM;
    }

    code = PE4302_DbToCode(db);

    return PE4302_WriteCode(dev, code);
}

/************************************************************
 * Function :       PE4302_SetAttenStep
 * Comment  :       按半dB步进值设置衰减量 (0~63对应0~31.5dB)
 * Parameter:       dev: PE4302驱动句柄; half_db_step: 半dB步进值
 * Return   :       PE4302_OK表示成功
 * Date     :       2026-07-13 V1
************************************************************/
PE4302_StatusTypeDef PE4302_SetAttenStep(PE4302_HandleTypeDef *dev,
                                         uint8_t half_db_step)
{
    if (dev == NULL)
    {
        return PE4302_ERROR_PARAM;
    }

    if (half_db_step > PE4302_CODE_MAX)
    {
        half_db_step = PE4302_CODE_MAX;
    }

    return PE4302_WriteCode(dev, half_db_step);
}

/************************************************************
 * Function :       PE4302_CodeToDb
 * Comment  :       将6位衰减码转换为dB值
 * Parameter:       code: 6位衰减码 (0~63)
 * Return   :       对应的衰减dB值 (0~31.5)
 * Date     :       2026-07-13 V1
************************************************************/
float PE4302_CodeToDb(uint8_t code)
{
    code &= PE4302_CODE_MAX;
    return (float)code * PE4302_ATTEN_STEP_DB;
}

/************************************************************
 * Function :       PE4302_DbToCode
 * Comment  :       将dB衰减值转换为最近的6位衰减码
 * Parameter:       db: 目标衰减值 (dB)
 * Return   :       6位衰减码 (0~63)
 * Date     :       2026-07-13 V1
************************************************************/
uint8_t PE4302_DbToCode(float db)
{
    int32_t code;

    if (db <= PE4302_ATTEN_MIN_DB)
    {
        return 0U;
    }

    if (db >= PE4302_ATTEN_MAX_DB)
    {
        return PE4302_CODE_MAX;
    }

    /* 四舍五入到最近的0.5dB步进 */
    code = (int32_t)(db / PE4302_ATTEN_STEP_DB + 0.5f);

    if (code < 0)
    {
        return 0U;
    }

    if (code > (int32_t)PE4302_CODE_MAX)
    {
        return PE4302_CODE_MAX;
    }

    return (uint8_t)code;
}

/************************************************************
 * Function :       PE4302_ConfigIsValid
 * Comment  :       校验配置参数有效性
 * Parameter:       cfg: PE4302配置结构体指针
 * Return   :       true=有效, false=无效
************************************************************/
static bool PE4302_ConfigIsValid(const PE4302_ConfigTypeDef *cfg)
{
    if (cfg == NULL)
    {
        return false;
    }

    if ((cfg->le_port   == NULL) || (cfg->le_pin   == 0U) ||
        (cfg->clk_port  == NULL) || (cfg->clk_pin  == 0U) ||
        (cfg->data_port == NULL) || (cfg->data_pin == 0U))
    {
        return false;
    }

    return true;
}

/************************************************************
 * Function :       PE4302_EnableGpioClk
 * Comment  :       使能指定GPIO端口的时钟
 * Parameter:       port: GPIO端口号
 * Return   :       null
************************************************************/
static void PE4302_EnableGpioClk(GPIO_TypeDef *port)
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    else if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    else if (port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    else if (port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    else if (port == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
}

/************************************************************
 * Function :       PE4302_Delay
 * Comment  :       软件时序延时 (~100ns @ 480MHz)
 *                   PE4302最大串行时钟为10MHz, 延时足够
************************************************************/
static void PE4302_Delay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 10U; i++)
    {
        __NOP();
    }
}
