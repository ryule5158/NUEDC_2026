#include "stm32h7xx_hal.h"

#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef GPIOD
#undef GPIOE
#undef GPIOH

#include "ti_msp_dl_config.h"
#include <stdbool.h>

/* 虚拟STM32引脚对应的真实TI端口和位掩码。 */
typedef struct {
    GPIO_Regs *port;
    uint32_t pin;
} NUEDC_TiPin_t;

/* 供公共驱动使用的虚拟STM32 GPIO端口。 */
GPIO_TypeDef NUEDC_STM32_GPIOA = {0U, "GPIOA"}; /* 虚拟GPIOA实例。 */
GPIO_TypeDef NUEDC_STM32_GPIOB = {1U, "GPIOB"}; /* 虚拟GPIOB实例。 */
GPIO_TypeDef NUEDC_STM32_GPIOC = {2U, "GPIOC"}; /* 虚拟GPIOC实例。 */
GPIO_TypeDef NUEDC_STM32_GPIOD = {3U, "GPIOD"}; /* 虚拟GPIOD实例。 */
GPIO_TypeDef NUEDC_STM32_GPIOE = {4U, "GPIOE"}; /* 虚拟GPIOE实例。 */
GPIO_TypeDef NUEDC_STM32_GPIOH = {7U, "GPIOH"}; /* 虚拟GPIOH实例。 */

/* FPGA专用SPI0对应的兼容句柄。 */
SPI_HandleTypeDef hspi1 = {
    (void *)0x40013000UL,
    {SPI_MODE_MASTER, SPI_DIRECTION_2LINES, SPI_DATASIZE_8BIT,
     SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_NSS_SOFT,
     SPI_BAUDRATEPRESCALER_8, SPI_FIRSTBIT_MSB}
};

/* ADS8363软件20位SPI对应的兼容句柄。 */
SPI_HandleTypeDef hspi2 = {
    (void *)0x40003800UL,
    {SPI_MODE_MASTER, SPI_DIRECTION_2LINES, SPI_DATASIZE_20BIT,
     SPI_POLARITY_LOW, SPI_PHASE_2EDGE, SPI_NSS_SOFT,
     SPI_BAUDRATEPRESCALER_8, SPI_FIRSTBIT_MSB}
};

/* 公共模块所需的兼容外设句柄。 */
UART_HandleTypeDef huart1 = {USART1}; /* 兼容串口句柄1。 */
UART_HandleTypeDef huart2 = {USART2}; /* 标准输出使用的兼容串口句柄。 */
UART_HandleTypeDef huart3 = {USART3}; /* 兼容串口句柄3。 */
ADC_HandleTypeDef hadc1 = {(void *)0x40012000UL}; /* 兼容ADC句柄。 */
TIM_HandleTypeDef htim3 = {(void *)0x40000400UL}; /* 兼容定时器句柄。 */
DWT_Type NUEDC_DWT = {0U, 0U};           /* Cortex-M0+软件周期计数器。 */
CoreDebug_Type NUEDC_CoreDebug = {0U};   /* CMSIS兼容调试寄存器占位。 */
uint32_t SystemCoreClock = CPUCLK_FREQ;  /* 公共驱动读取的系统主频。 */

#define NUEDC_CYCLES_PER_MS (CPUCLK_FREQ / 1000U) /* 每毫秒CPU周期数。 */

static volatile uint32_t s_hal_tick_ms;       /* 系统毫秒节拍。 */
static uint32_t s_gpio_shadow[8];             /* 未映射虚拟端口的输出影子值。 */
static UART_HandleTypeDef *s_uart_rx_huart;   /* 当前异步接收串口句柄。 */
static uint8_t *s_uart_rx_buf;                /* 当前异步接收缓存。 */
static uint16_t s_uart_rx_len;                /* 当前异步接收目标长度。 */
static uint16_t s_uart_rx_index;              /* 当前异步接收写入索引。 */

/* 使用SysTick当前值补足Cortex-M0+软件周期计数器的亚毫秒部分。 */
static void NUEDC_UpdateCycleCounter(void)
{
    uint32_t tick_before;
    uint32_t tick_after;
    uint32_t remaining_cycles;

    do {
        tick_before = s_hal_tick_ms;
        remaining_cycles = SysTick->VAL;
        tick_after = s_hal_tick_ms;
    } while (tick_before != tick_after);

    NUEDC_DWT.CYCCNT = (tick_before * NUEDC_CYCLES_PER_MS) +
        ((NUEDC_CYCLES_PER_MS - 1U) - remaining_cycles);
}

/* 把单一位掩码换算为0～15引脚序号。 */
static uint32_t NUEDC_PinMaskToIndex(uint32_t pin_mask)
{
    for (uint32_t i = 0U; i < 16U; i++) {
        if ((pin_mask & (1UL << i)) != 0U) {
            return i;
        }
    }
    return 16U;
}

/* 把公共驱动的虚拟STM32引脚映射到真实TI引脚。 */
static bool NUEDC_MapGpioPin(const GPIO_TypeDef *port,
                             uint32_t stm32_pin,
                             NUEDC_TiPin_t *mapped_pin)
{
    uint32_t pin_index;

    if ((port == NULL) || (mapped_pin == NULL)) {
        return false;
    }

    pin_index = NUEDC_PinMaskToIndex(stm32_pin);
    mapped_pin->port = NULL;
    mapped_pin->pin = 0U;

    if (port->port_index == 0U) {
        switch (pin_index) {
            case 0U:
                mapped_pin->port = GPIOB;
                mapped_pin->pin = DL_GPIO_PIN_1;
                break;
            case 4U:
                mapped_pin->port = GPIOB;
                mapped_pin->pin = DL_GPIO_PIN_0;
                break;
            case 5U:
                mapped_pin->port = GPIOB;
                mapped_pin->pin = DL_GPIO_PIN_6;
                break;
            case 6U:
                mapped_pin->port = GPIOB;
                mapped_pin->pin = DL_GPIO_PIN_4;
                break;
            case 7U:
                mapped_pin->port = GPIOA;
                mapped_pin->pin = DL_GPIO_PIN_7;
                break;
            case 15U:
                mapped_pin->port = GPIOA;
                mapped_pin->pin = DL_GPIO_PIN_15;
                break;
            default:
                break;
        }
    } else if (port->port_index == 1U) {
        mapped_pin->port = GPIOB;
        mapped_pin->pin = stm32_pin;
    } else if (port->port_index == 2U) {
        switch (pin_index) {
            case 1U:
                mapped_pin->port = GPIOA;
                mapped_pin->pin = DL_GPIO_PIN_16;
                break;
            case 6U:
                mapped_pin->port = GPIOA;
                mapped_pin->pin = DL_GPIO_PIN_17;
                break;
            case 7U:
                mapped_pin->port = GPIOA;
                mapped_pin->pin = DL_GPIO_PIN_9;
                break;
            case 8U:
                mapped_pin->port = GPIOA;
                mapped_pin->pin = DL_GPIO_PIN_8;
                break;
            case 13U:
                mapped_pin->port = GPIOB;
                mapped_pin->pin = DL_GPIO_PIN_13;
                break;
            default:
                break;
        }
    } else if (port->port_index == 4U) {
        mapped_pin->port = GPIOB;
        switch (pin_index) {
            case 2U:
                mapped_pin->pin = DL_GPIO_PIN_20;
                break;
            case 6U:
                mapped_pin->pin = DL_GPIO_PIN_2;
                break;
            case 8U:
                mapped_pin->pin = DL_GPIO_PIN_22;
                break;
            default:
                break;
        }
    } else if (port->port_index == 7U) {
        mapped_pin->port = GPIOB;
        switch (pin_index) {
            case 8U:
                mapped_pin->pin = DL_GPIO_PIN_23;
                break;
            case 9U:
                mapped_pin->pin = DL_GPIO_PIN_24;
                break;
            default:
                break;
        }
    }

    return ((mapped_pin->port != NULL) && (mapped_pin->pin != 0U));
}

/* 向一个已映射TI引脚写入电平。 */
static void NUEDC_GpioWriteMapped(const NUEDC_TiPin_t *mapped_pin,
                                  GPIO_PinState pin_state)
{
    if ((mapped_pin == NULL) || (mapped_pin->port == NULL) ||
        (mapped_pin->pin == 0U)) {
        return;
    }

    if (pin_state == GPIO_PIN_SET) {
        DL_GPIO_setPins(mapped_pin->port, mapped_pin->pin);
    } else {
        DL_GPIO_clearPins(mapped_pin->port, mapped_pin->pin);
    }
}

/* 读取一个已映射TI引脚的电平。 */
static GPIO_PinState NUEDC_GpioReadMapped(const NUEDC_TiPin_t *mapped_pin)
{
    if ((mapped_pin == NULL) || (mapped_pin->port == NULL) ||
        (mapped_pin->pin == 0U)) {
        return GPIO_PIN_RESET;
    }

    return ((DL_GPIO_readPins(mapped_pin->port, mapped_pin->pin) &
             mapped_pin->pin) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

/* 为ADS8363软件SPI提供短建立时间。 */
/*
 * The ADS8363 pin group is absent when the board uses the MSPM0G3507 ADCs.
 * Keep the legacy SPI entry point linkable, but only build its bit-banged bus
 * when SysConfig actually defines the external ADC pins.
 */
#if defined(GPIO_ADS8363_PORT)
static void NUEDC_SpiDelay(void)
{
    __NOP();
    __NOP();
}

/* 以GPIO时序收发一帧ADS8363数据。 */
static uint32_t NUEDC_Ads8363TransferBits(uint32_t tx_data, uint8_t bit_count)
{
    uint32_t rx_data = 0U;

    DL_GPIO_clearPins(GPIO_ADS8363_PORT, GPIO_ADS8363_SW_SPI_SCLK_PIN);
    NUEDC_SpiDelay();

    for (uint8_t i = bit_count; i > 0U; i--) {
        uint32_t bit_mask = 1UL << (i - 1U);

        if ((tx_data & bit_mask) != 0U) {
            DL_GPIO_setPins(GPIO_ADS8363_PORT,
                            GPIO_ADS8363_SW_SPI_MOSI_PIN);
        } else {
            DL_GPIO_clearPins(GPIO_ADS8363_PORT,
                              GPIO_ADS8363_SW_SPI_MOSI_PIN);
        }

        NUEDC_SpiDelay();
        DL_GPIO_setPins(GPIO_ADS8363_PORT, GPIO_ADS8363_SW_SPI_SCLK_PIN);
        NUEDC_SpiDelay();

        if ((DL_GPIO_readPins(GPIO_ADS8363_PORT,
                              GPIO_ADS8363_SW_SPI_MISO_PIN) &
             GPIO_ADS8363_SW_SPI_MISO_PIN) != 0U) {
            rx_data |= bit_mask;
        }

        DL_GPIO_clearPins(GPIO_ADS8363_PORT,
                          GPIO_ADS8363_SW_SPI_SCLK_PIN);
        NUEDC_SpiDelay();
    }

    DL_GPIO_clearPins(GPIO_ADS8363_PORT, GPIO_ADS8363_SW_SPI_MOSI_PIN);
    return rx_data;
}
#else
static uint32_t NUEDC_Ads8363TransferBits(uint32_t tx_data, uint8_t bit_count)
{
    UNUSED(tx_data);
    UNUSED(bit_count);
    return 0U;
}
#endif

/* 判断UART轮询是否已经超时。 */
static bool NUEDC_UartTimeoutExpired(uint32_t start_tick, uint32_t timeout_ms)
{
    if (timeout_ms == 0U) {
        return false;
    }
    return ((HAL_GetTick() - start_tick) >= timeout_ms);
}

/* 初始化兼容层毫秒和周期节拍。 */
void NUEDC_HAL_Init(void)
{
    s_hal_tick_ms = 0U;
    NUEDC_DWT.CYCCNT = 0U;
    (void)SysTick_Config(CPUCLK_FREQ / 1000U);
}

/* 更新周期计数并轮询异步串口接收。 */
void NUEDC_HAL_Service(void)
{
    NUEDC_UpdateCycleCounter();

    if ((s_uart_rx_buf == NULL) || (s_uart_rx_index >= s_uart_rx_len)) {
        return;
    }

    while (s_uart_rx_index < s_uart_rx_len) {
        uint8_t data;

        if (!DL_UART_Main_receiveDataCheck(UART_0_INST, &data)) {
            break;
        }

        s_uart_rx_buf[s_uart_rx_index] = data;
        s_uart_rx_index++;
    }

    if (s_uart_rx_index >= s_uart_rx_len) {
        UART_HandleTypeDef *rx_huart = s_uart_rx_huart;

        s_uart_rx_huart = NULL;
        s_uart_rx_buf = NULL;
        s_uart_rx_len = 0U;
        s_uart_rx_index = 0U;
        HAL_UART_RxCpltCallback(rx_huart);
    }
}

/* 返回32 MHz软件周期计数值。 */
uint32_t NUEDC_HAL_GetCycleCount(void)
{
    NUEDC_UpdateCycleCounter();
    return NUEDC_DWT.CYCCNT;
}

/* 每毫秒更新系统节拍和软件周期基值。 */
void SysTick_Handler(void)
{
    s_hal_tick_ms++;
    NUEDC_DWT.CYCCNT += NUEDC_CYCLES_PER_MS;
}

/* 返回兼容HAL使用的毫秒节拍。 */
uint32_t HAL_GetTick(void)
{
    return s_hal_tick_ms;
}

/* 提供兼容HAL的阻塞毫秒延时。 */
void HAL_Delay(uint32_t delay_ms)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < delay_ms) {
        __NOP();
    }
}

/* 返回固定32 MHz系统主频。 */
uint32_t HAL_RCC_GetSysClockFreq(void)
{
    return CPUCLK_FREQ;
}

/* 同步CMSIS兼容的系统主频变量。 */
void SystemCoreClockUpdate(void)
{
    SystemCoreClock = CPUCLK_FREQ;
}

/* 通过虚拟端口映射写入真实TI GPIO。 */
void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                       GPIO_PinState PinState)
{
    uint32_t pins = (uint32_t)GPIO_Pin;

    if ((GPIOx == NULL) || (GPIOx->port_index >= 8U)) {
        return;
    }

    if (PinState == GPIO_PIN_SET) {
        s_gpio_shadow[GPIOx->port_index] |= pins;
    } else {
        s_gpio_shadow[GPIOx->port_index] &= ~pins;
    }

    for (uint32_t i = 0U; i < 16U; i++) {
        uint32_t pin = 1UL << i;
        NUEDC_TiPin_t mapped_pin;

        if ((pins & pin) == 0U) {
            continue;
        }

        if (NUEDC_MapGpioPin(GPIOx, pin, &mapped_pin)) {
            NUEDC_GpioWriteMapped(&mapped_pin, PinState);
        }
    }
}

/* 通过虚拟端口映射读取真实TI GPIO。 */
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint32_t pins = (uint32_t)GPIO_Pin;
    bool has_mapped_pin = false;

    if ((GPIOx == NULL) || (GPIOx->port_index >= 8U)) {
        return GPIO_PIN_RESET;
    }

    for (uint32_t i = 0U; i < 16U; i++) {
        uint32_t pin = 1UL << i;
        NUEDC_TiPin_t mapped_pin;

        if ((pins & pin) == 0U) {
            continue;
        }

        if (NUEDC_MapGpioPin(GPIOx, pin, &mapped_pin)) {
            has_mapped_pin = true;
            if (NUEDC_GpioReadMapped(&mapped_pin) == GPIO_PIN_SET) {
                return GPIO_PIN_SET;
            }
        }
    }

    if (!has_mapped_pin && ((s_gpio_shadow[GPIOx->port_index] & pins) != 0U)) {
        return GPIO_PIN_SET;
    }

    return GPIO_PIN_RESET;
}

/* 使用ADS8363软件SPI发送兼容数据。 */
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi,
                                   const uint8_t *pData,
                                   uint16_t Size,
                                   uint32_t Timeout)
{
    UNUSED(hspi);
    UNUSED(Timeout);

    if ((pData == NULL) || (Size == 0U)) {
        return HAL_ERROR;
    }

    for (uint16_t i = 0U; i < Size; i++) {
        (void)NUEDC_Ads8363TransferBits((uint32_t)pData[i], 8U);
    }

    return HAL_OK;
}

/* 使用ADS8363软件SPI完成兼容全双工传输。 */
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
                                          uint8_t *pTxData,
                                          uint8_t *pRxData,
                                          uint16_t Size,
                                          uint32_t Timeout)
{
    UNUSED(Timeout);

    if ((hspi == NULL) || (pTxData == NULL) || (pRxData == NULL) ||
        (Size == 0U)) {
        return HAL_ERROR;
    }

    if ((hspi->Init.DataSize == SPI_DATASIZE_20BIT) && (Size == 1U)) {
        uint32_t tx_frame = ((uint32_t)pTxData[0]) |
                            ((uint32_t)pTxData[1] << 8U) |
                            ((uint32_t)pTxData[2] << 16U) |
                            ((uint32_t)pTxData[3] << 24U);
        uint32_t rx_frame = NUEDC_Ads8363TransferBits(tx_frame & 0x000FFFFFUL,
                                                      20U);

        pRxData[0] = (uint8_t)(rx_frame & 0xFFU);
        pRxData[1] = (uint8_t)((rx_frame >> 8U) & 0xFFU);
        pRxData[2] = (uint8_t)((rx_frame >> 16U) & 0xFFU);
        pRxData[3] = (uint8_t)((rx_frame >> 24U) & 0xFFU);
        return HAL_OK;
    }

    for (uint16_t i = 0U; i < Size; i++) {
        pRxData[i] = (uint8_t)NUEDC_Ads8363TransferBits((uint32_t)pTxData[i],
                                                        8U);
    }

    return HAL_OK;
}

/* 使用UART0完成兼容阻塞发送。 */
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
                                    uint8_t *pData,
                                    uint16_t Size,
                                    uint32_t Timeout)
{
    uint32_t start = HAL_GetTick();

    UNUSED(huart);

    if ((pData == NULL) || (Size == 0U)) {
        return HAL_ERROR;
    }

    for (uint16_t i = 0U; i < Size; i++) {
        while (DL_UART_Main_isTXFIFOFull(UART_0_INST)) {
            if (NUEDC_UartTimeoutExpired(start, Timeout)) {
                return HAL_TIMEOUT;
            }
        }

        DL_UART_Main_transmitData(UART_0_INST, pData[i]);
    }

    while (DL_UART_Main_isBusy(UART_0_INST)) {
        if (NUEDC_UartTimeoutExpired(start, Timeout)) {
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

/* 登记由主循环轮询完成的兼容异步接收。 */
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart,
                                      uint8_t *pData,
                                      uint16_t Size)
{
    if ((pData == NULL) || (Size == 0U)) {
        return HAL_ERROR;
    }

    s_uart_rx_huart = huart;
    s_uart_rx_buf = pData;
    s_uart_rx_len = Size;
    s_uart_rx_index = 0U;
    return HAL_OK;
}

/* 提供可由业务层覆盖的串口接收完成回调。 */
__weak void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}

/* 提供可由业务层覆盖的串口错误回调。 */
__weak void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}
