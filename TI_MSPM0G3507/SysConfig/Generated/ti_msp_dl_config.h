/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for ADC_SAMPLE_TIMER */
#define ADC_SAMPLE_TIMER_INST                                           (TIMG12)
#define ADC_SAMPLE_TIMER_INST_IRQHandler                       TIMG12_IRQHandler
#define ADC_SAMPLE_TIMER_INST_INT_IRQN                         (TIMG12_INT_IRQn)
#define ADC_SAMPLE_TIMER_INST_LOAD_VALUE                                   (31U)
#define ADC_SAMPLE_TIMER_INST_PUB_0_CH                                      (12)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD                                      (23)




/* Defines for FPGA_SPI */
#define FPGA_SPI_INST                                                      SPI0
#define FPGA_SPI_INST_IRQHandler                                SPI0_IRQHandler
#define FPGA_SPI_INST_INT_IRQN                                    SPI0_INT_IRQn
#define GPIO_FPGA_SPI_PICO_PORT                                           GPIOA
#define GPIO_FPGA_SPI_PICO_PIN                                   DL_GPIO_PIN_14
#define GPIO_FPGA_SPI_IOMUX_PICO                                (IOMUX_PINCM36)
#define GPIO_FPGA_SPI_IOMUX_PICO_FUNC                IOMUX_PINCM36_PF_SPI0_PICO
#define GPIO_FPGA_SPI_POCI_PORT                                           GPIOA
#define GPIO_FPGA_SPI_POCI_PIN                                   DL_GPIO_PIN_13
#define GPIO_FPGA_SPI_IOMUX_POCI                                (IOMUX_PINCM35)
#define GPIO_FPGA_SPI_IOMUX_POCI_FUNC                IOMUX_PINCM35_PF_SPI0_POCI
/* GPIO configuration for FPGA_SPI */
#define GPIO_FPGA_SPI_SCLK_PORT                                           GPIOA
#define GPIO_FPGA_SPI_SCLK_PIN                                   DL_GPIO_PIN_12
#define GPIO_FPGA_SPI_IOMUX_SCLK                                (IOMUX_PINCM34)
#define GPIO_FPGA_SPI_IOMUX_SCLK_FUNC                IOMUX_PINCM34_PF_SPI0_SCLK
#define GPIO_FPGA_SPI_CS0_PORT                                            GPIOA
#define GPIO_FPGA_SPI_CS0_PIN                                     DL_GPIO_PIN_2
#define GPIO_FPGA_SPI_IOMUX_CS0                                  (IOMUX_PINCM7)
#define GPIO_FPGA_SPI_IOMUX_CS0_FUNC                   IOMUX_PINCM7_PF_SPI0_CS0



/* Defines for ADC_VOLTAGE */
#define ADC_VOLTAGE_INST                                                    ADC0
#define ADC_VOLTAGE_INST_IRQHandler                              ADC0_IRQHandler
#define ADC_VOLTAGE_INST_INT_IRQN                                (ADC0_INT_IRQn)
#define ADC_VOLTAGE_ADCMEM_0                                  DL_ADC12_MEM_IDX_0
#define ADC_VOLTAGE_ADCMEM_0_REF                 DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_VOLTAGE_ADCMEM_0_REF_VOLTAGE_V                                     3.3
#define ADC_VOLTAGE_INST_SUB_CH                                             (12)
#define GPIO_ADC_VOLTAGE_C4_PORT                                           GPIOB
#define GPIO_ADC_VOLTAGE_C4_PIN                                   DL_GPIO_PIN_25
#define GPIO_ADC_VOLTAGE_IOMUX_C4                                (IOMUX_PINCM56)
#define GPIO_ADC_VOLTAGE_IOMUX_C4_FUNC            (IOMUX_PINCM56_PF_UNCONNECTED)

/* Defines for ADC_CURRENT */
#define ADC_CURRENT_INST                                                    ADC1
#define ADC_CURRENT_INST_IRQHandler                              ADC1_IRQHandler
#define ADC_CURRENT_INST_INT_IRQN                                (ADC1_INT_IRQn)
#define ADC_CURRENT_ADCMEM_0                                  DL_ADC12_MEM_IDX_0
#define ADC_CURRENT_ADCMEM_0_REF                 DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC_CURRENT_ADCMEM_0_REF_VOLTAGE_V                                     3.3
#define ADC_CURRENT_INST_SUB_CH                                             (12)
#define GPIO_ADC_CURRENT_C4_PORT                                           GPIOB
#define GPIO_ADC_CURRENT_C4_PIN                                   DL_GPIO_PIN_17
#define GPIO_ADC_CURRENT_IOMUX_C4                                (IOMUX_PINCM43)
#define GPIO_ADC_CURRENT_IOMUX_C4_FUNC            (IOMUX_PINCM43_PF_UNCONNECTED)



/* Defines for DMA_ADC_VOLTAGE */
#define DMA_ADC_VOLTAGE_CHAN_ID                                              (0)
#define ADC_VOLTAGE_INST_DMA_TRIGGER                  (DMA_ADC0_EVT_GEN_BD_TRIG)
/* Defines for DMA_ADC_CURRENT */
#define DMA_ADC_CURRENT_CHAN_ID                                              (1)
#define ADC_CURRENT_INST_DMA_TRIGGER                  (DMA_ADC1_EVT_GEN_BD_TRIG)


/* Port definition for Pin Group GPIO_FPGA_CONTROL */
#define GPIO_FPGA_CONTROL_PORT                                           (GPIOA)

/* Defines for FPGA_CS_N: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_FPGA_CONTROL_FPGA_CS_N_PIN                         (DL_GPIO_PIN_15)
#define GPIO_FPGA_CONTROL_FPGA_CS_N_IOMUX                        (IOMUX_PINCM37)
/* Port definition for Pin Group GPIO_BOARD */
#define GPIO_BOARD_PORT                                                  (GPIOB)

/* Defines for STATUS_LED: GPIOB.14 with pinCMx 31 on package pin 2 */
#define GPIO_BOARD_STATUS_LED_PIN                               (DL_GPIO_PIN_14)
#define GPIO_BOARD_STATUS_LED_IOMUX                              (IOMUX_PINCM31)
/* Defines for USER_TEST: GPIOB.27 with pinCMx 58 on package pin 29 */
#define GPIO_BOARD_USER_TEST_PIN                                (DL_GPIO_PIN_27)
#define GPIO_BOARD_USER_TEST_IOMUX                               (IOMUX_PINCM58)
/* Defines for SERIAL_CLOCK: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_DAC_SHARED_SERIAL_CLOCK_PORT                                (GPIOB)
#define GPIO_DAC_SHARED_SERIAL_CLOCK_PIN                         (DL_GPIO_PIN_6)
#define GPIO_DAC_SHARED_SERIAL_CLOCK_IOMUX                       (IOMUX_PINCM23)
/* Defines for SERIAL_DATA: GPIOA.7 with pinCMx 14 on package pin 49 */
#define GPIO_DAC_SHARED_SERIAL_DATA_PORT                                 (GPIOA)
#define GPIO_DAC_SHARED_SERIAL_DATA_PIN                          (DL_GPIO_PIN_7)
#define GPIO_DAC_SHARED_SERIAL_DATA_IOMUX                        (IOMUX_PINCM14)
/* Defines for DAC8831_CS: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_DAC_SHARED_DAC8831_CS_PORT                                  (GPIOB)
#define GPIO_DAC_SHARED_DAC8831_CS_PIN                           (DL_GPIO_PIN_0)
#define GPIO_DAC_SHARED_DAC8831_CS_IOMUX                         (IOMUX_PINCM12)
/* Defines for AD5687_CS: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_DAC_SHARED_AD5687_CS_PORT                                   (GPIOB)
#define GPIO_DAC_SHARED_AD5687_CS_PIN                           (DL_GPIO_PIN_22)
#define GPIO_DAC_SHARED_AD5687_CS_IOMUX                          (IOMUX_PINCM50)
/* Defines for SCK: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_AD9910_SCK_PORT                                             (GPIOB)
#define GPIO_AD9910_SCK_PIN                                     (DL_GPIO_PIN_20)
#define GPIO_AD9910_SCK_IOMUX                                    (IOMUX_PINCM48)
/* Defines for SDIO: GPIOB.2 with pinCMx 15 on package pin 50 */
#define GPIO_AD9910_SDIO_PORT                                            (GPIOB)
#define GPIO_AD9910_SDIO_PIN                                     (DL_GPIO_PIN_2)
#define GPIO_AD9910_SDIO_IOMUX                                   (IOMUX_PINCM15)
/* Defines for PWR: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_AD9910_PWR_PORT                                             (GPIOB)
#define GPIO_AD9910_PWR_PIN                                     (DL_GPIO_PIN_13)
#define GPIO_AD9910_PWR_IOMUX                                    (IOMUX_PINCM30)
/* Defines for DROVER: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_AD9910_DROVER_PORT                                          (GPIOB)
#define GPIO_AD9910_DROVER_PIN                                   (DL_GPIO_PIN_1)
#define GPIO_AD9910_DROVER_IOMUX                                 (IOMUX_PINCM13)
/* Defines for DRCTL: GPIOA.8 with pinCMx 19 on package pin 54 */
#define GPIO_AD9910_DRCTL_PORT                                           (GPIOA)
#define GPIO_AD9910_DRCTL_PIN                                    (DL_GPIO_PIN_8)
#define GPIO_AD9910_DRCTL_IOMUX                                  (IOMUX_PINCM19)
/* Defines for DRHOLD: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_AD9910_DRHOLD_PORT                                          (GPIOA)
#define GPIO_AD9910_DRHOLD_PIN                                  (DL_GPIO_PIN_16)
#define GPIO_AD9910_DRHOLD_IOMUX                                 (IOMUX_PINCM38)
/* Defines for RESET: GPIOB.4 with pinCMx 17 on package pin 52 */
#define GPIO_AD9910_RESET_PORT                                           (GPIOB)
#define GPIO_AD9910_RESET_PIN                                    (DL_GPIO_PIN_4)
#define GPIO_AD9910_RESET_IOMUX                                  (IOMUX_PINCM17)
/* Defines for PROFILE_1: GPIOA.17 with pinCMx 39 on package pin 10 */
#define GPIO_AD9910_PROFILE_1_PORT                                       (GPIOA)
#define GPIO_AD9910_PROFILE_1_PIN                               (DL_GPIO_PIN_17)
#define GPIO_AD9910_PROFILE_1_IOMUX                              (IOMUX_PINCM39)
/* Defines for PROFILE_2: GPIOA.9 with pinCMx 20 on package pin 55 */
#define GPIO_AD9910_PROFILE_2_PORT                                       (GPIOA)
#define GPIO_AD9910_PROFILE_2_PIN                                (DL_GPIO_PIN_9)
#define GPIO_AD9910_PROFILE_2_IOMUX                              (IOMUX_PINCM20)
/* Defines for PROFILE_0: GPIOB.5 with pinCMx 18 on package pin 53 */
#define GPIO_AD9910_PROFILE_0_PORT                                       (GPIOB)
#define GPIO_AD9910_PROFILE_0_PIN                                (DL_GPIO_PIN_5)
#define GPIO_AD9910_PROFILE_0_IOMUX                              (IOMUX_PINCM18)
/* Defines for OSK: GPIOB.7 with pinCMx 24 on package pin 59 */
#define GPIO_AD9910_OSK_PORT                                             (GPIOB)
#define GPIO_AD9910_OSK_PIN                                      (DL_GPIO_PIN_7)
#define GPIO_AD9910_OSK_IOMUX                                    (IOMUX_PINCM24)
/* Defines for IO_UPDATE: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_AD9910_IO_UPDATE_PORT                                       (GPIOB)
#define GPIO_AD9910_IO_UPDATE_PIN                                (DL_GPIO_PIN_8)
#define GPIO_AD9910_IO_UPDATE_IOMUX                              (IOMUX_PINCM25)
/* Defines for AD9910_CS_N: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_AD9910_AD9910_CS_N_PORT                                     (GPIOB)
#define GPIO_AD9910_AD9910_CS_N_PIN                              (DL_GPIO_PIN_9)
#define GPIO_AD9910_AD9910_CS_N_IOMUX                            (IOMUX_PINCM26)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_ADC_SAMPLE_TIMER_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_FPGA_SPI_init(void);
void SYSCFG_DL_ADC_VOLTAGE_init(void);
void SYSCFG_DL_ADC_CURRENT_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
