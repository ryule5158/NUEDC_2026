# MSPM0G3507 集成工程

> 本工程运行于 MSP-LITO-G3507，使用 Keil 编译、TI SysConfig 管理板级配置。
> 当前 `main.c` 初始化 AD9910，输出 **1 kHz 满幅正弦**，随后进入低功耗循环。

## 快速入口

| 项目 | 位置或状态 |
|---|---|
| Keil 工程 | `Keil/TI_MSPM0G3507.uvprojx` |
| SysConfig 源 | `SysConfig/MSP_LITO_G3507_Board.syscfg` |
| 自动生成 | `SysConfig/Generated/ti_msp_dl_config.c/.h` |
| 当前入口 | `Src/main.c`，AD9910 1 kHz 正弦 |
| 构建结果 | ArmClang 6.23：`0 Error(s), 0 Warning(s)` |

```powershell
cd D:\NUEDC\省赛\Integrated_Project\TI_MSPM0G3507
.\open_sysconfig.ps1       # 打开图形化配置
.\generate_sysconfig.ps1   # 手动生成配置
.\build_keil.ps1           # 生成配置并全量构建
```

`MSP_LITO_G3507_Board.syscfg` 是唯一硬件配置源。禁止手工修改
`SysConfig/Generated` 中的文件。

## SysConfig 配置速查

| 功能 | 配置 |
|---|---|
| 系统时钟 | SYSOSC 32 MHz，不启用 HFXT/PLL |
| UART0 | PA10 TX、PA11 RX，115200-8-N-1 |
| FPGA SPI0 | Mode 0、8 位、MSB、1 MHz |
| FPGA 片选 | PA15 GPIO，低有效 |
| 电压 ADC | PB25/J2-15，ADC0_CH4，DMA_CH0 |
| 电流 ADC | PB17/J2-28，ADC1_CH4，DMA_CH1 |
| ADC 触发 | TIMG12 同时触发，默认 1 MSPS |
| ADC 缓冲 | 每通道 512 点，连续模式使用双半缓冲 |
| 调试接口 | PA19/PA20 保留 SWD |

两路 ADC 的 `ADCMEM0` 必须保持 `TRIGGER_NEXT`；不要改为首次触发后自由运行的
`AUTO_NEXT`。

## TI 与 BX71 接线

| MSPM0G3507 | MSP-LITO 位置 | BX71 P2 / 球位 | 作用 |
|---|---|---|---|
| PA12 / SPI0_SCLK | J1-30 | P2-31 / K19 | SPI 时钟 |
| PA14 / SPI0_PICO | J1-32 | P2-37 / L19 | MOSI |
| PA13 / SPI0_POCI | J1-31 | P2-33 / K17 | MISO |
| PA15 / GPIO | J2-32 | P2-34 / K18 | 低有效片选 |
| GND | 任一 GND | 任一 GND | 必须共地 |

AD9708、AD9280 和 ProMax 共用 SPI0，必须在主循环中串行访问，不要在中断中调用。
125 MSPS DAC、32 MSPS ADC 和 ProMax 实时 DSP 均由 FPGA 执行，TI 只传配置和读取
结果。

## 模块接线速查

### AD9910

| 信号 | MSPM0G3507 | MSP-LITO 位置 |
|---|---|---|
| SCK / SDIO | PB20 / PB2 | J2-23 / J1-11 |
| PWR | PB13 | J1-26 |
| DROVER / DRCTL / DRHOLD | PB1 / PA8 / PA16 | J1-9 / J1-15 / J2-31 |
| RESET | PB4 | J1-13 |
| PF0 / PF1 / PF2 | PB5 / PA17 / PA9 | J1-14 / J2-30 / J1-16 |
| OSK / IO_UPDATE / CS | PB7 / PB8 / PB9 | J1-20 / J1-21 / J1-22 |

默认 `AD9910_SYSCLK_HZ=1 GHz`。模块参考时钟或 PLL 配置变化后必须同步修改该宏。

### AD5687、AD603 与 DAC8831

| 模块 | MSPM0G3507 接线 | 说明 |
|---|---|---|
| AD5687 | PB6/J1-19 SCK、PA7/J1-10 MOSI、PB22/J2-21 CS | 两路 CV 共用片选 |
| AD603 | AD5687 CV1 → P2-2，GND → P2-3 | 拔掉 P2 跳帽；推荐约 ±5.3 V、50 Ω |
| DAC8831 | PB6/J1-19 SCK、PA7/J1-10 MOSI、PB0/J1-8 CS | 与 AD5687 共用时钟和数据 |

AD5687 和 DAC8831 可分别静态输出，但两个软件波形任务不能并发运行。

### ADS8363 兼容边界

旧 ADS8363 接口使用的 PB17 已分配为片上 `ADC1_CH4`。当前工程不能直接启用
`NUEDC_ENABLE_ADS8363`；若赛题需要外置 ADS8363，应先在 SysConfig 中重新分配
无冲突引脚。

## 最小调用示例

```c
#include "ti_msp_dl_config.h"
#include "adc_app.h"
#include "ad603.h"
#include "ad9910_app.h"
#include "ad9708_app.h"
#include "ad9280_app.h"

SYSCFG_DL_init();
NUEDC_HAL_Init();

/* AD9910 输出 100 kHz 正弦。 */
AD9910_AppInit();
AD9910_AppOutputSine(100000U, AD9910_MAX_AMPLITUDE);

/* AD603：通过 AD5687 CV1 设置 10 dB。 */
AD603_HandleTypeDef ad603;       /* AD603 运行句柄。 */
AD603_ConfigTypeDef ad603_cfg;   /* 两点电压校准参数。 */

AD603_GetDefaultConfig(&ad603_cfg);
ad603_cfg.dac_output = AD5687_OUTPUT_CV1;
if (AD603_Init(&ad603, &ad603_cfg) == AD603_OK) {
    (void)AD603_SetGainMdb(&ad603, 10000L);
}

/* FPGA 高速 AD/DA。 */
if (AD9708_AppInit() == AD9708_OK) {
    (void)AD9708_AppOutputCalibrationCode(192U);
}
if (AD9280_AppInit() == AD9280_OK) {
    (void)AD9280_AppCapture(1024U, 1U, 100U);
}
```

AD603 默认把 -20～60 dB 映射为 0.400～1.446 V；`10000 mdB` 表示 10 dB。
实测端点不同只需修改校准结构体。

## 片上双 ADC

- TIMG12 同时触发 ADC0 和 ADC1，采样率范围 1 kSPS～1 MSPS。
- 单次模式采满 512 点后停止。
- 连续模式每 256 点发布一组配对数据。
- DMA 只搬运 12 位原始码；高速算法应处理原始码，浮点换算留给低速显示。

## FPGA ProMax

典型顺序：

1. 定义 `FpgaPromax` 句柄并调用 `FPGA_ProMax_Init()`；
2. 配置最多 8 路 DDC；
3. 装载最多 4 组、每组 32 抽头的匹配模板；
4. 启动数据面并读取功率、峰值及样本位置。

当前能力字为 `0x09200408`，不包含 UART、线级 CRC 或 FFT。相关高层接口返回
`FPGA_PROMAX_E_UNSUPPORTED` 属于正常能力边界。

## 资源边界

| 资源 | 当前能力 |
|---|---|
| MCU | 128 KB Flash、32 KB SRAM、2 KB 栈 |
| 片上 ADC | 双通道同步、12 位、最高 1 MSPS |
| 软件 DAC | 保守上限 1000 点/秒 |
| AD9280 | FPGA 内 32 MSPS；TI 可读取 1～4096 点块数据 |
| DSP | 512 点轻量实现，不复制 STM32 的 4096 点大工作区 |

1 MSPS 片上 ADC 无法为 100 kHz 基波的第五次谐波留下奈奎斯特保护带。高频 THD
应使用 STM32 的更高采样率方案或 FPGA 高速 ADC。

## 已完成验证

- SysConfig 1.26.2 校验并重新生成成功。
- 生成前后核心 `ti_msp_dl_config.c/.h` 未变化。
- TI 全量构建：0 错误、0 警告。
- 当前固件大小：Code 4624 B、RO 372 B、RW 56 B、ZI 2224 B。
- ProMax 主机协议测试和 FPGA 行为仿真通过。

上述结果证明配置与工程能够一致生成和编译。XDS110 下载、模块供电、模拟量程和
具体赛题指标仍需按实际接线上板验证。

## 本机工具

| 工具 | 路径 |
|---|---|
| TI SysConfig | `D:\ti\sysconfig_1.26.2\sysconfig_gui.bat` |
| SysConfig CLI | `D:\ti\sysconfig_1.26.2\sysconfig_cli.bat` |
| MSPM0 SDK | `D:\ti\mspm0_sdk_2_10_00_04` |
| Keil | `C:\Keil_v5\UV4\UV4.exe` |

本机未发现 CCS 或 UniFlash；当前工程使用 Keil + SysConfig，下载调试需外接
XDS110。
