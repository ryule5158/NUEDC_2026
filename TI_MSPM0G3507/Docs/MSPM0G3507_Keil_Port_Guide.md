# MSPM0G3507 Keil 移植说明

## 1. 工程与环境

工程根目录：

`D:\NUEDC\fpga\集成工程\TI_MSPM0G3507`

使用 Keil uVision 5.38、ARMClang 6.23、TI MSPM0G1X0X/G3X0X DFP 1.3.1。环境检查脚本是 `setup_keil_mspm0_env.ps1`，全量构建脚本是 `build_keil.ps1`。

当前 `SysConfig/MSP_LITO_G3507_Board.syscfg` 是工程唯一有效的硬件配置源。Keil 在构建前自动生成 `SysConfig/Generated/ti_msp_dl_config.h/.c`，生成文件不手工修改，避免图形配置和源码出现两套映射。

## 2. 移植结构

| 层 | 关键文件 | 作用 |
|---|---|---|
| TI图形配置 | `SysConfig/MSP_LITO_G3507_Board.syscfg`、`SysConfig/Generated/ti_msp_dl_config.*` | 时钟、GPIO、UART0、ADC0/ADC1、DMA、TIMG12、SPI0 |
| HAL兼容 | `Port/stm32h7xx_hal.h`、`Port/nuedc_hal_compat.c` | GPIO映射、SysTick、UART和旧接口兼容 |
| FPGA链路 | `Port/fpga_link_ti.c` | TI DriverLib SPI0、6字节帧、ACK与超时 |
| FPGA ProMax | `../Modules/fpga_promax*`、`Port/fpga_promax_link_ti.c` | 8路DDC与4×32匹配滤波控制、压缩结果读取 |
| 高速AD/DA | `../Modules/ad9708*`、`ad9280*` | TI工程内独立保存的协议和APP接口 |
| TI入口 | `Src/main.c` | 仅执行SysConfig初始化和空循环，不包含模块测试 |
| 片上ADC | `BSP/adc_app.c` | PB25电压、PB17电流，同步触发双DMA采样 |

TI工程编译共享的 AD9708/AD9280 源码，但不会编译 STM32 专用
`fpga_link.c`。APP层在 TI 平台选择 `hspi1`，它只作为独占 SPI0 的平台
句柄。正式LCR采样改用片上ADC后，ADS8363的旧兼容源码仍可编译，但
SysConfig不再分配其GPIO，不能与PB17电流输入同时启用。

ProMax 通用核心不依赖 STM32 HAL、FPU、DMA 或动态内存。TI 适配层把本地9字节逻辑事务映射到现有6字节SPI寄存器，并对稳定配置做写后读回校验。当前FPGA能力字为 `0x09200408`，只开放SPI实时DDC/匹配滤波；UART、线级CRC和FFT能力位保持清零。工程默认不调用 `FPGA_ProMax_Init(&句柄)`，使用者应在实际业务明确需要时初始化，并在主循环串行访问共享SPI0。

## 3. 系统引脚

| 功能 | MSPM0G3507 | MSP-LITO位置 |
|---|---|---|
| UART0 TX/RX | PA10 / PA11 | J1-17 / J1-18，同时接到J3调试口 |
| 电压 ADC0_CH4 | PB25 | J2-15 |
| 电流 ADC1_CH4 | PB17 | J2-28 |
| 状态灯 | PB14 | 板载D1 |
| FPGA SCLK/MOSI/MISO/CS | PA12 / PA14 / PA13 / PA15 | J1-30 / J1-32 / J1-31 / J2-32 |

FPGA SPI0：Controller、Motorola 4-wire Mode 0、8 bit、MSB first、1 MHz。
SysConfig把未外接的PA2保留为硬件CS0，以便原生生成MOTO4配置；BX71实际
片选仍由PA15在完整6字节帧前后拉低/拉高，PA2不需要外接。以调试接口在上、
按键在下的元件面观察：J1最下排左/右分别为PA13/PA14，J1倒数第二排右侧为
PA12，J2最下排右侧为PA15。

## 4. 外接模块映射

### AD9910

| 信号 | MSPM0G3507 | MSP-LITO位置 |
|---|---|---|
| SCK / SDIO | PB20 / PB2 | J2-23 / J1-11 |
| PWR | PB13 | J1-26 |
| DROVER / DRCTL / DRHOLD | PB1 / PA8 / PA16 | J1-9 / J1-15 / J2-31 |
| RESET | PB4 | J1-13 |
| PF0 / PF1 / PF2 | PB5 / PA17 / PA9 | J1-14 / J2-30 / J1-16 |
| OSK / IO_UPDATE / CS | PB7 / PB8 / PB9 | J1-20 / J1-21 / J1-22 |

DRCTL和PWR已从PA19/PA20迁到PA8/PB13，PA19/PA20完整保留给SWD调试。DROVER和RESET使用无板载负载且已引出的PB1/PB4，避开PA0的板载LED/上拉以及PA6的40 MHz晶振网络；SDIO使用PB2，避开直接连接用户按键S2的PB21。

### AD5687与DAC8831

- 共享软件SPI：PB6 SCK（J1-19）、PA7 MOSI（J1-10）。
- AD5687 CS：PB22（J2-21）。
- DAC8831 CS：PB0（J1-8）。
- 两者可静态输出，但不要并行启动两个软件波形任务。

PB0/PB6均从MSP-LITO排针引出且无默认板载负载；不使用连接32.768 kHz/40 MHz晶振网络的PA4～PA6。

### ADS8363旧接口

外置ADS8363不再属于当前LCR采样方案。原软件SPI的PB17已分配给
`ADC1_CH4`电流输入，因此不要启用`NUEDC_ENABLE_ADS8363`。若以后恢复
外置ADC，必须先重新选择一组无冲突引脚并在SysConfig中显式配置，不能直接
复用当前工程。

## 5. 空白应用入口

`Src/main.c`不提供模块初始化开关，也不包含串口菜单、波形输出、设备扫描或
其他测试逻辑。它只执行：

1. `SYSCFG_DL_init()`，加载图形化生成的硬件配置。
2. 在空循环中执行`__WFI()`。

ADC、FPGA、DDS、DAC和DSP源码仅作为库参与工程管理。正式业务需要哪个模块，
就由后续独立应用层明确初始化哪个模块。

## 6. 定时和资源边界

MSPM0G3507 Cortex-M0+没有DWT。兼容层以1 ms SysTick计数和当前递减值组合出亚毫秒软件周期计数，主循环不再使用阻塞LED延时。

| 能力 | STM32H743工程 | TI MSPM0G3507工程 | 移植结论 |
|---|---|---|---|
| AD9708/AD9280 FPGA协议与APP | 完整支持 | 复用同一高层源码 | 已完整移植，底层改为TI SPI0 |
| FPGA内部125/32 MSPS | FPGA执行 | FPGA执行 | 与MCU性能无关，功能一致 |
| FPGA ProMax实时DSP | FPGA并行执行 | MCU仅配置并读取压缩结果 | 支持8路DDC、4组32抽头匹配滤波，不支持当前档位之外的FFT |
| MCU片上ADC | 可使用DMA和更大缓存 | 双路同步、12位、512点/通道、1 kSPS～1 MSPS、DMA双半缓冲 | 满足LCR电压/电流同步采样 |
| 大规模DSP缓存 | H743 RAM允许更大规模 | 32 KB SRAM，保持512点轻量实现 | 未复制4096点STM32缓存 |
| 软件DAC连续送点 | 较高主频和定时器资源 | 保守限制1000点/秒 | 不宣称不可达的高速波形 |
| FPGA QSPI烧写 | 由BX71工具完成 | 仍由BX71工具完成 | 不移植到MSPM0 |

- AD5687/DAC8831的软件送点上限限定为1000点/秒。
- 片上ADC由TIMG12事件触发；`ADC_MODE_ONESHOT`采满512点后停止，
  `ADC_MODE_CONTINUOUS`每256点发布一次成对电压/电流数据。
- DMA只搬运12位原始码。高速实时算法应直接使用原始码，浮点电压换算留给
  单次调试或低速显示路径。
- FPGA ADC/DAC时钟完全由FPGA产生，不受MSPM0软件定时精度影响。

链接文件给 MSPM0 分配128 KB Flash、32 KB SRAM和2 KB栈。默认构建约使用12 KB代码与6 KB零初始化RAM；AD9280的4096点缓存和AD9708的1024点波表仅在调用对应功能时进入最终链接，资源仍低于32 KB。

## 7. 构建与实板验证

```powershell
powershell -ExecutionPolicy Bypass -File .\build_keil.ps1
```

必须得到 `0 Error(s), 0 Warning(s)`。生成的 `.hex` 用 XDS110/UniFlash 烧入 MSPM0G3507。数字编译、协议和资源验证不能替代模拟实测；接线、波形、ADC码值及故障定位按 `D:\NUEDC\fpga\集成工程\硬件测试指南.md` 执行。
