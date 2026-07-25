# MSPM0G3507 Keil 集成工程

本工程把原 TI 板卡算法与外设库整理到 MSP-LITO-G3507，并增加了与 BX71 FPGA 高速 AD/DA 工程配套的控制链路。

Keil 工程：

`Keil/TI_MSPM0G3507.uvprojx`

TI 图形化配置工程：

`SysConfig/MSP_LITO_G3507_Board.syscfg`

打开 SysConfig 图形界面：

```powershell
powershell -ExecutionPolicy Bypass -File .\open_sysconfig.ps1
```

Keil 每次构建前会自动运行 `generate_sysconfig.ps1`，把图形配置生成到
`SysConfig/Generated/ti_msp_dl_config.c/.h`。这两个文件是自动生成文件，
不要手工修改。详细使用与引脚核对见
`Docs/TI_SysConfig_图形化配置说明.md`。

BX71 的 SPI0 使用 PA15 GPIO 软件片选。Motorola 4-wire、Mode 0、1 MHz
全部直接配置在 `.syscfg` 中；SysConfig把未外接的PA2保留为SPI0_CS0，
从而原生生成MOTO4寄存器配置，不再使用任何手写板级配置补丁。

命令行强制全量构建：

```powershell
cd D:\NUEDC\fpga\集成工程\TI_MSPM0G3507
powershell -ExecutionPolicy Bypass -File .\build_keil.ps1
```

构建脚本会等待 uVision 完整结束；任何错误或警告都会使脚本返回失败。当前 ARMClang 6.23 结果为 `0 Error(s), 0 Warning(s)`。

## 当前启动入口

- `Src/main.c` 只调用 `SYSCFG_DL_init()`，随后进入低功耗空循环。
- 工程中不再保留FPGA、AD/DA、AD9910、串口屏等模块测试入口。
- 外设和DSP驱动作为可复用库保留，空白主函数不会主动初始化或访问它们。
- 后续LCR业务应放入独立应用层，不把临时测试重新写回主函数。
- LCR采样使用MSPM0G3507片上双ADC；电压与电流通道由同一定时器事件触发，并由两路DMA同步采集。

## 片上 ADC 采样

| 量 | ADC | 引脚 | DMA |
|---|---|---|---|
| 电压 | ADC0_CH4 | PB25 / J2-15 | DMA_CH0 |
| 电流 | ADC1_CH4 | PB17 / J2-28 | DMA_CH1 |

- `TIMG12` 同时触发 ADC0 和 ADC1，默认配置为 1 MSPS，驱动初始化时可在
  1 kSPS～1 MSPS 内修改采样率。
- 每个通道保存 512 个 12 位原始码；连续模式在 256 点和 512 点位置产生
  配对就绪事件，业务层始终取得同一时间段的电压/电流数据。
- `ADC_MODE_ONESHOT` 采满一帧后自动停止；`ADC_MODE_CONTINUOUS` 使用DMA
  循环双半缓冲。正式锁相/相关运算应直接读取原始码，避免在高速实时路径中
  逐点浮点换算。
- PB17 已用于片上电流ADC，外置ADS8363的软件SPI引脚组不再由SysConfig配置。

## TI 与 FPGA 接口

| MSPM0G3507 | MSP-LITO位置 | BX71 P2 | 作用 |
|---|---|---:|---|
| PA12 / SPI0_SCLK | J1-30 | 31 | SPI时钟 |
| PA14 / SPI0_PICO | J1-32 | 32 | MOSI |
| PA13 / SPI0_POCI | J1-31 | 33 | MISO |
| PA15 / GPIO | J2-32 | 34 | 低有效软件片选 |
| GND | 任一GND | 30 或 12 | 共地 |

SPI0 为独占硬件接口：模式0、8位、MSB优先、1 MHz。CS在完整6字节帧期间持续为低，响应头固定为 `A5 5A`。以调试接口在上、按键在下的元件面观察：J1最下排左/右分别为PA13/PA14，J1倒数第二排右侧为PA12，J2最下排右侧为PA15。高速 DAC 的 125 MSPS 发生和高速 ADC 的 32 MSPS 采集均在 FPGA 内完成，TI只配置寄存器并读取BRAM。

AD9708/AD9280及ProMax源码保存在本工程自己的 `Modules/`，TI工程不引用STM32工程目录。TI底层链路由 `Port/fpga_link_ti.c` 实现，两套MCU工程分别保存自身驱动副本，并使用相同的设备标识、寄存器和状态接口。

## FPGA ProMax 接口

`fpga_promax.c/.h` 是从最新版 BX71 DSP ProMax 合并的纯 C 控制核心；TI 适配层为 `Port/fpga_promax_link_ti.c`。调用方按需定义 `FpgaPromax` 句柄并执行 `FPGA_ProMax_Init(&句柄)`，随后可配置 8 路 DDC、装载 4 组 32 抽头匹配模板、启停/清理数据面，并读取 FPGA 生成的功率、匹配峰值和样本位置。初始化只校验设备标识、版本和能力，不会自动启动计算。

TI 与 FPGA 仍使用同一套四线、1 MHz、6 字节寄存器链路。适配层在本地生成并校验通用 9 字节逻辑帧，再映射到 FPGA 物理寄存器；稳定配置写入后会按硬件有效位读回确认。当前整合 bitstream 的能力字为 `0x09200408`：支持 SPI 和实时 DDC/匹配滤波，不支持 ProMax UART、线级 CRC 或 FFT profile，因此 FFT 高层接口会明确返回“不支持”。

ProMax 不应在中断中调用，也不能与 AD9708/AD9280 驱动并发访问 SPI0；本裸机工程应在主循环中串行调用。TI 只传配置和压缩结果，不搬运 32 MSPS 原始流。

## TI 能力边界

- 片上 ADC：双通道同步触发，12位，双DMA，512点/通道，支持单次及连续双半缓冲，最高1 MSPS。
- AD5687/DAC8831：软件送点上限1000点/秒，16点表最高约62 Hz。
- DSP：保留轻量数学层和512点规模，不复制STM32H743的大缓存/DMA实现。
- THD：`DSP/Measure.h` 与 STM32 共用 THD5、H1~H16 归一化幅度、亚 bin 跟踪和状态接口；512 点频谱可在满足分辨率/奈奎斯特条件的频带内使用。片上 ADC 的 1 MSPS 上限不能为 100 kHz 基波的第 5 次谐波留出保护带。
- ProMax：MCU 仅承担控制和结果读取，8 路 DDC、4×32 匹配滤波由 FPGA 实时完成；当前档位不提供 FFT。
- AD9280：可读1～4096点、可抽取和触发，但不能连续接收32 MSPS，也没有预触发。
- 2 KB栈；默认固件RAM占用远低于32 KB。按需链接ADC/DAC大缓存后仍在器件边界内。

详细端口映射见 `Docs/MSPM0G3507_Keil_Port_Guide.md`，完整系统联测见上一级 `../硬件测试指南.md`。
