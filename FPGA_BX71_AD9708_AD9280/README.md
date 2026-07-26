# BX71 高速 AD/DA 通用工程

本工程在 BX71-V4（Zynq-7020）上同时实现 AD9708 波形发生和 AD9280 高速采集。它不包含 2024 C 题的 SD/SM、调幅、时延或 AD9910 同步逻辑。

## 已实现功能

- AD9708：125 MSPS，支持恒定码、正弦波、三角波、方波、锯齿波、SINC、任意波、相位、Q8.8 幅度/偏置和硬件扫频。
- AD9280：32 MSPS，1～4096 点 BRAM 缓存，1～65535 倍抽取，立即/上升沿/下降沿/任意边沿触发，并统计最小值、最大值、累加和与 OTR 越界次数。
- DSP ProMax 实时档：原始 32 MSPS ADC 样本经 32×18 位 XPM 异步 FIFO 送入 50 MHz 数据面，同时运行 8 路 DDC、16 个 I/Q CIC、8 路复功率统计和 4 组×32 抽头通用 FIR/匹配滤波。4 组 FIR 每个有效样点都并行执行，共 128 个硬件乘法通道。
- 连续测量：FPGA 对满速样本流逐块计算最值、和、平方和、自动门限、占空比计数和上升沿周期累计；STM32 只读取结果快照，即可换算 DC、RMS、AC RMS、Vpp、频率和占空比。
- STM32 接口：SPI 模式 0、MSB first、6 字节寄存器帧，固定 ACK 为 `0xA5 0x5A`。
- 时钟：由 BX71 本地 50 MHz 时钟生成 DAC 125 MHz 和 ADC 32 MHz，不需要 AD9910 `SYNC_CLK`。

AD9708 通用协议固件版本为 `0x00020003`，设备标识为 `0xAD970802`。ADC 寄存器区使用 `0x20`～`0x2E`。

## DSP ProMax 寄存器

ProMax 复用原 6 字节 SPI 总线，未引入第二套协议。版本为 `1.1`，能力值为 `0x29200408`，表示物理 SPI、实时数据面、连续波形统计、32 抽头、4 个 FIR/匹配滤波组和 8 路 DDC。该档位不包含 FFT、UART 或物理帧 CRC；这些能力位保持为 0。

| 地址 | 作用 |
|---:|---|
| `0x30`～`0x32` | 标识 `BXPM`、版本 `1.1`、能力值 |
| `0x33`～`0x37` | 运行/清零、状态、快照、32 MHz 采样率、FIFO 状态 |
| `0x38`～`0x3F` | 8 路 NCO 相位增量 |
| `0x40`～`0x47` | 8 路 NCO 相位偏移 |
| `0x48`～`0x49` | FIR/匹配滤波组、抽头选择和18位 Q1.16 系数 |
| `0x4A`～`0x4F` | 保留，读回 0 |
| `0x50`～`0x5F` | 8 路37位功率快照的低/高字 |
| `0x60`～`0x67` | 4 组带符号峰值分数和峰值样本位置 |
| `0x68`～`0x72` | 连续波形统计控制、状态及完整结果快照 |

NCO 只在 FIFO 输出有效样本时推进，因此相位增量按实际 ADC 采样率计算：`round(f / 32000000 × 2^32)`。STM32 逻辑地址和物理地址的映射由 `fpga_promax_link_map.h` 管理，业务代码不要直接写上述物理地址。比赛题型、职责边界和入门调用见 `DSP_PROMAX_CONTEST_GUIDE.md`。

## 最终验证（2026-07-23）

- Vivado 2018.3 含 PS7 全量构建成功，路由后全局建立/保持余量为 `+0.342/+0.021 ns`；DAC 为 `+1.632/+2.158 ns`，ADC 为 `+1.008/+1.518 ns`。
- 顶层实际使用 `33,526 LUT`、`31,069 FF`、`1 RAMB36`、`163 DSP48E1`，其中 ProMax 实时数据面保留 `160 DSP48E1`。
- AD9280采集、AD9708幅度/偏置及ProMax集成三项自检均通过；ProMax定向CDC报告没有Critical，完整证据见 `output/verification/`。
- `BOOT_PS7.bin` 已重新打包，可按下文步骤写入 QSPI；静态验证和数字仿真不替代最终板上模拟链路实测。

## 目录

- `Vivado/BX71_AD9708_AD9280_PS7/`：含 PS7 的固定 Vivado 工程，直接打开同名 `.xpr`。
- `Vivado/BX71_AD9708_AD9280_PL/`：运行纯 PL 构建脚本后生成的固定 Vivado 工程。
- `rtl/`：AD9708 DDS、AD9280 采集核、时钟和 SPI 寄存器从机。
- `rtl_ps7/`：带 PS7 配置的持久化启动顶层。
- `constraints/`：BX71 P2/P3 引脚和 AD/DA 时序约束。
- `sim/`：AD9708 数字幅度及 AD9280 采集自检仿真。
- `scripts/`：Vivado 构建和 QSPI 启动镜像打包脚本。
- `启动支持/`：本 FPGA 工程使用的 FSBL 等启动支持文件。
- `output/`：构建后的 `.bit`、`BOOT_PS7.bin`和 SHA-256。

## 构建与固化

双击 `scripts/Build-QspiBoot.cmd`，脚本会在 `Vivado/BX71_AD9708_AD9280_PS7/` 创建或复用固定工程，构建 PS7 集成位流并生成 `output/BOOT_PS7.bin`。

JTAG 临时调试应下载最终的 `output/BX71_V4_U3_tail_AD9708_PS7.bit`。掉电保存使用以下任一入口，不需要打开 SDK、不需要 SDK workspace，也不需要导入 HDF：

- `scripts/Program-Qspi.cmd`：烧录现有的 `output/BOOT_PS7.bin`，适合确认镜像后重复烧录。
- `scripts/Build-And-Program-Qspi.cmd`：重新构建位流、打包完整启动镜像、烧录 QSPI 并写后校验。
- `scripts/Probe-Qspi.cmd`：只检查工具、JTAG 和 XC7Z020，不擦除、不写入 Flash。

脚本固定使用 BX71 所需的 `qspi-x1-single`、Offset 0 和工程内 `启动支持/boot/FSBL.elf`。它会自动寻找任意盘符及常见安装目录中的 Vivado/SDK 2018.3，自动连接或启动 `hw_server`，确认 JTAG 链中存在 XC7Z020，自动选择唯一的 Zynq ARM DAP，并把烧录输入暂存到纯 ASCII 短路径，避免工程目录变化、空格或中文路径导致 Xilinx 2018.3 失败。烧录成功后默认执行 Xilinx 写后校验，并在 `output/` 保存本机日志及 `LAST_PROGRAMMED.txt`。

跨电脑使用时只需完整复制/拉取本工程，并安装 Vivado 2018.3 与 SDK 2018.3。工具可安装在不同盘符；若采用非常规目录，可先运行 Xilinx 的 `settings64.bat`，或直接给 PowerShell 脚本传入 `-Vivado`、`-Bootgen`、`-ProgramFlash`、`-HwServer` 的完整路径。烧录时只能连接一块 Zynq；检测到多块 Zynq 时脚本会拒绝继续，避免写错设备。

## 连接

高速 AD/DA 模块在 U3 与 BX71 P3 尾端对齐直插，不翻面、不错位。STM32 与 BX71 使用：

| STM32H743 | BX71 P2 | 作用 |
|---|---:|---|
| PI1 / SPI2_SCK | 31 | SPI 时钟 |
| PI3 / SPI2_MOSI | 32 | STM32 写 FPGA |
| PI2 / SPI2_MISO | 33 | FPGA 返回数据 |
| PE8 / FPGA_CS | 34 | 低有效片选 |
| PH9 / FPGA_IRQ | 35 | 可选DAC PLL锁定指示 |
| GND | 30 或 12 | 共地 |

TI LP-MSPM0G3507 也可独立控制同一接口：

| MSPM0G3507 | LaunchPad位置 | BX71 P2 | 作用 |
|---|---|---:|---|
| PA12 / SPI0_SCLK | BoosterPack 32 | 31 | SPI 时钟 |
| PA14 / SPI0_PICO | 板底 J28-7 | 32 | TI 写 FPGA |
| PA13 / SPI0_POCI | BoosterPack 31 | 33 | FPGA 返回数据 |
| PA15 / GPIO | BoosterPack 30 | 34 | 低有效软件片选 |
| GND | 任一 GND | 30 或 12 | 共地 |

STM32 与 TI 板不能同时连接并驱动这组 SPI。TI SPI0 使用模式0、8位、MSB优先、1 MHz。J28 位于 LaunchPad 板底且没有信号丝印，PA14 必须按排针缺口和 1 脚方向定位。

S1 为 AD9708 DAC 输出，S2 为 AD9280 ADC 输入。模块使用稳定 +5 V 供电，FPGA GPIO 仅允许 3.3 V 电平。
