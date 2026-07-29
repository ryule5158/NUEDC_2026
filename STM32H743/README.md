# STM32H743 集成工程

> 当前工程固定运行远端 G 题方案：AD9280 → FPGA → SPI2 → STM32 周期信号分析。
> SPI2 MOSI 使用 **PB15 → BX71 L19/P2-37**。

## 快速入口

| 项目 | 位置或状态 |
|---|---|
| Keil 工程 | `MDK-ARM/STM32H743.uvprojx` |
| CubeMX 配置 | `STM32H743.ioc` |
| 默认功能 | GContest：AD9280/FPGA 采集与周期信号分析 |
| 构建结果 | ArmClang 6.23：`0 Error(s), 0 Warning(s)` |
| 固件输出 | `output/STM32H743.hex`、`output/STM32H743.axf` |

```powershell
Set-Location C:\Users\21274\Documents\省赛\d_repo_latest_precision\STM32H743
.\build_keil.ps1
```

## 默认串口屏与 USART1 测试命令

陶晶驰串口屏是默认交互入口，连接 USART2（PA2 TX、PA3 RX，115200-8-N-1）。
屏幕按钮发送 `G1`、`G3`、`GS`，分别请求 1 周期波形、3 周期波形和频谱页面。
USART1 串口助手与屏幕同时保持可用，不需要改宏或重新编译；连接 PA9 TX、PA10 RX
（115200-8-N-1）后，正常测试只需发送一个数字：

```text
1         采集分析一次，打印 1 个完整周期，然后自动打印测量值和频谱
3         采集分析一次，打印 3 个完整周期，然后自动打印测量值和频谱
```

数字 `1`、`3` 可以直接作为单字节发送，不要求添加回车或换行。发送 `1` 后不需要再发任何内容，单片机会自动完成采样、分析、输出一个周期、输出峰峰值/有效值/基频，再输出 10 kHz～500 kHz 内检测到的各个频谱峰值。

辅助诊断命令 `HELP`、`INFO`、`STATUS`、`PING` 仍然可用，文字命令需要以回车或换行结束；它们不启动采集。

### 原始 Q8.8 两点电压标定

串口助手还提供独立的直流标定模式，所有文字命令均以回车或换行结束：

```text
CAL       进入标定模式
Q         连续采集 16 帧，每帧 4096 点，打印原始 Q8.8 统计值
EXIT      退出标定模式，恢复 1/3 正常测量
```

进入后先把稳定直流加到 AD9280 S2，并用万用表在 S2 实测电压，再发送 `Q`。最终
`[CAL] RESULT Q=...` 是 16 个帧均值的中值，`pooled_Q` 是 65536 点合并平均值，
`frame_span_Q` 用来观察漂移。该路径直接读取 FPGA 的整数累加和、最小值、最大值
和 OTR，不进行名义伏特换算、CIC 频响补偿或经验校准，也不搬运 16×4096 个波形点。
只有 `clip=0`、`overrange=0` 且帧间跨度稳定的结果可用于

```text
a = (VH - VL) / (QH - QL)
b = VL - a * QL
V = aQ + b
```

当前固件只负责可靠打印 Q，不会自动保存或启用尚未复核的 `a`、`b`。

上电初始化成功后会收到：

```text
[CLI] READY USART1 115200 8N1; send 1/3, or CAL for raw Q
```

发送 `1` 后输出顺序固定为：

```text
[CLI] command=1 accepted; capture starts now
[G] frame=1 capture_analyze=... model_lines=...
[WAVE] begin cycles=1 points_per_cycle=256 ...
-466.930695
-466.863434
...
[WAVE] end
[MEAS] Vpp=...mV RMS=...mV F0=...Hz residual_RMS=...mV clip=...
[REJECT] expected=1000000Hz detected=... frequency=...Hz post_dsp_peak=...mV post_dsp_rms=...mV
[SPEC] begin band=10000..500000Hz count=... columns=...
SPEC,谐波序号,频率_Hz,峰值幅度_mV,分量峰峰值_mV
...
[SPEC] end
[DONE] frame=1
```

每周期输出 256 个等相位点，`[WAVE] begin` 和 `[WAVE] end` 之间每行只有一个电压数值，单位为 mV，可直接复制到串口绘图或数据处理软件。这里的波形是根据检测到的基波和谐波拟合后重建的完整周期，因此起点和终点严格按一个周期组织，不受原始 4096 点采样帧是否刚好从周期边界开始的影响。频谱按频率升序打印 10 kHz～500 kHz 内全部有效峰，同时给出峰值和峰峰值；`H1`、`H2`、`H3` 等已识别谐波使用联合拟合幅值，其余有效峰标为 `C1`、`C2` 等并使用 FFT 峰值。低于 0.5 mV 峰值或低于最强分量 1% 的噪声峰不作为有效频率分量打印。

### 精度与校准状态

- 当前明确关闭频率响应幅相校准表：`cal_count = 0`。启动日志会打印
  `empirical_frequency_calibration=OFF analytic_response_compensation=ON
  voltage_transfer=nominal`。
- AD9280 的物理分辨率仍是 8 位，名义物理步长为
  `10 V / 256 = 39.0625 mV/code`。FPGA CIC3/R16 的输出改为 signed Q8.8，
  传输步长为 `10 V / 65536 = 0.152588 mV`，保留 16 次过采样产生的亚码信息。
  这不会把 ADC 变成真正的 16 位器件。
- FPGA/STM32 协议固定为 v1.4（`0x00010004`），数字响应 profile 固定为
  CIC3/R16（`0x00030010`）。驱动还会核对只读抽取元数据必须为 `16`；
  任一项不匹配都明确返回 `SAMPLE_FORMAT`，因此 FPGA 与 STM32 必须配套更新。
- 当前伏特换算仍使用模块名义 `±5 V` 量程，启动日志标记为
  `voltage_transfer=nominal`。当前实物信号在采集前没有放大，程序报告的是
  AD9280 模块输入端的名义电压；`0 V` 附近对应居中的 signed Q8.8 数据。
- 4096 点数据先由 FFT 找候选谱线，再对全部有效谐波做双精度联合最小二乘。
  基频通过多级、受频带约束的残差最小化继续细化，最终幅值、相位、RMS 和重建
  波形都使用同一组联合拟合参数。
- CIC3/R16 的幅值与线性相位是确定的，AD9280 后端按精确解析式向分析器提供
  `|H(f)|` 和 `arg H(f)`，分析器对每条拟合谱线施加复数逆响应。该步骤会撤销
  10 kHz～500 kHz 内的 CIC 幅相变化，不依赖经验校准表。
- 1 MHz 抗干扰不是对采样数据做低通或陷波。分析器在所有联合最小二乘中加入
  一个不参与被测结果的 nuisance 正弦分量；在 2 MSPS 的精确奈奎斯特点只加入
  可观测的交替序列列。这样被测分量仍按原频率拟合，关闭/开启且没有干扰时结果
  保持一致。`[REJECT]` 只报告被分离出的干扰，不加入 10～500 kHz 频谱。
  其中 `post_dsp_peak/rms` 是 CIC 输出采样域诊断值，不做 CIC 逆响应或经验
  校准；精确 1 MHz 时只代表实际可观测的奈奎斯特交替投影。
- 在整机扫频和幅值标定完成前，程序不会猜测模拟链路、ADC 模块增益和非线性；
  削顶、混叠和模拟误差也不能由软件补偿消除。

### ADC 后端切换

G 题业务层只依赖 [`BSP/signal_adc.h`](BSP/signal_adc.h)，接收单位为伏特的
`float` 数据、该帧真实采样率以及后端声明的确定性数字幅相响应。当前唯一参与编译的实现是
[`BSP/signal_adc_ad9280.c`](BSP/signal_adc_ad9280.c)，AD9280 的 8 位物理码、
32 MSPS、FPGA signed Q8.8 传输、CIC3/R16、名义电压换算和削顶阈值都只存在
于这个后端及其底层驱动中。

更换 ACM9238/AD9238 时，新建 `signal_adc_acm9238.c` 并实现同一组
`SignalAdc_*` 函数，然后在 Keil 工程中用它替换
`signal_adc_ad9280.c`。两个适配器不能同时参与编译；DSP、串口命令和
`g_contest_app.c` 不需要修改。新后端必须如实填写物理采样率、输出采样率、
码宽、传输 LSB、原始统计采集、削顶条件及覆盖 10～500 kHz 的确定性数字响应回调；如果新链路
在该频带内是平坦直通，可返回单位幅值和零相位。若并行位宽、时序或满量程不同，
还需增加对应的 FPGA 输入适配和底层寄存器驱动，但这些变化不进入分析器业务层。

## G 题接线与信号路径

```text
被测电压（当前无模拟放大） -> 8位 AD9280 -> FPGA 固定 CIC3/R16
FPGA signed Q8.8 4096点缓存 -> SPI2 -> STM32 解析幅相补偿
-> FFT/联合谐波与1MHz nuisance拟合 -> USART2串口屏 + USART1串口助手
```

### STM32 与 BX71

| STM32H743 | BX71 P2 / 球位 | 作用 |
|---|---|---|
| PI1 / SPI2_SCK | P2-31 / K19 | SPI 时钟 |
| PB15 / SPI2_MOSI | P2-37 / L19 | 写寄存器和波形数据 |
| PI2 / SPI2_MISO | P2-33 / K17 | 状态与寄存器回读 |
| PE8 / GPIO | P2-34 / K18 | 低有效片选 |
| GND | 任一 GND | 必须共地 |

默认串口屏为 USART2：PA2 TX、PA3 RX；调试串口为 USART1：PA9 TX、PA10 RX；
两路均为 115200-8-N-1。

### 模拟输入安全

- AD9280 输入电压就是当前固件换算的被测电压，输入必须满足模块允许范围，并先用示波器检查过冲。
- 输入信号不能让 AD9280 输出码值长期贴近 0 或 255，否则会产生削顶。
- 信号源、STM32、BX71、AD/DA 模块和示波器必须共地。
- 当前高频采样统一使用 FPGA + AD9280，STM32 片上 ADC 不参与 G 题主流程。

## 模块接线速查

| 模块 | STM32 接线 | 说明 |
|---|---|---|
| FPGA / AD9280 G题采集 | PI1 SCK、PB15 MOSI、PI2 MISO、PE8 CS | 当前生产位流只提供 G 题 ADC 寄存器 |
| AD5687 | PA5 SCK、PA7 MOSI、PE4 CS | 两路 CV 共用片选 |
| AD603 | AD5687 CV1 → P2-2，GND → P2-3 | 拔掉 P2 电位器跳帽；推荐约 ±5.3 V、50 Ω |
| DAC8831 | PA5 SCK、PA7 MOSI、PA4 CS | 与 AD5687 共用时钟和数据 |
| ADS8363 | SPI2、PB12 CS、PH8 CONVST、PH9 BUSY | 与 FPGA 共用 SPI2，切换后要恢复配置 |
| ADS8688 | PA5 SCK、PA6 MISO、PB5 MOSI、PA4 CS | 使用 SPI1 |
| PE4302 | LE、CLK、DATA 由配置结构体绑定空闲 GPIO | 0～31.5 dB，0.5 dB 步进 |

### AD9910 接线

| 信号 | STM32H743 |
|---|---|
| SCK / SDIO / CS | PE2 / PE6 / PB9 |
| RESET / IO_UPDATE | PA6 / PB8 |
| PWR / DROVER | PC13 / PA0 |
| DRCTL / DRHOLD | PC8 / PC1 |
| PF0 / PF1 / PF2 | PB5 / PC6 / PC7 |
| OSK | PB7 |

当前 PLL 不调用 AD9910，驱动仅为其他题目保留。默认
`AD9910_SYSCLK_HZ=1 GHz`，更改模块时钟后必须同步修改该宏。

## 最小调用示例

```c
#include "Modules_Header.h"
#include "BSP_Header.h"

/* AD603：通过 AD5687 CV1 设置 10 dB。 */
AD603_HandleTypeDef ad603;       /* AD603 运行句柄。 */
AD603_ConfigTypeDef ad603_cfg;   /* 两点电压校准参数。 */

AD603_GetDefaultConfig(&ad603_cfg);
ad603_cfg.dac_output = AD5687_OUTPUT_CV1;
if (AD603_Init(&ad603, &ad603_cfg) == AD603_OK) {
    (void)AD603_SetGainMdb(&ad603, 10000L);
}

/* FPGA 高速 ADC：固定 CIC3/R16，采集 1024 点，超时 100 ms。 */
if (AD9280_AppInit() == AD9280_OK) {
    (void)AD9280_AppCapture(1024U, AD9280_DECIMATION_FIXED, 100U);
}
```

AD603 默认把 -20～60 dB 映射为 0.400～1.446 V；增益单位为 mdB，
`10000` 表示 10 dB。实测端点不同只需修改校准结构体。

## 板载存储

| 存储器 | 器件 | 容量 | 接口 |
|---|---|---:|---|
| SDRAM | W9825G6KH | 32 MB | FMC SDRAM Bank1 |
| NAND | MT29F4G08 | 512 MB | FMC NAND Bank3 |
| QSPI Flash | W25Q256 | 32 MB | QUADSPI Bank1 |

```c
BSP_StorageProbeResult probe; /* 三类存储器的探测结果。 */

if (BSP_Storage_InitAll(&probe) != BSP_STORAGE_OK) {
    Error_Handler();
}
```

NAND 驱动包含坏块检查和硬件 ECC，但不包含 FTL 或文件系统。

> **PE8 分时复用：** PE8 同时连接 FPGA_CS 和核心板 FMC_D5。两类功能均保留，
> 但不能同时工作。使用存储前先停止 FPGA SPI；切回 FPGA 时必须重新配置
> FMC/GPIO，并恢复 PE8 输出和 SPI2。

## CubeMX 配置边界

`.ioc` 已与以下可再生配置一致：

- SPI2：PI1 SCK、PB15 MOSI、PI2 MISO；
- FPGA 片选：PE8，初始高电平；
- AD5687 片选：PE4，初始高电平；
- STM32 只通过 SPI2 读取 FPGA 已完成固定 CIC3/R16 抽取的 4096 点数据。

以下是保留在源码中的手工板级配置，不能由当前 `.ioc` 完整再生：

- AD9280 原始 32 MSPS 数据由 FPGA 完成固定 CIC3/R16 抽取；
- STM32 负责名义电压换算、CIC 解析幅相补偿、FFT、谐波/1 MHz 联合拟合、
  串口屏显示和串口助手输出。
- SDRAM、NAND、QSPI。

因此不要直接让 CubeMX 覆盖生成。若必须生成，先备份并复核
`main.c`、`adc.c`、`tim.c`、`dac.c`、HAL 宏和 Keil 源文件列表。

## DSP 库选择

| 模块 | 用途 | 重要限制 |
|---|---|---|
| `FFT` | FFT、Goertzel、谐波分析 | 使用全局工作区，不可重入 |
| `FilterEx` | FIR/SOS、RMS、包络、分数延时、抽取 | 对象和缓冲区由调用者持有 |
| `IQ` | 单频幅相、双通道相差 | 采样率必须真实准确 |
| `SoftPll` | 软件锁相控制 | 调用者负责更新 DDS/NCO |
| `Demod` | AM/FM/BPSK 解调 | 连续流应保留 Context |
| `Measure` | RMS、频率、THD、SNR、SINAD、ENOB | 高频 THD 注意奈奎斯特边界 |
| `Adaptive` | LMS、自适应陷波和正弦分离 | 步长需上板整定 |

## 已完成验证

- STM32 全量构建：0 错误、0 警告。
- FPGA/STM32 signed Q8.8 协议版本一致，旧 8 位格式会被拒绝。
- THD C 测试及数值回归通过。
- 周期分析器主机回归通过。
- `FilterEx` 分数延时 C 测试通过。
- 自适应算法数值回归通过。

本次修改没有重新完成全部实体板复测；实际模拟增益、噪声和整板 QSPI 冷启动
仍应赛前复验。

## 本机工具

| 工具 | 路径 |
|---|---|
| Keil | `C:\Keil_v5\UV4\UV4.exe` |
| ArmClang | `C:\Keil_v5\ARM\ARMCLANG\bin\armclang.exe` |
| STM32CubeMX | `D:\cubemx\新建文件夹\STM32CubeMX.exe` |
| CubeProgrammer | `C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32CubeProgrammer.exe` |
