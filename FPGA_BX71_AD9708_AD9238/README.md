# BX71 + ACM9238：2026 G 题 FPGA 工程

这是从 `FPGA_BX71_AD9708_AD9280` 独立复制并改名得到的 AD9238 版本。原 AD9280 工程不参与本目录的构建，也未被本次迁移覆盖。

本工程只改变 ADC 相关链路：ACM9238 接口、12 位采样、50 MHz 时钟、R=25 抽取、寄存器身份和板级约束。G 题既有 SPI、4096 点缓存、Q8.8 数据格式、PS7/QSPI 启动流程和静默 AD9708 输出保持原设计意图。

## 硬件结论

依据 `C:\Users\LENOVO\Desktop\Electronic_Modules\ACM9238` 中的原理图和例程：

- ACM9238 为双通道、12 位、偏移二进制输出；CH1 保持原校准测量链，CH2 接 15 MHz 模拟低通后的无放大信号，只用于抗混叠一致性判决。
- 模块例程给两个 ADC 通道都提供 50 MHz 时钟；本工程同样驱动 `CH1_CLK` 与 `CH2_CLK`。
- 模块前端标称 ±5 V：SMA 后为 4 kΩ/1 kΩ 衰减、AD8065、AD8138 差分驱动以及 40 Ω/20 pF ADC 输入网络。
- 不通过降低 ADC 时钟获得 2 MSPS；FPGA 在 50 MSPS 下使用 CIC3/R25 抽取。

AD9238 官方器件资料：双通道 12 位流水线 ADC，输出延迟约 2–6 ns。模块上的 DFS 接地，因此使用偏移二进制。器件页：<https://www.analog.com/en/products/AD9238.html>。

## ACM9238 直插 BX71 P3

P1 与 P3 按同号引脚对齐；P1-11/P3-11 为 +5 V，P1-12/P3-12 为 GND。

| ACM9238 P1 | 信号 | BX71 FPGA |
|---:|---|---|
| 1 | CH1_D0 | P15 |
| 2 | CH1_CLK | P14 |
| 3 | CH1_D2 | N17 |
| 4 | CH1_D1 | P16 |
| 5 | CH1_D4 | R18 |
| 6 | CH1_D3 | P18 |
| 7 | CH1_D6 | U17 |
| 8 | CH1_D5 | T17 |
| 9 | CH1_D8 | Y19 |
| 10 | CH1_D7 | W19 |
| 13 | CH1_D10 | V18 |
| 14 | CH1_D9 | R16 |
| 15 | CH1_OTR | W16 |
| 16 | CH1_D11 | T16 |
| 17 | CH2_D1 | T15 |
| 18 | CH2_D0 | U15 |
| 19 | CH2_D3 | T14 |
| 20 | CH2_D2 | U14 |
| 21 | CH2_D5 | W18 |
| 22 | CH2_D4 | T12 |
| 23 | CH2_D7 | R14 |
| 24 | CH2_D6 | U13 |
| 25 | CH2_D9 | V17 |
| 26 | CH2_D8 | Y18 |
| 27 | CH2_D11 | V16 |
| 28 | CH2_D10 | Y17 |
| 31 | CH2_OTR | U19 |
| 32 | CH2_CLK | Y16 |

完整约束见 `constraints/acm9238_p3_spi_gpio0.xdc`。两路数据均进入顶层；CH2 使用 13 个 DSP48 的多相 FIR307/R25 参考核，在抽取前抑制 1 MHz 以上分量，同时避免复制 CH1 的 154-DSP 全速 FIR。

ACM9238 占用 P3 后，AD9708 不能再与它同时直插 P3。为了保留原 DAC 逻辑，本工程把静默中点输出映射到 P2 `GPIO0_0…8`（数据 L14…F17，时钟 E17）；G 题镜像不会启用实验 AM/FM 输出。

## 采样与协议

- ADC 物理采样：50,000,000 sample/s，12 位偏移二进制。
- G 题分析流：FIR307 抗混叠后接 CIC3/R25，2,000,000 sample/s，最多 4096 点。
- 原始旁路和 CIC 输出都使用 16 位有符号 Q8.8 传输。
- 12 位中心码 `2048` 映射为 `0`；一个原生 ADC 码映射为 16 个 Q8.8 传输计数。
- 标称 ±5 V 时，原生 ADC LSB 为 `10/4096 V`；Q8.8 传输 LSB 为 `10/65536 V`。16 位传输保留 12 个原生信息位和 4 个 CIC 分数位，并未退回 AD9280 的 8 位精度；实际测量仍需做增益/零点标定。

寄存器 `0x20…0x2F` 沿用原地址，AD9238 版本的关键变化如下：

| 地址 | 含义 | AD9238 值/布局 |
|---:|---|---|
| `0x23` | 抽取比 | G 模式必须为 25 |
| `0x24` | 触发 | `[17:16]` 模式，`[11:0]` 12 位阈值 |
| `0x29` | ADC 时钟 | `50,000,000` |
| `0x2A` | 设备 ID | `0xAD923801` |
| `0x2B` | 协议版本 | `0x00020004` |
| `0x2F` | 滤波 | bit0：使能 FIR307 + CIC3/R25 |
| `0x30` | 读回通道 | bit0：0=CH1，1=CH2；启动、配置和终止命令仍同时作用于两路 |

`READ_DATA` 仍为 `[31:28]=0`、`[27:16]=地址标签`、`[15:0]=有符号 Q8.8`，SPI 仍按 MSB first 发送。

## 构建与验证

在 PowerShell 中执行：

```powershell
cd C:\Users\LENOVO\Desktop\NUEDC_2026\FPGA_BX71_AD9708_AD9238
& .\scripts\Build-QspiBoot.ps1
```

脚本使用 Vivado/SDK 2018.3，重新发现 RTL/XDC/BD 输入、创建干净工程、完成综合布线、检查全局与 ADC setup/hold、检查 DRC Error、生成 PS7 bitstream 和 QSPI BOOT 镜像，并写出源码快照与 SHA256：

- `output/BX71_G_ANALYZER_AD9238_PS7.bit`
- `output/BOOT_G_ANALYZER_AD9238.bin`
- `output/LATEST_G_AD9238_QSPI_BUILD.txt`
- `output/SOURCE_SNAPSHOT_G_AD9238_QSPI.txt`
- `output/SHA256SUMS_G_AD9238_QSPI.txt`

单独仿真入口：

```powershell
vivado -mode batch -source .\scripts\run_g_fir_sim.tcl
vivado -mode batch -source .\scripts\run_g_cic_sim.tcl
vivado -mode batch -source .\scripts\run_ch2_reference_sim.tcl
vivado -mode batch -source .\scripts\run_legacy_sim.tcl
vivado -mode batch -source .\scripts\run_promax_sim.tcl
```

STM32 配套副本位于同级 `STM32H743_AD9238`，必须与设备 ID `0xAD923801`、协议 `0x00020004`、50 MHz/R25 参数成套使用。

### FIR307 + CIC3/R25 抗干扰、通带与相位

为满足题目第三问，滤波必须发生在 50 MSPS 抽取之前；仅在 2 MSPS 数据上处理无法阻止 1.6 MHz 等干扰混叠到 400 kHz。当前链路为：

`AD9238 12-bit @ 50 MSPS -> FIR307(Q16, linear phase) -> CIC3/R25 -> S16Q8.8 @ 2 MSPS`

- FIR 通带 `0..500 kHz`，阻带从 `1 MHz` 开始。量化后通带纹波 `0.017978 dB`，通带最大偏差 `0.014214 dB`，最差阻带衰减 `52.73 dB`。
- FIR 群延迟固定为 153 个 50 MHz 时钟；当前 CIC 实际信号延迟为 38 个时钟，因此总线性相位延迟为 191 个时钟（3.82 us）。STM32 按相同系数逐频点补偿幅度和这 191 时钟相位。共同线性延迟不会改变 `phi_h-h*phi_1`，也不会破坏一周期/三周期完整波形的谐波相对相位。
- FIR 输出保留 signed Q4，CIC 使用 `274878/2^32` 实现 `/15625`。实际 CIC 直流比例为 `274878*15625/2^32=1.0000003385357559`，没有额外乘 16 或高频增益。
- FIR 独立实现使用 154 个 DSP48E1；独立 XC7Z020 布局布线在 50 MHz 下 WNS `+9.090 ns`、WHS `+0.052 ns`。
- 当前双通道抗混叠顶层布局布线后使用 12,621 LUT、15,806 FF、169 DSP48E1、4 RAMB36；其中 CH2 参考 FIR 使用 1,674 LUT、904 FF、13 DSP48E1。WNS +6.104 ns、WHS +0.060 ns，0 个时序失败端点、0 个 DRC Error。

位精确仿真在 1、1.6、5、10 MHz 对约 200 mVpp 干扰的 FIR 后残余分别为 `0.266164、0.218046、0、0.375694 mVpp`。100/200/400/500 kHz 通带、307 点填充、9 拍实现流水、153 点冲激群延迟、CIC 弱信号分数精度和完整采集协议均有独立回归。

数字滤波只能处理已被 50 MSPS ADC 正确采样的 1–25 MHz 干扰；超过 25 MHz 的模拟信号若先在 ADC 处混叠进 0–500 kHz，数字逻辑无法恢复，仍需模块模拟前端承担最终抗混叠边界。
## 验证边界

通过 RTL 仿真、Vivado 构建和生成 BOOT 镜像，只能证明数字逻辑、静态时序、DRC 与可打包性。它不等于已经写入 QSPI，也不等于实板模拟精度已验证。上板后仍需用已知幅值/频率源核对 CH1 时钟、D0…D11、OTR、零点、满量程和 G 题 5 mV 指标；未完成这些测量前，不应把本工程描述为硬件闭环通过。
