# DSP ProMax 电赛使用指南

这份指南面向“不需要先会写 FPGA”的 STM32 使用者。当前实时档的目标不是把
STM32 的 DSP 代码照搬进 FPGA，而是把必须逐样点、持续、高并发执行的部分交给
FPGA；STM32 负责参数、校准、题目状态机、识别决策和串口屏。

## 1. 数据如何流动

```text
AD9238 50 MSPS
      |
      +--> 8 路 DDC --> 16 个 I/Q CIC --> 8 路功率统计 --+
      |                                                  |
      +--> 4 路 × 32 阶 FIR/相关器（128 路乘法）--------+--> 结果快照 --> SPI --> STM32
      |                                                  |
      +--> 连续波形统计（DC/RMS/Vpp/频率/占空比）--------+
```

三条路径对同一个 ADC 样本流同时运行。配置完成后，STM32 不需要按样点发送
数据，也不需要定时“调用一次滤波”；FPGA 每得到一个有效样点就推进全部流水线。
SPI 只传几十字节的配置和结果，因此不会成为 50 MSPS 数据面的瓶颈。

## 2. 真正体现 FPGA 并行能力的部分

| 硬件路径 | 并行规模 | 适合任务 | STM32 接口 |
|---|---:|---|---|
| 通用 FIR | 4 组 × 32 阶 | 低通、高通、带通、脉冲整形、边沿增强 | `FpgaPromax_LoadFirBank()` |
| 相关/匹配滤波 | 4 组 × 32 点 | 帧头、码型、脉冲、同步序列检测 | `FpgaPromax_LoadCorrelationTemplate()` |
| 选频功率 | 8 路 DDC + 16 个 CIC | 多音判别、FSK、谐波/边带监测 | `FpgaPromax_ConfigureToneBank()` |
| 连续测量 | 1 条满速统计流水线 | DC、RMS、AC RMS、Vpp、频率、占空比 | `FpgaPromax_ReadWaveStats()` |

4 路 FIR 是同时执行，不是先算 bank 0 再轮询 bank 1。每路每拍接收一个样点，
32 阶系数均为 Q1.16，整数 `65536` 表示 `1.0`，允许范围为
`-131072`～`131071`。普通 FIR 按原顺序装载；相关检测会自动把模板反序。

当前 SPI 快照提供 FIR 的峰值和峰值样本位置，适合检测、测量和触发。它没有把
50 MSPS 的连续滤波后波形送回 STM32，因为普通 SPI 无法承载该速率。如果题目
要求“把滤波后的连续波形输出到接口”，应在 FPGA 内把某个 FIR 输出接到 BRAM、
AD9708 或其他硬件后级，这是后续数据面扩展，不应尝试通过 SPI 搬运全部样点。

## 3. STM32 与 FPGA 的职责边界

根据工程现有 STM32 DSP 库以及往年题目资料，推荐边界如下。

| 电赛考点 | 首选执行端 | 原因/现有入口 |
|---|---|---|
| 连续实时滤波、并行滤波、相关触发 | FPGA | 必须逐样点持续运行；使用 4 路 FIR |
| 多频点同时监测、FSK/多音预判 | FPGA | 8 路 DDC 同时工作，不需要做整帧 FFT |
| 高频波形 DC/RMS/Vpp/频率/占空比 | FPGA 统计，STM32 换算 | 不搬运采样块，`ComputeWaveMetrics()` 做标定 |
| AM 载波与上下边带功率 | FPGA 采集，STM32 计算 | 3 路 DDC 并行，`ReadAmEstimate()` |
| FFT 频谱、THD、H1～H5、波形识别 | STM32 | 现有 `DSP/FFT`、`Measure`、`THD` 已成熟；当前 FPGA 位流无 FFT |
| AM/FM/BPSK 完整解调 | STM32 | 现有 `Demod`、`IQ`、`SoftPll` 更适合帧级状态算法 |
| 自适应滤波系数更新 | STM32 决策 | `Adaptive` 已有算法；若采样率很高，可把固定后的系数下发 FPGA FIR |
| 拟合、模型识别、参数估计 | STM32 | `Fit`、`ModelFit`、`Correlate` 易改、浮点方便 |
| UI、自动量程、校准、题目流程 | STM32 | 低速控制和复杂分支不是 FPGA 优势 |

不要仅为了“用了 FPGA”重复实现已经稳定的 STM32 FFT、THD 或拟合。FPGA 的
优势应通过满速、连续、并行数据面体现；STM32 的优势是算法灵活、浮点和业务状态
机易维护。

## 4. 往年题型覆盖

下表是对资料目录中 2011～2025 年信号类考点的归纳。它表示“现有积木如何组合”，
不是声称所有整题已经自动完成。

| 题型 | 推荐积木 | 当前状态 |
|---|---|---|
| 频率计、占空比、脉冲参数 | 连续统计 + STM32 显示/量程 | 可直接调用 |
| 频谱仪、谐波、THD、功率分析 | FPGA 8 点预监测 + STM32 FFT/THD | 可组合 |
| AM/FM 识别与测量 | FPGA 载波/边带预检 + STM32 Demod/SoftPll | AM 边带接口已提供；整机需标定 |
| ASK/FSK/PSK、位同步、码型识别 | DDC + 相关器 + STM32 判决/PLL | 硬件预处理可用；协议状态机按题实现 |
| 自适应滤波、信号分离 | STM32 更新系数 + FPGA 4 路 FIR | 固定系数实时路径可用 |
| TDR、线缆衰减、脉冲定位 | 相关峰值与样本位置 + STM32 距离换算 | 短模板可用；模拟前端/标定另做 |
| 声音定位、音调识别 | STM32 音频 DSP；高并发场景可用 DDC | 现有 STM32 更合适 |
| 幅频扫频、网络/模型拟合 | AD9708/AD9910 激励 + FPGA 测量 + STM32 Fit | 需按题编写扫频状态机 |
| 眼图、波形采集与显示 | AD9238 触发采集 + STM32/UI | 采集可用；眼图叠加属业务层 |

## 5. 初学者固定调用顺序

1. `FPGA_ProMax_Init(&dev)`，确认返回 `FPGA_PROMAX_OK`。
2. `FpgaPromax_HasRealtime(&dev)` 和 `FpgaPromax_HasWaveStats(&dev)` 必须为 1。
3. `FpgaPromax_SetRun(&dev, 0)` 停止数据面。
4. 装载全部 FIR 系数、相关模板和 DDC 频点。
5. `FpgaPromax_SetWaveBlockLog2(&dev, N)` 设置 `2^N` 点统计块。
6. `FpgaPromax_ClearState(&dev)` 清掉配置期间的旧峰值。
7. `FpgaPromax_SetRun(&dev, 1)` 一次启动，之后连续运行。
8. 主循环按 UI 刷新频率读取快照；不要在中断内操作 SPI。

可直接复制的代码见
`STM32H743_AD9238/Examples/fpga_promax_contest_example.c`。

## 6. 常用配置建议

- 波形统计默认 `N=12`，即 4096 点；50 MSPS 下约 81.92 us 更新一次。
- 测低频时增大块长，保证一个块内至少出现两个完整上升沿。
- 测快速变化时减小块长；允许范围是 `N=6`～`25`。
- FIR 系数先在 MATLAB/Python/STM32 工具中归一化到 Q1.16，确保单个系数不越界，
  同时评估总增益，避免 24 位输出饱和。
- 带通采样输入可直接传真实 RF 频率给 `ConfigureToneBank()`，库会折叠到
  `0～fs/2`。模拟前端必须先保证目标带宽满足带通采样条件并抑制混叠干扰。
- AM 调制度公式假设普通单音 AM 且三个 DDC 通道增益一致；最终成绩必须用已知
  调制度信号做整机增益校准。

## 7. 当前明确未实现的能力

- 当前实时位流能力字是 `0x29200408`，没有硬件 FFT；FFT API 会返回
  `FPGA_PROMAX_E_UNSUPPORTED`。
- 4 路 FIR 只有峰值/位置快照，没有通过 SPI 返回连续滤波波形。
- 没有硬件 LMS/RLS 系数自更新；由 STM32 计算系数后下发。
- 数字仿真通过不等于模拟链路已校准，AD9238 前端、输入幅度、带宽和混叠仍需实测。

这些边界是有意保留的：先把可复用且确实体现 FPGA 优势的实时流水线做好，再针对
新题增加必要的数据出口或专用核。

## 8. 本次扩展验证（2026-07-26）

- STM32 ProMax 主机回归：`FPGA_PROMAX_HOST_TEST_PASS`。
- 现有 THD 数值回归：`measure_thd_numeric_test: PASS`。
- Keil ArmClang 6.24 全量重建：`0 Error(s), 0 Warning(s)`。
- Vivado 2018.3 行为仿真：`PROMAX_INTEGRATION_TEST_PASS` 与
  `PROMAX_SIM_PASS` 均出现；目标 DDC 功率为 `2423158473`，非目标通道为
  `61527`。
- 仿真覆盖寄存器、异步 FIFO、4 路 FIR 配置、快照冻结、DDC 选频、波形
  min/max/和/平方和、50% 占空比、上升沿周期和样本数。

本次没有执行综合、实现、生成 bitstream 或重新打包 `BOOT_PS7.bin`，因此板上
当前运行/固化的仍是修改前镜像。需要板上使用这些新能力时，必须另行全量构建并
烧录；执行前应先确认将被覆盖的 bitstream 和启动镜像。
