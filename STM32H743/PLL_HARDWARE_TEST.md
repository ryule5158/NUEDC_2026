# STM32H743 双 ADC + AD9910 PLL 实测说明

## 先确认反馈电压安全

`PC4`、`PC5` 只能接受 `0–VDDA`。AD9910 模块的 SMA 输出通常经过变压器或
交流耦合，可能带负半周，不能未经测量直接接入 `PC5`。

先只把 AD9910 输出接示波器，确认幅度和偏置。送入 `PC5` 的最终波形建议以
1.65 V 为中心、约 1.0–2.0 Vpp，并确保过冲始终在 0–3.3 V 内。

一种适合低频自测的反馈调理方式：

```text
AD9910 OUT ── 隔直电容 ── 1 kΩ ──┬── PC5
                                  ├── 10 kΩ ── 3.3 V
                                  └── 10 kΩ ── GND
```

需要时在 PC5 对 3.3 V/GND 增加肖特基钳位。隔直电容可先用 1–10 µF；
测试 10 Hz 时优先使用 10 µF 双极性电容或合适的有源偏置电路。示波器先验证
PC5 节点安全，再连接 MCU。

## 信号和测量接线

所有设备必须共地，断电完成接线。

1. 信号源 OUT 同时接示波器 CH2 和 `PC4`（ADC1_INP4，参考输入）。
2. 信号源 GND、示波器 GND、AD9910 GND、STM32 GND、CH340 GND 共地。
3. AD9910 OUT 接示波器 CH1；另一路经过上述衰减/隔直/1.65 V 偏置后接
   `PC5`（ADC2_INP8，反馈输入）。
4. CH340 TXD → `PA10`（USART1_RX）。
5. CH340 RXD → `PA9`（USART1_TX）。
6. CH340 使用 3.3 V TTL，不要把 CH340 的 5 V 接到 MCU。

信号源送入 PC4 时同样必须满足 0–3.3 V。建议从 1 kHz 正弦、
1.65 V Offset、1.0 Vpp、High-Z 显示开始。

## AD9910 控制线

当前 `Modules/ad9910.c` 的引脚定义如下：

| AD9910 信号 | STM32H743 |
| --- | --- |
| SCK | PE2 |
| SDIO | PE6 |
| CS | PB9 |
| IO_UPDATE | PB8 |
| RESET | PA6 |
| PF0/PF1/PF2 | PB5/PC6/PC7 |
| OSK | PB7 |
| PWR | PC13 |
| DRCTL/DRHOLD | PC8/PC1 |
| DROVER | PA0 |

AD9910 的参考时钟和供电按模块要求连接。驱动当前按 40 MHz 参考、
片内 PLL 后 `AD9910_SYSCLK_HZ = 1 GHz` 计算 FTW；如果你的模块参考时钟
不同，必须先修改该宏和 CFR3，不能靠环路参数补偿时钟比例错误。

## 当前固件结构

- `PC4=ADC1`、`PC5=ADC2`，由同一个 TIM6 TRGO 以 500 kS/s regular
  simultaneous 同步采样。
- ADC 使用 circular DMA 双半区，每半区 1024 组同步样本；CPU 处理已释放
  半区时，DMA 同时采集另一半区。
- AD9910 只在启动时初始化一次。PLL 更新使用单次 Profile0 写入，同时更新
  FTW、POW、ASF，并只产生一次 IO_UPDATE；不会周期复位 DDS。
- AD9910 自己连续产生波形，因此不需要片上 DAC 的输出 DMA 双缓冲。
- `SoftPll` 仍只输出 `dds_freq_cmd_hz`、`phase_step_deg`，不依赖 AD9910。
- 参考输入可为正弦、方波或三角波；AD9910 输出为锁定其基波的正弦。
- 参考消失后进入 `HOLD`，保持最后的连续 AD9910 输出，不执行间歇复位。
- ADS8688 代码和工程项全部保留；片上 DAC 库也保留，但本测试不初始化 PA4
  或 DAC1。

当前直接反馈测试上限为 50 kHz，并非 AD9910 的上限。片上 ADC 以
500 kS/s 直接采样无法无歧义锁定 0–40 MHz。0–40 MHz 最终版本必须使用
外部高速 ADC、模拟下变频、欠采样加明确带通约束，或数字鉴相器，再把相位和
频率测量量交给同一个 `SoftPll`。AD9910 的射频输出链路也不能输出可反馈的 DC；
无交流参考时固件进入 HOLD。

## CH340 串口

参数：`115200, 8N1`，当前 Windows 识别为 `COM19`。

启动信息：

```text
[PLL_TEST] READY PC4=ADC1_INP4 PC5=ADC2_INP8 OUT=AD9910 ...
```

遥测：

```text
[PLL] mode=AC lock=1 fin_mHz=... fout_mHz=... err_mdeg=...
      drift_mHz=... ref_mV=... fb_mV=...
      adc_ovr=0 dds_err=0 dds_upd=... dds_us=... asf=16383
```

- `adc_ovr` 必须为 0。
- `dds_err` 是驱动参数/发送流程错误计数；软件 SPI 返回成功不能单独证明芯片
  通信健康，必须同时看到连续 SMA 输出。
- `dds_us` 是最近一次 FTW+POW+ASF 原子更新耗时，应明显小于 2048 µs 的
  ADC 半缓冲周期。
- `lock=1` 只代表软件锁定判据满足，最终相差以示波器 CH1/CH2 为准。

在线命令：

```text
help
status
bw 10
kp 0.25
ki 10.0
kd 0.2
pkp 0.12
phase 0
cal 0
telem 100
```

本测试把 AD9910 ASF 固定为满幅字 `16383 (0x3FFF)`，不提供在线幅度命令。
`cal` 用于补偿 AD9910 模拟输出、反馈调理和布线的固定相移。

## 实测顺序

1. 暂时不接 PC5，只接示波器确认 AD9910 连续输出和反馈调理节点为
   0–3.3 V。
2. 接入 PC5，输入 1 kHz 正弦，确认 `adc_ovr=0`、`dds_err=0`，
   且 `dds_us` 小于 2048 µs。
3. 观察 `lock` 从 0 变 1，记录 CH1/CH2 相差和锁定时间。
4. 使用 `cal <deg>` 消除固定模拟链路相移。
5. 测试 100 Hz、1 kHz、10 kHz、50 kHz，再切换方波、三角波。
6. 每个频点记录 `err_mdeg`、`drift_mHz`、示波器相差、锁定时间，再调整
   `bw/kp/ki/kd/pkp`。

当前 `.ioc` 保留通用工程配置。实测完成前不要在 CubeMX 中重新生成代码，
否则会覆盖同步 ADC 和 TIM6 的源码级测试配置。
