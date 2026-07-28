# STM32H743 双 ADC + FPGA/AD9708 PLL 实测说明

## 1. 当前测试链路

```text
信号源 ──┬── 示波器参考通道
         └── PC4 / ADC1_INP4

BX71 AD9708 模拟输出 ──┬── 示波器输出通道
                       └── 调理后送 PC5 / ADC2_INP8

PC4+PC5 同步 ADC DMA → SoftPll → SPI2 → FPGA DDS → AD9708
```

AD9910 已从当前测试系统拆除。`Modules/ad9910*` 为兼容其他工程保留，但 `BSP/pll_test_app.c` 不再调用它。

## 2. 模拟输入安全

`PC4`、`PC5` 只能接受 `0..VDDA`。信号源和 AD9708 输出在进入 ADC 前必须经过合适的衰减、偏置或钳位，使波形始终位于 `0..3.3 V`。推荐 ADC 节点以约 1.65 V 为中心，并先用示波器确认没有负压和超过 VDDA 的过冲。

所有设备必须共地：信号源、示波器、STM32、BX71、AD9708、CH340。

## 3. 当前有效数字接线

| STM32H743 | FPGA 球位 | 方向/用途 |
|---|---|---|
| PI1 / SPI2_SCK | K19 | STM32→FPGA，SPI 时钟 |
| PB15 / SPI2_MOSI | L19 | STM32→FPGA，寄存器和波形数据 |
| PI2 / SPI2_MISO | K17 | FPGA→STM32，状态和寄存器读回 |
| PE8 / FPGA_CS | K18 | STM32→FPGA，低有效片选 |
| GND | GND | 必须共地 |

串口使用 USART1：PA9→CH340 RXD，PA10←CH340 TXD，COM19，115200 8N1。CH340 使用 3.3 V TTL，不要把 5 V 接到 MCU。

## 4. 固件结构与双缓冲

- `BSP/adc_dual_capture.c/.h` 使 ADC1/ADC2 由同一个 TIM6 TRGO 触发，以 regular simultaneous 模式真正同步采样 PC4/PC5。
- DMA 为循环双半区：每半区 1024 组配对样本。CPU 只处理已经释放的一半，DMA 同时采集另一半。
- `DSP/SoftPll.c/.h` 保持硬件无关，只产生频率和相位校正命令。
- `BSP/pll_test_app.c` 将命令映射到 FPGA/AD9708；原有 `PllTest_*` API 保持不变。
- `BSP/fpga_boot_restore.c/.h` 每次上电重新下发 ProMax FIR、DDC 音调和运行参数。
- `AD9708_Init()` 校验设备 ID、协议版本、125 MHz DAC 时钟和 PLL ready；随后 STM32 重新生成并下发 1024 点正弦表。
- `AD9708_UpdateDdsFast()` 在一个 SPI 会话中只写变化的 32 位频率字、相位字或使能位。每个写事务仍检查 FPGA ACK，但运行环不做冗余读回。
- 频率搜索期间关闭 AD9708，防止旧输出耦合到 PC4；识别输入后重新使能并进入闭环。参考消失时进入 HOLD，保持最后有效输出。

## 5. AD9708 输出配置

- FPGA DDS/DAC 时钟：125 MHz。
- PLL 软件输出上限：40 MHz。
- AD9708 驱动保护上限：50 MHz；数字 Nyquist 为 62.5 MHz。
- 波形：自动识别并重建正弦、方波、三角波、锯齿波；协议 `0x00020004` 还可在 FPGA 125 MSPS 数据面重建单音 AM/FM。
- 幅度：`amplitude=128`、`offset=128`，覆盖 8 位 DAC 码 `0..255` 的满幅范围。
- 运行时频率量化分辨率约为 `125 MHz / 2^32 = 0.0291 Hz`。

片上 ADC 的自动频率搜索配置到 6.5 MHz，用于保留曾验证过的 6 MHz 兼容点；正式目标仍是 5 MHz 以下。超过 5 MHz 的可靠频率/相位采集应切换到 FPGA 高速 ADC。不能仅凭 AD9708 能输出 40 MHz，就宣称片上 ADC 已验证全 40 MHz 锁相。

## 6. 串口日志

正常启动应依次看到：

```text
[FPGA_BOOT] READY attempt=1 id=0x4258504D ...
[AD9708] READY id=0xAD970802 version=0x00020003 dac_Hz=125000000 wave=sine points=1024 amp=128 offset=128
[PLL_TEST] READY PC4=ADC1_INP4 PC5=ADC2_INP8 OUT=FPGA_AD9708 ...
[ADC_DUAL] sync=1 ...
```

运行日志示例：

```text
[PLL] mode=AC lock=1 fin_Hz=100000 fout_Hz=100000 ...
      err_mdeg=-8 drift_mHz=-6 ... ref_mV=551 fb_mV=197 ...
      adc_sync=1 adc_ovr=0 adc_err=0x00 out_err=0 out_us=1 amp=128
```

关键字段：

- `fin_Hz`：PC4 估计输入频率。
- `fout_Hz`：实际量化后的 FPGA DDS 输出频率。
- `lock`：软件锁定判据；最终相差仍应以示波器为准。
- `adc_sync=1`：ADC1/ADC2 使用同一硬件触发并成对传输。
- `adc_ovr=0`、`adc_err=0x00`：采集没有溢出或 HAL 错误。
- `out_err=0`：FPGA/AD9708 更新没有 SPI/ACK 错误。
- `out_us`：最近一次快速输出更新耗时。实际写频率字约 139–141 μs；无需写寄存器时约 0–1 μs。
- `amp=128`：AD9708 满幅数字幅度。

## 7. 在线命令

```text
help
status
scan
hint 100000
spec 0
bw 10
kp 0.25
ki 10.0
kd 0.2
pkp 0.12
phase 0
cal 0
telem 100
```

一般优先调 `bw`；`kp/ki/kd/pkp` 用于实机实验。`cal <deg>` 补偿 AD9708 模拟输出、反馈调理和布线的固定相移。

## 8. 已完成的物理证据

2026-07-27，100 kHz 正弦输入：

- FPGA ProMax 和 AD9708 设备 ID、协议版本读回正确；
- PC5 反馈约 197 mV；
- `fin_Hz=100000`、`fout_Hz=100000`、`lock=1`；
- 相位误差多数约在 ±0.2°内；
- `out_err=0`；
- `adc_sync=1`、`adc_ovr=0`、`adc_err=0x00`；
- Keil ArmClang 6.24 构建为 0 errors、0 warnings；STM32 下载、校验、复位成功。

这些早期证据只确认当时的 100 kHz 正弦物理闭环。后续已人工通过 5 MHz 以下正弦、100 kHz/1 MHz/3 MHz/5 MHz 方波、100 kHz 三角波和 100 kHz 锯齿波；AM/FM 仍须使用新 `0x00020004` 位流完成板上验收。

## 9. QSPI 持久化边界

最新 PB15/L19 `BOOT_PS7.bin` 已于 2026-07-27 14:10 以 `qspi-x1-single`、Offset 0 写入，并通过 Xilinx 逐块 Verify；镜像 SHA256 为 `70088EC85FA7204E1F63FB3102BD055D02C3CA48D0D4A6BBA3ADE0E14BD77326`。烧录后已用同一 BIT 恢复当前 JTAG 运行态并再次锁住 100 kHz，但尚未进行完整断电冷启动。

最终冷启动验收步骤：

1. 整板完全断电；
2. 拨码保持 QSPI 启动；
3. 重新上电；
4. 再次确认 `[FPGA_BOOT] READY`、`[AD9708] READY` 和 100 kHz `lock=1`。

## 10. 建议复测顺序

1. 保持 100 kHz 正弦，记录锁定时间、示波器相差和串口稳定性。
2. 依次测试 1 kHz、10 kHz、50 kHz、100 kHz、500 kHz、1 MHz、2 MHz、5 MHz。
3. 每次切频记录 `fin_Hz/fout_Hz/err_mdeg/drift_mHz/out_err/adc_ovr`。
4. AM 建议首测 `fc=100 kHz, fm=1 kHz, m=50%`；FM 建议首测 `fc=100 kHz, fm=1 kHz, Δf=1 kHz`。串口必须同时满足载波锁定和调制相位锁定后才显示 `lock=1`。
5. 超过 5 MHz 时改用 FPGA 高速 ADC，不把片上 ADC 欠采样结果当作无条件唯一解。

不要在未同步回填 `.ioc` 配置的情况下用 CubeMX 覆盖生成代码；否则可能破坏双 ADC、TIM6、PB15 SPI2_MOSI 或 DMA 的手工验证配置。
## 11. 上电与运行期自动恢复

固件现已使用 `FpgaBootRestore_WaitReady(0, 500)` 等待 QSPI/FPGA 稳定，并在首次 READY 后再次完整恢复易失参数。错误设备 ID 在启动期可重试，默认首次等待与重试间隔均为 500 ms。

若 FPGA 独立掉电或被 JTAG 重配，AD9708 连续 4 次 ACK 写失败会自动暂停双 ADC DMA，重新执行 FPGA FIR/DDC/运行寄存器恢复和 AD9708 1024 点正弦 RAM/满幅配置，然后重启双缓冲采集。该路径已通过不复位 STM32、仅 JTAG 重配 FPGA 的实测：`READY attempt=8`、`AD9708 RECOVERED`，约 3.1308 MHz 重新 `lock=1`，`adc_sync=1`，历史 `out_err=4` 不再增长。

新 HEX SHA256：`0E6CF91D1442F44B0E63AA651F0E6F7BF2B55EA0048AD02FB4A93E4BC67FD01A`。仍需断开 ST-LINK、CH340 等可能反向供电路径，完成一次 MCU 与 FPGA 同时掉电的最终 QSPI 冷启动复测。
