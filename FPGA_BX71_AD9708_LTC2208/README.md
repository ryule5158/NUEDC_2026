# BX71 + ClearDream Dual LTC2208：2026 G 题 FPGA 工程

本目录从远端 `origin/main` 最新的 `FPGA_BX71_AD9708_AD9238` 独立复制、重命名并迁移得到。原 AD9238、AD9280 工程不在本目录内修改，也不参与本版本构建。

本次只替换 ADC 采集和抽取滤波链。STM32 SPI 寄存器区、4096 点缓存、PS7/QSPI 启动流程、AD9708 静默中点输出和 G 题上层接口保持兼容；样本契约升级为 signed-S20 Q3，在完整 16 位 ADC 动态范围之外保留三位滤波小数和一位保护余量。

## 硬件链路与边界

当前使用 ClearDream Circuit Dual LTC2208 模块 A 通道：

`BNC/SMA -> 外接 OPA690 跟随器 -> 模块 ADA4932 差分驱动/低通/100 Ω 差分终端 -> LTC2208 -> BX71`

- LTC2208：16 位、130 MSPS、芯片端标称 `2.25 Vpp` 差分满量程。
- 模块 MODE 为偏移二进制并启用 DCS；RTL 用 `adc_data ^ 16'h8000` 转为二进制补码。
- 外接 OPA690 只作跟随器，软件标称增益 `1.0`。
- 模块资料不足以证明 SMA 到 ADC 的精确闭环增益、输入阻抗和全部低通元件值；`2.25/65536 V` 只是 ADC 芯片端名义刻度，BNC 参考面的最终增益必须实测。
- 仅使用 A 通道，B 通道不进入交付镜像。

## 严禁同号直插 U8 与 BX71 P3

ClearDream U8 与 BX71 P3 **不兼容**。同号直插会把电源接到 FPGA 数据脚，存在损坏模块和 FPGA 的风险。必须使用交叉转接板或逐线飞线，上电前逐根检查通断、短路和电源极性。

| ClearDream U8 | 信号 | BX71 P3 | XC7Z020 |
|---:|---|---:|---|
| 17 | A_D0 | 1 | P15 |
| 18 | A_D1 | 4 | P16 |
| 15 | A_D2 | 3 | N17 |
| 16 | A_D3 | 6 | P18 |
| 13 | A_D4 | 5 | R18 |
| 14 | A_D5 | 8 | T17 |
| 11 | A_D6 | 7 | U17 |
| 12 | A_D7 | 10 | W19 |
| 9 | A_D8 | 9 | Y19 |
| 10 | A_D9 | 14 | R16 |
| 7 | A_D10 | 13 | V18 |
| 8 | A_D11 | 16 | T16 |
| 5 | A_D12 | 15 | W16 |
| 6 | A_D13 | 19 | T14 |
| 3 | A_D14 | 20 | U14 |
| 4 | A_D15 | 21 | W18 |
| 20 | AD_CLKA | 2 | P14 |
| 2 | +5 V | 11 | 仅供电 |
| 1 | GND | 12 | 仅供电 |

U8-39、U8-40 是模块 3.3 V 输出，必须悬空。完整约束见 `constraints/ltc2208_u8_to_bx71_p3_spi_gpio0.xdc`。130 MHz 时钟和 16 位数据应使用短、等长、有连续地参考面的转接板；长杜邦线不属于已验证用法。

P3 被 LTC2208 占用后，AD9708 静默中点输出保留在 P2 飞线引脚；G 题镜像不会启用实验 AM/FM 核。

## 采样、滤波与数据格式

生产链路：

`LTC2208 16-bit @ 130 MSPS -> CIC5/R65 -> FIR81(Q47, inverse-CIC + LPF) -> signed-S20 Q3 @ 2 MSPS`

- 物理采样率 `130,000,000 sample/s`，输出采样率 `2,000,000 sample/s`。
- 4096 点频谱间隔 `488.28125 Hz`，满足题目 `<=500 Hz`。
- 一帧有效数据 `2.048 ms`；滤波填充、SPI 和 MCU 分析仍远低于 2 秒预算。
- CIC 使用 56 位内部累加；FIR 直接处理未提前截位的 CIC 结果，只在最终输出处从 Q47 最近舍入、饱和到 signed-S20 Q3。
- FIR81 为线性相位时分复用结构，不会把 16 位输入先降成 12 位或 Q8.8。
- 稳态群延迟为 CIC `160` 个 ADC 时钟加 FIR `40` 个 2 MSPS 样本，共 `21.230769 us`。共同线性延迟不改变谐波相对相位，也不破坏一周期/三周期波形重建。

### 相对 AD9238 的代码侧精度优势

以仓库最新版配置比较：

| 项目 | AD9238 | LTC2208 |
|---|---:|---:|
| ADC 物理位数 | 12 | 16 |
| ADC 满量程 | 10 Vpp | 2.25 Vpp |
| 外部前端配置增益 | 9.55 | 1.0 |
| BNC 输入折算物理 LSB | 约 255.6 uV | 34.33 uV |
| 滤波后传输 LSB | 约 15.978 uV | 4.2915 uV |
| 5 mV 峰值所占物理码 | 约 19.6 | 约 145.6 |

当前名义配置的物理码宽约细 `7.45` 倍，滤波后传输网格仍比 AD9238 细 `3.72` 倍。Q3 小数来自 65 倍抽取和 FIR 累加结果，不是把同一个 ADC 码重复插值；额外整数保护位覆盖滤波瞬态和微小通带增益。官方最大 `250 mVpp` 占 LTC2208 名义满量程 `11.1%`；设计压力值 `450 mVpp` 也只占 `20%`，不会削顶。精度预算脚本和 5 mV 位精确 RTL 回归用于防止后续修改丢失这一优势。

这只证明数字链不会比 AD9238 更早受量化或传输网格限制，不能替代模块 ENOB、时钟抖动、OPA690、供电、接线和 BNC 增益实测。模拟噪声或增益误差较大时，16 位名义值不会自动变成更好的整机结果。

## 通带与抗干扰

量化生产系数的 500 Hz 步进密集扫描：

- `10~500 kHz`：`+0.008866~+0.019868 dB`，纹波 `0.011002 dB`。
- `1~65 MHz` 最差混叠泄漏 `-42.620160 dB`，约在 `1.43 MHz`。
- 对 `200 mVpp` 单频干扰，数字链最差残留 `1.479183 mVpp`。

滤波在 130 MSPS 抽取前完成，避免 ADC 奈奎斯特以内的干扰先混叠到 0~500 kHz。超过 65 MHz 的输入若已在 ADC 采样处混叠进带内，FPGA/MCU 无法恢复，最终边界仍由模块模拟低通和实板接线决定。

## SPI 寄存器契约

地址 `0x20~0x2F` 保持 STM32 传输兼容：

| 地址 | 含义 | LTC2208 值/布局 |
|---:|---|---|
| `0x23` | 抽取比 | G 模式必须为 `65` |
| `0x24` | 触发 | `[17:16]` 模式，`[15:0]` 16 位偏移二进制阈值 |
| `0x29` | ADC 时钟 | `130,000,000` |
| `0x2A` | 设备 ID | `0x22080001` |
| `0x2B` | 协议版本 | `0x00030002` |
| `0x2F` | 滤波 | bit0：`CIC5/R65 + FIR81` |

`READ_DATA` 为 `[31:20]=12 位地址标签`、`[19:0]=signed-S20 Q3`，SPI 按 MSB first。`MIN/MAX/SUM/LATEST` 诊断寄存器仍保持物理 raw S16 语义，避免破坏既有调试工具。配置总线在 start toggle 前稳定并在 ADC 域锁存；最终统计先冻结，随后才同步 completion toggle，避免跨时钟撕裂结果。

## 构建与验证

```powershell
python .\scripts\analyze_ltc2208_filter_response.py
vivado.bat -mode batch -source .\scripts\run_ltc2208_filter_sim.tcl
vivado.bat -mode batch -source .\scripts\run_ltc2208_capture_spi_sim.tcl
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\Build-QspiBoot.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\Program-Qspi.ps1 -ValidateOnly
```

`scripts/ltc2208_release_inputs.txt` 是 QSPI 发布源唯一清单。PowerShell 指纹与 Vivado 允许列表必须完全一致，否则失败。完整构建会删除并重建专用 Vivado 工程，检查结构、全局/ADC setup/hold 和 DRC Error，生成：

- `output/BX71_G_ANALYZER_LTC2208_PS7.bit`
- `output/BOOT_G_ANALYZER_LTC2208.bin`
- `output/LATEST_G_LTC2208_QSPI_BUILD.txt`
- `output/SOURCE_SNAPSHOT_G_LTC2208_QSPI.txt`
- `output/SHA256SUMS_G_LTC2208_QSPI.txt`
- `output/g_ltc2208_qspi_verification/`

配套 STM32 工程是同级 `STM32H743_LTC2208`。设备 ID、协议版本、130 MHz/R65、2 MSPS 和 signed-S20 Q3 必须成套匹配。

## 自动 QSPI 烧录

1. BX71 完全断电，切到 JTAG。
2. 连接 JTAG 后上电。
3. 双击 `scripts/Program-Qspi.cmd`；它会完整重建、打包、校验最新 LTC2208 镜像，再写入 QSPI 并 readback Verify。
4. 只有看到 `QSPI PROGRAMMING AND READBACK VERIFY PASSED` 才能断电。
5. 断电切回 QSPI，再上电做真实冷启动和串口 ID/版本检查。

不能带电切换 JTAG/QSPI。`Build-QspiBoot.ps1` 或 `-ValidateOnly` 成功没有接触板上 Flash。

## 上板精度验收

在声称“整机优于 AD9238”前至少完成：

1. 检查零输入噪声、中心码、16 位极性、削顶门限和 130 MHz 时钟。
2. 扫描 ADC 采样相位并做长时间码流测试，禁止用单次启动成功代替接口裕量证明。
3. 用比赛 50 Ω 信号源及整套线缆，在 BNC 参考面扫 `10/50/100/200/300/400/450/484.5/500 kHz`。
4. 对最小 `5 mV` 峰值分量、强弱分量组合和最大整体 Vpp 检查频谱峰值、总 Vpp、真 RMS 和频率；整机幅值误差目标留出余量到 `<=3 mV`，不要贴着题目 `5 mV` 上限。
5. 叠加 `>=1 MHz、200 mVpp` 干扰，检查带内幅值、完整周期和额外谱线。
6. 只填写 LTC2208 专用校准点，禁止复用 AD9238 表。
7. 重复采集、冷热启动和不同相位组合，确认稳定且每项小于 2 秒。

RTL 仿真、Vivado 时序和 Keil 编译不能替代物理校准。完成 BNC 验收前，应描述为“数字链已验证、实板精度待闭环”，不能直接宣称题目 `<=5 mV` 已实测通过。
