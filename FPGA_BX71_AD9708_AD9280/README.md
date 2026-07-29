# BX71 2026 G 题高速采集工程

当前生产构建在 BX71-V4（Zynq-7020）上只综合 G 题需要的
AD9280 采集链路。顶层为 `g_analyzer_zynq_top`；AD9708 输出固定在中点码，
ProMax 和实验 AM/FM 不进入交付 `.bit/.bin`。目录中仍有通用 RTL 源文件供
以后复用，但它们不是当前镜像的一部分。

## 当前 G 题数据链路

- AD9280 物理采样为 8 位、32 MSPS。
- FPGA 使用 CIC3/R16 抗混叠抽取，输出 2 MSPS，最多缓存 4096 点。
- CIC 结果以 signed Q8.8 保存和传输，采用对称最近偶数舍入及有符号饱和；
  不再先压回 8 位整数，因此保留了 16 次过采样形成的小数信息。
- 立即/上升沿/下降沿/任意边沿触发、最小值、最大值、累加和、最新值和
  OTR 统计均使用同一 signed Q8.8 定义。
- STM32 接口为 SPI 模式 0、MSB first、6 字节寄存器帧，ACK 为
  `0xA5 0x5A`。
- BX71 本地 50 MHz 时钟生成 ADC 32 MHz；闲置 DAC 仍输出合法时钟和中点码。

ADC 协议版本固定为 `0x00010003`，寄存器区为 `0x20`～`0x2F`。
旧版 `0x00010001/0x00010002` 是 8 位样本布局，STM32/TI 驱动会明确返回
`SAMPLE_FORMAT`，不会把旧数据误当作新格式。

| 地址 | v1.3 数据定义 |
|---:|---|
| `0x26` | `[31:28]=0`、`[27:16]=12 位 RAM 地址`、`[15:0]=signed Q8.8 样本` |
| `0x2C` | `[31:16]=signed Q8.8 最大值`、`[15:0]=signed Q8.8 最小值` |
| `0x2D` | 4096 点以内的 signed Q8.8 累加和 |
| `0x2E` | 符号扩展后的最新 signed Q8.8 样本 |
| `0x2F` | bit0：使能 G 题 CIC3/R16 |

Q8.8 是传输精度，不代表 8 位 AD9280 变成了真正的 16 位 ADC。物理一整码仍约
为 `10 V / 256 = 39.0625 mV`；Q8.8 的一个传输 LSB 约为
`10 V / 65536 = 0.152588 mV`，用于保留抽取平均得到的亚码信息。

### 未参与当前 G 镜像：AD9708 硬件调制寄存器

以下内容只描述目录中的历史通用 RTL；当前 G 题位流不提供这些寄存器。

| 地址 | 作用 |
|---:|---|
| `0x15` | 调制控制：`0=关闭`、`1=AM`、`2=FM`；最后写入并原子提交全部调制参数 |
| `0x16` | 调制 NCO 的 32 位频率字 |
| `0x17` | `{AM 调制增益, AM 载波增益}`，两个字段均为无符号 Q1.15 |
| `0x18` | FM 峰值频偏对应的 32 位频率字 |
| `0x19` | 调制 NCO 的 32 位相位偏移字 |

AM 使用归一化的 `A/(1+m)·(1+m·sin)`，因此 100% 调制度仍不会数字削顶；FM 每个 125 MHz DAC 周期直接调制载波相位增量。参数寄存器先暂存，写 `0x15` 时一次性跨时钟域提交，避免半套新旧参数混合。

## 未参与当前 G 镜像：DSP ProMax 寄存器

以下内容只描述目录中的历史通用 RTL；当前 G 题位流不提供 ProMax。

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

## 最终验证（2026-07-29）

- Vivado 2018.3 已完成 G 题 PS7 顶层全量综合、布局、布线和 bitgen。
- 路由后全局建立/保持余量为 `+1.008/+0.081 ns`；ADC 输入路径为
  `+1.008/+1.518 ns`，全部用户时序约束满足。
- 顶层实际使用 `845 LUT`、`882 FF`、`2 RAMB36`、`0 DSP48E1`。
- CIC signed Q8.8 舍入/饱和、10/20/30 kHz 幅值、采集寄存器和原 DAC
  回归仿真均通过。
- `BOOT_G_ANALYZER.bin` 已使用该 G 题位流重新打包。静态验证和数字仿真不替代最终板上模拟链路实测。
- `output/G_QSPI_HARDWARE_VALIDATION_20260729.txt` 保留的是更早镜像的板上冷启动记录；
  其哈希与当前 Q8.8 交付件不同，当前镜像仍需重新烧录并完成掉电启动复验。

## 目录

- `Vivado/BX71_G_ANALYZER_PS7/`：含 PS7 的固定 Vivado 工程，直接打开同名 `.xpr`。
- `Vivado/BX71_AD9708_AD9280_PL/`：运行纯 PL 构建脚本后生成的固定 Vivado 工程。
- `rtl/`：AD9708 DDS、AD9280 采集核、时钟和 SPI 寄存器从机。
- `rtl_ps7/`：带 PS7 配置的持久化启动顶层。
- `constraints/`：BX71 P2/P3 引脚和 AD/DA 时序约束。
- `sim/`：AD9708 数字幅度及 AD9280 采集自检仿真。
- `scripts/`：Vivado 构建和 QSPI 启动镜像打包脚本。
- `启动支持/`：本 FPGA 工程使用的 FSBL 等启动支持文件。
- `output/`：构建后的 `.bit`、`BOOT_G_ANALYZER.bin`和 SHA-256。

## 构建与固化

双击 `scripts/Build-QspiBoot.cmd`，脚本会递归发现当前 RTL/XDC/IP 输入，在
`Vivado/BX71_G_ANALYZER_PS7/` 创建干净工程，构建 G 题 PS7 集成位流，并生成
`output/BOOT_G_ANALYZER.bin` 与 `output/SOURCE_SNAPSHOT_G_QSPI.txt`。
支持范围和必须重新签核的结构变化见 `ONE_CLICK_QSPI_CHANGE_COVERAGE.md`。

JTAG 临时调试应下载最终的 `output/BX71_G_ANALYZER_PS7.bit`。掉电保存使用以下任一入口，不需要打开 SDK、不需要 SDK workspace，也不需要导入 HDF：

- `scripts/Program-Qspi.cmd`：烧录现有的 `output/BOOT_G_ANALYZER.bin`，适合确认镜像后重复烧录。
- `scripts/Build-And-Program-Qspi.cmd`：重新构建位流、打包完整启动镜像、烧录 QSPI 并写后校验。
- `scripts/Probe-Qspi.cmd`：只检查工具、JTAG 和 XC7Z020，不擦除、不写入 Flash。

脚本固定使用 BX71 所需的 `qspi-x1-single`、Offset 0 和工程内 `启动支持/boot/FSBL.elf`。它会自动寻找任意盘符及常见安装目录中的 Vivado/SDK 2018.3，自动连接或启动 `hw_server`，确认 JTAG 链中存在 XC7Z020，自动选择唯一的 Zynq ARM DAP，并把烧录输入暂存到纯 ASCII 短路径，避免工程目录变化、空格或中文路径导致 Xilinx 2018.3 失败。烧录成功后默认执行 Xilinx 写后校验，并在 `output/` 保存本机日志及 `LAST_PROGRAMMED_G_QSPI.txt`。

跨电脑使用时只需完整复制/拉取本工程，并安装 Vivado 2018.3 与 SDK 2018.3。工具可安装在不同盘符；若采用非常规目录，可先运行 Xilinx 的 `settings64.bat`，或直接给 PowerShell 脚本传入 `-Vivado`、`-Bootgen`、`-ProgramFlash`、`-HwServer` 的完整路径。烧录时只能连接一块 Zynq；检测到多块 Zynq 时脚本会拒绝继续，避免写错设备。

## 连接

高速 AD/DA 模块在 U3 与 BX71 P3 尾端对齐直插，不翻面、不错位。STM32 与 BX71 使用：

| STM32H743 | BX71 P2 | 作用 |
|---|---:|---|
| PI1 / SPI2_SCK | 31（K19） | SPI 时钟 |
| PB15 / SPI2_MOSI | 37（L19） | STM32 写 FPGA |
| PI2 / SPI2_MISO | 33（K17） | FPGA 返回数据 |
| PE8 / FPGA_CS | 34（K18） | 低有效片选 |
| PH9 / FPGA_IRQ | 35（L16） | 可选DAC PLL锁定指示 |
| GND | 30 或 12 | 共地 |

TI LP-MSPM0G3507 也可独立控制同一接口：

| MSPM0G3507 | LaunchPad位置 | BX71 P2 | 作用 |
|---|---|---:|---|
| PA12 / SPI0_SCLK | BoosterPack 32 | 31（K19） | SPI 时钟 |
| PA14 / SPI0_PICO | 板底 J28-7 | 37（L19） | TI 写 FPGA |
| PA13 / SPI0_POCI | BoosterPack 31 | 33（K17） | FPGA 返回数据 |
| PA15 / GPIO | BoosterPack 30 | 34（K18） | 低有效软件片选 |
| GND | 任一 GND | 30 或 12 | 共地 |

STM32 与 TI 板不能同时连接并驱动这组 SPI。TI SPI0 使用模式0、8位、MSB优先、1 MHz。J28 位于 LaunchPad 板底且没有信号丝印，PA14 必须按排针缺口和 1 脚方向定位。

S1 为 AD9708 DAC 输出，S2 为 AD9280 ADC 输入。模块使用稳定 +5 V 供电，FPGA GPIO 仅允许 3.3 V 电平。
