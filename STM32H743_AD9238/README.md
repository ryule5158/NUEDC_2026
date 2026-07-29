# STM32H743 + AD9238：2026 G 题配套工程

这是从 `STM32H743` 独立复制并改名得到的 AD9238 配套版本；原 STM32 工程未被本次迁移覆盖。它与同级 `FPGA_BX71_AD9708_AD9238` 成套使用，不能与旧 AD9280 FPGA 镜像混用。

## 为什么 STM32 也需要新副本

ACM9238 把 ADC 从 8 位/32 MSPS 改为 12 位/50 MSPS，FPGA 的 G 题抽取比随之从 16 改为 25。虽然 STM32 最终仍接收 2 MSPS、16 位有符号 Q8.8 数据，但设备身份、协议版本、触发阈值位宽和物理 LSB 都发生了变化，因此必须同步修改驱动。

配套参数：

| 项目 | AD9238 版本 |
|---|---:|
| ADC 物理采样率 | 50,000,000 sample/s |
| 原生分辨率 | 12 bit |
| G 题抽取 | CIC3/R25 |
| STM32 分析流 | 2,000,000 sample/s |
| FPGA 设备 ID | `0xAD923801` |
| FPGA 协议版本 | `0x00020001` |
| 触发寄存器 | 模式 `[17:16]`，阈值 `[11:0]` |
| 标称原生 LSB | `10/4096 V` |
| Q8.8 传输 LSB | `10/65536 V` |

## 主要文件

- `Modules/ad9238.c/.h`：FPGA 寄存器驱动、设备/协议核验、12 位触发配置、Q8.8 解码。
- `Modules/ad9238_app.c/.h`：0…4095 原生码的电压标定和阻塞采集流程。
- `BSP/signal_adc_ad9238.c`：G 题通用采样后端，固定 50 MHz/R25→2 MHz。
- `MDK-ARM/STM32H743_AD9238.uvprojx`：已重命名的 Keil 工程与目标。
- `STM32H743_AD9238.ioc`：已重命名的 CubeMX 工程描述。

`SignalAdc_*` 与上层 `g_contest_app`、周期信号分析和显示接口保持不变，因此 G 题业务代码不需要知道底层 ADC 型号。初始化时会严格检查设备 ID、协议版本、50 MHz 采样时钟、16 位样本字和 8 位小数位；旧 FPGA 镜像会被拒绝。

## STM32 ↔ BX71 SPI

沿用现有 P2 连接：

| STM32H743 | BX71 P2 FPGA | 作用 |
|---|---|---|
| SPI2_SCK | K19 / GPIO0_26 | SCK |
| SPI2_MOSI | L19 / GPIO0_32 | MOSI |
| SPI2_MISO | K17 / GPIO0_28 | MISO |
| CS GPIO | K18 / GPIO0_29 | CS_N |
| IRQ GPIO | L16 / GPIO0_30 | FPGA ready/IRQ |
| GND | GND | 共地 |

ACM9238 与 BX71 的 P3 直插连接由 FPGA 工程约束管理；STM32 不直接连接 D0…D11 或 ADC 时钟。

## 构建

```powershell
cd C:\Users\LENOVO\Desktop\NUEDC_2026\STM32H743_AD9238
& .\build_keil.ps1
```

脚本重建 `STM32H743_AD9238.uvprojx`，只有日志包含 `0 Error(s), 0 Warning(s)` 才成功，并复制以下产物及 SHA256 到 `output`。如果机器上已经打开 uVision，脚本会等待本次新日志出现最终汇总，而不会等待或强制关闭用户的 GUI 进程：

- `STM32H743_AD9238.axf`
- `STM32H743_AD9238.hex`
- `STM32H743_AD9238.map`
- `build.log`
- `SHA256SUMS.txt`

## 串口调试反馈

USART1 使用 115200 8N1。输入 `HELP` 可查看命令；输入 `ADCDBG` 会输出最近一次缓存的 AD9238/FPGA 调试快照，包括设备 ID、协议版本、物理采样时钟、Q8.8 字宽、R25 配置、触发状态、采集/读取计数、OTR 计数以及 min/max/latest/mean。该命令不会发起新的 SPI 事务，也不会移动 FPGA 块 RAM 读指针，适合在采集成功、超时或链路错误后留存现场。

上电初始化成功时还会主动输出 `0xAD923801`、协议 `0x00020001`、`S16Q8` 和 ready 状态，便于在测试前立即发现烧错 FPGA 镜像或时钟未锁定。

## 使用边界

代码和 Keil 构建通过不代表已经烧录 STM32，也不代表 ACM9238 模拟幅值精度已经达到 G 题 5 mV 指标。首次上板必须与 AD9238 FPGA 镜像配套，先核对设备 ID/版本，再用已知直流与周期信号做零点、增益和频响标定；标称 ±5 V/`10/4096 V` 只用于初始换算，不能替代实测校准。