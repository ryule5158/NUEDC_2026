# NUEDC 2026 集成工程

本目录以本地已验证集成工程为基础，合入
[`ryule5158/NUEDC_2026`](https://github.com/ryule5158/NUEDC_2026) 最新内容。
新对话或新成员应先阅读根目录的 `交接文件.md`，其中包含实际硬件清单、软件架构、
工具路径、验证状态、资源冲突和赛题协作流程。

## 2026 G 题专用入口

当前工作树采用远端仓库的 G 题周期信号分析方案：

- Zynq7020/AD9280 以 32 MSPS 采集，经 FPGA CIC3/R16 抽取为
  4096 点、2 MSPS 的 signed Q8.8 数据，再交给 STM32 分析。
- STM32 `main.c` 直接进入 `GContest` 串口测试流程；工程中不再保留
  编译期方案选择宏。
- 默认 FPGA 生产顶层只综合 G 题采集链路；AD9708 保持中点静默，
  ProMax 和实验 AM/FM 不进入交付位流。

数字接线、六阶抗干扰模拟前端、构建和实物校准步骤见
[G题_MCU_FPGA接线与使用.md](G题_MCU_FPGA接线与使用.md)。

| 子工程 | 工程入口 | 当前用途 |
|---|---|---|
| STM32H743 | `STM32H743/MDK-ARM/STM32H743.uvprojx` | 运行 FPGA/AD9280 G 题采集、分析和串口测试 |
| TI MSPM0G3507 | `TI_MSPM0G3507/Keil/TI_MSPM0G3507.uvprojx` | SysConfig 管理的轻量 MCU 工程；当前 main 输出 AD9910 1 kHz 正弦 |
| BX71 Zynq-7020 | `FPGA_BX71_AD9708_AD9280/Vivado/BX71_AD9708_AD9280_PS7/BX71_AD9708_AD9280_PS7.xpr` | G 题 AD9280/CIC3 与 PS7/QSPI 固化工程 |

STM32 与 TI 各自保存独立的 `Modules/`，互不跨目录编译；两者仅与 FPGA 使用相同
寄存器协议。各 MCU 工程内只保留一份项目说明：

- `STM32H743/README.md`
- `TI_MSPM0G3507/README.md`

最终 MCU-FPGA SPI 连接为：

| 信号 | STM32H743 | MSPM0G3507 | BX71 |
|---|---|---|---|
| SCK | PI1 | PA12 | P2-31 / K19 |
| MOSI | PB15 | PA14 | P2-37 / L19 |
| MISO | PI2 | PA13 | P2-33 / K17 |
| CS_N | PE8 | PA15 | P2-34 / K18 |

STM32 PE8 同时连接核心板 FMC_D5，FPGA 与 SDRAM/NAND 只能分时使用；两个功能都
保留，切换时必须停止访问并重新配置 GPIO/FMC。

全量构建：

```powershell
cd D:\NUEDC\省赛\Integrated_Project\STM32H743
.\build_keil.ps1

cd D:\NUEDC\省赛\Integrated_Project\TI_MSPM0G3507
.\build_keil.ps1
```

最近一次最终检查中，两套 Keil 工程均为 0 错误、0 警告；MCU 主机回归、
FPGA 行为仿真和 G 题 PS7 全量构建通过，已布线时序满足。模拟量程、
模块供电和 QSPI 冷启动仍需在实物上复测。
