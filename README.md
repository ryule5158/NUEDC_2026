# NUEDC 2026 集成工程

本目录以本地已验证集成工程为基础，合入
[`ryule5158/NUEDC_2026`](https://github.com/ryule5158/NUEDC_2026) 最新内容。
新对话或新成员应先阅读根目录的 `交接文件.md`，其中包含实际硬件清单、软件架构、
工具路径、验证状态、资源冲突和赛题协作流程。

## 2026 G 题专用入口

当前工作树采用远端仓库的 G 题周期信号分析方案：

- Zynq7020/AD9280 以 32 MSPS 采集。FPGA 数据通路固定为 CIC3/R16，
  输出 4096 点、2 MSPS 的 signed Q8.8 数据，不存在旁路、可变抽取或滤波器选择。
- FPGA/STM32 采集协议固定为 v1.4（`0x00010004`），响应 profile 固定为
  CIC3/R16（`0x00030010`）。STM32 根据解析式逐频恢复 CIC 的确定性幅值和相位响应，
  因而 10 kHz～500 kHz 的测量结果不保留 CIC 下垂。
- STM32 `main.c` 直接进入 `GContest`，不再保留编译期方案选择宏。陶晶驰串口屏
  USART2 是默认交互入口，USART1 串口助手的 `1`/`3` 测试命令同时可用；
  `CAL/Q/EXIT` 可采集未经换算的 Q8.8 两点直流标定数据。
- 尚未完成整机扫频前，经验幅相校准表保持关闭；1 MHz 干扰由联合最小二乘中的
  nuisance（干扰）分量分离，不对被测波形做低通、陷波或频谱删点。
- 采集业务通过 `signal_adc.h` 与 AD9280 解耦；以后切换 ACM9238/AD9238 时，
  只替换 ADC/FPGA 后端并声明其采样率、量化、原始统计和确定性数字响应。
- 默认 FPGA 生产顶层只综合 G 题采集链路；AD9708 保持中点静默，
  ProMax 和实验 AM/FM 不进入交付位流。

数字接线、当前无放大采集路径、构建和后续实物校准边界见
[G题_MCU_FPGA接线与使用.md](G题_MCU_FPGA接线与使用.md)。

| 子工程 | 工程入口 | 当前用途 |
|---|---|---|
| STM32H743 | `STM32H743/MDK-ARM/STM32H743.uvprojx` | 运行 FPGA/AD9280 G 题采集、分析和串口测试 |
| TI MSPM0G3507 | `TI_MSPM0G3507/Keil/TI_MSPM0G3507.uvprojx` | 远端保留的独立辅助工程，不参与当前 G 题生产构建 |
| BX71 Zynq-7020 | `FPGA_BX71_AD9708_AD9280/Vivado/BX71_G_ANALYZER_PS7/BX71_G_ANALYZER_PS7.xpr` | G 题固定 AD9280/CIC3/R16 与 PS7/QSPI 固化工程 |

STM32 与 TI 各自保存独立的 `Modules/`，互不跨目录编译。当前 G 题协议 v1.4
只在 STM32H743 路径完成了迁移和验证；TI 目录中的旧 AD9280 副本不能控制本次
生产位流，除非单独完成同等迁移与回归。各 MCU 工程内只保留一份项目说明：

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
cd C:\Users\21274\Documents\省赛\d_repo_latest_precision\STM32H743
.\build_keil.ps1

cd C:\Users\21274\Documents\省赛\d_repo_latest_precision\TI_MSPM0G3507
.\build_keil.ps1
```

最近一次最终检查中，MCU 主机回归、FPGA 行为仿真和 G 题 PS7 全量构建通过，
已布线时序满足；CIC 响应补偿与 1 MHz 联合干扰模型也已通过主机/行为仿真矩阵。
模拟量程、整机传递增益、模块供电和当前 QSPI 镜像的冷启动仍需在实物上复测。
