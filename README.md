# NUEDC 2026 集成工程

这是从 2024 C 题工程独立整理出的通用库模板，原 `2024C` 目录没有被修改。顶层按硬件平台平级组织，不使用“主工程/从工程”关系。

## 2026 G 题专用入口

当前工作树已启用 STM32H743 + Zynq7020/AD9280 的 G 题周期信号分析代码，不包含屏幕驱动。数字接线、六阶抗干扰模拟前端、构建和实物校准步骤见 [G题_MCU_FPGA接线与使用.md](G题_MCU_FPGA接线与使用.md)。

G 题配置使用 32 MSPS ADC、FPGA CIC3/R16 和 4096 点/2 MSPS 分析，频率间隔 488.28125 Hz。板上满量程、滤波器幅相和 5 mV 误差仍需按文档现场校准。

| 子工程 | 路径 | 用途 |
|---|---|---|
| TI MSPM0G3507 | `TI_MSPM0G3507/Keil/TI_MSPM0G3507.uvprojx` | TI 板卡轻量 DSP、通用外设及独立 SPI0 FPGA 联动 |
| STM32H743 | `STM32H743/MDK-ARM/STM32H743.uvprojx` | STM32H743 通用库及高速 AD/DA APP 接口 |
| FPGA BX71 | `FPGA_BX71_AD9708_AD9280/Vivado/BX71_AD9708_AD9280_PS7/BX71_AD9708_AD9280_PS7.xpr` | AD9708、AD9280、ProMax 和 Zynq PS7 固化工程 |

三个工程各自保存源码、工程配置、构建脚本和输出目录。TI 与 STM32 各有自己的 `Modules/`，任何 Keil 工程都不引用另一个板卡目录；它们仅在接口协议层与 FPGA 兼容。FPGA 的 Vivado 工程、启动支持和输出也全部位于 FPGA 自己的目录内。

三个正式工程均不依赖本机空的 `第三方参考库/` 目录；仓库没有 `.gitmodules`，克隆后无需执行子模块命令。

工程已去除 C 题的 SD/SM、多径时延、AD9910 同步启动、C 题业务参数以及串口屏页面逻辑。当前稳定 AD9708 位流协议为 `0x00020003`，ADC 与 ProMax 共用同一 SPI 链路。实验 RTL 中保留了 `0x00020004` AM/FM 数据面和 STM32 锁相草案，但默认宏关闭且尚未板上验收，详见 `STM32H743/EXPERIMENTAL_AMFM_WIP.md`。

## 当前稳定交付边界

- STM32 双 ADC 使用同一 TIM6 触发沿真正同步采集 PC4/PC5，并由 DMA 半满/全满形成 ping-pong 双缓冲。
- 已人工通过：5 MHz 以下正弦；100 kHz、1 MHz、3 MHz、5 MHz 方波；100 kHz 三角波；100 kHz 锯齿波。
- PC4 断线或弱串扰不会再报告锁定：参考 IQ 门限按现场数据设为 `0.030`。
- 默认构建不启用 AM/FM 实验锁相；稳定 `.bit/.bin` 也没有被暂停的实验构建覆盖。
- STM32 ArmClang 6.24 默认配置构建为 `0 Error(s), 0 Warning(s)`。

构建和最小调用示例见各子工程的 `README.md`；完整接线、测试步骤和预期现象见 `硬件测试指南.md`。
最终全量编译、仿真、时序和镜像校验结果见 `验证结果.md`。
本轮 PLL 联调的稳定功能、暂停项和继续开发入口见 `发布交接说明.md`。

## DSP ProMax 实时档

本工程已从 `ryule5158/BX71-DSP-ProMax` 的 `3ff859aa888a7e77996f6fc3b32b92be8352f69c` 合并经过复核的实时数据面：8 路 DDC、每路 CIC64 滤波与功率统计，以及 4 组 32 抽头匹配滤波和峰值保持。AD9280 的原始 32 MSPS 样本通过异步 FIFO 送入 50 MHz DSP 域；原有触发采集和 DAC 波形发生不受影响。

当前能力字为 `0x29200408`。它真实声明 SPI、实时档和连续波形统计，不声明 ProMax UART、线级 CRC 或 FFT；两套 MCU 的 FFT 接口会返回 `FPGA_PROMAX_E_UNSUPPORTED`。MCU 只配置频率/模板并读取压缩结果，不搬运连续 32 MSPS 数据，也不在 MCU 上复制 160 个并行乘法器的工作。

STM32H743 与 MSPM0G3507 继续使用原有 6 字节物理寄存器链路，接线没有增加。公共 `FpgaPromax_*` 逻辑 API 在 MCU 本地校验 9 字节帧，再安全映射到现有寄存器；因此本地 CRC 不是 SPI 线上端到端 CRC。默认 `main` 只保留库，不自动启动 ProMax。

## 重要硬件约定

- 高速 AD/DA 模块与 BX71 P3 尾端对齐直插，模块供电为 +5 V，数字 IO 为 3.3 V。
- STM32 与 FPGA 的 SPI2 使用 PI1/PB15/PI2/PE8（SCK/MOSI/MISO/CS），必须共地；当前实测 MOSI 已从 PI3 改接 PB15，对应 FPGA P2 的 L19。
- TI 与 FPGA 使用独占 SPI0：PA12（BP32）/PA14（板底 J28-7）/PA13（BP31）/PA15（BP30），必须共地；J28 无表面丝印，应按缺口和 1 脚方向定位；不能与 STM32 同时驱动 BX71 SPI。
- AD9910 若在新题目中使用，`IO_UPDATE` 应连 STM32 PB8；不应再连本 FPGA 的原 C 题同步引脚，否则可能出现输出争用。

## 代码约定

新增业务代码继续遵守：尽可能简洁、真实实现、优先可读性，并为全局变量、宏、函数、枚举和结构体添加简洁中文注释。APP 层只公开业务层直接调用的接口，私有辅助函数使用 `static`。
