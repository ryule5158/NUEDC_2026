# STM32H743 通用库工程

Keil 工程入口为 `MDK-ARM/STM32H743.uvprojx`，CubeMX 工程入口为 `STM32H743.ioc`。`main.c` 仅完成 CubeMX 外设初始化，不自动运行任何题目业务，方便在新题目中按需调用 `Modules` 中的 APP 层。

命令行强制全量构建：

```powershell
cd D:\NUEDC\fpga\集成工程\STM32H743
powershell -ExecutionPolicy Bypass -File .\build_keil.ps1
```

最后一次验证生成的 `.hex` 和带调试符号的 `.axf` 保存在 `output/`；Keil 工作目录中的标准中间文件用于后续增量构建，可由 uVision 的 Clean 功能随时重新生成。

本工程只引用自己的 `BSP/`、`DSP/`、`Modules/`、`Core/` 和 `Drivers/`，不引用 TI 工程目录。

## 高速 AD/DA 用法

```c
/* 初始化 FPGA 高速 DAC，用原始码先检查数字链路。 */
if (AD9708_AppInit() == AD9708_OK)
{
  (void)AD9708_AppOutputCalibrationCode(192U);
}

/* 立即触发采集 1024 点，不抽取，超时 100 ms。 */
if (AD9280_AppInit() == AD9280_OK)
{
  (void)AD9280_AppCapture(1024U, 1U, 100U);
}
```

AD9708 电压输出接口故意要求校准，避免将不同模块电位器的偏差当作固定参数。如果只需确认数字链路，可用 `AD9708_AppOutputCalibrationCode()` 直接输出原始码。

## FPGA ProMax 用法

`Modules/fpga_promax.c/.h` 是最新版 BX71 DSP ProMax 的通用控制核心，`fpga_promax_link.c` 把它接到现有 SPI2 六字节链路。调用 `FPGA_ProMax_Init(&句柄)` 后可配置 8 路 DDC、装载 4 组 32 抽头模板，并读取 FPGA 产生的功率和匹配峰值；初始化不会自动启动数据面。

当前 FPGA 实时档能力字为 `0x09200408`，不包含 UART、线级 CRC 或 FFT。`FpgaPromax_SetFft()` 等 FFT 接口会返回 `FPGA_PROMAX_E_UNSUPPORTED`，不能把它当作已实现的硬件 FFT。完整闭环代码、判据和预期值见上一级 `硬件测试指南.md`。

ProMax、AD9708 和 AD9280 共用 SPI2，裸机业务必须在主循环中串行调用；不要在中断中调用，也不要从多个任务并发访问。若以后加入 RTOS，应由业务层用同一把互斥锁保护整条 FPGA 链路。

工程原有 `DSP/DSP_ProMax.c/.h` 是 STM32H743 对内存数组执行的一次性 CPU 统计/FFT工具；新增 `Modules/fpga_promax.c/.h` 则是 FPGA 实时加速器控制 API。两者用途不同，均未互相替换。

## 通用库边界

- 保留 AD9910、AD9708、AD9280、AD5687、DAC8831、ADS8363、ADS8688、PE4302、BSP、CPU DSP 和 FPGA ProMax 控制库。
- CPU DSP 的 THD 接口已按 2020 E 题与 2021 A 题升级：支持 THD5、H1~H5 归一化幅度、1 kHz~100 kHz 限频搜索、单周期波形与自动测量状态；用法见 `DSP/THD.md`。
- 删除 C 题任务、C 题串口屏交互和六档 SM 外部延时封装。
- AD9910 使用 STM32 PB8 直接输出 `IO_UPDATE`，不再依赖 FPGA 同步寄存器。
- `Modules_Header.h` 只汇总 APP 层与必要的通用接口；底层寄存器与链路细节留在各自驱动内。
