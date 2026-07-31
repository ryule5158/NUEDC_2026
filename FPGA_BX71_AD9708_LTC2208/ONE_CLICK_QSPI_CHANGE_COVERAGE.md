# LTC2208 一键构建与 QSPI 烧录覆盖范围

入口：双击 `scripts/Program-Qspi.cmd` 或 `scripts/Build-And-Program-Qspi.cmd`。

## 实际参与发布的文件

唯一清单是 `scripts/ltc2208_release_inputs.txt`。当前包含：

- G 题顶层、STM32 SPI 从机、AD9708 静默时钟、LTC2208 时钟/采集/CIC5-FIR81 RTL。
- PS7 包装顶层 `rtl_ps7/g_analyzer_zynq_top.v`。
- LTC2208 U8 到 BX71 P3 交叉转接约束。
- `ps7/sys.bd`。

PowerShell 在构建前后对清单、清单内文件、构建 Tcl 和实际 FSBL 计算 SHA-256；Vivado 启动时再把自己的显式允许列表与同一清单逐项比较。任一边新增、删除、改名或内容变化而另一边未同步，流程失败，不打包、不烧录。

新建 RTL 不会被脚本擅自加入发布镜像。需要有意增加源文件时，必须同时更新 `ltc2208_release_inputs.txt` 和 `build_ps7_project.tcl`，再接受结构、时序、DRC、仿真和硬件复验。

成功时输入清单保存为 `output/SOURCE_SNAPSHOT_G_LTC2208_QSPI.txt`。

## 构建安全门禁

新位流只在以下条件全部满足后生成：

- 顶层为 `g_analyzer_zynq_top`，器件为 `xc7z020clg400-2`。
- PS7 存在，LTC2208 采集核和 CIC5/FIR81 各恰好一个。
- 时分复用 FIR 的 DSP48 数量在预期范围，实验 AD9708 AM/FM 核为 0。
- 全局 setup/hold 和 LTC2208 数据接口 setup/hold 均非负。
- DRC Error 为 0。
- 构建期间发布输入摘要没有变化。
- 位流、FSBL、BOOT、清单和 SHA-256 相互匹配。

烧录阶段还要求：

- JTAG 链只有目标 XC7Z020。
- `F800025C=0x00000000`，确认真实 JTAG 启动模式。
- QSPI offset 0 写入成功，并通过 Xilinx readback Verify。

任一步失败都返回非零退出码。完整重建开始时会删除本版本命名的旧交付物，失败后不会把旧文件冒充新构建继续烧录。

## 不会自动认定正确的变化

以下变化必须人工重新签核，脚本不会猜测：

- 更换 FPGA、板卡、Flash 类型或 offset。
- 改顶层、PS7、DDR、MIO、QSPI、PS 时钟或 FSBL。
- 改 ADC/DAC 端口、位宽、采样时钟、U8/P3 转接或时序模型。
- 改滤波架构、抽取比、输出采样率、样本格式或 STM32 协议。
- 启用实验 AM/FM 或 ProMax 数据面。
- 模拟前端增益、噪声、ENOB、输入阻抗和抗混叠性能。

## 使用顺序

1. 保存源文件，关闭仍会自动改写它们的工具。
2. BX71 断电切到 JTAG，再上电并连接 JTAG。
3. 双击 `scripts/Program-Qspi.cmd`。
4. 看到 `QSPI PROGRAMMING AND READBACK VERIFY PASSED` 后断电。
5. 切回 QSPI，再上电完成冷启动、设备 ID `0x22080001`、协议 `0x00030002` 和真实 ADC/STM32 联调。

构建、打包和 Verify 不等于 BNC 精度已达标。10~500 kHz 幅值、5 mV 最小分量、>=1 MHz 干扰和完整周期波形仍必须上板验收。
