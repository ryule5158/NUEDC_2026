# FPGA G 题最终验证证据

最终复核日期为 2026-07-29。工具为 Vivado/Bootgen 2018.3，目标器件为
XC7Z020-2CLG400，生产顶层为 `g_analyzer_zynq_top`。

## 当前生产镜像

- Vivado 工程只加载 G 题顶层及其直接依赖：SPI 寄存器从机、AD9280 时钟、
  CIC3/R16、16 位采集缓存和 PS7 启动配置。
- ProMax、通用波形发生和实验 AM/FM 不在当前工程源文件集中。
- AD9708 端口只保持合法时钟和中点静默码，不提供控制寄存器。
- AD9280 协议为 `0x00010003`，缓存及统计采用 signed Q8.8。

## 数字仿真

- `g_cic_sim_final.log` 包含 `G_CIC3_DECIMATOR_TEST_PASS` 和
  `G_CIC_SIM_PASS`；覆盖对称最近偶数舍入、饱和及 10/20/30 kHz 幅值。
- `legacy_sim_final.log` 包含 `AD9280_CAPTURE_TEST_PASS` 和
  `LEGACY_SIM_PASS`；覆盖 Q8.8 RAM 数据、地址标签、最值、和、最新值、
  触发、OTR 和终止。
- 同一日志中的 DAC 用例只是未参与生产镜像的源码库回归，不代表当前位流提供
  DAC/AM/FM 控制功能。

## 路由后实现结果

- 全局建立/保持余量：`+1.008/+0.081 ns`。
- AD9280 输入建立/保持余量：`+1.008/+1.518 ns`。
- 顶层资源：`845 LUT`、`882 FF`、`2 RAMB36`、`0 DSP48E1`。
- 全部用户时序约束满足；最终构建为 `0 Critical Warning`、`0 Error`。
- `timing_final.rpt`、`drc_final.rpt`、`utilization_final.rpt` 和三份 CDC
  报告保留 Vivado 原始结果。数字仿真和静态报告不能替代模拟输入实测。

## 镜像

- JTAG 临时下载：`output/BX71_G_ANALYZER_PS7.bit`。
- QSPI 掉电启动：`output/BOOT_PS7.bin`。
- 两个文件的当前长度和 SHA-256 以 `output/SHA256SUMS.txt` 为准。

以后双击 `scripts/Build-QspiBoot.cmd`，成功构建会刷新位流、启动镜像、哈希和
六份路由后报告。
