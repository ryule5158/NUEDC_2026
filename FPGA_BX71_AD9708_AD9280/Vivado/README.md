# Vivado 工程入口

正式掉电启动工程：

`BX71_G_ANALYZER_PS7/BX71_G_ANALYZER_PS7.xpr`

该生产工程只综合 Zynq PS7、AD9280 和固定 CIC3/R16 G 题链路；AD9708 保持
中点静默，ProMax 与实验 AM/FM 不进入位流。源文件按相对路径引用本 FPGA 目录内
的 `rtl/`、`rtl_ps7/`、`ps7/` 与 `constraints/`，不引用 TI 或 STM32 工程。

推荐通过上一级 `scripts/Build-QspiBoot.ps1` 全量构建。脚本仅在运行 Vivado 2018.3 时建立临时英文目录联接，以规避旧版本工具对中文路径的兼容问题；`.xpr` 和全部正式输出仍保存在本 FPGA 工程内。

如只需纯 PL 调试，可运行 `scripts/build_g_analyzer.tcl`，工程会生成到
`BX71_G_ANALYZER_PL/`；该纯 PL 结果不能代替正式 PS7/QSPI 交付镜像。

行为仿真临时工程统一生成到 `Simulation/`，该目录可随时重建且不会提交到 Git。
