# FPGA 最终验证证据

原始验证日期：2026-07-22。当前稳定镜像复核日期：2026-07-27。工具为 Vivado/Bootgen 2018.3，目标器件为 XC7Z020-2CLG400。

## 当前集成工程仿真

- `legacy_sim_final.log`：包含 `AD9280_CAPTURE_TEST_PASS` 和 `TEST_PASS: Q8.8 amplitude and offset scaling`。
- `promax_sim_final.log`：包含 `PROMAX_INTEGRATION_TEST_PASS target=2425739991 other=60458`。

ProMax 集成仿真实际覆盖四组行为：

1. `BXPM`标识、版本、能力值和寄存器地址映射；
2. ADC偏移二进制转18位有符号数，以及64个异步FIFO样本的顺序和数量；
3. 第2组第5抽头系数写入、快照保持/释放和代次更新；
4. 实际32 MSPS下 `fs/8` 目标DDC的选择性，目标功率为 `2425739991`，非目标通道为 `60458`。

`PROMAX_UPSTREAM_RECHECK.txt` 是上游原库可行性复核，记录了上游5项独立仿真和100 MHz综合。上游UART、9字节控制面及FFT没有集成到本工程，不能据此视为本工程功能。

## 路由后实现结果

- 全局建立/保持余量：`+0.404/+0.023 ns`。
- AD9708建立/保持余量：`+1.632/+2.158 ns`。
- AD9280建立/保持余量：`+1.008/+1.518 ns`。
- 顶层资源：`34,086 LUT`、`31,587 FF`、`1 RAMB36`、`163 DSP48E1`；其中ProMax实时数据面使用`160 DSP48E1`。
- DRC：`0 Error`。778条Warning由449条DSP输入流水建议、160条PREG建议、162条MREG建议及7条原AD9280缓存全局复位提示组成；50 MHz/125 MHz实际时序均已满足。

定向CDC结果：

- ADC→配置域：ProMax状态均识别为两级`ASYNC_REG`，无Critical；18条CDC-15来自XPM异步FIFO内部双口存储结构。另有6条CDC-5属于原AD9280在完成翻转量保护下读取的稳定统计快照。
- 配置域→ADC域：ProMax清零翻转量和XPM复位同步均识别为两级`ASYNC_REG`，无Critical；4条CDC-5属于原AD9280在启动翻转量保护下传递的稳定配置。
- `cdc_global_final.rpt`仍如实保留全局异步`reset_n`、SPI首级同步和XPM复位引出的CDC-1/7/13，不能宣称全工程“0 CDC”。跨数据时钟域安全性应以两份定向报告为准。

## 镜像

- `BX71_V4_U3_tail_AD9708_PS7.bit`：4,045,675字节，SHA-256 `AB757F2AD33D9FC92567F438BB629EF47EE3885CA5432D140A2E66C3E1512643`。
- `BOOT_PS7.bin`：4,149,824字节，SHA-256 `70088EC85FA7204E1F63FB3102BD055D02C3CA48D0D4A6BBA3ADE0E14BD77326`。

JTAG临时调试也应使用上述最终PS7位流；QSPI掉电保存烧录`BOOT_PS7.bin`。旧纯PL位流不交付，避免误烧。

## 文件说明

- `timing_final.rpt`、`drc_final.rpt`、`utilization_final.rpt`：PS7最终路由结果。
- `cdc_global_final.rpt`：全局CDC原始报告。
- `cdc_adc_to_cfg_final.rpt`、`cdc_cfg_to_adc_final.rpt`：排除复位端口影响后的双向数据时钟域报告。
- `vivado_ps7_build_final.log`：最终综合、布局布线、位流及Bootgen日志。

以后双击 `scripts/Build-QspiBoot.cmd`，成功构建会自动刷新六份路由后报告。数字仿真与静态报告不能替代P3映射、供电、ADC模拟输入和DAC模拟输出的实板测试。
