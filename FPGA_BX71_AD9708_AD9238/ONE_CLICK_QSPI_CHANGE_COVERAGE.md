# FPGA 一键构建与 QSPI 烧录的改动覆盖范围

入口：双击 `scripts/Build-And-Program-Qspi.cmd`。

## 会自动覆盖的改动

- `rtl/`、`rtl_ps7/` 内已有 RTL 的任何内容修改。
- 上述目录中新建的 `.v`、`.sv`、`.vhd`、`.vhdl`、`.vh`、`.svh` 文件，包括子目录。
- `constraints/` 内新增或修改的全部 `.xdc`。
- 可选 `ip/` 内新增或修改的 `.xci`、`.xcix`、`.dcp`、`.edf`、`.edif`。
- 固定的 `ps7/sys.bd`、构建 Tcl 和实际选中的 `FSBL.elf`。

每次运行都会删除并重建专用 Vivado 工程，不依赖 GUI `.xpr` 中残留的源文件列表。构建前后会分别计算全部输入的 SHA-256 总摘要；只要构建期间任何输入发生变化，流程立即失败，不会打包或写入 QSPI。成功时输入清单保存为 `output/SOURCE_SNAPSHOT_G_AD9238_QSPI.txt`。

## 安全门禁

脚本只在以下条件全部满足后生成新位流：

- 顶层仍是 `g_analyzer_zynq_top`，目标器件仍是 `xc7z020clg400-2`；
- PS7、G 题采集核和当前硬件约束存在，实验 AM/FM 核未进入该交付镜像；
- 路由后的全局及 AD9238 接口 setup/hold 均非负；
- DRC Error 为 0；
- 位流、FSBL、BOOT 镜像和输入摘要相互匹配；
- 仅检测到目标 XC7Z020，且启动模式寄存器 `F800025C` 为 JTAG 模式 `0x00000000`；
- QSPI offset 0 写入完成并通过 Xilinx Verify。

任一步失败都返回非零退出码，旧产物不会被当成新工程继续烧录。

## 不能无条件自动推断的结构变化

以下变化会触发明确失败或需要同步修改发布配置，不能由脚本安全猜测：

- 更换 FPGA 型号、板卡、Flash 类型或写入 offset；
- 重命名/替换顶层，改变 PS7 block design、DDR/MIO/时钟，或更换必须匹配的新 FSBL；
- 更换 ADC/DAC 接口、端口名、位宽或时钟后，需要重新签核 XDC 和接口时序门禁；
- 有意启用实验 AM/FM 核时，需要单独的已验证交付配置；
- RTL 的算法功能正确性、外部接线和模拟性能。

这些边界是安全设计：脚本可以保证“当前声明的工程输入全部被构建、产物不是旧文件、时序/DRC/烧录校验通过”，但不能把任意硬件架构变化自动认定为正确。

## 使用顺序

1. 保存所有 FPGA 源文件，关闭仍会自动改写源文件的编辑操作。
2. BX71 完全断电，拨到 JTAG 启动模式，再上电并连接 JTAG。
3. 双击 `scripts/Build-And-Program-Qspi.cmd`。
4. 看到 `QSPI PROGRAMMING AND READBACK VERIFY PASSED` 后再断电。
5. 拨回 QSPI 启动模式并重新上电，完成冷启动和应用寄存器/串口版本验证。

构建成功不等于板上功能已经物理验证；掉电启动和实际 ADC/通信链路仍需按第 5 步确认。