# BX71 可移植 QSPI 掉电保存烧录

本工程不再依赖 Xilinx SDK workspace，也不需要为了烧录导入 HDF。烧录由
Xilinx SDK 2018.3 自带的命令行 `program_flash.bat` 完成。

## 新电脑所需内容

1. 完整工程目录。
2. Vivado 2018.3。
3. SDK 2018.3（必须包含 `bootgen.bat`、`program_flash.bat` 和
   `hw_server.bat`）。
4. Xilinx/Digilent JTAG 驱动。
5. BX71、JTAG 下载器和电源。

Vivado/SDK 可以装在不同盘符和不同父目录。脚本会依次从以下位置查找：

1. PowerShell 参数显式指定的路径。
2. `XILINX_VIVADO`、`XILINX_SDK` 环境变量。
3. `PATH`。
4. 每个本地盘符下常见的 `Xilinx/...`、`Tools/Xilinx/...` 或直接
   `Vivado/...`、`SDK/...` 目录。

## 日常使用

### 只烧录已有镜像

1. 断电，把 BX71 启动模式切到 JTAG。
2. 连接 JTAG 和电源并开机。
3. 双击 `scripts/Program-Qspi.cmd`。
4. 看到 `QSPI PROGRAMMING AND READBACK VERIFY PASSED` 后才能断电。
5. 断电，把启动模式切到 QSPI。
6. 再次上电，进行真实冷启动功能检查。

如果只想检查新电脑的环境和连接，双击 `scripts/Probe-Qspi.cmd`。它会完成
工具自动定位、`hw_server` 连接、JTAG 链和 XC7Z020 检查，但不会擦除或写入
Flash。

### 修改 FPGA 代码后完整发布

1. 保持 JTAG 模式并上电。
2. 双击 `scripts/Build-And-Program-Qspi.cmd`。
3. 脚本会依次完成：
   - 构建含 BX71 PS7 配置的位流；
   - 用工程内 FSBL 打包 `output/BOOT_G_ANALYZER.bin`；
   - 自动连接或启动 `hw_server`；
   - 确认 JTAG 链中存在 XC7Z020；
   - 写入 `qspi-x1-single`、Offset 0；
   - 执行 Xilinx 写后 Verify。
4. 成功后断电切换到 QSPI，再上电验证。

不能在 QSPI 与 JTAG 之间带电切换启动拨码。

## 为什么不再需要 HDF

SDK 图形界面的 `Program Flash` 需要 workspace 中存在 Hardware Platform，
所以没有 HDF 时会报 “No hardware specification in the workspace”。

命令行 `program_flash.bat` 只需要：

- 完整的 `BOOT_G_ANALYZER.bin`；
- 可执行的 `FSBL.elf`；
- Flash 类型；
- JTAG 目标。

这些输入全部在工程或 Xilinx 安装目录中，脚本可自行定位，因此与 SDK
workspace 无关。HDF 仍可用于建立 SDK 软件工程，但不是本工程烧录 QSPI 的
必要文件。

## 输出和证据

- `output/BOOT_G_ANALYZER.bin`：写入 QSPI 的完整启动镜像。
- `output/SHA256SUMS_G_QSPI.txt`：位流和启动镜像的 SHA-256。
- `output/program_g_qspi_*.log`：本机烧录日志。
- `output/LAST_PROGRAMMED_G_QSPI.txt`：最后一次烧录的时间、目标、长度、哈希和
  Verify 状态。

日志和 `LAST_PROGRAMMED_G_QSPI.txt` 是本机运行证据，不上传 Git。

## 非常规安装目录

如果自动查找失败，可以直接指定工具：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\Build-QspiBoot.ps1 `
  -Vivado 'D:\MyXilinx\Vivado\2018.3\bin\vivado.bat' `
  -Bootgen 'D:\MyXilinx\SDK\2018.3\bin\bootgen.bat'

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\Program-Qspi.ps1 `
  -ProgramFlash 'D:\MyXilinx\SDK\2018.3\bin\program_flash.bat' `
  -HwServer 'D:\MyXilinx\SDK\2018.3\bin\hw_server.bat'
```

如果同时连接多块 Zynq，脚本会拒绝继续，防止烧错板。烧录时请断开其他
Zynq，只保留目标 BX71。可用以下命令检查当前 JTAG 链：

```powershell
program_flash.bat -jtagtargets -url tcp:localhost:3121
```

Xilinx 2018.3 存在已确认的兼容性问题：`-jtagtargets` 列出的目标名再传给
`-target_name` 仍可能报 `Given target do not exist`，且错误退出码可能仍为
0。因此脚本先确认只有一条含 XC7Z020 的 Zynq 链，再提取探测结果左侧的
ARM DAP 数值 ID 传给 `-target_id`，并额外扫描原始输出中的 `ERROR`。

## 修改范围与重新生成

- 只改 PL RTL/XDC：运行完整构建和烧录入口。
- 只需要把同一个镜像烧到另一块板：运行 `Program-Qspi.cmd`。
- 修改 PS7、DDR、MIO、QSPI 或 PS 时钟：必须重新生成匹配的 PS7 位流和
  FSBL，再替换工程内 FSBL。
- 修改 HDF 或 SDK workspace：不会影响上述命令行烧录流程。
