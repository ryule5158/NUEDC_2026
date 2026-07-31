# BX71 LTC2208 可移植 QSPI 掉电保存烧录

本工程使用 Xilinx SDK 2018.3 的命令行工具完成 BOOT 打包、JTAG 探测、QSPI 写入和 readback Verify，不依赖 SDK workspace，也不需要为了烧录导入 HDF。

## 所需环境

- 完整的 `FPGA_BX71_AD9708_LTC2208` 目录。
- Vivado 2018.3。
- SDK 2018.3，包含 `bootgen.bat`、`program_flash.bat`、`hw_server.bat` 和 `xsdb.bat`。
- Xilinx/Digilent JTAG 驱动、BX71、JTAG 下载器和稳定电源。

脚本依次从显式参数、`XILINX_VIVADO`/`XILINX_SDK`、`PATH` 和常见本地安装目录寻找工具。

## 自动重建并烧录

1. BX71 完全断电。
2. 把启动模式切到 JTAG，再连接下载器并上电。
3. 双击 `scripts/Program-Qspi.cmd`。
4. 脚本先按 `scripts/ltc2208_release_inputs.txt` 完整重建 PS7 位流，核对源码指纹、时序、DRC、BOOT 和 SHA-256，然后才访问 Flash。
5. 只有看到 `QSPI PROGRAMMING AND READBACK VERIFY PASSED` 才能断电。
6. 断电切回 QSPI，重新上电做真实冷启动、FPGA ID 和协议版本验证。

不能带电切换 JTAG/QSPI。`Program-Qspi.cmd` 与 `Build-And-Program-Qspi.cmd` 都会执行完整重建；不会直接烧录未经源指纹校验的旧 BIN。

## 只读或不接板验证

只检查 JTAG 工具、链路、XC7Z020 和启动模式，不写 Flash：

```powershell
.\scripts\Probe-Qspi.cmd
```

只验证现有位流、源码指纹、BOOT 和哈希，不访问 JTAG/QSPI：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\Program-Qspi.ps1 -ValidateOnly
```

只构建和打包，不烧录：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\Build-QspiBoot.ps1
```

## 发布产物

- `output/BX71_G_ANALYZER_LTC2208_PS7.bit`：含 PS7 的发布位流。
- `output/BOOT_G_ANALYZER_LTC2208.bin`：写入 QSPI offset 0 的完整启动镜像。
- `output/LATEST_G_LTC2208_QSPI_BUILD.txt`：构建时间、源码摘要、长度和哈希。
- `output/SOURCE_SNAPSHOT_G_LTC2208_QSPI.txt`：实际发布输入的逐文件 SHA-256。
- `output/SHA256SUMS_G_LTC2208_QSPI.txt`：位流和 BOOT SHA-256。
- `output/g_ltc2208_qspi_verification/`：路由后时序、DRC、CDC 和资源报告。

`output/program_g_qspi_*.log` 与 `output/LAST_PROGRAMMED_G_QSPI.txt` 是本机烧录证据，已被忽略，不应提交 Git。

## 工具在非常规目录

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\Build-QspiBoot.ps1 `
  -Vivado 'D:\MyXilinx\Vivado\2018.3\bin\vivado.bat' `
  -Bootgen 'D:\MyXilinx\SDK\2018.3\bin\bootgen.bat'

powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\scripts\Program-Qspi.ps1 `
  -ProgramFlash 'D:\MyXilinx\SDK\2018.3\bin\program_flash.bat' `
  -HwServer 'D:\MyXilinx\SDK\2018.3\bin\hw_server.bat' `
  -Xsdb 'D:\MyXilinx\SDK\2018.3\bin\xsdb.bat'
```

如果连接多块 Zynq，脚本会拒绝继续，防止烧错板。烧录前只保留目标 BX71。脚本还会读取 `F800025C`，只有真实 JTAG 启动模式 `0x00000000` 才允许写 Flash。

## HDF、FSBL 与改动边界

命令行 `program_flash.bat` 只需要匹配的 BOOT、FSBL、Flash 类型和 JTAG 目标，所以不依赖 SDK 图形 workspace。当前工程内 FSBL 与 `ps7/sys.bd` 配套；若修改 DDR、MIO、QSPI 或 PS 时钟，必须重新生成并替换匹配的 FSBL。

修改 PL RTL/XDC 后应运行完整入口。若需要把完全相同且源指纹仍匹配的镜像烧到另一块板，可直接调用 `Program-Qspi.ps1`；常规使用仍推荐 `Program-Qspi.cmd`。
