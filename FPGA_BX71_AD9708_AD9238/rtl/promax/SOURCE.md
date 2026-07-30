# DSP ProMax RTL 来源

- 上游仓库：`git@github.com:ryule5158/BX71-DSP-ProMax.git`
- 上游提交：`3ff859aa888a7e77996f6fc3b32b92be8352f69c`
- 上游集成范围：实时 DDC、CIC、功率检测和匹配滤波数据面所需的 10 个 RTL 文件。
- 本工程新增：`dsp_stream_stats.sv`，在满速采样流上连续完成测量统计；它不是上游文件。
- 未集成：上游 9 字节 SPI/UART 控制面、FFT IP、演示顶层和板级约束。

上游文件的运算逻辑保持不变，仅补充了简洁中文注释；本工程通过
`ad9238_promax_core.sv` 接入现有 AD9238 数据流和 6 字节寄存器协议，并将
4 组匹配滤波器同时作为通用 32 阶 FIR 使用。
