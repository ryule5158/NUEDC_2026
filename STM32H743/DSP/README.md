# STM32H7 电赛 DSP 库

本目录面向电赛信号类题目的 STM32H7 裸机工程，覆盖采样频谱、滤波、IQ、AM/FM/BPSK 解调、测量、相关、拟合、自适应处理和软件锁相。代码默认使用单精度浮点，热路径不使用动态内存。

## 模块选择

| 模块 | 用途 | 重要约束 |
|---|---|---|
| `FFT.*` | 64～4096 点 FFT、Goertzel、窗函数、谐波提取 | 使用全局工作区，不可重入；旧入口固定 4096 点，新代码优先 `FFT_StartN()` |
| `ADC_FFT.*` | 对板载 ADC 的 4096 点数据做频谱和谐波分析 | 当前基波搜索范围固定为 50～1000 Hz，静音帧返回错误 |
| `Filter.*` | 兼容旧工程的 FIR/IIR、中值、滑动平均、抽取 | 保留旧 API；新设计优先使用 `FilterEx` |
| `FilterEx.*` | 流式 FIR/SOS、Butterworth、RMS、包络、抗混叠抽取 | 对象和工作区由调用者持有；初始化/设计与热路径分离 |
| `IQ.*` | 已知频率的幅相测量、双通道相位差 | 输入需为有限值；非整周期采样会产生泄漏误差 |
| `SoftPll.*` | 与 HAL 解耦的软件相位/频率控制器 | 调用者负责把命令施加到 DDS/NCO/PWM；无效输入会拒绝更新 |
| `Demod.*` | AM/FM/BPSK 解调、载波恢复、频偏/调制度估计 | FFT Hilbert 需要 `2*n` float；PLL/Costas 必须使用 Context 跨块保存状态 |
| `Measure.*` | RMS、Vpp、频率、THD、H1~H16、SNR、SINAD、ENOB、单周期波形 | THD 支持限频搜索、亚 bin 谐波跟踪、噪声底扣除和明确错误状态；详见 `THD.md` |
| `DSP_ProMax.*` | 一次聚合时域、可变点数 FFT、THD 和质量分析结果 | 支持 64~4096 点二次幂，使用全局 FFT 工作区，不可重入 |
| `Correlate.*` | 时延、互相关、自相关、基频估计 | 时域复杂度约为 `O(len*max_lag)` |
| `Fit.*` / `ModelFit.*` | 标定曲线和二阶网络模型拟合 | 拟合前应归一化自变量；无效样本会被拒绝或跳过 |
| `Adaptive.*` | LMS、正弦分离、自适应陷波、在线统计 | LMS 步长需要按输入功率和收敛速度实测整定 |

## 推荐调用顺序

1. 在采集边界完成 ADC 电压换算，并确认采样率是真实值。
2. 需要选频或降采样时先用 `FilterEx` 设计抗混叠滤波器。
3. 已知单频幅相优先使用 `IQ`；未知频率先用 FFT/相关法粗估，再做 IQ 精测。
4. AM/FM 连续流优先使用 `Demod_AMContext` / `Demod_FMContext`；无状态 Hilbert API 适合独立帧。
5. PLL/Costas 未锁定样本以 NaN 表示。送入 FFT/Goertzel 前调用 `Demod_ReplaceNaN()`，或在采集层明确处理无效点。
6. 所有 `Init`/`Design` 在配置阶段调用；逐点 `Process` 才放入高频循环或 ISR。

## 兼容接口与新代码

- `arm_fir_f32_lp()`、`IIR_*()` 和 `Filter_Decimate()` 为旧工程兼容入口，函数名和参数保持不变。
- `arm_fir_f32_lp()` 现在可安全处理最后不足 32 点的尾块。
- `Filter_Decimate()` 严格返回 `floor(len/factor)`，输出数组按该长度分配。
- 新滤波链优先使用 `FilterEx_FIR`、`FilterEx_Decimator` 和 `FilterEx_SOS`，它们有明确状态、容量和错误码。
- `FFT_Start()` / `FFT_StartEx()` 仍固定处理 4096 点；需要其他合法点数时使用 `FFT_StartN()`。

## 内存、并发与实时性

- `FFT` 的全局输入、复数工作区和幅度谱约占 68 KiB RAM，不可在 ISR 与主循环并发调用。
- `Demod` 的旧百分位/兼容接口含静态 4096 点工作区，也不可重入；Context 只解决环路和滤波状态，不会把 FFT Hilbert 变成流式 overlap-save。
- `Measure_THDFromSpectrum()` 和 `Measure_Quality()` 不使用动态内存或大频点位图；普通测量接口可重入。
- `FilterEx`、`IQ`、普通 `Measure` 和调用者持有状态的自适应模块可以各实例独立使用。

## BPSK 当前适用范围

`Demod_PSK_Demodulate()` 使用平方载波恢复、可变点数 FFT、I/Q 混频和星座轴投影。`if_search_min_hz` / `if_search_max_hz` 表示二倍中频搜索范围，不是原始中频范围。当前码率判决针对候选基频 3/4/5 kHz，对应 6/8/10 kbps；若赛题码率集合变化，应先扩展配置，而不是直接套用结果。

## 已知限制

- THD 有 PC 数值回归和 C 测试向量；当前桌面若缺少 MSVC C 运行库头文件，会运行数值回归并由 Arm Compiler 6 全工程构建覆盖嵌入式 C 编译。
- `ADC_FFT` 的基波搜索范围仍为 50～1000 Hz，超出范围应使用通用 FFT/Goertzel 接口。
- FFT Hilbert 采用整帧周期延拓，帧边缘会有误差；分析时使用 `edge_discard_samples`，真正连续处理需 FIR Hilbert 或 overlap-save。
- 旧 `Filter` 保留全局缓冲和简化高阶设计以兼容已有工程；要求可重入、精确高阶 Butterworth 或明确容量时使用 `FilterEx`。

## 构建验证

2026-07-24 使用 Keil μVision / ArmClang 6.23 构建 `MDK-ARM/STM32H743.uvprojx`：`0 Error(s), 0 Warning(s)`；THD 数值回归通过。
