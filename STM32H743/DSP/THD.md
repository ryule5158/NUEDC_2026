# THD 自动测量与电赛使用指南

本模块按 2020 年 E 题《放大器非线性失真研究装置》和 2021 年 A 题《信号失真度测量装置》的口径升级。默认 THD 为基波到 5 次谐波的近似值：

```text
THD5 = sqrt(U2^2 + U3^2 + U4^2 + U5^2) / U1
```

`Measure_THDFromSpectrum()` 同时给出比值、百分数和 dB：

```text
thd         = THD5
thd_percent = THD5 * 100
thd_db      = 20*log10(THD5)
```

## 往年题目覆盖

| 考察点 | 赛题要求 | 库内支持 |
|---|---|---|
| 2020 E 题 | 1 kHz 输入；对无明显失真、顶部、底部、双向、交越失真自动测 THD | 基波限频搜索、H1~H5、噪声底扣除、一键聚合接口 |
| 2021 A 题基本部分 | 300~600 mVpp、1 kHz、THD 5%~50%、绝对误差不大于 5 个百分点、10 秒内显示 | THD%、有效性状态、幅度门限、4096 点分析远低于 10 秒预算 |
| 2021 A 题发挥部分 | 30~600 mVpp、1 kHz~100 kHz、误差不大于 3 个百分点 | 可配置基波频带、亚 bin 基频、可变 64~4096 点 FFT |
| 2021 A 题发挥部分 | 显示一个周期波形 | `Measure_ExtractPeriod()` 固定点数重采样 |
| 2021 A 题发挥部分 | 显示基波及谐波归一化幅值，只到 5 次 | `harmonic_ratio[0..4]` 分别为 1、U2/U1、…、U5/U1 |
| 自动测量 | 启动后不得人工干预，10 秒内完成 | `DSP_ProMax_AnalyzeEx()` 一次完成时域、FFT、THD 和质量指标 |

失真波形类别本身由模拟电路工作状态决定；DSP 的职责是对任意周期波形统一测得 H1~H5 和 THD，不依赖预存波形或人工选择类别。

## 推荐调用

```c
DSP_ProMaxConfigTypeDef config;
DSP_ProMaxResultTypeDef result;

DSP_ProMax_ConfigDefault(&config);
config.thd.fund_min_hz = 900.0f;
config.thd.fund_max_hz = 1100.0f;
config.thd.min_fundamental_amplitude = 0.005f; /* 按 ADC 电压标定后调整 */

DSP_ProMaxStatusTypeDef status =
    DSP_ProMax_AnalyzeEx(adc_voltage,
                         4096,
                         200000.0f,
                         &config,
                         &result);

if (status == DSP_PROMAX_OK && result.thd.valid != 0u) {
    float thd_percent = result.thd.thd_percent;
    float h2_over_h1 = result.thd.harmonic_ratio[1];
    float h5_over_h1 = result.thd.harmonic_ratio[4];
}
```

`DSP_ProMax_Analyze()` 仍然可用，它现在使用默认 THD5 配置，并接受 `FFT_StartN()` 支持的 64~4096 点二次幂长度。旧的 `Measure_THD()` 幅值数组接口保持不变。

只已有单边幅度谱时，直接调用：

```c
Measure_THDConfig_t config;
Measure_THDResult_t result;

Measure_THDConfigDefault(&config);
config.fund_min_hz = 1000.0f;
config.fund_max_hz = 100000.0f;

Measure_THDStatus_t status =
    Measure_THDFromSpectrum(magnitude, fft_len, sample_rate, &config, &result);
```

## 算法升级

旧实现先把最大整数 bin 当作基波，再用 `k0*h` 作为 h 次谐波中心。非整周期采样时，真实基波位于小数 bin，高次谐波的位置误差会随次数累积，5 次谐波最容易偏出积分窗口。

新实现采用以下流程：

1. 只在配置的基波频带内找峰，避免带外杂散被误判。
2. 用三点抛物线插值得到小数基波 bin。
3. 用“小数基波 bin × 谐波次数”预测 H2~H16，再在附近做局部峰值跟踪。
4. 对每个主瓣积分功率；默认适配 `FFT_StartN()` 的 Hann 窗，半宽为 2 bin。
5. 用非谐波 bin 估计噪声底，并从各谐波簇中扣除等带宽噪声，降低低 THD 时的正偏差。
6. 输出 H1~Hn 的频率、积分幅度和归一化幅度，以及 THD、THD%、THD dB。

该实现不使用动态内存，也不再使用 4097 字节或 516 字节的频点标记工作区；THD 核心可重入。

## 采样率与点数

要按赛题定义测到 5 次谐波，必须满足：

```text
5*f0 < fs/2
```

即理论上 `fs > 10*f0`。考虑抗混叠滤波过渡带和频率偏差，建议 `fs >= 12*f0`。

同时，H1 与 H2 的主瓣不能重叠。默认搜索半宽 1 bin、积分半宽 2 bin，要求基波至少约 7 个 FFT bin：

```text
f0*N/fs >= 7
```

因此不能用同一组 `fs=1.2 MSPS, N=4096` 同时覆盖 1 kHz 和 100 kHz：

| 基频范围 | 建议采样率 | 点数 | 说明 |
|---|---:|---:|---|
| 约 1 kHz | 100~200 kSPS | 4096 | 基波有 20 个以上 bin，H5 带宽充足 |
| 10~50 kHz | 600 kSPS | 2048 或 4096 | 可按量程切换采样率 |
| 约 100 kHz | 1.2 MSPS 或更高 | 4096 | H5=500 kHz，留出奈奎斯特保护带 |

TI MSPM0G3507 当前片上 ADC 上限为 1 MSPS，无法在 100 kHz 基波处为 H5 留出奈奎斯特保护带；发挥部分上限建议使用 STM32H743 的更高采样率方案或外部高速 ADC。TI 轻量库仍可在其可用频带内调用相同的 `Measure_THDFromSpectrum()`。

## 窗函数与配置

- `FFT_StartN()` 内置 Hann 窗，默认 `integration_half_width_bins=2`。
- 整周期同步采样并使用矩形窗时可设为 0~1。
- Blackman-Harris 窗主瓣更宽，应设为 4；它适合观察很小的谐波，但不能仍按 Hann 的 ±2 bin 积分。
- `search_half_width_bins=1` 通常足够补偿插值与时钟误差。过宽会把邻近非谐波杂散当成谐波。
- 设定 `fund_min_hz`/`fund_max_hz` 是自动测量可靠性的关键；不要在已知赛题频带时仍全带搜索。
- 30 mVpp 档必须在模拟前端提供合适增益、偏置和抗混叠滤波。DSP 无法恢复被 ADC 量化噪声淹没或已经削顶的信号。

## 状态处理

| 状态 | 含义 | 建议处理 |
|---|---|---|
| `MEASURE_THD_OK` | 请求的最高谐波可用，结果有效 | 显示 THD% 和 H1~Hn |
| `MEASURE_THD_ERROR_PARAM` | 指针、频带、次数或采样率无效 | 检查配置 |
| `MEASURE_THD_ERROR_LENGTH` | FFT 点数过小或不是偶数 | 使用受支持点数 |
| `MEASURE_THD_ERROR_NO_FUNDAMENTAL` | 频带内没有正幅度基波 | 检查接线、量程和频带 |
| `MEASURE_THD_ERROR_LOW_AMPLITUDE` | 基波低于设定门限 | 切换前端增益或拒绝本帧 |
| `MEASURE_THD_ERROR_RESOLUTION` | 相邻谐波主瓣会重叠 | 降低采样率、增加点数或缩窄积分宽度 |
| `MEASURE_THD_ERROR_BANDWIDTH` | 要求的高次谐波达到奈奎斯特边界 | 提高采样率；低次结果仍保留但 `valid=0` |

## 单周期波形

```c
float display_wave[256];
uint32_t count = Measure_ExtractPeriod(adc_voltage,
                                       sample_count,
                                       sample_rate,
                                       result.thd.fund_freq,
                                       display_wave,
                                       256);
```

函数从第一个有完整后续周期的上升过均值点开始，线性插值为固定点数，输出不重复周期末端点，适合直接发送到串口屏或手机端。

## 验证

```powershell
Set-Location .\STM32H743
powershell -ExecutionPolicy Bypass -File .\tests\run_measure_thd_host_test.ps1
powershell -ExecutionPolicy Bypass -File .\build_keil.ps1
```

数值回归覆盖非整周期 1 kHz/100 kHz、H1~H5 归一化幅度、强带外杂散、带宽不足和分辨率不足；Keil 全量构建用于验证实际 ArmClang 工程。
