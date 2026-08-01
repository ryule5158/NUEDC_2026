# 无源 LC 滤波器双通道幅相校准

本工程支持使用 AD9238 的 CH1、CH2 同步采集滤波前后信号，测量

\[
H(f)=\frac{Y_{CH2}(f)}{X_{CH1}(f)}
\]

并输出 9 kHz～501 kHz 的幅频、相频曲线。校准采集固定使用 4096 点、2 MSPS；两路均采用相同的原始 R25 抽取，不经过单通道 FIR/CIC，因此两路数字处理延时一致。普通 G 题测量结束后仍自动恢复 FIR307+CIC3/R25。

## 为什么要测两遍

双路 ADC、连接线和模拟前端自身存在增益差与固定相位差。只测一次滤波器，会把这些误差一起算进 LC 响应。因此必须完成：

1. `THRU` 直通基准：同一个正弦信号用等长线缆同时接 CH1、CH2，不接 LC。
2. `FILTER` 滤波器测量：CH1 接滤波前节点，CH2 接滤波后节点。

后处理按 `H_LC=H_FILTER/H_THRU` 消除通道增益差、ADC 时钟/布线偏差和线缆固定时延。只要两次测量期间接线、阻抗、采样时钟和通道没有变化，该方法能够满足相位校准需要。

## 接线与信号源设置

- 信号源输出单一正弦波，关闭调制、扫幅和直流偏置。
- 扫频范围设为 9 kHz～501 kHz，建议从低到高连续慢扫 60～120 s。
- 目标电平为 250 mVpp。若信号源幅度定义为“接 50 Ω 负载时的电压”，而 ADC 输入为高阻，实际端口电压可能加倍；应把信号源负载显示设为 High-Z，或用示波器确认 ADC 插座处约为 250 mVpp。
- 幅值的绝对准确度不会直接影响传递函数比值，但 CH1、CH2 都必须不削顶，滤波后信号也应有足够信噪比。
- 直通与滤波器测量应使用相同线缆、转接头、终端方式和信号源幅度。
- 无源 LC 的响应取决于源阻抗和负载阻抗。校准时必须复现装置实际工作的源、负载和终端条件，否则测得曲线不能代表装机后的响应。
- CH1 对滤波前节点形成额外负载。若 LC 对该负载敏感，应使用高阻缓冲或高阻探测点，并在实际工作接线中保持等效负载一致。

## 串口采集

USART1 使用 115200、8N1。终端打开日志保存后执行：

```text
BODE THRU START
```

开始 9 kHz→501 kHz 的直通慢扫，扫完后发送：

```text
BODE STOP
```

接入 LC，并保持其余条件不变，再执行：

```text
BODE FILTER START
```

完成第二遍慢扫后再次发送 `BODE STOP`。若使用信号源手动逐点调节，可用：

```text
BODE THRU ONCE
BODE FILTER ONCE
```

每个有效点输出一行：

```text
BODE,模式,频率,CH1峰峰值,CH2峰峰值,增益,增益dB,包裹相位,展开相位,CH1残差,CH2残差,CH1削顶,CH2削顶,质量标志
```

`quality=1` 表示未削顶、正弦拟合残差合格且滤波后信号幅度足够；默认后处理只使用这些点。

## 生成校准曲线

假设串口日志保存为 `bode.log`：

```powershell
python .\tools\process_bode_calibration.py .\bode.log --output-dir .\bode_output
```

也可把直通和滤波器日志作为两个输入文件：

```powershell
python .\tools\process_bode_calibration.py .\thru.log .\filter.log --output-dir .\bode_output
```

脚本生成：

- `lc_response_corrected.csv`：去除双通道差异后的完整 LC 幅频、相频数据；
- `lc_response_corrected.svg`：幅频与连续相频曲线；
- `lc_inverse_calibration_8points.csv`：当前固件 8 个校准频点对应的幅值和相位逆补偿量。

其中 `amplitude_correction=1/|H_LC|`，`phase_correction_deg=-∠H_LC`。在把这组数据写入正式测量补偿表前，应先检查曲线是否连续、直通增益是否接近 1、直通相位是否平滑，以及 9 kHz、501 kHz 两端是否都有覆盖。
