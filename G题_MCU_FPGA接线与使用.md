# 2026 电赛 G 题：STM32H743 + Zynq7020/AD9280 接线与使用

> 本文只覆盖单片机、FPGA、AD9280 和模拟前端；不包含任何串口屏代码。当前数字代码已经通过主机算法测试、Vivado 行为仿真和 Keil 编译，但模拟前端仍必须按本文的校准步骤做实物扫频，才能把误差压进赛题的 5 mV 指标。

## 1. 最终系统结构

```text
G题 BNC 输入
  -> 交流耦合并偏置到 2.5 V
  -> 三节 OPA690 单位增益 Sallen-Key 低通（约 650 kHz，六阶）
  -> OPA695 同相放大 8 倍
  -> 交流耦合
  -> AD9280 模块 S2
  -> Zynq7020：32 MSPS 采样 + CIC3/R16 抗混叠抽取
  -> STM32H743：4096 点、2 MSPS、FFT 搜索 + 谐波模型 + 联合最小二乘拟合
  -> USART1 输出测量结果/由上层读取结果结构体
```

为什么增益级放在低通后面：当被测信号 250 mVpp 和 1 MHz 干扰 200 mVpp 同时出现时，先滤除大部分干扰再放大，可避免单 5 V 供电的高速运放在前级削顶。

数字参数已经固定为：AD9280 32 MSPS，FPGA CIC 三阶 16 倍抽取，送给 STM32 的等效采样率 2 MSPS，4096 点频率间隔 488.28125 Hz，满足题目 500 Hz 分辨率要求。

## 2. BX71、AD9280 模块和 STM32H743 接线

### 2.1 AD/DA 模块插接

- 高速 AD/DA 模块的 U3 排针尾端与 BX71 的 P3 尾端对齐后直插，不翻面、不左右错位。
- 模块使用稳定的 +5 V 电源。
- `S2` 是 AD9280 模拟输入；`S1` 是 AD9708 输出，本题不用 S1。
- FPGA GPIO 只能是 3.3 V 电平，绝不允许把 5 V 接到 P2 信号脚。

### 2.2 STM32H743 到 BX71 P2

| STM32H743 | BX71 P2 | FPGA 管脚 | 方向/作用 |
|---|---:|---:|---|
| PI1 / SPI2_SCK | P2-31 | K19 | STM32 -> FPGA，SPI 时钟 |
| PB15 / SPI2_MOSI | P2-37 | L19 | STM32 -> FPGA，命令/写数据 |
| PI2 / SPI2_MISO | P2-33 | K17 | FPGA -> STM32，读回数据 |
| PE8 / FPGA_CS | P2-34 | K18 | STM32 -> FPGA，低有效片选 |
| PH9 / FPGA_IRQ | P2-35 | L16 | 可选状态输入；G 题轮询采集时可以不接 |
| 任一 GND | P2-30 或 P2-12 | GND | 两板必须共地 |

注意：

- 只让 STM32H743 驱动这组 SPI。MSPM0G3507 不得同时接入并驱动这些线。
- 固件中的 SPI2 已配置为主机、MSB first；通信层会按 FPGA 六字节寄存器帧工作，不要临时改变 SPI2 管脚或 CS。
- 若上电提示 `AD9280 init failed`，先测 P2 共地、PE8 空闲时是否为高、K19/L19 是否有 3.3 V SPI 波形，再检查 FPGA 镜像版本。

### 2.3 调试串口（不是屏幕）

| STM32H743 | USB 转串口 | 参数 |
|---|---|---|
| PA9 / USART1_TX | RX | 115200, 8N1 |
| PA10 / USART1_RX | TX，可不接 | 115200, 8N1 |
| GND | GND | 必须共地 |

正常启动第一行应包含：

```text
[G] READY AD9280=32MSPS CIC3/R16 Fs=2000000 N=4096 df=488.28125Hz
```

每帧输出基波频率、合成波形 Vpp、真有效值、1~3 条谱线的频率与正弦峰值。结果也可由 `GContest_GetLastResult()` 和 `GContest_GetLastSamples()` 直接读取，不需要屏幕驱动。

## 3. 洞洞板模拟前端

### 3.1 电源和虚拟地

整块前端只用题目允许的 +5 V 单电源：

- +5 V 到 GND：入口放 `100 uF || 10 uF || 100 nF`。
- `VBIAS = 2.5 V`：+5 V 到 VBIAS 用 330 ohm，VBIAS 到 GND 用 330 ohm。
- VBIAS 到 GND：紧靠分压点放 `100 uF || 10 uF || 100 nF`。
- 每一片 OPA690/OPA695 的 7 脚与 4 脚之间紧贴芯片放 100 nF，旁边再放 4.7 uF。
- 这条 VBIAS 是交流“地”，滤波电容和运放反馈的参考端都接 VBIAS，不能误接真实 GND。

洞洞板布局必须短：BNC、三节滤波和增益级按信号方向排成一条线；每级输出不要绕回输入；100 nF 去耦引脚长度尽量小于 1 cm。

### 3.2 输入与第一级

- BNC 外壳 -> 模拟 GND。
- BNC 中心 -> 1 uF 薄膜/无极性电容 -> `IN` 节点。
- `IN` -> 100 kohm -> VBIAS，给运放输入提供直流偏置。
- `IN` 直接进入第一节 OPA690 低通。
- 默认不要并 49.9 ohm 输入终端，保持高输入阻抗，避免把 50 ohm 信号源幅度再分掉一半。若现场信号源的幅度定义明确要求 50 ohm 端接，可用跳帽临时并入 49.9 ohm，并在该状态下重新做整机幅度校准。

### 3.3 三节 OPA690 六阶低通

OPA690 的 SOIC-8 接法：7 脚 +5 V，4 脚 GND，3 脚同相输入，2 脚反相输入，6 脚输出，8 脚 DIS 悬空或通过 10 kohm 接 +5 V，1/5 脚不接。

每一节均按下面拓扑连接：

```text
上一级输出 -- R1 -- 节点 A -- R2 -- 节点 B -> OPA690 pin 3
                       |                    |
                       C1                   C2
                       |                    |
                 OPA690 pin 6             VBIAS

OPA690 pin 6 直接回接 pin 2（单位增益跟随器）
```

三节必须按低 Q 到高 Q 的顺序串联：

| 顺序 | R1 | R2 | C1（节点 A 到本级输出） | C2（节点 B 到 VBIAS） | 目标 Q |
|---:|---:|---:|---:|---:|---:|
| 1 | 255 ohm | 255 ohm | 1.0 nF | 910 pF | 0.524 |
| 2 | 232 ohm | 232 ohm | 1.5 nF | 750 pF | 0.707 |
| 3 | 352 ohm | 352 ohm | 2.7 nF | 180 pF | 1.936 |

若没有精确阻值，优先用 1% 电阻串/并联凑值；电容优先 C0G/NP0。不要随意把最后一节高 Q 放到最前面，否则更容易因洞洞板寄生和过载出现振铃。

### 3.4 OPA695 八倍增益

OPA695 的 SOIC-8：7 脚 +5 V，4 脚 GND，3 脚同相输入，2 脚反相输入，6 脚输出，8 脚 DIS 悬空或经 10 kohm 接 +5 V，1/5 脚不接。

连接：

- 第三节 OPA690 的 6 脚 -> OPA695 3 脚。
- OPA695 6 脚 -> 348 ohm -> OPA695 2 脚。
- OPA695 2 脚 -> 49.9 ohm -> VBIAS。
- 电压增益约为 `1 + 348/49.9 = 7.974`；软件按整机实测结果校准，不依赖理想值。
- OPA695 6 脚 -> 2.2 uF 薄膜/无极性隔直电容 -> 49.9 ohm 串联电阻 -> AD9280 模块 S2 中心。
- S2 外壳 -> 模拟 GND。
- 若 S2 的隔直后端没有模块自带直流回路，在 S2 中心侧加 100 kohm 到 GND；先用万用表断电测阻确认，避免和模块现有偏置重复。

通电前逐点查短路。首次通电不要接 AD9280，先看 VBIAS 和四个运放输出的直流是否都接近 2.5 V；再用示波器检查各级没有数十 MHz 自激，最后才接 S2。

## 4. 代码位置

### FPGA/Zynq

- `FPGA_BX71_AD9708_AD9280/rtl/cic3_decimator_16.v`：三阶 CIC、R=16、32 MSPS 到 2 MSPS。
- `FPGA_BX71_AD9708_AD9280/rtl/ad9280_capture_core.v`：采集、4096 点 BRAM、CIC 使能寄存器 `0x2F`、协议版本 `0x00010002`。
- `FPGA_BX71_AD9708_AD9280/sim/tb_cic3_decimator_16.v`：CIC 直流增益和第一零点仿真。
- `FPGA_BX71_AD9708_AD9280/sim/tb_ad9280_capture_core.v`：原始采集与“中止后重启+CIC”端到端回归。

### STM32H743

- `STM32H743/BSP/g_contest_app.c/.h`：整帧采集、换算、分析、结果接口和串口诊断。
- `STM32H743/DSP/periodic_analyzer.c/.h`：FFT 搜索、基波/谐波判定、联合最小二乘幅相拟合、真 RMS 与重建 Vpp。
- `STM32H743/Core/Inc/g_contest_config.h`：采样率、点数、前端增益、ADC 两点校准和限幅阈值。
- `STM32H743/Core/Src/main.c`：G 模式入口；当前 `G_CONTEST_ENABLE=1`。

没有加入陶晶驰、LCD、键盘或任何屏幕代码。

## 5. 构建与下载

### 5.1 STM32H743

在仓库根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\STM32H743\build_keil.ps1
```

烧录文件：`STM32H743/output/STM32H743.hex`。当前工程由 Keil/ArmClang 构建；下载成功只代表数字固件写入，不代表 5 mV 模拟精度已经通过。

### 5.2 FPGA 临时 JTAG 调试

纯 PL 全量构建：

```powershell
Set-Location .\FPGA_BX71_AD9708_AD9280
E:\Vivado\2018.3\bin\vivado.bat -mode batch -source scripts\build_g_analyzer.tcl -nojournal -nolog
```

输出位流为 `FPGA_BX71_AD9708_AD9280/output/BX71_G_ANALYZER.bit`。这是只保留 G 题 ADC/SPI/CIC 链路的时序收敛镜像，未使用的 AD9708 固定在中间码。用 JTAG 下载后保持 BX71 供电，再验证寄存器版本、采集和测量。通用 `build_project.tcl` 仍含实验 AM/FM 与 ProMax，不是 G 题交付入口。

### 5.3 当前采用 JTAG，不误烧旧 QSPI 镜像

本次 G 题交付的是纯 PL 的 `BX71_G_ANALYZER.bit`，用于 JTAG 下载；纯 PL 位流不能单独作为 Zynq 的 QSPI/PCAP 掉电启动镜像。现有 `BOOT_PS7.bin` 属于此前通用工程，不要拿它冒充 G 题镜像，也不要运行 `Program-Qspi.cmd` 覆盖当前板卡。比赛时下载 G 位流后保持 BX71 供电即可。若后续确需掉电自启，应另行把 `g_analyzer_top` 集成到 PS7 顶层、全量时序通过并用 FSBL 重新打包后再烧录。

## 6. 现场校准（决定能否达到 5 mV）

### 6.1 AD9280 两点电压校准

用稳定直流或已校准信号取得两个已知输入电压及对应 ADC 码，更新 `g_contest_config.h`：

- `G_CONTEST_CAL_LOW_CODE / G_CONTEST_CAL_LOW_V`
- `G_CONTEST_CAL_HIGH_CODE / G_CONTEST_CAL_HIGH_V`

当前 `0 -> -5 V、255 -> +4.9609375 V` 只是模块名义初值，不能当作比赛实测标定。

### 6.2 整机扫频幅相校准

当前 `g_contest_app.c` 内置的是“六阶模拟低通 + CIC3/R16”的理论补偿。按 10、50、100、200、300、400、500 kHz 逐点输入单音：

1. 用可信示波器读取 G 题 BNC 入口的真实正弦峰值。
2. 读取程序给出的 `Apeak`。
3. 新的 `amplitude_correction = 真实峰值 / 未补偿测得峰值`。
4. 将实测补偿值替换 `GContest_LoadInitialCalibration()` 中对应点。
5. 重新编译并复测 10~500 kHz；重点看 400/500 kHz，因为 CIC 下垂最大。

如果只能先改一个地方，先校正整机 8 倍增益；如果 500 kHz 误差大，再修高频补偿表。每次更换 50 ohm 端接状态、运放、电阻电容或 AD9280 模块后都必须重新标定。

### 6.3 抗干扰验收

分别测试：

- 被测信号 50、100、250 mVpp；频率 10、200、500 kHz。
- 单基波、基波+二次、基波+三次、基波+二次+三次。
- 再叠加 200 mVpp 的 1.0、1.2、1.5、2.0 MHz 干扰。

每组检查：

- `clip=0`；若为 1，先查模拟前端/ADC 削顶，不要用软件补偿掩盖。
- 基波误差不超过 1 kHz。
- Vpp、真 RMS、各正弦峰值与示波器/信号源真值的绝对误差均不超过 5 mV。
- 单帧 `latency` 明显小于 2000 ms。

## 7. 当前验证边界

已经完成的数字验证：

- CIC3/R16 行为仿真通过；直流码保持，2 MHz 第一零点符合预期。
- AD9280 采集核心回归通过，包括触发、抽取、统计、OTR、中止后重新采集和 CIC 读回。
- 周期分析器五组主机矩阵通过：10 kHz 边缘、接近 500 kHz、基波弱于谐波、二/三条谱线和 AD9280 8 位量化条件下，频率、真 RMS、Vpp 与各分量幅度均满足题目阈值断言。
- STM32H743 Keil 全量构建为 0 error、0 warning。

尚未替你完成的只有必须依赖实物的部分：洞洞板寄生/自激检查、AD9280 模块实测转换曲线、整机扫频补偿以及真实 1 MHz 干扰压制。满分目标靠这些校准闭环完成，不能只凭编译和仿真宣称已经满分。
## 8. 芯片官方资料

- [TI OPA690 数据手册](https://www.ti.com/lit/ds/symlink/opa690.pdf)
- [TI OPA695 数据手册](https://www.ti.com/lit/ds/symlink/opa695.pdf)
- [Analog Devices AD9280 数据手册](https://www.analog.com/media/en/technical-documentation/data-sheets/AD9280.pdf)