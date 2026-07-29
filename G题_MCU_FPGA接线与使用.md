# 2026 电赛 G 题：STM32H743 + Zynq7020/AD9280 接线与使用

> 本文以当前实物为准：信号在进入 AD9280 前没有放大，程序按 AD9280 模块输入端
> 的名义电压报告。陶晶驰串口屏为默认入口，USART1 串口助手同时保留。数字代码已
> 通过主机算法测试和 Vivado 行为仿真；模拟量程、传递增益、非线性和抗混叠能力仍
> 必须实测，不能把软件构建或理论值写成 5 mV 实物精度。

## 1. 最终系统结构

```text
G题 BNC 输入
  -> 当前无放大，直接进入 AD9280 模块 S2
  -> Zynq7020：32 MSPS 采样 + 固定 CIC3/R16
  -> signed Q8.8、4096 点、2 MSPS
  -> STM32H743：CIC 解析幅相逆响应 + FFT 搜索
  -> 被测谐波与 1 MHz nuisance 分量联合最小二乘拟合
  -> USART2 陶晶驰串口屏 + USART1 串口助手
```

数字参数固定为：AD9280 32 MSPS、FPGA CIC 三阶 16 倍抽取、STM32 等效
采样率 2 MSPS、4096 点，FFT 频率间隔 488.28125 Hz。协议为 v1.4
（`0x00010004`），响应 profile 为 `0x00030010`。`R=16` 和 CIC3 是生产数据
格式的一部分，不是可选方案。

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

### 2.3 串口屏与串口助手

| STM32H743 | 对端 | 参数/用途 |
|---|---|---|
| PA2 / USART2_TX | 陶晶驰 RX | 115200-8-N-1，默认结果显示 |
| PA3 / USART2_RX | 陶晶驰 TX | 接收屏幕的 `G1`、`G3`、`GS` |
| PA9 / USART1_TX | USB 转串口 RX | 115200-8-N-1，文本诊断和数据 |
| PA10 / USART1_RX | USB 转串口 TX | 发送 `1`、`3`、`CAL/Q/EXIT` 或诊断命令 |
| GND | 两个对端 GND | 必须共地 |

正常启动日志应包含类似内容：

```text
[G] READY ADC=AD9280+FPGA-CIC3-R16-S16Q8.8 physicalFs=32000000Hz Fs=2000000Hz N=4096 df=488.28125Hz
[G] empirical_frequency_calibration=OFF analytic_response_compensation=ON voltage_transfer=nominal
[G] interference_rejection=joint_model expected=1000000Hz search=+/-5000Hz ...
[CLI] READY USART1 115200 8N1; send 1/3, or CAL for raw Q
```

串口助手发送 `1` 或 `3` 后，每帧输出完整周期波形、基频、重建波形 Vpp、真
有效值、10～500 kHz 分量和 `[REJECT]` 干扰诊断；波形区每行只有一个 mV 数值。
屏幕与串口助手触发同一套采集/分析流程，不存在两套测量算法。
`[REJECT]` 的 `post_dsp_peak/rms` 是 CIC 输出采样域的诊断值，不是经过 CIC
逆响应或整机标定后的 1 MHz 源幅值。

直流两点标定时，串口发送 `CAL` 加回车，稳定输入后发送 `Q` 加回车。固件打印
16×4096 点的原始 Q8.8 统计结果，不经过伏特或频响校准；记录
`[CAL] RESULT Q=...`、`clip` 和 `overrange`。发送 `EXIT` 返回正常测量。

## 3. 当前模拟输入与未来抗混叠边界

当前连接是信号源/被测端直接接 AD9280 模块 S2，没有软件假定的 8 倍增益，也没有
已确认装入的六阶低通。因此：

- 用示波器在 S2 端确认真实 Vpp、直流偏置和过冲；程序当前只按模块名义
  `10 V / 256 code` 换算，不能替代整机电压标定。
- 信号源、BX71、STM32、AD9280 模块和示波器必须共地；不确定端接方式时，同时
  记录信号源的 High-Z/50 Ω 设置和 S2 实测幅度。
- `clip=1`、OTR 有效或数据贴近量程边界时，必须先降低输入或修复硬件，软件不能
  恢复已经削顶的波形。
- FPGA CIC3/R16 的解析补偿只纠正确定性数字幅相响应，不会纠正 AD9280 模块、
  线缆、端接和未来模拟前端的增益/相位误差。
- CIC 抽取前若存在会混叠入 0～1 MHz 的带外能量，软件不能区分其来源。未来增加
  抗混叠模拟前端时，必须先实测 10～500 kHz 通带平坦度、群延迟、噪声、削顶和
  1 MHz 以上抑制度，再决定是否启用独立的经验校准；不能把某个理论六阶网络写成
  已装、已标定或已达到精度。

## 4. 代码位置

### FPGA/Zynq

- `FPGA_BX71_AD9708_AD9280/rtl/cic3_decimator_16.v`：三阶 CIC、R=16、32 MSPS 到 2 MSPS。
- `FPGA_BX71_AD9708_AD9280/rtl/ad9280_capture_core.v`：采集、4096 点 BRAM、
  v1.4（`0x00010004`）协议、只读固定 R16 元数据和响应 profile
  `0x00030010`；`0x23`/`0x2F` 写入均被忽略。
- `FPGA_BX71_AD9708_AD9280/sim/tb_cic3_decimator_16.v`：CIC signed Q8.8
  舍入/饱和及 10～500 kHz 七点解析幅相补偿仿真。
- `FPGA_BX71_AD9708_AD9280/sim/tb_ad9280_capture_core.v`：协议/profile、采集、
  触发、统计、读回与中止后重启回归。

### STM32H743

- `STM32H743/BSP/signal_adc.h` 与 `signal_adc_ad9280.c`：ADC 无关契约和当前
  AD9280 后端，封装采样率、码制、名义电压换算、削顶及 CIC 解析响应。
- `STM32H743/BSP/g_contest_app.c/.h`：整帧采集、分析配置、结果接口和串口诊断。
- `STM32H743/DSP/periodic_analyzer.c/.h`：FFT 搜索、基波/谐波判定、CIC 复响应
  补偿、1 MHz nuisance 联合最小二乘、真 RMS 与重建 Vpp。
- `STM32H743/Core/Inc/g_contest_config.h`：点数、分析频带、有效分量门限和
  1 MHz 干扰模型参数。
- `STM32H743/Core/Src/main.c`：直接进入 G 题应用，不保留编译期方案选择；
  USART2 串口屏为默认入口，USART1 串口助手并存。

## 5. 构建与下载

### 5.1 STM32H743

在仓库根目录运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\STM32H743\build_keil.ps1
```

烧录文件：`STM32H743/output/STM32H743.hex`。当前工程由 Keil/ArmClang 构建；下载成功只代表数字固件写入，不代表 5 mV 模拟精度已经通过。

### 5.2 FPGA 构建与临时 JTAG 调试

含 PS7/QSPI 启动配置的 G 题全量构建：

```powershell
Set-Location .\FPGA_BX71_AD9708_AD9280
.\scripts\Build-QspiBoot.cmd
```

JTAG 临时下载使用
`FPGA_BX71_AD9708_AD9280/output/BX71_G_ANALYZER_PS7.bit`。该生产镜像只保留
G 题 ADC/SPI/固定 CIC3/R16 链路，未使用的 AD9708 固定在中间码。下载后先验证
协议 v1.4、响应 profile `0x00030010`、采集和测量；通用 ProMax/AM/FM 工程不是
G 题交付入口。

### 5.3 QSPI 掉电启动

QSPI 只烧录当前构建生成的
`FPGA_BX71_AD9708_AD9280/output/BOOT_G_ANALYZER.bin`，不要使用历史
`BOOT_PS7.bin`。QSPI 探测或烧录前必须断电，将 BX71 启动拨码切到 JTAG，只连接
目标板的 JTAG 后再上电；不能带电切换拨码。先运行 `scripts/Probe-Qspi.cmd`
做只读链路/启动模式检查，再运行 `scripts/Program-Qspi.cmd` 烧录现有镜像；
重新构建并烧录使用 `scripts/Build-And-Program-Qspi.cmd`。烧录和 Verify 完成
前不得断电或拔 JTAG；成功后断电、切回 QSPI、重新上电做冷启动验收。单纯用
Vivado Hardware Manager 临时下载 `.bit` 不写 Flash，但仓库没有自动 JTAG
下载脚本，不能把临时配置当作掉电固化。

## 6. 现场校准（决定能否达到 5 mV）

### 6.1 当前电压换算状态

当前 `signal_adc_ad9280.c` 按 signed Q8.8 传输 LSB 的名义值
`10 V / 65536` 将样本换算为 AD9280 模块输入电压，采集前增益按 `1.0` 处理。
这不是两点实测校准。正式验收前应在保持相同端接方式的条件下，用可信示波器取得
S2 端真实电压并建立 ADC 后端的电压传递标定；标定属于后端，不应把 AD9280 的
常数写进 `periodic_analyzer`。

### 6.2 数字响应与经验校准

- CIC3/R16 的确定性幅相变化已由解析式逐频补偿，不能再用一张近似表重复补偿。
- 当前经验频响表明确为 `cal_count = 0`，启动日志打印
  `empirical_frequency_calibration=OFF`。
- 按 10、50、100、200、300、400、500 kHz 逐点记录 S2 实测峰值、程序峰值、
  Vpp、RMS 和相位。先确认端接、削顶、采样时钟及解析 CIC 补偿无误，再判断剩余
  误差是否稳定、可重复。
- 只有整机硬件固定且误差重复性足够时，才建立独立的经验校准数据；更换 ADC
  后端、端接、线缆或模拟前端后必须重新验证。当前版本不要启用临时校准表。

### 6.3 抗干扰验收

分别测试：

- 被测信号 50、100、250 mVpp；频率 10、200、500 kHz。
- 单基波、基波+二次、基波+三次、基波+二次+三次。
- 再叠加约 1 MHz、200 mVpp 干扰，并覆盖配置搜索范围内的轻微频偏。

每组检查：

- `clip=0`；若为 1，先查输入/ADC 削顶，不要用软件补偿掩盖。
- 基波误差不超过 1 kHz。
- Vpp、真 RMS、各正弦峰值与示波器/信号源真值的绝对误差均不超过 5 mV。
- 单帧 `latency` 明显小于 2000 ms。

## 7. 当前验证边界

已经完成的数字验证：

- CIC3/R16 行为仿真覆盖 10～500 kHz 七个频点，解析幅值/相位补偿通过。
- AD9280 采集核心回归通过，包括 v1.4、固定 R16、响应 profile、触发、统计、
  OTR、中止后重新采集和读回。
- 周期分析器主机矩阵覆盖频带边缘、强谐波、多谱线、CIC 解析补偿、经验校准组合、
  无干扰开关不变性、精确奈奎斯特及其附近/混叠后的 1 MHz 干扰、低于门限干扰，
  均通过。
- STM32H743 Keil 全量构建为 0 error、0 warning。

仍需依赖实物完成：AD9280 模块输入转换曲线、端接/线缆影响、10～500 kHz 扫频、
削顶/非线性、当前无放大路径下的噪声，以及真实 1 MHz 干扰叠加验收。1 MHz 联合
nuisance 算法已完成数字回归，但这不等于模拟链路已经实测通过。

## 8. 芯片官方资料

- [Analog Devices AD9280 数据手册](https://www.analog.com/media/en/technical-documentation/data-sheets/AD9280.pdf)
