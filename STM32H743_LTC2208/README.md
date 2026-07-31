# STM32H743 + LTC2208：2026 G 题配套工程

本目录是从远端 `origin/main` 的最新 `STM32H743_AD9238` 独立复制并重命名得到的 LTC2208 版本。原 AD9238 工程不在本目录内修改。串口屏、自适应绘图、波形横向方向修复和通用周期分析算法均继承最新 AD9238 版本；本次只替换 ADC/FPGA 采集适配层，并把外部前端电压增益设为 1。

## 固件配套关系

本工程只能与同级 `FPGA_BX71_AD9708_LTC2208` 固件配套：

| 项目 | LTC2208 版本 |
|---|---|
| FPGA 设备 ID | `0x22080001` |
| FPGA 协议版本 | `0x00030002` |
| LTC2208 物理采样时钟 | `130 MHz` |
| FPGA 输出采样率 | `2 MHz` |
| FPGA 数字链路 | `CIC5/R65 + FIR81` |
| MCU 样本格式 | signed-20 Q3（int32_t），`fraction_bits=3` |
| 标称 ADC 满量程 | `2.25 Vpp` |
| ADC 物理每码电压 | `2.25/65536 V = 34.3323 uV` |
| Q3 传输网格 | `2.25/(65536*8) V = 4.29153 uV` |
| 外接 OPA690 | 电压跟随器，标称增益 `1.0` |

读取寄存器的 32 位负载由高 12 位 RAM 地址和低 20 位 signed Q3 样本组成。一个物理 LTC2208 码对应 8 个传输计数，MCU 用 int32_t 保留 CIC5/FIR81 的小数结果；MIN/MAX/SUM/LATEST 统计寄存器仍明确使用原始 signed-S16 物理码。Q3 只避免数字抽取结果再次量化，并不等价于 ADC 增加 3 位 ENOB。

## 重要的幅值边界

代码当前按 LTC2208 芯片端 `2.25 Vpp` 满量程和 OPA690 跟随器增益 1 换算。ClearDream 模块在 SMA 与 LTC2208 之间还有 ADA4932、串联电阻、低通网络和差分终端；现有模块资料没有给出足以证明 SMA 到 ADC 的精确闭环增益及输入阻抗的全部阻容值。因此：

- `2.25/65536 V` 只是 ADC 芯片端的名义刻度，不能直接证明 BNC 输入端也是这个刻度。
- `G_CONTEST_FRONTEND_VOLTAGE_GAIN=1.0f` 表示外接 OPA690 按跟随器使用，不代表板载 ADA4932 链路已经实测为精确 1 倍。
- AD9238 的经验校准点已禁用，不能移植到 LTC2208。
- 首次上板必须在 BNC 参考面完成增益、零点和 10~500 kHz 频响实测，再决定是否填写 LTC2208 专用校准表。

## 主要修改点

- `Modules/ltc2208.c/.h`：设备/协议核验、16 位触发、signed-20 Q3 样本解码、raw-S16 统计和 FPGA 滤波控制。
- `Modules/ltc2208_app.c/.h`：阻塞采集封装和名义两点电压换算。
- `BSP/signal_adc_ltc2208.c`：130 MHz、R65、2 MSPS，捕获样本按 `2.25/(65536*8)`、统计值按 `2.25/65536` 换算。
- `Core/Inc/g_contest_config.h`：外部前端增益为 `1.0f`，LTC2208 经验校准点默认为 0。
- 初始化诊断会沿用驱动刚读到的 READY 快照，因此成功初始化后不会再误报 `ready=0`。

## STM32 与 BX71 SPI 接线

沿用 AD9238 版本的 P2 连接：

| STM32H743 | BX71 P2 FPGA | 作用 |
|---|---|---|
| SPI2_SCK | K19 / GPIO0_26 | SCK |
| SPI2_MOSI | L19 / GPIO0_32 | MOSI |
| SPI2_MISO | K17 / GPIO0_28 | MISO |
| CS GPIO | K18 / GPIO0_29 | CS_N |
| IRQ GPIO | L16 / GPIO0_30 | FPGA ready/IRQ |
| GND | GND | 共地 |

LTC2208 模块与 BX71 的 16 位并行数据、采样时钟和电源转接由 FPGA 工程约束管理。STM32 不直接连接 ADC 数据引脚。

## 构建与产物

在本目录执行：

```powershell
.\build_keil.ps1
```

脚本重建 `MDK-ARM/STM32H743_LTC2208.uvprojx`，只有 Keil 最终汇总为 `0 Error(s), 0 Warning(s)` 才算成功。成功后 `output` 内只保留本版本的新产物：

- `STM32H743_LTC2208.axf`
- `STM32H743_LTC2208.hex`
- `STM32H743_LTC2208.map`
- `build.log`
- `SHA256SUMS.txt`

## 代码侧精度与主机回归

```powershell
python .\tests\analyze_ltc2208_precision_budget.py
.\tests\run_g_tjc_plot_scale_host_test.ps1
.\tests\run_periodic_analyzer_host_test.ps1
```

精度预算会读取当前 LTC2208 和仓库最新版 AD9238 的真实宏定义，门禁 signed-20 Q3、协议 `0x00030002`、OPA690 增益 1、LTC2208 `4.2915 uV` 传输网格，以及 AD9238 Q8.8 输入折算约 `15.978 uV`。它只证明代码保留了 FPGA 滤波的小数精度，不代表新增 ADC 位数或模拟前端已经校准。

串口屏和通用周期分析测试用于确认继承的横向显示修复、自适应绘图和 DSP 没有退化；AD9238 理论补偿函数虽然仍保留在通用分析器中，但 LTC2208 配置不启用它们。

## 上板验收顺序

1. 先烧录匹配的 LTC2208 FPGA 镜像，再烧录本 STM32 固件。
2. 串口确认 ID=`0x22080001`、协议=`0x00030002`、字宽=20、小数位=3、物理时钟=130 MHz、`ready=1`。
3. 用零输入检查中心偏置和噪声；用已知小幅正弦检查极性、码宽及削顶状态。
4. 在信号源 50 欧设置和实际比赛接线下，按 10、50、100、200、300、400、450、484.5、500 kHz 扫频，记录 BNC 设定值与装置峰值读数。
5. 用 5 mV 峰值弱分量、250 mVpp 整体波形及强弱分量组合验证，整机幅值误差以 `<=3 mV` 为内部放行目标，给题目 `<=5 mV` 留出余量。
6. 再叠加 `>=1 MHz`、`200 mVpp` 正弦干扰，确认 500 kHz 以下幅值、频率和完整周期波形仍满足题目指标。

Keil 构建、主机算法测试和 FPGA 仿真都不能替代真实 LTC2208 模块、OPA690、线缆、供电和信号源条件下的物理校准。尤其是高于 65 MHz 奈奎斯特频率后已经混叠进带内的干扰，无法仅靠 MCU/FPGA 数字滤波恢复。
