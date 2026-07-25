# MSP-LITO-G3507 图形化配置说明

## 1. 工程结构

本工程已经由手写 `ti_msp_dl_config.c/.h` 迁移到 TI SysConfig：

- 图形配置源文件：`SysConfig/MSP_LITO_G3507_Board.syscfg`
- 自动生成源码：`SysConfig/Generated/ti_msp_dl_config.c`
- 自动生成头文件：`SysConfig/Generated/ti_msp_dl_config.h`
- 手动生成脚本：`generate_sysconfig.ps1`
- 打开图形界面：`open_sysconfig.ps1`
- Keil 工程：`Keil/TI_MSPM0G3507.uvprojx`

图形配置文件是引脚、时钟和外设参数的唯一配置源。不要手工修改
`SysConfig/Generated` 中的文件，也不要再在 `Src` 或 `Inc` 中建立另一套
`ti_msp_dl_config.c/.h`。

## 2. 本机官方工具

当前脚本默认使用：

- TI SysConfig：`D:\ti\sysconfig_1.26.2`
- MSPM0 SDK：`D:\ti\mspm0_sdk_2_10_00_04`
- Keil uVision：`C:\Keil_v5\UV4\UV4.exe`

这些路径只指向官方工具，不指向 STM32 工程、FPGA 工程或其他 TI 工程。
如以后更换工具位置，可设置下列环境变量，或者给脚本传同名参数：

```powershell
$env:NUEDC_TI_SYSCONFIG_ROOT = "D:\ti\sysconfig_1.26.2"
$env:NUEDC_TI_MSPM0_SDK_ROOT = "D:\ti\mspm0_sdk_2_10_00_04"
```

## 3. 打开图形配置

在 `TI_MSPM0G3507` 目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\open_sysconfig.ps1
```

打开后可在左侧模块中查看：

- `UART_0`
- `FPGA_SPI`
- `ADC_VOLTAGE`
- `ADC_CURRENT`
- `ADC_SAMPLE_TIMER`
- `GPIO_FPGA_CONTROL`
- `GPIO_BOARD`
- `GPIO_DAC_SHARED`
- `GPIO_AD9910`

在右侧 PinMux/器件封装视图中可直接查看每个功能对应的物理引脚。

如果希望从 Keil 的 Tools 菜单打开，可在 uVision 的
`Tools -> Customize Tools Menu` 中导入
`Keil/SysConfig_DriveD_Menu.cfg`。推荐优先使用 `open_sysconfig.ps1`，
因为它会把图形界面的生成目录固定为工程内的 `SysConfig/Generated`。

## 4. 保存、生成和编译

图形界面保存 `.syscfg` 后，在工程根目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\generate_sysconfig.ps1
powershell -ExecutionPolicy Bypass -File .\build_keil.ps1
```

也可以直接在 Keil 中 Build/Rebuild。Keil 的 Before Build 已接入
`generate_sysconfig.ps1`，会先严格校验并重新生成配置，再编译固件。
SysConfig 出现错误或警告时，构建会停止，避免使用过期生成文件。

## 5. 已保留的实测硬件配置

| 功能 | 配置 |
|---|---|
| 系统时钟 | 内部 SYSOSC 32 MHz，不启用 HFXT/PLL |
| UART0 | PA10 TX、PA11 RX，115200-8-N-1，FIFO 1/2 阈值 |
| FPGA SPI0 | PA12 SCLK、PA14 PICO/MOSI、PA13 POCI/MISO；PA2保留为未外接CS0 |
| FPGA SPI 参数 | Controller，Motorola 4-wire，Mode 0，8 位，MSB，1 MHz |
| FPGA 软件片选 | PA15，低有效，上电先置高再使能输出 |
| 片上电压 ADC | ADC0_CH4，PB25/J2-15，12 位无符号，VDDA 3.3 V |
| 片上电流 ADC | ADC1_CH4，PB17/J2-28，12 位无符号，VDDA 3.3 V |
| ADC 触发 | TIMG12 零事件同时触发两路ADC，事件通道12 |
| ADC DMA | 电压DMA_CH0、电流DMA_CH1，各512点，半满与全满中断 |
| 默认采样率 | 1 MSPS；驱动可把TIMG12改为1 kSPS～1 MSPS |
| 调试保留 | PA19/PA20 保留 SWD，PA18 不占用 |

SPI0的帧格式直接由SysConfig生成。图形配置选择Motorola 4-wire，并把
SPI0_CS0分配到未外接的PA2；实际BX71片选仍由PA15 GPIO在完整6字节帧前后
控制。这样生成代码天然包含`MOTO4_POL0_PHA0`，不需要在初始化后修改SPI
寄存器。PA2不需要接到BX71。

其余图形 GPIO 已按真实器件功能命名，包括 AD9910、AD5687、DAC8831
和板载状态脚。AD9910及DAC的低有效片选均配置为空闲高电平。原
ADS8363软件SPI使用的PB17已改作片上电流ADC，因此SysConfig不再创建
`GPIO_ADS8363`，两套采样路径不会复用同一引脚。

## 6. 生成后检查

每次修改引脚或外设后至少检查：

1. SysConfig 校验为 0 error、0 warning。
2. Keil 全量构建为 `0 Error(s), 0 Warning(s)`。
3. 生成头文件中仍有 `ADC_VOLTAGE_INST=ADC0`、
   `ADC_CURRENT_INST=ADC1`、`DMA_ADC_VOLTAGE_CHAN_ID=0` 和
   `DMA_ADC_CURRENT_CHAN_ID=1`。
4. 生成源码中的SPI帧格式仍为`DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0`。
5. 实板分别给PB25、PB17输入0～3.3 V信号，确认两路原始码范围均为
   0～4095，并检查连续采集无DMA错误或缓冲超限。
6. 在独立应用层接入FPGA驱动后，再读取一次设备标识，确认通信正常。
