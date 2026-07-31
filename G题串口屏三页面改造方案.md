# G题串口屏三页面最小改造方案

适用文件：`G题串口屏交互7寸.HMI`

页面名称必须严格使用小写的 `index`、`main`、`set`。STM32 只跟踪
页面状态，不接管页面跳转；所有跳转按钮由串口屏本地执行，因此即使
MCU正在打印串口诊断，进入和返回页面也不会被阻塞。

## 一、上电初始化

保留全局程序中的：

```text
baud=115200
dim=100
recmod=0
```

把原来的：

```text
page 0
```

改成：

```text
page index
```

## 二、index 页面

新建页面，名称为 `index`。

建议控件：

| 名称 | 类型 | x | y | w | h | 显示文字 | txt_maxl |
|---|---|---:|---:|---:|---:|---|---:|
| `t_index_title` | 文本 | 120 | 90 | 560 | 50 | 周期信号分析装置 | 32 |
| `t_index_sub` | 文本 | 180 | 160 | 440 | 36 | G题测量系统 | 32 |
| `b_enter` | 按钮 | 270 | 250 | 260 | 72 | 进入测量 | 32 |

`index` 页面的后初始化完成事件：

```text
bkcmd=0
prints "PI",0
printh FF FF FF
```

`b_enter` 的弹起事件：

```text
page main
```

## 三、main 页面

波形、坐标和数据显示区均保持当前位置，不移动。只改底部按钮行。

### 1. 波形控件网格

`s0` 保持：

```text
id=1
x=84
y=80
w=660
h=220
ch=1
```

把网格间隔改为：

```text
gdw=165
gdh=55
```

这样宽度和高度均为四等分：

- 横向中央网格：330像素，对应 `txmid`。
- 纵向中央网格：110像素，对应 `tymid=0`。
- STM32生成的波形0值也映射到约110。

### 2. 底部五按钮布局

按钮统一使用 `y=408、h=56`：

| 名称 | x | w | 文字 | 事件 |
|---|---:|---:|---|---|
| `b_back` | 84 | 72 | 返回 | `page index` |
| `b0` | 164 | 130 | 1周期 | 保留原G1报文 |
| `b1` | 302 | 130 | 3周期 | 保留原G3报文 |
| `b2` | 440 | 224 | 频谱 | 保留原GS报文 |
| `b_set` | 672 | 72 | 设置 | `page set` |

`b0` 弹起事件保持：

```text
prints "G1",0
printh FF FF FF
```

`b1` 弹起事件保持：

```text
prints "G3",0
printh FF FF FF
```

`b2` 弹起事件保持：

```text
prints "GS",0
printh FF FF FF
```

`b_back` 弹起事件：

```text
page index
```

`b_set` 弹起事件：

```text
page set
```

`main` 页面的后初始化完成事件改为：

```text
bkcmd=0
cle 1,0
prints "PM",0
printh FF FF FF
```

不要把页面通知写在前初始化事件中。后初始化通知可以保证MCU收到
`PM`时，`tview`、`s0`、坐标文本和测量文本已经可以安全访问。

## 四、set 页面

新建页面，名称为 `set`。现在只保留导航和设置预留区，不创建任何
设置报文，避免以后确定设置项目时受当前协议限制。

建议控件：

| 名称 | 类型 | x | y | w | h | 显示文字 | txt_maxl |
|---|---|---:|---:|---:|---:|---|---:|
| `t_set_title` | 文本 | 84 | 30 | 632 | 48 | 设置 | 16 |
| `t_set_hint` | 文本 | 140 | 160 | 520 | 50 | 设置功能待定 | 32 |
| `b_return` | 按钮 | 84 | 408 | 120 | 56 | 返回 | 16 |

`set` 页面的后初始化完成事件：

```text
bkcmd=0
prints "PS",0
printh FF FF FF
```

`b_return` 的弹起事件：

```text
page main
```

## 五、页面协议

| HMI发给STM32 | 含义 |
|---|---|
| `PI FF FF FF` | index页面已经加载 |
| `PM FF FF FF` | main页面已经加载 |
| `PS FF FF FF` | set页面已经加载 |
| `G1 FF FF FF` | 测量并显示1周期 |
| `G3 FF FF FF` | 测量并显示3周期 |
| `GS FF FF FF` | 测量并显示频谱 |

进入 `main` 时，STM32会自动恢复最近一次有效测量结果；没有结果时
显示 `WAIT` 并清空曲线。进入 `index` 或 `set` 后，STM32不会访问
`main` 页的私有控件。设置项协议暂不定义。

## 六、必须保持不变的项目

- USART2：115200、8N1。
- 波形控件名称 `s0`、ID 1、通道 0。
- `tview`、`tupp`、`trms`、`tc1`、`tc2`、`tc3`。
- `tyunit`、`tymax`、`tymid`、`tymin`、`tx0`、`txmid`、`txmax`。
- `b0`、`b1`、`b2` 的 G1/G3/GS 报文。
- 不要重新增加 `tf1`。
