# STM32F103 BLDC 速度闭环 (PID)

基于正点原子 STM32F1 精英开发板 + ATK-PD6010B 电机驱动板的 BLDC (无刷直流) 电机速度闭环项目。

6 步梯形换相 + 霍尔传感器 + TIM1 高级定时器 PWM + 编码器测速 + PID 调速。

## 硬件

- **MCU**: STM32F103ZE (Cortex-M3, 512KB Flash, 64KB SRAM)
- **驱动板**: ATK-PD6010B
- **电机**: 霍尔传感器 BLDC, 编码器 500 P/R

## 引脚分配

| 引脚 | 功能 |
|------|------|
| PA6/PA7/PB0 | 霍尔 U/V/W (TIM3 CH1/2/3) |
| PA8/PA9/PA10 | U/V/W 高侧 PWM (TIM1 CH1/2/3) |
| PB13/PB14/PB15 | U/V/W 低侧 (TIM1 CH1N/2N/3N) |
| PB11 | CTRL_SD (驱动使能) |
| PB12 | TIM1_BKIN 刹车 |
| PB6/PB7 | 编码器 A/B (TIM4 CH1/2) |
| PE3/PE4 | KEY1/KEY0 (调目标转速) |

## 定时器分配

| 定时器 | 用途 | 频率/周期 |
|--------|------|----------|
| TIM1 | 3 相互补 PWM | 5 kHz (arr=7200, psc=1) |
| TIM3 | 霍尔捕获 + 换相触发 | 霍尔接口, Slave Reset 模式 |
| TIM4 | 编码器计数 | TI12 4 倍频, 2000 计数/圈 |
| TIM6 | 测速节拍 | 100 ms 周期 ISR |

## 构建

- IDE: **Keil MDK uVision 5**, 工程文件 `TASK1.uvprojx`
- 编译器: ARMCLANG V6.24
- Device Pack: Keil.STM32F1xx_DFP.2.4.1

> 注意: 仓库只包含源码, 固件库 (CMSIS + StdPeriph_Driver) 和 Keil 工程文件未上传. 上传源码后需要在本地 Keil 中补齐 STM32 固件库并配置好 include 路径.

## 操作

- **KEY0 (PE4)**: 目标转速 +100 RPM
- **KEY1 (PE3)**: 目标转速 -100 RPM
- 默认目标: 1000 RPM, 上限 3000 RPM, 下限 0 RPM

## PID 参数

控制周期 100 ms (由 TIM6 中断驱动):

```c
#define TS  0.1f
#define KP  1.5f
#define KI  0.8f
#define KD  0.15f
```

- **KP**: 比例, 响应快慢
- **KI**: 积分, 消静差
- **KD**: 微分, 防冲过头

占空比限幅: `[0, MAX_PWM_DUTY/2]` ≈ `[0, 4800]`

积分限幅: ±4800

## 源码结构

```
bldc_main.c       # main() + PID 速度环
bldc.c / bldc.h   # GPIO 初始化, 霍尔读取, 6 个换相函数
bldc_tim.c/.h     # TIM1/3/4/6 初始化 + ISR
delay.c / delay.h # SysTick 阻塞延时
key.c / key.h     # 按键扫描 (10 ms 消抖)
sys.h             # 位带操作宏
stm32f10x_conf.h  # StdPeriph 模块包含配置
```

## 许可

仅供学习参考.
