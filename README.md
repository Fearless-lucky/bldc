# BLDC 双电机分拣线控制固件 (STM32F103ZE)

传送带电机（Hall 六步换相 + 编码器 PI 速度闭环）与旋转挡板电机（编码器 FOC/SVPWM
位置闭环）的统一控制固件，单 Keil 工程驱动两台电机，共享 USART2 命令接口。

## 硬件

- **MCU**: STM32F103ZE (Cortex-M3, 512KB Flash, 64KB SRAM)
- **电机1 传送带**: 霍尔传感器 BLDC + 增量编码器 500 P/R
- **电机2 旋转挡板**: 三相 FOC 电机 + 增量编码器 500 P/R

## 引脚分配

### 传送带电机（六步换相）

| 引脚 | 功能 |
|------|------|
| PA6/PA7/PB0 | 霍尔 U/V/W (TIM3 CH1/2/3) |
| PA8/PA9/PA10 | U/V/W 高侧 PWM (TIM1 CH1/2/3) |
| PB13/PB14/PB15 | U/V/W 低侧 (TIM1 CH1N/2N/3N) |
| PB11 | 驱动使能 CTRL_SD |
| PB12 | TIM1_BKIN 刹车输入 |
| PB6/PB7 | 编码器 A/B (TIM4 CH1/2) |

### 旋转挡板电机（FOC/SVPWM）

| 引脚 | 功能 |
|------|------|
| PC6/PC7/PC8 | 三相 PWM (TIM8 CH1/2/3, 单端输出) |
| PA0/PA1 | 编码器 A/B (TIM2 CH1/2) |
| PC0/PC1 | 相电流采样 A/B (ADC2 IN10/IN11) |
| PB4 | 驱动使能 SD (需关闭 JTAG 保留 SWD) |
| PB5 | 故障输入 |

### 公共

| 引脚 | 功能 |
|------|------|
| PA2/PA3 | USART2 TX/RX (115200-8-N-1) |
| PE3/PE4 | KEY1/KEY0 (急停) |

## 定时器分配

| 定时器 | 用途 | 频率/周期 |
|--------|------|----------|
| TIM1 | 传送带三相互补 PWM | 5 kHz |
| TIM3 | 霍尔捕获 + COM 换相触发 | 霍尔接口模式 |
| TIM4 | 传送带编码器 (清零式测速) | TI12 4 倍频 |
| TIM8 | 挡板 SVPWM (中心对齐) | 10 kHz |
| TIM2 | 挡板编码器 (模 60000 差分测速) | TI12 4 倍频 |
| TIM6 | 系统节拍: 挡板速度 10ms + 传送带 PI 100ms | 10 ms |

## 命令接口 (USART2, 行结束 `\r`)

### 传送带电机

```
SPD <rpm>          设置基准带速(堆积度=0时)
ACC <0-100>        下游堆积度 -> 自动折算速度目标(≥90 停带)
PID <kp> <ki> <kd> 在线修改速度环增益
TEST <n>           固定负载下重复相同速度命令 n 次, 输出响应曲线
LOG <0|1>          开/关遥测流 (C,<run>,<t>,<tgt>,<rpm>,<duty>)
RUN / STOP         启停
?                  查询状态
SAVE               保存标定值与增益到 Flash
```

### 旋转挡板电机

```
#1 <pos> <speed>   目标位置 + 轨迹速度(梯形轨迹)
#1 V <label>       视觉标签 1~8 -> 挡板角度 0~315°
#1 PID <loop> <kp> <ki> [kd]   在线修改增益(loop: 0位置 1速度 2/3电流)
#1 TEST <n>        重复相同位置命令 n 次, 输出响应曲线
#1 LOG <0|1>       开/关遥测流 (D,<axis>,<run>,<t>,<tgt>,<pos>,<rpm>,<sp_ref>)
#1 CAL             转子对齐 + 电角度零点标定
#1 HOME / STOP / ? 对齐 / 停止(含故障恢复) / 查询
```

## 标定与保护

- **电流零点校准**: 上电输出关断时采样 ADC 偏置（16 次均值）
- **电角度零标定**: 对齐矢量拉转子到 d 轴后自动捕获编码器零点
- **电流标定**: `svpwm.h` 中 `CURR_SENSE_R` / `CURR_AMP_GAIN` 按实际电路修改
- **Hall 丢失保护**: 运行中 1s 无霍尔边沿即切断输出
- **挡板堵转保护**: 目标远离且 2s 不动 -> 故障切断, `#1 STOP` 恢复
- **看门狗**: IWDG 2s 超时, 主循环喂狗
- **Flash 参数**: `SAVE` 保存标定值与增益, 上电自动加载

## 调参验证工作流

1. 固定负载下 `TEST 5`（或 `#1 TEST 5`），串口记录响应曲线
2. `PID` 命令在线修改增益
3. 重跑相同 `TEST`，两段曲线按 `!RUN` 标记对齐叠加对比
   （命令、负载、初始条件完全一致，隔离控制器效果与工况差异）
4. 调好后 `SAVE` 掉电保持

## 构建

- IDE: **Keil MDK uVision 5**, 工程文件 `TASK1.uvprojx`
- 编译器: ARMCLANG V6
- 仓库已包含 CMSIS + StdPeriph_Driver 固件库, 克隆后直接打开工程即可编译

## 源码结构

```
svpwm_main.c          # main(): 双电机初始化 + 主控制循环 + 堵转/看门狗
svpwm.c / svpwm.h     # 挡板 FOC: Clarke/Park, PID, 电机状态机, SVPWM
svpwm_tim.c / .h      # 定时器/ADC 统一配置(TIM1/2/3/4/6/8) + 电流采样
bldc.c / bldc.h       # 传送带六步换相(霍尔查表)
conveyor.c / .h       # 传送带应用层: 堆积度调速 + TEST 序列 + 遥测
sorter.c / sorter.h   # 挡板应用层: 视觉标签映射 + TEST 序列 + 遥测
uart_cmd.c / .h       # 双电机命令接口(中断收发, 环形缓冲)
trajectory.c / .h     # 梯形速度轨迹规划
delay / key / sys     # 延时 / 按键 / 系统工具(含 Flash 参数存储)
Device/ Core/         # STM32F10x 标准外设库 + CMSIS
```

## 许可

仅供学习参考.
