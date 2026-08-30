#ifndef __CONVEYOR_H_
#define __CONVEYOR_H_

#include "stm32f10x.h"

/* 下游堆积度达到该级别时暂停传送带, 防止堵料 */
#define CONV_ACC_STOP_LEVEL   90

/* 带速斜坡: 目标每100ms最多变化量(RPM), 防止堆积度突变冲击传动 */
#define CONV_SLEW_RPM_PER_TICK  30

/* 双轴联动: 挡板动作期间传送带限速(比例因子) */
#define CONV_GATE_SLOW_FACTOR   0.3f

/* 调参验证测试序列时序(单位: 100ms tick) */
#define TEST_SETTLE_TICKS     10    /* 起始/停止稳定时间 1s */
#define TEST_HOLD_TICKS       30    /* 速度阶跃保持时间 3s */

void conveyor_init(void);
void conveyor_set_base_speed(int16_t rpm);      /* 设置基准速度(堆积度=0时) */
void conveyor_set_accumulation(uint8_t level);  /* 设置下游堆积度 0~100 */
void conveyor_set_log(uint8_t on);              /* 开/关响应曲线遥测 */
void conveyor_test_start(uint8_t runs);         /* 固定负载下重复相同速度命令 */
void conveyor_poll(void);                       /* 主循环周期调用 */

/* 双轴联动: 主循环每周期传入挡板状态 */
void conveyor_link_update(uint8_t gate_busy, uint8_t gate_fault);
/* 联动急停(挡板故障时): 切断传送带 */
void conveyor_emergency_stop(void);

int16_t  conveyor_base_speed(void);
uint8_t  conveyor_accumulation(void);

#endif
