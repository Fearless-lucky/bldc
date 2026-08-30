#ifndef __SORTER_H_
#define __SORTER_H_

#include "stm32f10x.h"

/* ---------- 旋转挡板(分拣门)参数 ---------- */
#define GATE_ENCODER_CPR     2000    /* 编码器每转计数(500线 x4倍频) */
#define GATE_LABEL_NUM       8       /* 视觉标签数量 */
#define GATE_MOVE_ACCEL      3000.0f /* 挡板动作加速度上限 counts/s^2 */

/* 自适应轨迹: 按转动角度自动选取速度(短程三角波慢, 长程梯形波快) */
#define GATE_V_MIN           200.0f  /* 短程(45°)对应速度 counts/s */
#define GATE_V_MAX           800.0f  /* 长程(≥270°)对应速度 counts/s */

/* ---------- 到位判定 ---------- */
#define GATE_INPOS_WINDOW    8       /* 到位窗口 ±counts */
#define GATE_INPOS_HOLD_MS   50      /* 窗口内持续时长, 排除振荡 */

/* ---------- 命令队列 ---------- */
#define GATE_QUEUE_DEPTH     4       /* 挂起的目标队列深度 */

/* ---------- 调参验证测试参数 ---------- */
#define TEST_DELTA_POS       2000    /* 每次重复的相同位置阶跃幅度 counts */
#define TEST_SETTLE_MS       500     /* 轨迹到位后的稳定等待时间 ms */
#define TEST_TELEM_PERIOD    10      /* 遥测采样周期 ms */

/* 挡板状态查询(供双轴联动) */
uint8_t gate_busy(void);             /* 挡板运动中(轨迹执行或未到位) */
uint8_t gate_fault(void);            /* 挡板故障态 */

/* 视觉标签 -> 挡板目标角度(编码器计数) */
int32_t gate_label_to_counts(uint8_t label);

/* 视觉标签 -> 自适应梯形轨迹运动, 成功返回1; 队列满返回0 */
uint8_t gate_move_to_label(uint8_t axis, uint8_t label);

/* 直接位置命令入队(成功返回1) */
uint8_t gate_queue_move(uint8_t axis, int32_t pos, float speed);

/* 查询挂起队列数 */
uint8_t gate_queue_len(void);

/* 主循环周期调用(约1ms): 到位判定/队列推进/测试序列/遥测 */
void sorter_poll(void);

/* 调参验证: 固定负载下重复相同位置命令 n 次 */
void sorter_test_start(uint8_t axis, uint8_t runs);

/* 手动开/关响应曲线遥测流 */
void sorter_log_ctrl(uint8_t axis, uint8_t on);

/* 到位状态查询(1=已到位且无挂起命令) */
uint8_t gate_in_position(void);

#endif
