#ifndef __SORTER_H_
#define __SORTER_H_

#include "stm32f10x.h"

/* ---------- 旋转挡板(分拣门)参数 ---------- */
#define GATE_ENCODER_CPR     2000    /* 编码器每转计数(500线 x4倍频) */
#define GATE_LABEL_NUM       8       /* 视觉标签数量 */
#define GATE_MOVE_SPEED      600.0f  /* 挡板动作速度 counts/s */
#define GATE_MOVE_ACCEL      3000.0f /* 挡板动作加速度 counts/s^2 */

/* ---------- 调参验证测试参数 ---------- */
#define TEST_DELTA_POS       2000    /* 每次重复的相同位置阶跃幅度 counts */
#define TEST_SETTLE_MS       500     /* 轨迹到位后的稳定等待时间 ms */
#define TEST_TELEM_PERIOD    10      /* 遥测采样周期 ms */

/* 视觉标签 -> 挡板目标角度(编码器计数) */
int32_t gate_label_to_counts(uint8_t label);

/* 视觉标签 -> 梯形轨迹运动到对应挡板角度, 成功返回1 */
uint8_t gate_move_to_label(uint8_t axis, uint8_t label);

/* 调参验证: 固定负载下重复相同位置命令 n 次, 输出响应曲线遥测 */
void sorter_test_start(uint8_t axis, uint8_t runs);

/* 手动开/关响应曲线遥测流 */
void sorter_log_ctrl(uint8_t axis, uint8_t on);

/* 主循环周期调用(约1ms): 驱动测试序列与遥测输出 */
void sorter_poll(void);

#endif
