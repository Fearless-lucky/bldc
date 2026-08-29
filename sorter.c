#include "sorter.h"
#include "svpwm.h"
#include "trajectory.h"
#include "uart_cmd.h"
#include "stdio.h"

extern Trajectory_t traj[2];
extern uint32_t sys_tick;

/* 视觉标签 -> 挡板目标角度(度)查找表, 标签1~8对应8个分拣流向 */
static const uint16_t gate_label_angle[GATE_LABEL_NUM] = {
    0, 45, 90, 135, 180, 225, 270, 315
};

/* 调参验证测试序列状态 */
typedef struct {
    uint8_t  active;       /* 测试进行中 */
    uint8_t  runs_total;   /* 重复次数 */
    uint8_t  run_idx;      /* 当前重复序号(从1起) */
    uint8_t  phase;        /* 0:发令 1:正向运动 2:到位稳定 3:返回 4:回位稳定 */
    uint8_t  log_en;       /* 遥测流开关 */
    int32_t  base_pos;     /* 测试基准位置 */
    uint32_t test_t0;      /* 测试起始时刻(遥测时间基准) */
    uint32_t phase_t0;     /* 当前阶段起始时刻 */
    uint32_t last_telem;   /* 上次遥测时刻 */
} SorterTest_t;

static SorterTest_t stest[2];

int32_t gate_label_to_counts(uint8_t label)
{
    float angle = (float)gate_label_angle[label - 1];
    return (int32_t)(angle * GATE_ENCODER_CPR / 360.0f);
}

uint8_t gate_move_to_label(uint8_t axis, uint8_t label)
{
    if (axis > 1 || label < 1 || label > GATE_LABEL_NUM) return 0;

    int32_t target = gate_label_to_counts(label);
    Motor_SetTarget(&motor[axis], target);
    /* 执行梯形速度轨迹, 位置与状态反馈由主循环闭环跟踪 */
    traj_plan(&traj[axis], (float)target, (float)motor[axis].pos,
              GATE_MOVE_SPEED, GATE_MOVE_ACCEL);
    return 1;
}

static void test_issue_cmd(uint8_t axis, int32_t pos)
{
    Motor_SetTarget(&motor[axis], pos);
    traj_plan(&traj[axis], (float)pos, (float)motor[axis].pos,
              GATE_MOVE_SPEED, GATE_MOVE_ACCEL);
}

void sorter_test_start(uint8_t axis, uint8_t runs)
{
    if (axis > 1 || runs == 0) return;

    stest[axis].runs_total = runs;
    stest[axis].run_idx = 0;
    stest[axis].base_pos = motor[axis].pos;
    /* 编码器计数在60000处回绕: 基准靠近回绕点时移到中段, 保证测试目标单调可达 */
    if (stest[axis].base_pos + TEST_DELTA_POS >= 59000) {
        stest[axis].base_pos = 30000;
    }
    stest[axis].test_t0 = sys_tick;
    stest[axis].phase = 0;
    stest[axis].phase_t0 = sys_tick;
    stest[axis].last_telem = sys_tick;
    stest[axis].log_en = 1;
    stest[axis].active = 1;

    char buf[32];
    sprintf(buf, "!TEST%d,%d\r\n", axis, runs);
    UART_SendString(buf);
}

void sorter_log_ctrl(uint8_t axis, uint8_t on)
{
    if (axis > 1) return;
    stest[axis].log_en = on ? 1 : 0;
}

static void test_finish(uint8_t axis)
{
    stest[axis].active = 0;
    stest[axis].log_en = 0;
    char buf[32];
    sprintf(buf, "!TDONE%d\r\n", axis);
    UART_SendString(buf);
}

/* 遥测一行: D,<axis>,<run>,<t_ms>,<target>,<pos>,<rpm>,<speed_ref>
 * 记录增益调整前/后的整段曲线即可叠加对比 */
static void telemetry_line(uint8_t axis)
{
    /* 手工拼接替代sprintf: 无FPU环境下约快5倍, 降低遥测对控制环的CPU挤占 */
    char buf[80];
    char *p = buf;
    p = fmt_chr(p, 'D');   p = fmt_chr(p, ',');
    p = fmt_int(p, axis);  p = fmt_chr(p, ',');
    p = fmt_int(p, stest[axis].active ? stest[axis].run_idx : 0); p = fmt_chr(p, ',');
    p = fmt_uint(p, sys_tick - stest[axis].test_t0); p = fmt_chr(p, ',');
    p = fmt_int(p, motor[axis].target_pos); p = fmt_chr(p, ',');
    p = fmt_int(p, motor[axis].pos); p = fmt_chr(p, ',');
    p = fmt_int(p, motor[axis].speed_rpm); p = fmt_chr(p, ',');
    p = fmt_int(p, motor[axis].speed_ref);
    p = fmt_chr(p, '\r'); p = fmt_chr(p, '\n');
    *p = 0;
    UART_SendString(buf);
}

static void test_poll(uint8_t axis)
{
    SorterTest_t *t = &stest[axis];

    /* 电机被停止/故障时中止测试, 避免测试命令覆盖停机命令(发令阶段除外, 其本身会恢复运行态) */
    if (t->phase != 0 && motor[axis].state != MOTOR_RUN) {
        test_finish(axis);
        return;
    }

    switch (t->phase) {
    case 0: /* 发出本回合的相同位置命令 */
        t->run_idx++;
        t->phase_t0 = sys_tick;
        {
            char buf[32];
            sprintf(buf, "!RUN%d,%d\r\n", axis, t->run_idx);
            UART_SendString(buf);
        }
        test_issue_cmd(axis, t->base_pos + TEST_DELTA_POS);
        t->phase = 1;
        break;

    case 1: /* 正向梯形轨迹执行中 */
        if (traj_done(&traj[axis])) {
            t->phase_t0 = sys_tick;
            t->phase = 2;
        }
        break;

    case 2: /* 到位稳定, 剥离瞬态后准备返回 */
        if (sys_tick - t->phase_t0 >= TEST_SETTLE_MS) {
            test_issue_cmd(axis, t->base_pos);
            t->phase = 3;
        }
        break;

    case 3: /* 返回基准位置 */
        if (traj_done(&traj[axis])) {
            t->phase_t0 = sys_tick;
            t->phase = 4;
        }
        break;

    case 4: /* 回位稳定, 判断是否继续下一回合 */
        if (sys_tick - t->phase_t0 >= TEST_SETTLE_MS) {
            if (t->run_idx >= t->runs_total) {
                test_finish(axis);
                return;
            }
            t->phase = 0;
        }
        break;

    default:
        t->phase = 0;
        break;
    }
}

void sorter_poll(void)
{
    uint8_t axis;
    for (axis = 0; axis < 2; axis++) {
        if (stest[axis].active) {
            test_poll(axis);
        }
        if ((stest[axis].active || stest[axis].log_en) &&
            (sys_tick - stest[axis].last_telem >= TEST_TELEM_PERIOD)) {
            stest[axis].last_telem = sys_tick;
            telemetry_line(axis);
        }
    }
}
