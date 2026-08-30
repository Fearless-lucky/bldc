#include "sorter.h"
#include "svpwm.h"
#include "trajectory.h"
#include "uart_cmd.h"
#include "stdio.h"

extern Trajectory_t traj[2];
extern uint32_t sys_tick;
extern uint8_t ff_reset_req;   /* 定义于svpwm_main.c: 轨迹重规划时复位前馈差分基线 */

/* 视觉标签 -> 挡板目标角度(度)查找表, 标签1~8对应8个分拣流向 */
static const uint16_t gate_label_angle[GATE_LABEL_NUM] = {
    0, 45, 90, 135, 180, 225, 270, 315
};

/*---------- 到位判定状态 ----------*/
typedef struct {
    uint8_t  in_pos;        /* 当前判定为到位 */
    uint16_t hold_ms;       /* 窗口内持续时间 */
    uint8_t  done_reported; /* 本次运动的 !DONE 已上报(避免重复) */
} InPos_t;

static InPos_t ginpos;

/*---------- 命令队列 ----------*/
typedef struct {
    int32_t pos;
    float   speed;
} GateCmd_t;

static GateCmd_t gqueue[GATE_QUEUE_DEPTH];
static uint8_t gq_head = 0, gq_tail = 0, gq_len = 0;

/*---------- 调参验证测试序列状态 ----------*/
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

/*====================================================================
 * 内部: 执行一段轨迹(直接, 不经队列)
 *====================================================================*/
static void gate_execute(int32_t pos, float speed)
{
    Motor_SetTarget(&motor[1], pos);
    traj_plan(&traj[1], (float)pos, (float)motor[1].pos, speed, GATE_MOVE_ACCEL);
    ginpos.done_reported = 0;   /* 新运动开始, 允许再次上报 */
    ff_reset_req = 1;           /* 复位前馈差分基线 */
}

/*====================================================================
 * 自适应轨迹: 按转动距离选取巡航速度
 * 短程(45°, 250计数)用慢速防过冲, 长程(≥270°, 1500计数)用快速
 * 中间线性插值
 *====================================================================*/
static float gate_adaptive_speed(int32_t from, int32_t to)
{
    int32_t dist = to - from;
    if (dist < 0) dist = -dist;
    if (dist > 1500) dist = 1500;
    if (dist < 0) dist = 0;
    return GATE_V_MIN + (GATE_V_MAX - GATE_V_MIN) * (float)dist / 1500.0f;
}

/*====================================================================
 * 命令队列: 运动中收到新目标先挂起, 当前运动到位后依次执行
 *====================================================================*/
uint8_t gate_queue_move(uint8_t axis, int32_t pos, float speed)
{
    (void)axis;
    if (gq_len >= GATE_QUEUE_DEPTH) return 0;

    gqueue[gq_tail].pos = pos;
    gqueue[gq_tail].speed = speed;
    gq_tail = (uint8_t)((gq_tail + 1) % GATE_QUEUE_DEPTH);
    gq_len++;
    return 1;
}

uint8_t gate_queue_len(void)
{
    return gq_len;
}

/* 空闲(已到位且无队列)时立即执行, 否则入队 */
static uint8_t gate_submit(int32_t pos, float speed)
{
    if (gate_in_position() && !traj[1].running) {
        gate_execute(pos, speed);
        return 1;
    }
    return gate_queue_move(1, pos, speed);
}

int32_t gate_label_to_counts(uint8_t label)
{
    float angle = (float)gate_label_angle[label - 1];
    return (int32_t)(angle * GATE_ENCODER_CPR / 360.0f);
}

uint8_t gate_move_to_label(uint8_t axis, uint8_t label)
{
    if (axis > 1 || label < 1 || label > GATE_LABEL_NUM) return 0;

    int32_t target = gate_label_to_counts(label);
    /* 自适应速度: 按当前位置到标签角度的距离选取 */
    float speed = gate_adaptive_speed(motor[1].pos, target);
    return gate_submit(target, speed);
}

uint8_t gate_in_position(void)
{
    return ginpos.in_pos;
}

uint8_t gate_busy(void)
{
    /* 运行态下: 轨迹执行中 / 未到位 / 队列有挂起命令
     * (对齐/停止/故障态不算busy: 上电对齐期间不应触发传送带联动限速) */
    if (motor[1].state != MOTOR_RUN) return 0;
    return (uint8_t)(traj[1].running || !ginpos.in_pos || gq_len > 0);
}

uint8_t gate_fault(void)
{
    return (uint8_t)(motor[1].state == MOTOR_FAULT);
}

/*====================================================================
 * 到位判定: 位置进入窗口并保持 GATE_INPOS_HOLD_MS 才算到位
 *====================================================================*/
static void inpos_poll(void)
{
    Motor_t *m = &motor[1];

    if (m->state != MOTOR_RUN) {
        ginpos.in_pos = 0;
        ginpos.hold_ms = 0;
        return;
    }

    int32_t err = m->target_pos - m->pos;
    if (err < 0) err = -err;

    if (!traj[1].running && err <= GATE_INPOS_WINDOW) {
        if (ginpos.hold_ms < 60000) ginpos.hold_ms++;
        if (ginpos.hold_ms >= GATE_INPOS_HOLD_MS) {
            if (!ginpos.in_pos) {
                ginpos.in_pos = 1;
                /* 本次运动首次到达窗口: 上报到位事件 */
                if (!ginpos.done_reported) {
                    ginpos.done_reported = 1;
                    UART_SendString("!DONE1\r\n");
                }
            }
        }
    } else {
        /* 跟踪中: 位置环目标仍在变或误差超窗 */
        ginpos.in_pos = 0;
        ginpos.hold_ms = 0;
    }
}

/*====================================================================
 * 队列推进: 到位后自动执行下一条挂起命令
 *====================================================================*/
static void queue_poll(void)
{
    if (gq_len > 0 && ginpos.in_pos && !traj[1].running) {
        GateCmd_t *c = &gqueue[gq_head];
        gq_head = (uint8_t)((gq_head + 1) % GATE_QUEUE_DEPTH);
        gq_len--;
        gate_execute(c->pos, c->speed);
    }
}

/*====================================================================
 * 调参验证测试
 *====================================================================*/
static void test_issue_cmd(uint8_t axis, int32_t pos)
{
    /* 测试命令用固定速度(非自适应): 保证增益调整前后命令严格一致 */
    Motor_SetTarget(&motor[axis], pos);
    traj_plan(&traj[axis], (float)pos, (float)motor[axis].pos,
              GATE_V_MAX, GATE_MOVE_ACCEL);
    if (axis == 1) ff_reset_req = 1;
}

void sorter_test_start(uint8_t axis, uint8_t runs)
{
    if (axis > 1 || runs == 0) return;

    /* 清空挂起队列: 测试期间不响应普通运动命令 */
    gq_head = gq_tail = gq_len = 0;

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
    /* 中止/结束时清空挂起队列, 防止残留命令在测试后突然执行 */
    gq_head = gq_tail = gq_len = 0;
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

    /* 到位判定与队列推进(仅挡板轴) */
    inpos_poll();
    queue_poll();
}
