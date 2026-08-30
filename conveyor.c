/* conveyor.c - 分拣线传送带应用层
 * 六步换相+编码器PI速度环(bldc.c/svpwm_tim.c)之上实现:
 *  1. 下游堆积度 -> 传送带速度目标映射(堆积越严重带速越低, 满级停带)
 *  2. 调参验证: 固定负载下重复相同速度阶跃命令, 输出响应曲线遥测,
 *     供增益调整前后对比
 */
#include "conveyor.h"
#include "bldc.h"
#include "uart_cmd.h"
#include "stm32f10x_tim.h"
#include "stdio.h"

extern volatile uint32_t tick_100ms;   /* 定义于 svpwm_tim.c, 100ms全局时基 */

static int16_t  base_speed  = 0;    /* 堆积度为0时的基准带速 RPM */
static uint8_t  acc_level   = 0;    /* 下游堆积度 0~100 */
static uint8_t  log_en      = 0;    /* 手动遥测开关 */

/* 双轴联动状态 */
static uint8_t  link_gate_busy = 0;  /* 挡板运动中(传送带限速) */
static uint8_t  link_gate_fault = 0; /* 挡板故障(传送带停机) */

/* 斜坡状态: 期望目标 vs 当前目标(每tick限幅逼近) */
static float    slew_current = 0.0f;

static uint8_t  test_active = 0;    /* 调参测试进行中 */
static uint8_t  test_runs   = 0;
static uint8_t  test_run_idx = 0;
static uint8_t  test_phase  = 0;
static uint32_t phase_t0    = 0;

/* 计算期望速度目标(堆积度折算 + 双轴联动限速), 不直接写入 */
static float conveyor_desired(void)
{
    if (link_gate_fault) {
        return 0.0f;    /* 挡板故障: 传送带联动停机 */
    }

    /* 堆积度越高带速越低; 达到暂停阈值时停带防止下游堵料 */
    float v;
    if (acc_level >= CONV_ACC_STOP_LEVEL) {
        v = 0.0f;
    } else {
        v = (float)(base_speed * (100 - acc_level) / 100);
    }

    /* 挡板动作期间限速: 避免物料在挡板切换中到达 */
    if (link_gate_busy) {
        v *= CONV_GATE_SLOW_FACTOR;
    }
    return v;
}

/* 斜坡逼近: 每100ms最多变化 CONV_SLEW_RPM_PER_TICK */
static void conveyor_slew(void)
{
    float desired = conveyor_desired();
    float d = desired - slew_current;

    if (d > CONV_SLEW_RPM_PER_TICK) {
        slew_current += CONV_SLEW_RPM_PER_TICK;
    } else if (d < -CONV_SLEW_RPM_PER_TICK) {
        slew_current -= CONV_SLEW_RPM_PER_TICK;
    } else {
        slew_current = desired;
    }
    pid_target = slew_current;
}

void conveyor_init(void)
{
    base_speed = (int16_t)pid_target;
    acc_level = 0;
    log_en = 0;
    test_active = 0;
    link_gate_busy = 0;
    link_gate_fault = 0;
    slew_current = 0.0f;
    pid_target = 0.0f;   /* 上电从静止斜坡起步 */
}

void conveyor_set_base_speed(int16_t rpm)
{
    if (rpm < 0) rpm = 0;
    if (rpm > 3000) rpm = 3000;
    base_speed = rpm;
    /* 目标由 conveyor_poll 的斜坡逐步逼近, 无需立即写 pid_target */
}

void conveyor_set_accumulation(uint8_t level)
{
    if (level > 100) level = 100;
    acc_level = level;
}

void conveyor_set_log(uint8_t on)
{
    log_en = on ? 1 : 0;
}

int16_t conveyor_base_speed(void)
{
    return base_speed;
}

void conveyor_link_update(uint8_t gate_busy_flag, uint8_t gate_fault_flag)
{
    link_gate_busy = gate_busy_flag ? 1 : 0;
    link_gate_fault = gate_fault_flag ? 1 : 0;
}

void conveyor_emergency_stop(void)
{
    /* 联动急停: 挡板故障时切断传送带输出 */
    link_gate_fault = 1;
    slew_current = 0.0f;
    pid_target = 0.0f;
    stop_motor1();
    motor1.run_flag = STOP;
    UART_SendString("!CSTOP\r\n");
}

uint8_t conveyor_accumulation(void)
{
    return acc_level;
}

void conveyor_test_start(uint8_t runs)
{
    if (runs == 0 || base_speed <= 0) return;

    /* 测试需要电机处于运行状态: 停机时先启动并完成首次换相 */
    if (motor1.run_flag != RUN) {
        motor1.run_flag = RUN;
        start_motor1();
        UVW_6_Step_Ponoff();
        TIM_GenerateEvent(TIM1, TIM_EventSource_COM);
    }

    test_runs = runs;
    test_run_idx = 0;
    test_phase = 0;
    test_active = 1;
    {
        char buf[24];
        sprintf(buf, "!TEST,%d\r\n", runs);
        UART_SendString(buf);
    }
}

/* 主循环调用, 内部按100ms节拍推进: 测试状态机 + 遥测输出 */
void conveyor_poll(void)
{
    static uint32_t last_tick = 0;
    uint32_t now = tick_100ms;
    if (now == last_tick) return;
    last_tick = now;

    if (test_active) {
        /* 电机被STOP或Hall异常停机时中止测试, 避免后台继续写目标与输出误导性遥测 */
        if (motor1.run_flag != RUN) {
            test_active = 0;
            slew_current = 0.0f;   /* 恢复后从静止斜坡 */
            UART_SendString("!TABORT\r\n");
        } else {
        switch (test_phase) {
        case 0: /* 新回合: 先回静止, 保证每回合初始条件一致 */
            test_run_idx++;
            {
                char buf[24];
                sprintf(buf, "!RUN,%d\r\n", test_run_idx);
                UART_SendString(buf);
            }
            pid_target = 0.0f;
            phase_t0 = now;
            test_phase = 1;
            break;

        case 1: /* 停稳后发出本回合的相同速度命令(测试需阶跃, 绕过斜坡) */
            if (now - phase_t0 >= TEST_SETTLE_TICKS) {
                pid_target = (float)base_speed;
                slew_current = (float)base_speed;
                phase_t0 = now;
                test_phase = 2;
            }
            break;

        case 2: /* 阶跃保持, 采集完整升速响应曲线 */
            if (now - phase_t0 >= TEST_HOLD_TICKS) {
                pid_target = 0.0f;
                slew_current = 0.0f;
                phase_t0 = now;
                test_phase = 3;
            }
            break;

        case 3: /* 停稳后判断是否进入下一回合 */
            if (now - phase_t0 >= TEST_SETTLE_TICKS) {
                if (test_run_idx >= test_runs) {
                    test_active = 0;
                    UART_SendString("!TDONE\r\n");
                    /* 恢复正常调速(下一tick起从静止斜坡回升) */
                    slew_current = 0.0f;
                    pid_target = 0.0f;
                    return;
                }
                test_phase = 0;
            }
            break;

        default:
            test_phase = 0;
            break;
        }
        }
    } else {
        /* 正常运行: 堆积度/联动折算 + 斜坡逼近速度目标 */
        conveyor_slew();
    }

    /* 响应曲线遥测: C,<run>,<t_100ms>,<target_rpm>,<speed_rpm>,<pwm_duty>
     * (前缀C=传送带, 与挡板遥测前缀D区分)
     * 记录增益调整前后各一轮TEST即可叠加对比响应曲线 */
    if (test_active || log_en) {
        /* 手工拼接替代sprintf: 降低遥测对控制环的CPU挤占 */
        char buf[64];
        char *p = buf;
        p = fmt_chr(p, 'C');   p = fmt_chr(p, ',');
        p = fmt_int(p, test_active ? test_run_idx : 0); p = fmt_chr(p, ',');
        p = fmt_uint(p, now); p = fmt_chr(p, ',');
        p = fmt_int(p, (int)pid_target); p = fmt_chr(p, ',');
        p = fmt_int(p, motor1.speed); p = fmt_chr(p, ',');
        p = fmt_uint(p, motor1.pwm_duty);
        p = fmt_chr(p, '\r'); p = fmt_chr(p, '\n');
        *p = 0;
        UART_SendString(buf);
    }
}
