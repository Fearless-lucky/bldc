/* svpwm_main.c - 分拣线双电机控制主程序(合并版)
 *
 * 电机1(传送带): Hall六步换相 + 编码器PI速度环
 *   TIM1 PWM + TIM3 Hall + TIM4 编码器, 100ms PI(见svpwm_tim.c)
 *   应用层: conveyor.c(下游堆积度->带速, 调参TEST序列)
 *
 * 电机2(旋转挡板): 编码器FOC/SVPWM位置控制
 *   TIM8 PWM + TIM2 编码器 + ADC2 电流环, 位置/速度/电流三环
 *   应用层: sorter.c(视觉标签->挡板角度, 调参TEST序列)
 *
 * 命令接口: USART2 115200 (uart_cmd.c)
 */
#include "svpwm.h"
#include "svpwm_tim.h"
#include "bldc.h"
#include "trajectory.h"
#include "uart_cmd.h"
#include "conveyor.h"
#include "sorter.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_iwdg.h"

u16 arr = 7199, psc = 0;
float T0 = 0, T1 = 0, T2 = 0;
float Udc = 24.0f;

/* 前馈差分基线复位标志: 轨迹重规划时置1, 主循环首拍消费后清零 */
uint8_t ff_reset_req = 0;

Trajectory_t traj[2];
uint32_t sys_tick = 0;

/* 传送带 PWM 频率 5kHz(arr=7200,psc=1), 挡板 SVPWM 10kHz(arr=7199,psc=0) */
static u16 conv_arr = 7200, conv_psc = 1;

/*====================================================================
 * 运行数据环形日志: 每10ms采样, 记录触发前历史(故障回溯用)
 *====================================================================*/
#define LOG_DEPTH 100

typedef struct {
    int32_t pos;
    int16_t speed_rpm;
    int16_t speed_ref;
    float   iq;
} LogSlot_t;

static LogSlot_t dlog[LOG_DEPTH];
static uint8_t dlog_idx = 0;
static uint8_t dlog_enable = 1;    /* 0=已冻结(故障后保留现场) */

void dlog_freeze(void)
{
    dlog_enable = 0;    /* 停止采样, 保留故障现场供 DUMP 导出 */
}

void dlog_resume(void)
{
    dlog_enable = 1;    /* 恢复采样(故障处理完毕后) */
}

void dlog_dump(void)
{
    /* 按时间顺序导出: 从最旧到最新; 导出后自动恢复采样 */
    char buf[72];
    uint8_t i;
    UART_SendString("!LOG\r\n");
    for (i = 0; i < LOG_DEPTH; i++) {
        uint8_t k = (uint8_t)((dlog_idx + i) % LOG_DEPTH);
        char *p = buf;
        p = fmt_chr(p, 'L');   p = fmt_chr(p, ',');
        p = fmt_int(p, i);     p = fmt_chr(p, ',');
        p = fmt_int(p, dlog[k].pos); p = fmt_chr(p, ',');
        p = fmt_int(p, dlog[k].speed_rpm); p = fmt_chr(p, ',');
        p = fmt_int(p, dlog[k].speed_ref); p = fmt_chr(p, ',');
        p = fmt_int(p, (int32_t)(dlog[k].iq * 1000.0f));
        p = fmt_chr(p, '\r'); p = fmt_chr(p, '\n');
        *p = 0;
        UART_SendString(buf);
    }
    UART_SendString("!LOGEND\r\n");
    dlog_enable = 1;    /* 导出即处理完毕, 恢复采样 */
}

static void dlog_sample(void)
{
    if (!dlog_enable) return;
    LogSlot_t *s = &dlog[dlog_idx];
    s->pos = motor[1].pos;
    s->speed_rpm = motor[1].speed_rpm;
    s->speed_ref = motor[1].speed_ref;
    s->iq = motor[1].iq;
    dlog_idx = (uint8_t)((dlog_idx + 1) % LOG_DEPTH);
}

/*====================================================================
 * 上电自检: Hall有效性 / 编码器响应 / 电流采样范围
 * 返回故障码(FAULT_NONE=通过)
 *====================================================================*/
static uint8_t power_on_selftest(void)
{
    uint8_t fault = FAULT_NONE;

    /* 1) 传送带Hall静态检查: 000/111 为断线特征 */
    {
        u8 hs = hallsensor_get_state();
        if (hs == 0 || hs == 7) {
            fault |= FAULT_HALL_LOST;
        }
    }

    /* 2) 挡板编码器静态检查: 引脚电平需为确定态(非浮空) */
    /* 注: 编码器动态响应需手动转动检测, 静态仅查电流采样 */

    /* 3) 电流采样范围: 偏置应在ADC中段(500~3500), 否则采样链路异常 */
    {
        float off_a = 0.0f, off_b = 0.0f;
        ADC_CurrentOffsetCalib(&off_a, &off_b);
        motor[1].i_offset_a = off_a;
        motor[1].i_offset_b = off_b;
        if (off_a < 500.0f || off_a > 3500.0f ||
            off_b < 500.0f || off_b > 3500.0f) {
            fault |= FAULT_DRV;
        }
    }

    return fault;
}

static void update_encoder_angle(void)
{
    Motor_t *m = &motor[1];
    TIM_TypeDef *TIMx = m->tim_enc;
    uint16_t cnt = TIM_GetCounter(TIMx);
    m->enc_raw = cnt;

    /* 减去标定的电角度零点(转子d轴位置), 保证theta_elec=0对应d轴 */
    int32_t rel = (int32_t)cnt - m->theta_offset;
    if (m->dir == CW) {
        m->theta_elec = -6.2832f * ((rel % 1000 + 1000) % 1000) / 1000.0f;
    } else {
        m->theta_elec = 6.2832f * (((-rel) % 1000 + 1000) % 1000) / 1000.0f;
    }
}

int main(void)
{
    key_init();
    delay_init(72);      /* F103主频72MHz(168为F4模板错误值, 会导致所有延时慢2.33倍) */

    /* ---- 挡板电机(FOC/SVPWM): TIM8 + TIM2 + ADC2 ---- */
    svpwm_gpio2_init();
    SVPWM_TIM8_Init(arr, psc);
    PWM_RESOLUTION = arr;
    Encoder_Init_TIM2();

    /* ---- 传送带电机(六步换相): TIM1 + TIM3 Hall + TIM4 ---- */
    bldc_gpio_init();
    TIM1_PwmoutCNF_OnLibFunc(conv_arr - 1, conv_psc - 1);
    Time3_HalldetectCNF();
    Encoder_Init_TIM4();

    /* ---- 公共: 10ms节拍 + 电流采样 + NVIC + UART ---- */
    Init_TIM6();
    ADC_Axis_Init();

    TIM2_NVIC_Config();
    TIM3_NVIC_Config();
    TIM4_NVIC_Config();
    TIM6_NVIC_Config();
    TIM8_NVIC_Config();

    UART_Init();
    conveyor_init();      /* 传送带: 以默认目标速度初始化堆积度调速 */

    /* ---- 挡板电机初始化 ---- */
    Motor_Init(&motor[1], TIM8, TIM2, 2, 3);
    motor[1].dir = CCW;
    traj_init(&traj[1]);

    /* 上电自检 + 电流零点校准(输出关断状态采样, 须在使能驱动器前) */
    {
        uint8_t selftest_fault = power_on_selftest();
        if (selftest_fault != FAULT_NONE) {
            char buf[40];
            /* 自检失败: 记录故障码并上报, 挡板保持禁用 */
            motor[1].fault_code = selftest_fault;
            UART_SendString("!SELFTEST FAIL,0x");
            {
                char hx[3] = {"0123456789ABCDEF"};
                buf[0] = hx[(selftest_fault >> 4) & 0xF];
                buf[1] = hx[selftest_fault & 0xF];
                buf[2] = 0;
                UART_SendString(buf);
            }
            UART_SendString("\r\n");
        } else {
            UART_SendString("!SELFTEST OK\r\n");
        }
    }

    /* Flash参数加载: 有效则覆盖标定值与传送带PID增益(调好的参数掉电保持) */
    {
        ParamBlock_t pb;
        if (param_load(&pb)) {
            motor[1].theta_offset = (int32_t)pb.theta_offset;
            motor[1].i_offset_a = pb.i_offset_a;
            motor[1].i_offset_b = pb.i_offset_b;
            motor[1].enc_dir = (pb.enc_dir >= 0) ? 1 : 255;   /* 存+1/-1, 归一化 */
            Kp = pb.conv_kp;
            Ki = pb.conv_ki;
            Kd = pb.conv_kd;
        }
    }

    delay_ms(1000);
    TIM_SetCounter(TIM2, 0);
    motor[1].theta_elec = 0;

    TIM_Cmd(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE);

    motor[1].state = MOTOR_ALIGN;
    motor[1].align_ms = 0;
    /* MOE必须由软件置位(AOE仅在刹车事件后恢复): 否则TIM8永无PWM输出 */
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    GPIO_WriteBit(AXIS2_SD_PORT, AXIS2_SD_PIN, Bit_SET);

    /* ---- 传送带电机: 上电保持停止, 由 RUN/SPD 命令或按键启动 ---- */
    motor1.pwm_duty = 350;    /* 六步换相初值(电机静止时无Hall边沿) */
    motor1.dir = CW;
    motor1.run_flag = STOP;
    stop_motor1();

    delay_ms(500);

    /* 看门狗: 主循环1ms喂狗, 程序跑飞约2s后自动复位(LSI≈40kHz/64=625Hz) */
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload(1250);
    IWDG_ReloadCounter();
    IWDG_Enable();

    while (1) {
        uart_poll();

        uint8_t key = key_scan(0);
        if (key == KEY0_PRES) {
            Motor_Stop(&motor[1]);   /* 挡板急停 */
        } else if (key == KEY1_PRES) {
            stop_motor1();           /* 传送带急停 */
            motor1.run_flag = STOP;
        }

        /* ---- 挡板: 电角度更新 + 轨迹/位置环 ---- */
        update_encoder_angle();
        motor[1].pos = TIM_GetCounter(TIM2);

        /* ---- 应用层: 传送带堆积度调速/测试序列 + 挡板测试序列 ---- */
        conveyor_poll();
        sorter_poll();

        /* ---- 双轴联动: 挡板状态 -> 传送带限速/急停 ---- */
        conveyor_link_update(gate_busy(), gate_fault());
        if (gate_fault() && motor1.run_flag == RUN) {
            conveyor_emergency_stop();   /* 挡板故障联动切断传送带 */
        }

        /* ---- 挡板速度环(10ms) ---- */
        if (tim6_flag) {
            Motor_SpeedLoop(&motor[1], 0.01f);
            tim6_flag = 0;
        }

        /* ---- 挡板对齐超时 -> 运行态: 捕获电角度零点并保持当前位置 ---- */
        if (motor[1].state == MOTOR_ALIGN) {
            if (++motor[1].align_ms >= 800) {
                motor[1].state = MOTOR_RUN;
                motor[1].align_ms = 0;
                /* 转子已被固定矢量拉到d轴: 此刻编码器读数即电角度零点 */
                motor[1].theta_offset = (int32_t)(TIM_GetCounter(TIM2) % 1000);
                motor[1].target_pos = motor[1].pos;
                traj_init(&traj[1]);
                traj[1].target_pos = (float)motor[1].pos;
            }
        }

        /* ---- 挡板轨迹跟踪 + 位置环 + 前馈 + 堵转/过流保护 + 日志采样 ---- */
        if (motor[1].state == MOTOR_RUN) {
            /* 轨迹前馈数据: v_ff/a_ff 由轨迹相位计算(简化差分)
             * 首拍跳过差分(避免与上段轨迹/静止位置的伪差分产生巨大加速度) */
            {
                static float last_ref = 0;
                static uint8_t ff_init = 0;
                float ref_pos = traj_step(&traj[1]);
                if (ff_reset_req) {
                    /* 新轨迹首拍: 速度/加速度前馈从0起(轨迹本身从静止/当前速度平滑加速) */
                    motor[1].v_ff = 0.0f;
                    motor[1].a_ff = 0.0f;
                    ff_init = 1;
                    ff_reset_req = 0;
                }
                if (ff_init) {
                    float v_now = (ref_pos - last_ref) * 1000.0f;      /* counts/s */
                    motor[1].a_ff = (v_now - motor[1].v_ff) * 1000.0f; /* counts/s^2 */
                    motor[1].v_ff = v_now;
                } else {
                    motor[1].v_ff = 0.0f;
                    motor[1].a_ff = 0.0f;
                    ff_init = 1;
                }
                last_ref = ref_pos;
                motor[1].target_pos = (int32_t)ref_pos;
            }
            Motor_PositionLoop(&motor[1]);

            /* 堵转保护: 目标远离且位置持续不动则故障切断 */
            {
                Motor_t *m = &motor[1];
                int32_t err = m->target_pos - m->pos;
                if (err > STALL_ERR_COUNTS || err < -STALL_ERR_COUNTS) {
                    int32_t dp = m->pos - m->stall_last_pos;
                    if (dp < 4 && dp > -4) {
                        if (++m->stall_ms >= STALL_TIME_MS) {
                            Motor_SetFault(m, FAULT_STALL);
                            dlog_freeze();
                            UART_SendString("!FAULT1,STALL\r\n");
                        }
                    } else {
                        m->stall_ms = 0;
                        m->stall_last_pos = m->pos;
                    }
                } else {
                    m->stall_ms = 0;
                    m->stall_last_pos = m->pos;
                }
            }

            /* 过流监测: 实测q轴电流持续越限(指令限幅之外的硬件异常) */
            {
                Motor_t *m = &motor[1];
                float iq_abs = m->iq;
                if (iq_abs < 0) iq_abs = -iq_abs;
                if (iq_abs > OC_LEVEL_AMP) {
                    if (++m->oc_ms >= OC_TIME_MS) {
                        Motor_SetFault(m, FAULT_OC);
                        dlog_freeze();
                        UART_SendString("!FAULT1,OC\r\n");
                    }
                } else {
                    m->oc_ms = 0;
                }
            }
        }

        /* 运行数据日志: 每10ms采样一次(tim6_flag节拍对齐后近似) */
        {
            static uint8_t log_div = 0;
            if (++log_div >= 10) {
                log_div = 0;
                dlog_sample();
            }
        }

        IWDG_ReloadCounter();   /* 喂狗 */

        delay_ms(1);
        sys_tick++;
    }
}
