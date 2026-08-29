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

Trajectory_t traj[2];
uint32_t sys_tick = 0;

/* 传送带 PWM 频率 5kHz(arr=7200,psc=1), 挡板 SVPWM 10kHz(arr=7199,psc=0) */
static u16 conv_arr = 7200, conv_psc = 1;

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

    /* 电流零点校准: 输出关断状态下采样ADC偏置(须在使能驱动器前) */
    ADC_CurrentOffsetCalib(&motor[1].i_offset_a, &motor[1].i_offset_b);

    /* Flash参数加载: 有效则覆盖标定值与传送带PID增益(调好的参数掉电保持) */
    {
        ParamBlock_t pb;
        if (param_load(&pb)) {
            motor[1].theta_offset = (int32_t)pb.theta_offset;
            motor[1].i_offset_a = pb.i_offset_a;
            motor[1].i_offset_b = pb.i_offset_b;
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

        /* ---- 挡板轨迹跟踪 + 位置环 + 堵转保护 ---- */
        if (motor[1].state == MOTOR_RUN) {
            float ref_pos = traj_step(&traj[1]);
            motor[1].target_pos = (int32_t)ref_pos;
            Motor_PositionLoop(&motor[1]);

            /* 堵转保护: 目标远离且位置持续不动则故障切断 */
            {
                Motor_t *m = &motor[1];
                int32_t err = m->target_pos - m->pos;
                if (err > STALL_ERR_COUNTS || err < -STALL_ERR_COUNTS) {
                    int32_t dp = m->pos - m->stall_last_pos;
                    if (dp < 4 && dp > -4) {
                        if (++m->stall_ms >= STALL_TIME_MS) {
                            Motor_Fault(m);
                            UART_SendString("!FAULT1\r\n");
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
        }

        IWDG_ReloadCounter();   /* 喂狗 */

        delay_ms(1);
        sys_tick++;
    }
}
