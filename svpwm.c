#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "svpwm.h"

extern u16 PWM_RESOLUTION;
extern float Udc;
extern float T0, T1, T2;

Motor_t motor[2];
volatile uint8_t tim6_flag = 0;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->last_error = 0;
    pid->limit = limit;
}

float PID_Calc(PID_t *pid, float ref, float fb, float dt)
{
    float error = ref - fb;
    float p_out = pid->kp * error;
    pid->integral += pid->ki * error * dt;
    if (pid->integral > pid->limit) pid->integral = pid->limit;
    if (pid->integral < -pid->limit) pid->integral = -pid->limit;
    float d_out = pid->kd * (error - pid->last_error) / dt;
    float out = p_out + pid->integral + d_out;
    if (out > pid->limit) out = pid->limit;
    if (out < -pid->limit) out = -pid->limit;
    pid->last_error = error;
    return out;
}

void Clarke(float ia, float ib, float *ialpha, float *ibeta)
{
    float ic = -(ia + ib);
    *ialpha = ia;
    *ibeta = (ia + 2.0f * ib) / 1.7320508f;
}

void Park(float ialpha, float ibeta, float theta, float *id, float *iq)
{
    float c = cosf(theta), s = sinf(theta);
    *id =  ialpha * c + ibeta * s;
    *iq = -ialpha * s + ibeta * c;
}

void InvPark(float vd, float vq, float theta, float *valpha, float *vbeta)
{
    float c = cosf(theta), s = sinf(theta);
    *valpha = vd * c - vq * s;
    *vbeta  = vd * s + vq * c;
}

void Motor_Init(Motor_t *m, TIM_TypeDef *tim_pwm, TIM_TypeDef *tim_enc, uint8_t ch_a, uint8_t ch_b)
{
    m->tim_pwm = tim_pwm;
    m->tim_enc = tim_enc;
    m->adc_ch_a = ch_a;
    m->adc_ch_b = ch_b;
    m->theta_elec = 0;
    m->theta_mech = 0;
    m->enc_raw = 0;
    m->pos = 0;
    m->speed_rpm = 0;
    m->ia = m->ib = 0;
    m->id = m->iq = 0;
    m->id_ref = 0;
    m->iq_ref = 0;
    m->ud_ref = m->uq_ref = 0;
    m->target_pos = 0;
    m->speed_ref = 0;
    m->state = MOTOR_STOP;
    m->dir = CCW;
    m->align_ms = 0;
    m->theta_offset = 0;
    m->i_offset_a = 0.0f;
    m->i_offset_b = 0.0f;
    m->stall_ms = 0;
    m->stall_last_pos = 0;
    m->v_ff = 0.0f;
    m->a_ff = 0.0f;
    m->oc_ms = 0;
    m->enc_dir = 1;
    m->fault_code = FAULT_NONE;

    PID_Init(&m->pid_c_d, 0.5f, 0.01f, 0.0f, 12.0f);
    PID_Init(&m->pid_c_q, 0.5f, 0.01f, 0.0f, 12.0f);
    PID_Init(&m->pid_speed, 0.02f, 0.5f, 0.0f, I_MAX);
    PID_Init(&m->pid_pos, 0.1f, 0.001f, 0.0f, 1500.0f);
}

void Motor_CurrentLoop(Motor_t *m)
{
    if (m->state != MOTOR_RUN && m->state != MOTOR_ALIGN) return;

    if (m->state == MOTOR_ALIGN) {
        /* 对齐阶段: 施加固定角度0的电压矢量, 转子稳定后编码器读数即电角度零点 */
        float valpha, vbeta;
        InvPark(2, 0, 0.0f, &valpha, &vbeta);
        SVPWM_Update_Duty(m, valpha, vbeta);
        return;
    }

    float ialpha, ibeta, id, iq;
    Clarke(m->ia, m->ib, &ialpha, &ibeta);
    Park(ialpha, ibeta, m->theta_elec, &id, &iq);
    m->id = id; m->iq = iq;

    float vd = PID_Calc(&m->pid_c_d, m->id_ref, id, 0.0001f);
    float vq = PID_Calc(&m->pid_c_q, m->iq_ref, iq, 0.0001f);
    m->ud_ref = vd; m->uq_ref = vq;

    float valpha, vbeta;
    InvPark(vd, vq, m->theta_elec, &valpha, &vbeta);

    SVPWM_Update_Duty(m, valpha, vbeta);
}

void Motor_SetTarget(Motor_t *m, int32_t pos)
{
    m->target_pos = pos;
    if (m->state == MOTOR_ALIGN || m->state == MOTOR_STOP) {
        m->state = MOTOR_RUN;
    }
}

void Motor_SpeedLoop(Motor_t *m, float dt)
{
    if (m->state != MOTOR_RUN) return;

    /* 反馈控制 + 加速度前馈: 轨迹规划加速度直接前馈到q轴电流
     * 减小跟踪滞后, 位置环同理有速度前馈 */
    float fb = PID_Calc(&m->pid_speed, (float)m->speed_ref, (float)m->speed_rpm, dt);
    float ff = FF_ACC_KT * m->a_ff;
    m->iq_ref = fb + ff;
    if (m->iq_ref > I_MAX) m->iq_ref = I_MAX;
    if (m->iq_ref < -I_MAX) m->iq_ref = -I_MAX;
}

void Motor_PositionLoop(Motor_t *m)
{
    if (m->state != MOTOR_RUN) return;
    /* 位置误差PID + 轨迹速度前馈: 跟随误差显著减小 */
    float fb = PID_Calc(&m->pid_pos, (float)m->target_pos, (float)m->pos, 0.01f);
    float ff = FF_VEL_KP * m->v_ff * 0.06f;   /* counts/s -> rpm 近似换算(2000计数/转) */
    float out = fb + ff;
    if (out > 1500.0f) out = 1500.0f;
    if (out < -1500.0f) out = -1500.0f;
    m->speed_ref = (int16_t)out;
}

void Motor_Stop(Motor_t *m)
{
    m->state = MOTOR_STOP;
    m->iq_ref = 0;
    m->id_ref = 0;
    m->ud_ref = 0;
    m->uq_ref = 0;
    /* 三相占空比置为相等(零矢量): 撤除输出电压, 电机真正停转 */
    m->tim_pwm->CCR1 = 0;
    m->tim_pwm->CCR2 = 0;
    m->tim_pwm->CCR3 = 0;
}

void Motor_Fault(Motor_t *m)
{
    m->state = MOTOR_FAULT;
    TIM_Cmd(m->tim_pwm, DISABLE);
    /* 挡板电机: 同时切断驱动器使能, 双重保护 */
    if (m->tim_pwm == TIM8) {
        GPIO_WriteBit(AXIS2_SD_PORT, AXIS2_SD_PIN, Bit_RESET);
    }
}

void Motor_SetFault(Motor_t *m, uint8_t code)
{
    if (m->state != MOTOR_FAULT) {
        m->fault_code = code;
        Motor_Fault(m);
    }
}

/* 挡板电机GPIO: TIM8_PWM(PC6/7/8) + 编码器TIM2(PA0/PA1) + 驱动SD(PB4)/故障(PB5) */
void svpwm_gpio2_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);

    /* PB4复用为NJTRST: 必须关闭JTAG(保留SWD)才能作为GPIO驱动挡板SD */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitTypeDef g;
    g.GPIO_Speed = GPIO_Speed_50MHz;

    /* 挡板电机 PWM: TIM8_CH1/CH2/CH3 = PC6/PC7/PC8(单端输出, 无互补通道) */
    g.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    g.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOC, &g);

    /* 挡板编码器输入: TIM2_CH1/CH2 = PA0/PA1 下拉输入 */
    g.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    g.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &g);

    /* 挡板驱动器使能 SD=PB4, 故障输入=PB5 */
    g.GPIO_Pin = AXIS2_SD_PIN;
    g.GPIO_Mode = GPIO_Mode_Out_PP;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(AXIS2_SD_PORT, &g);
    GPIO_WriteBit(AXIS2_SD_PORT, AXIS2_SD_PIN, Bit_RESET);

    g.GPIO_Pin = AXIS2_BKIN_PIN;
    g.GPIO_Mode = GPIO_Mode_IPD;
    g.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(AXIS2_BKIN_PORT, &g);
}


uint8_t SVPWM_Sector_Calc(float Ualpha, float Ubeta)
{
    uint8_t sector = 0;
    float Vref1 = Ubeta;
    float Vref2 = (1.7320508f * Ualpha - Ubeta) / 2.0f;
    float Vref3 = (-1.7320508f * Ualpha - Ubeta) / 2.0f;
    if (Vref1 >= 0) sector |= 1;
    if (Vref2 >= 0) sector |= 2;
    if (Vref3 >= 0) sector |= 4;
    return sector;
}


void SVPWM_Update_Duty(Motor_t *m, float ualpha, float ubeta)
{
    float X, Y, Z, Tcmp1 = 0, Tcmp2 = 0, Tcmp3 = 0;

    X = 1.7320508f * ubeta / Udc;
    Y = (3.0f * ualpha + 1.7320508f * ubeta) / (2.0f * Udc);
    Z = (-3.0f * ualpha + 1.7320508f * ubeta) / (2.0f * Udc);

    uint8_t sector = SVPWM_Sector_Calc(ualpha, ubeta);

    switch (sector) {
        case 1: T1 = Z; T2 = Y; break;
        case 2: T1 = Y; T2 = -X; break;
        case 3: T1 = -Z; T2 = X; break;
        case 4: T1 = -X; T2 = Z; break;
        case 5: T1 = X; T2 = -Y; break;
        case 6: T1 = -Y; T2 = -Z; break;
        default: T1 = 0; T2 = 0; break;
    }

    if ((T1 + T2) > 1.0f) {
        T1 = T1 / (T1 + T2);
        T2 = T2 / (T1 + T2);
    }

    T0 = 1.0f - T1 - T2;
    T0 = (T0 < 0) ? 0 : ((T0 > 1) ? 1 : T0);
    T1 = (T1 < 0) ? 0 : ((T1 > 1) ? 1 : T1);
    T2 = (T2 < 0) ? 0 : ((T2 > 1) ? 1 : T2);

    float ta = T0 / 4.0f;
    float tb = ta + T1 / 2.0f;
    float tc = tb + T2 / 2.0f;

    switch (sector) {
        case 1: Tcmp1 = tb; Tcmp2 = ta; Tcmp3 = tc; break;
        case 2: Tcmp1 = ta; Tcmp2 = tc; Tcmp3 = tb; break;
        case 3: Tcmp1 = ta; Tcmp2 = tb; Tcmp3 = tc; break;
        case 4: Tcmp1 = tc; Tcmp2 = tb; Tcmp3 = ta; break;
        case 5: Tcmp1 = tc; Tcmp2 = ta; Tcmp3 = tb; break;
        case 6: Tcmp1 = tb; Tcmp2 = tc; Tcmp3 = ta; break;
    }

    TIM_TypeDef *TIMx = m->tim_pwm;
    TIMx->CCR1 = (uint16_t)(Tcmp1 * PWM_RESOLUTION);
    TIMx->CCR2 = (uint16_t)(Tcmp2 * PWM_RESOLUTION);
    TIMx->CCR3 = (uint16_t)(Tcmp3 * PWM_RESOLUTION);
}
