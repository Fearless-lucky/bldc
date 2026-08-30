/* svpwm_tim.c - 定时器/ADC 资源统一配置(双电机合并)
 *
 * 资源分配(STM32F103ZE):
 *   传送带电机(六步换相+PI速度环):
 *     TIM1 PWM(PA8/9/10 + PB13/14/15) + TIM3 Hall(PA6/PA7/PB0)
 *     TIM4 编码器(PB6/7, 2000计数/转) + TIM1_BKIN(PB12)
 *   挡板电机(FOC/SVPWM位置控制):
 *     TIM8 PWM(PC6/7/8, 单端输出) + TIM2 编码器(PA0/PA1, 2000计数/转)
 *     ADC2 注入采样(PC0/PC1 相电流)
 *   TIM6: 10ms 节拍 —— 挡板速度计算(每周期) + 传送带速度PI(每10周期)
 *
 * 注: TIM8 不使用互补输出与硬件刹车(PA6/PA7/PB0 引脚让给传送带Hall),
 *     下桥死区由外部栅极驱动器保证。
 */
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_adc.h"
#include "svpwm_tim.h"
#include "svpwm.h"
#include "bldc.h"
#include "misc.h"

u16 PWM_RESOLUTION;

/* 100ms 全局时基, 供传送带应用层(conveyor.c)使用 */
volatile uint32_t tick_100ms = 0;

/* 传送带速度PI静态状态(积分分离) */
static int16_t e_prev = 0;
static float   integral = 0;

/*==================================================================
 * 传送带电机: TIM1 PWM 配置(六步换相, COM事件由TIM3 Hall触发)
 *==================================================================*/
void TIM1_PwmoutCNF_OnLibFunc(int arr, int psc)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitTypeDef t;
    t.TIM_Period = arr;
    t.TIM_Prescaler = psc;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    t.TIM_CounterMode = TIM_CounterMode_Up;
    t.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &t);

    /* TIM3 TRGO(TI1F_ED边沿)触发COM事件, 实现Hall同步换相 */
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Trigger);
    TIM_SelectInputTrigger(TIM1, TIM_TS_ITR2);
    TIM_SelectCOM(TIM1, ENABLE);
    TIM_CCPreloadControl(TIM1, ENABLE);
    TIM_ARRPreloadConfig(TIM1, DISABLE);

    TIM_OCInitTypeDef o;
    o.TIM_OCMode = TIM_OCMode_PWM1;
    o.TIM_OutputState = TIM_OutputState_Enable;
    o.TIM_OutputNState = TIM_OutputNState_Enable;
    o.TIM_OCPolarity = TIM_OCPolarity_High;
    o.TIM_OCNPolarity = TIM_OCNPolarity_High;
    o.TIM_OCIdleState = TIM_OCIdleState_Reset;
    o.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    o.TIM_Pulse = 0;
    TIM_OC1Init(TIM1, &o);
    TIM_OC2Init(TIM1, &o);
    TIM_OC3Init(TIM1, &o);

    TIM_OC1FastConfig(TIM1, DISABLE);
    TIM_OC2FastConfig(TIM1, DISABLE);
    TIM_OC3FastConfig(TIM1, DISABLE);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_BDTRInitTypeDef b;
    TIM_BDTRStructInit(&b);
    b.TIM_OSSRState = TIM_OSSRState_Enable;
    b.TIM_OSSIState = TIM_OSSIState_Enable;
    b.TIM_LOCKLevel = TIM_LOCKLevel_1;
    b.TIM_DeadTime = 11;
    b.TIM_Break = TIM_Break_Enable;
    b.TIM_BreakPolarity = TIM_BreakPolarity_High;
    b.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM1, &b);

    TIM_ClearFlag(TIM1, TIM_FLAG_Update | TIM_FLAG_COM | TIM_FLAG_Break);
    TIM_Cmd(TIM1, ENABLE);
}

/*==================================================================
 * 传送带电机: TIM3 Hall 传感器检测
 * Hall 任意边沿复位CNT并触发TRGO -> TIM1 COM换相
 *==================================================================*/
void Time3_HalldetectCNF(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    TIM_TimeBaseInitTypeDef t;
    t.TIM_Period = 65535;
    t.TIM_Prescaler = 0;
    t.TIM_ClockDivision = 0;
    t.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &t);

    TIM_SelectHallSensor(TIM3, ENABLE);
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
    TIM_SelectInputTrigger(TIM3, TIM_TS_TI1F_ED);

    TIM_ICInitTypeDef ic;
    ic.TIM_Channel = TIM_Channel_1;
    ic.TIM_ICFilter = 4;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInit(TIM3, &ic);

    TIM_OCInitTypeDef oc;
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_Pulse = 10;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM3, &oc);

    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC4Ref);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ITConfig(TIM3, TIM_IT_Trigger, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

/* Hall边沿计数: 用于传送带Hall丢失检测 */
static uint32_t hall_edges = 0;

void TIM3_IRQHandler(void)
{
    hall_edges++;
    UVW_6_Step_Ponoff();
    TIM_ClearITPendingBit(TIM3, TIM_FLAG_Trigger);
}

void TIM3_NVIC_Config(void)
{
    NVIC_InitTypeDef n;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    n.NVIC_IRQChannel = TIM3_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 0;
    n.NVIC_IRQChannelSubPriority = 3;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
}

/*==================================================================
 * 传送带电机: TIM4 编码器(PB6/7, 500P/R x4 = 2000计数/转)
 * 速度采样为100ms清零式: PI环读取后清零计数器
 *==================================================================*/
void Encoder_Init_TIM4(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef g;
    g.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    g.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &g);

    TIM_TimeBaseInitTypeDef t;
    TIM_TimeBaseStructInit(&t);
    t.TIM_Prescaler = 0;
    t.TIM_Period = 65535;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    t.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &t);

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICInitTypeDef ic;
    TIM_ICStructInit(&ic);
    ic.TIM_ICFilter = 10;
    TIM_ICInit(TIM4, &ic);

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_SetCounter(TIM4, 0);
    TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
    /* 清零式采样下计数不会到达回绕点, 仅作保险清除 */
    TIM_ClearITPendingBit(TIM4, TIM_FLAG_Update);
}

void TIM4_NVIC_Config(void)
{
    NVIC_InitTypeDef n;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    n.NVIC_IRQChannel = TIM4_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 0;
    n.NVIC_IRQChannelSubPriority = 3;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
}

/*==================================================================
 * 挡板电机: TIM8 PWM(PC6/7/8 单端输出, 中心对齐)
 *==================================================================*/
void SVPWM_TIM8_Init(int arr, int psc)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    TIM_TimeBaseInitTypeDef t;
    t.TIM_Period = arr;
    t.TIM_Prescaler = psc;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    t.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    t.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM8, &t);

    TIM_ARRPreloadConfig(TIM8, DISABLE);

    /* 单端输出: PA6/PA7/PB0 让给传送带Hall, 不使用互补通道 */
    TIM_OCInitTypeDef o;
    o.TIM_OCMode = TIM_OCMode_PWM2;
    o.TIM_OutputState = TIM_OutputState_Enable;
    o.TIM_OutputNState = TIM_OutputNState_Disable;
    o.TIM_OCPolarity = TIM_OCPolarity_High;
    o.TIM_OCIdleState = TIM_OCIdleState_Reset;
    o.TIM_Pulse = 0;
    TIM_OC1Init(TIM8, &o);
    TIM_OC2Init(TIM8, &o);
    TIM_OC3Init(TIM8, &o);

    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

    /* 硬件刹车禁用: TIM8_BKIN(PA6)已被传送带Hall占用 */
    TIM_BDTRInitTypeDef b;
    TIM_BDTRStructInit(&b);
    b.TIM_OSSRState = TIM_OSSRState_Enable;
    b.TIM_OSSIState = TIM_OSSIState_Enable;
    b.TIM_LOCKLevel = TIM_LOCKLevel_1;
    b.TIM_DeadTime = 0;
    b.TIM_Break = TIM_Break_Disable;
    b.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRConfig(TIM8, &b);

    TIM_ClearFlag(TIM8, TIM_FLAG_Update | TIM_FLAG_COM | TIM_FLAG_Break);
    TIM_Cmd(TIM8, ENABLE);
}

/*==================================================================
 * 挡板电机: TIM2 编码器(PA0/PA1, 2000计数/转, 周期60000)
 *==================================================================*/
void Encoder_Init_TIM2(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef t;
    TIM_TimeBaseStructInit(&t);
    t.TIM_Prescaler = 0;
    t.TIM_Period = 59999;
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    t.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &t);

    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICInitTypeDef ic;
    TIM_ICStructInit(&ic);
    ic.TIM_ICFilter = 10;
    TIM_ICInit(TIM2, &ic);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}

void TIM2_IRQHandler(void)
{
    /* 挡板编码器溢出: 速度按模60000差值计算, 仅清标志 */
    TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);
}

void TIM2_NVIC_Config(void)
{
    NVIC_InitTypeDef n;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    n.NVIC_IRQChannel = TIM2_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 0;
    n.NVIC_IRQChannelSubPriority = 3;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
}

/*==================================================================
 * TIM6: 10ms 系统节拍
 *   每周期: 挡板速度计算(TIM2) + tim6_flag(挡板速度环)
 *   每10周期: 传送带速度计算(TIM4清零式) + 传送带PI + tick_100ms
 *==================================================================*/
void Init_TIM6(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_TimeBaseInitTypeDef t;
    TIM_TimeBaseStructInit(&t);
    t.TIM_Prescaler = 71;    /* 72MHz/72 = 1MHz */
    t.TIM_Period = 9999;     /* 10000计数 = 10ms */
    t.TIM_ClockDivision = TIM_CKD_DIV1;
    t.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM6, &t);

    TIM_ClearFlag(TIM6, TIM_FLAG_Update);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM6, ENABLE);
}

void TIM6_NVIC_Config(void)
{
    NVIC_InitTypeDef n;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    n.NVIC_IRQChannel = TIM6_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 0;
    n.NVIC_IRQChannelSubPriority = 3;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
}

/* 挡板编码器周期60000的模差值: 正确处理0/60000回绕 */
static int32_t enc_delta(uint16_t now, uint16_t last)
{
    int32_t d = (int32_t)now - (int32_t)last;
    if (d > 30000) d -= 60000;
    else if (d < -30000) d += 60000;
    return d;
}

void TIM6_IRQHandler(void)
{
    static uint16_t enc1_last = 0;
    static uint8_t div100 = 0;

    /* ---- 挡板速度: 每10ms, TIM2模60000差值, 2000计数/转, 方向自整定系数修正 ---- */
    uint16_t enc1 = TIM_GetCounter(TIM2);
    int32_t d1 = enc_delta(enc1, enc1_last);
    int32_t d1s = (motor[1].enc_dir >= 0) ? d1 : -d1;   /* 编码器方向修正 */
    motor[1].speed_rpm = (int16_t)(60 * 100 * (motor[1].dir == CW ? d1s : -d1s) / 2000);
    if (motor[1].speed_rpm < 0) motor[1].speed_rpm = 0;
    enc1_last = enc1;

    tim6_flag = 1;   /* 主循环执行挡板速度环 */

    /* ---- 传送带速度+PI: 每100ms(10分频) ---- */
    if (++div100 >= 10) {
        div100 = 0;

        /* Hall丢失保护: 运行且有速度指令时, 1s内无任何Hall边沿则切断输出 */
        {
            static uint32_t last_edges = 0;
            static uint8_t hall_lost_ticks = 0;
            if (motor1.run_flag == RUN && pid_target > 0) {
                if (hall_edges == last_edges) {
                    if (++hall_lost_ticks >= 10) {
                        stop_motor1();
                        motor1.run_flag = STOP;
                        hall_lost_ticks = 0;
                    }
                } else {
                    hall_lost_ticks = 0;
                }
            }
            last_edges = hall_edges;
        }

        u16 encoder_pos = TIM_GetCounter(TIM4);
        /* 编码器2000计数/转, 100ms窗口: rpm = cnt*60*10/2000 */
        if (motor1.dir == CW) {
            motor1.speed = (int16_t)(60 * 10 * encoder_pos / 2000);
        } else {
            motor1.speed = (int16_t)(60 * 10 * (encoder_pos - 65536) / 2000);
        }
        TIM_SetCounter(TIM4, 0);

        /* 传送带速度环 PI: u(n) = Kp*e + Ki*Sum(e)*T + Kd*de/T, T=0.1s */
        if (motor1.run_flag == RUN && pid_target > 0) {
            float T = 0.1f;
            int16_t e = (int16_t)pid_target - motor1.speed;

            float p = Kp * e;

            /* 积分分离 + 限幅 */
            if (e < 200 && e > -200) {
                integral += e;
                if (integral > 40000) integral = 40000;
                if (integral < -40000) integral = -40000;
            }
            float i = Ki * integral * T;

            float d = Kd * (e - e_prev) / T;

            float out = p + i + d;
            if (out < 0) out = 0;
            if (out > 4800) out = 4800;

            motor1.pwm_duty = (uint16_t)out;
            e_prev = e;
        } else {
            /* 停机时清零积分和微分历史, 并撤除PWM防止堵转 */
            e_prev = 0;
            integral = 0;
            motor1.pwm_duty = 0;
        }

        tick_100ms++;   /* 传送带应用层时基 */
    }

    TIM_ClearITPendingBit(TIM6, TIM_FLAG_Update);
}

/*==================================================================
 * 挡板电机: ADC2 注入采样(PC0/PC1 两相电流)
 *==================================================================*/
void ADC_Axis_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

    ADC_InitTypeDef a;
    ADC_StructInit(&a);
    a.ADC_Mode = ADC_Mode_Independent;
    a.ADC_ScanConvMode = DISABLE;
    a.ADC_ContinuousConvMode = DISABLE;
    a.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    a.ADC_DataAlign = ADC_DataAlign_Right;
    a.ADC_NbrOfChannel = 1;
    ADC_Init(ADC2, &a);

    ADC_InjectedSequencerLengthConfig(ADC2, 2);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_10, 1, ADC_SampleTime_239Cycles5);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_239Cycles5);
    ADC_ExternalTrigInjectedConvCmd(ADC2, DISABLE);

    ADC_Cmd(ADC2, ENABLE);
    ADC_ResetCalibration(ADC2);
    while (ADC_GetResetCalibrationStatus(ADC2));
    ADC_StartCalibration(ADC2);
    while (ADC_GetCalibrationStatus(ADC2));
}

/* 电流零点校准: 输出关断状态下采样16次取均值作为偏置(ADC原始码)
 * 需在使能驱动器与TIM8中断之前调用 */
void ADC_CurrentOffsetCalib(float *off_a, float *off_b)
{
    uint32_t sum_a = 0, sum_b = 0;
    int i;
    for (i = 0; i < 16; i++) {
        ADC_SoftwareStartInjectedConvCmd(ADC2, ENABLE);
        while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC));
        sum_a += ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
        sum_b += ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_2);
        ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);
    }
    *off_a = (float)(sum_a / 16);
    *off_b = (float)(sum_b / 16);
}

/* 挡板电流环: TIM8 更新中断内采样并执行FOC电流环
 * 采样值减去零点偏置并按 ADC_TO_AMP 标定为安培 */
void TIM8_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM8, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
        static uint8_t toggle = 0;
        toggle ^= 1;
        if (toggle) return;
        if (motor[1].state != MOTOR_RUN && motor[1].state != MOTOR_ALIGN) return;
        ADC_SoftwareStartInjectedConvCmd(ADC2, ENABLE);
        while (!ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC));
        int32_t raw_a = (int32_t)ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_1);
        int32_t raw_b = (int32_t)ADC_GetInjectedConversionValue(ADC2, ADC_InjectedChannel_2);
        ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);
        motor[1].ia = ((float)raw_a - motor[1].i_offset_a) * ADC_TO_AMP;
        motor[1].ib = ((float)raw_b - motor[1].i_offset_b) * ADC_TO_AMP;
        Motor_CurrentLoop(&motor[1]);
    }
}

void TIM8_NVIC_Config(void)
{
    NVIC_InitTypeDef n;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    n.NVIC_IRQChannel = TIM8_UP_IRQn;
    n.NVIC_IRQChannelPreemptionPriority = 0;
    n.NVIC_IRQChannelSubPriority = 0;
    n.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&n);
}
