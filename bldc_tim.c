/**
 ****************************************************************************************************
 * @file        bldc_tim.c
 * @author
 * @version     V1.1
 * @date        2025-12-28
 * @brief       定时器 驱动程序
 * @license     湖南机电工程学院
 ****************************************************************************************************
 *
 * 实验平台:正点原子STM32F1精英开发板ATK-PD6010B电机驱动板
 * HALL传感器通过TIM3_CH1(PA6),TIM3_CH2(PA7)和TIM3_CH3(PB0)接入
 * SHUTDOWN_PIN为PB12(TIM1_BKIN 默认复PB12)
 * PWM信号通过TIM1_CH1(PA8)、TIM1_CH2(PA9)、TIM1_CH3(PA10)输出
 * U、V、W相下桥臂分别由stm32的TIM1_CH1N(PB13)、TIM1_CH2N(PB14)、TIM1_CH3N(PB15)输出
 *
 * V1.0 20251228
 * V1.1 20260604  新增 g_tim6_tick 用于主循环 PID 同步
 *
 ****************************************************************************************************
 */

#include "stm32f10x_gpio.h"
#include "bldc_tim.h"
#include "bldc.h"
#include "misc.h"

/************************* 电机运行状态相关结构体 **********************************************/

extern BLDC_Motor_State motor1;

volatile uint32_t g_tim6_tick = 0;   /* TIM6 中断计数器，每 100ms 递增，供主循环 PID 同步 */

/******************************************************************************************
 * @brief       高级定时器TIMX PWM输出初始化函数
 * @note
 *              高级定时器的时钟来源APB2, 而PCLK2 = 72Mhz, 没有经过PPRE2再分频, 因此
 *              高级定时器时钟 = 72Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值
 * @param       psc: 时钟预分频数
 * @retval      无
 */

void TIM1_PwmoutCNF_OnLibFunc(int arr, int psc)
{
	//TIM1 PWM 两路互补初始化;arr 设自动重装值;psc 设时钟预分频数.

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE); //TIM1时钟使能

	//初始化 TIM1 时间基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = arr;                       // 设置在自动重装载周期值
	TIM_TimeBaseStructure.TIM_Prescaler =psc;                    //设置预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 设置时钟分割 :TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;                  /* 初始重复计数*/
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);              // 初始化 TIMx

	TIM_SelectSlaveMode(TIM1,TIM_SlaveMode_Trigger); //选择从处理模式
	TIM_SelectInputTrigger(TIM1,TIM_TS_ITR2);        //选择TIM3的TRGO作为触发源自动换向COM

	//选择TIM 更新或COM事件，通过设置TIMx_CR2的位CCUS为1，可以做到：在COM位变成有效时(TRGI的一个上升沿)重载预装载.
	TIM_SelectCOM(TIM1, ENABLE);

//设置捕获/比较预装载控制位TIMx_CCPC,使OCxE、OCxNE、OCxM位可以通过预装载得到
	TIM_CCPreloadControl(TIM1, ENABLE);

	TIM_ARRPreloadConfig(TIM1,DISABLE);//设定为影子寄存器的寄存器

	//初始化 TIM1_CH1-CH3 PWM模式初始化
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   // 选择 PWM 模式 1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能，设置CCER的位CC1E
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; // 互补比较输出使能，设置CCER的位CC1NE
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性高，设置CCER的位CC1P=0
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse=0;                               //此值装入捕获/比较寄存器作为比较值
	TIM_OC1Init(TIM1, &TIM_OCInitStructure); //初始化外设 TIM1 OC1
	TIM_OC2Init(TIM1, &TIM_OCInitStructure); //初始化外设 TIM1 OC2
	TIM_OC3Init(TIM1, &TIM_OCInitStructure); //初始化外设 TIM1 OC3

	TIM_OC1FastConfig(TIM1,DISABLE);
	TIM_OC2FastConfig(TIM1,DISABLE);
	TIM_OC3FastConfig(TIM1,DISABLE);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); // 使能预装载寄存器
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable); // 使能预装载寄存器
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); // 使能预装载寄存器

	TIM_BDTRInitTypeDef TIM_BDTRInitStructure;     /* 刹车和死区参数设置 */
	TIM_BDTRStructInit(&TIM_BDTRInitStructure);
	TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;

	TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime = 11;  // 死区时间值
	TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;    //刹车使能
	TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	TIM_ClearFlag(TIM1, TIM_FLAG_Update |TIM_FLAG_COM| TIM_FLAG_Break);   // 清中断标志
	TIM_Cmd(TIM1, ENABLE);       // 使能 TIM1
}


void UVW_6_Step_Ponoff(void)
{
  if(motor1.run_flag == RUN)
    {
      if(motor1.dir == CCW)                                     /* 反转 */
      {
        motor1.step_sta= hallsensor_get_state();     /* 顺序6,2,3,1,5,4 motor1.step_sta*/
       }
      else                                                            /* 正转 */
       {
         motor1.step_sta = 7 - hallsensor_get_state(); /* 顺序5,1,3,2,6,4 即用7减去读值pfunclist_m1对应的顺序 实际霍尔值为：2,6,4,5,1,3*/
        }

      if((motor1.step_sta <= 6)&&(motor1.step_sta >= 1))/* 判断霍尔读值是否合法 */
        {
          pfunclist[motor1.step_sta-1]();                   /* 通过函数指针表调对应的函数指针 */
        }
      else                                                            /* 霍尔传感器故障、接触不良或电源断开 */
        {
          stop_motor1();
          motor1.run_flag = STOP;
        }
    }
}


//TIM4用于对光电编码器进行计数

void Encoder_Init_TIM4(void)
 {  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器2的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PB端口时钟
	  GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;    //端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);                          //根据设定参数初始化PB6和PB7

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器
    TIM_TimeBaseStructure.TIM_Period = 65535; //设定计数器自动重装值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频，不分频
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);//使用编码器模式3
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    TIM_SetCounter(TIM4,0);    //Reset counter
    TIM_Cmd(TIM4, ENABLE);
 }

 //TIM6用于定时读取为增量编码器模式的TIM4的计数值以计算速度，本实验定时0.1秒，即100ms
 void Init_TIM6(void)
 {
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);//使能定时器6的时钟

		 TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
     TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
     TIM_TimeBaseStructure.TIM_Prescaler = 0x99; // 预分频器
     TIM_TimeBaseStructure.TIM_Period = 35999; //设定计数器自动重装值
     TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频，不分频
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数
     TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

     TIM_ClearFlag(TIM6, TIM_FLAG_Update);//清除TIM的更新标志位
     TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
     TIM_Cmd(TIM6, ENABLE);
 }

 void TIM6_NVIC_Config(void)
 {
	 NVIC_InitTypeDef NVIC_Struct;
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	 NVIC_Struct.NVIC_IRQChannel=TIM6_IRQn;
	 NVIC_Struct.NVIC_IRQChannelPreemptionPriority=0;
	 NVIC_Struct.NVIC_IRQChannelSubPriority=3;
	 NVIC_Struct.NVIC_IRQChannelCmd=ENABLE;
	 NVIC_Init(&NVIC_Struct);
 }

void TIM6_IRQHandler(void)
{
	u16 encoder_pos = TIM_GetCounter(TIM4);
//0.1s采集1次，每分钟的转速
	if(motor1.dir==CW)
  motor1.speed=(int16_t)(60*10*encoder_pos/2000); //编码器线数500P/r，编码器模式3使得计数脉冲数为2000P/r.
	else motor1.speed=(int16_t) (60*10*(encoder_pos-65536)/2000);
	TIM_SetCounter(TIM4,0);    //Reset counter
	TIM_ClearITPendingBit(TIM6,TIM_FLAG_Update);

	g_tim6_tick++;   /* PID 时间基准：每 100ms 递增一次 */
}

//TIM3用于检测霍尔传感器的变化，使用定时器的计数值复位为零，同时设置CCR4为相应值
//使CNT复位为0的同时通过TRGO触发TIM1的COM事件。
void Time3_HalldetectCNF(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //使能定时器 3 时钟

	//初始化TIM3	时间基单元
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 65535;              // 设置在自动重装载周期值
	TIM_TimeBaseStructure.TIM_Prescaler =0;                //设置预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;           // 设置时钟分割 :TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM 向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);              // 初始化 TIMx

	TIM_SelectHallSensor(TIM3, ENABLE);  //使能霍尔传感器
	TIM_SelectSlaveMode(TIM3,TIM_SlaveMode_Reset);  //选择复位模式
	TIM_SelectInputTrigger(TIM3,TIM_TS_TI1F_ED);  //选择触发源为TI1F_ED

	//设置CC1,目的不捕获不比较只对该引脚低通滤波，防止电机换相
	TIM_ICInitTypeDef TIM_ICInitStructure1;
	TIM_ICInitStructure1.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStructure1.TIM_ICFilter=4;                   //滤波
	TIM_ICInitStructure1.TIM_ICPrescaler=TIM_ICPSC_DIV1;  //不分频
	TIM_ICInit(TIM3,&TIM_ICInitStructure1);

	//设置CC4并初始化TIM3 Channel4 PWM 模式
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;   // 选择PWM模式 2
	TIM_OCInitStructure.TIM_Pulse=10;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // 比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出高电平为有效电平
	TIM_OC4Init(TIM3, &TIM_OCInitStructure); //初始化外设 TIM3 OC4

	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_OC4Ref); //选择OC4Ref为触发输出信号来生成COM信号
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable); // 使能预装载寄存器
	TIM_ITConfig(TIM3,TIM_IT_Trigger, ENABLE);
	TIM_Cmd(TIM3,ENABLE);    //使能定时器3
}

void TIM3_NVIC_Config(void)
 {
	 NVIC_InitTypeDef NVIC_Struct;
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	 NVIC_Struct.NVIC_IRQChannel=TIM3_IRQn;
	 NVIC_Struct.NVIC_IRQChannelPreemptionPriority=0;
	 NVIC_Struct.NVIC_IRQChannelSubPriority=3;
	 NVIC_Struct.NVIC_IRQChannelCmd=ENABLE;
	 NVIC_Init(&NVIC_Struct);
 }

void TIM3_IRQHandler(void)
{
	UVW_6_Step_Ponoff();
	TIM_ClearITPendingBit(TIM3,TIM_FLAG_Trigger);

}
