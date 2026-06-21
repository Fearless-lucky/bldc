/*****************************************************************************************************
 * @file        bldc.c
 * @author      
 * @version     V1.0
 * @date        2026-02-06
 * @brief       BLDC 驱动代码
 * @license     
 *
 * 实验平台:正点原子STM32F1精英版开发板和ATK-PD6010B电机驱动板
 * HALL传感器通过TIM3_CH1(PA6),TIM3_CH2(PA7)和TIM3_CH3(PB0)输入
 * TIM1_BKIN:默认复用PB12
 * PWM信号通过TIM1_CH1(PA8)，TIM1_CH2(PA9)和TIM1_CH3(PA10)输出
 * U，V和W相的下半桥分别由stm32的TIM1_CH1N(PB13)，TIM1_CH2N(PB14)和TIM1_CH3N(PB15)控制
 * 
 * V1.0 20260206
 *
 ****************************************************************************************************/
 
#include "bldc.h"
#include "bldc_tim.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "delay.h"

BLDC_Motor_State motor1 = {STOP,0,0,0,0,0,CCW,0,0,0,0,0,0,0};   /* 电机结构体初始值 */

/** @brief       无刷电机初始化，包括定时器，霍尔接口以及SD引脚初始化
 * @param       arr: 自动重装值
 * @param       psc: 时钟预分频数
 * @retval      无
 */

/***************************************** 霍尔传感器接口 *************************************************/

#define HALL_U_TIM3_CH1_PIN       GPIO_Pin_6     /* U 默认复用PA6*/
#define HALL_U_TIM3_CH1_GPIO      GPIOA

#define HALL_V_TIM3_CH2_PIN       GPIO_Pin_7     /* V 默认复用PA7*/
#define HALL_V_TIM3_CH2_GPIO      GPIOA

#define HALL_W_TIM3_CH3_PIN       GPIO_Pin_0     /* W 默认复用PB0*/
#define HALL_W_TIM3_CH3_GPIO      GPIOB

extern u16 Max_arr;

void bldc_gpio_init(void)
{       
/* 霍尔传感器输入PA6、PA7、PB0和急刹引脚PB12、上臂GPIOA.8-10和下臂GPIOB.13-15引脚的时钟使能 */

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);  
	
  GPIO_InitTypeDef gpio_init_struct;
	
/************************************ 霍尔接口初始化 ******************************************/	

  /* 霍尔通道1(PA6)、霍尔通道2(PA7) 和霍尔通道3(PB0)引脚初始化为输入上拉 */
  gpio_init_struct.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;  //PA6
  gpio_init_struct.GPIO_Mode =GPIO_Mode_IPU; 
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &gpio_init_struct);
  gpio_init_struct.GPIO_Pin = GPIO_Pin_0;    //PB0
  GPIO_Init(GPIOB, &gpio_init_struct);
		
/*********************** 驱动板CDRL_SD控制引脚PB11初始化为推挽输出*********************************/

  gpio_init_struct.GPIO_Pin = GPIO_Pin_11;      //PB11
  gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
  gpio_init_struct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &gpio_init_struct); 

/*********************** TIM1刹车引脚PB12初始化为推挽输出*********************************************/
	gpio_init_struct.GPIO_Pin = GPIO_Pin_12;      //PB12
  gpio_init_struct.GPIO_Mode =GPIO_Mode_IPD;    //与TIM_BDTRConfig()设置的TIM_BreakPolarity为高地有关
	gpio_init_struct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &gpio_init_struct); 

/********************* 设置上臂GPIOA.8-10引脚（TIM_CH1-CH3）为复用输出功能 ***********************/ 	

	gpio_init_struct.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;   
	gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;       // 复用推挽输出
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio_init_struct);               //初始化 GPIO
	
/**** 设置下臂GPIOB.13-15引脚（TIM_CH1N-CH3N)为推挽输出功能(COM触发换相，使用TIM1的互补输出) ****/

	gpio_init_struct.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;             
	gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;            //  软件换相控制时，初始化为GPIO_Mode_Out_PP;   
	gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &gpio_init_struct);               //初始化 GPIO
}

/** @brief   获取霍尔传感器引脚状态
 * @param   电机编号motor_id
 * @retval  霍尔传感器引脚状态
 */
u8 hallsensor_get_state(void)
{
  u8 state  = 0;    
  if(GPIO_ReadInputDataBit(HALL_U_TIM3_CH1_GPIO,HALL_U_TIM3_CH1_PIN) != Bit_RESET)  /* 霍尔HA状态获取 */
   {
     state |= 0x01U;
    }
  if(GPIO_ReadInputDataBit(HALL_V_TIM3_CH2_GPIO,HALL_V_TIM3_CH2_PIN) != Bit_RESET)  /* 霍尔HB状态获取 */
   {
    state |= 0x02U;
   }
  if(GPIO_ReadInputDataBit(HALL_W_TIM3_CH3_GPIO,HALL_W_TIM3_CH3_PIN) != Bit_RESET)  /* 霍尔HC状态获取 */
   {
     state |= 0x04U;
    }    
  return state;
}

/************************************* BLDC相关控制函数 *************************************
* @brief  关闭电机运转
* @param  无
* @retval 无
*/
void stop_motor1(void)
{  
  Driver_Turnoff;     /* PB11=0，则CTRL_SD=0，关闭驱动板三相H桥芯片输出 */
  TIM_CtrlPWMOutputs(TIM1,DISABLE);    /* ENABLE:MOE=1,使能PWM输出;DISABLE:MOE=0,关闭PWM输出 */
	
/* 因TIM1采用PWM1输出，CNT<CCRx时为OCxREF=1,CNT>=CCRx时为OCxREF=0,所以下面设置使上桥臂全部关断 */
  TIM1->CCR2 = 0;
  TIM1->CCR1 = 0;
  TIM1->CCR3 = 0;
	
//下面设置使下桥臂全部关断 */	
	
	TIM1->CCER &=~(0x1<<2);     //CC1NE=0，输出禁止
	TIM1->CCER &=~(0x1<<6);     //CC2NE=0，输出禁止
	TIM1->CCER &=~(0x1<<10);       //CC3NE=0,输出禁止;
	
//下面使用软件使用通用I/O控制下桥臂关断	
//  GPIO_WriteBit(M1_DWN_SIDE_U_PORT,M1_DWN_SIDE_U_PIN,Bit_RESET);
//  GPIO_WriteBit(M1_DWN_SIDE_V_PORT,M1_DWN_SIDE_V_PIN,Bit_RESET);
//  GPIO_WriteBit(M1_DWN_SIDE_W_PORT,M1_DWN_SIDE_W_PIN,Bit_RESET);
}

/** @brief  开启电机运转
 * @param  无
 * @retval 无
 */
void start_motor1(void)
{
  Driver_Turnon;   /* PB11=1，则CTRL_SD=1，使能驱动板三相H桥芯片输出 */
	TIM_CtrlPWMOutputs(TIM1,ENABLE);  /* ENABLE:TIMx_BDTR的MOE=1,使能PWM输出;DISABLE:MOE=0,关闭PWM输出 */
}

/*************************** 上下桥臂的导通情况，共6种，也称为6步换向（接口一） ****************************/

/*  六步换向函数指针数组 */
pctr pfunclist[6] ={&U_up_W_dwn,&V_up_U_dwn,&V_up_W_dwn,&W_up_V_dwn,&U_up_V_dwn,&W_up_U_dwn};
//HCHBHC                 110       101          100        011          010         001

/** @brief  U相上桥臂导通，V相下桥臂导通
 * @param  无
 * @retval 无
 */
void U_up_V_dwn(void)
{
    TIM1->CCR1 = motor1.pwm_duty;     /* U相上桥臂PWM */
    TIM1->CCR2 = 0;                         //V相输出低电平
    TIM1->CCR3 = 0;                        //W相输出低电平

		TIM1->CCER &=~(0x1<<3);     //CC1NP=0;
		TIM1->CCER |=0x1;           //CC1E=1
		TIM1->CCER &=~(0x1<<2);     //CC1NE=0
	
		TIM1->CCER |=0x1<<7;     		//CC2NP=1;
		TIM1->CCER &=~(0x1<<4);     //CC2E=0    //PWM1,OCNP高电平有效，CCR2=0,，故OC2REF=0；
		TIM1->CCER |=0x1<<6;        //CC2NE=1，OC2=OC2REF XOR CC2NP 此时OC2=1
	
		TIM1->CCER &=~(0x1<<11);     //CC3NP=0;	
		TIM1->CCER &=~(0x1<<8);     //CC3E=0,输出禁止;
		TIM1->CCER &=~(0x1<<10);       //CC3NE=0,输出禁止;
	
//    GPIO_WriteBit(M1_DWN_SIDE_V_PORT,M1_DWN_SIDE_V_PIN,Bit_SET);   /* V相下桥臂导通 */
//    GPIO_WriteBit(M1_DWN_SIDE_U_PORT,M1_DWN_SIDE_U_PIN,Bit_RESET); /* U相下桥臂关闭 */
//    GPIO_WriteBit(M1_DWN_SIDE_W_PORT,M1_DWN_SIDE_W_PIN,Bit_RESET); /* W相下桥臂关闭 */	
	
}

/** @brief  U相上桥臂导通，W相下桥臂导通
 * @param  无
 * @retval 无
 */
void U_up_W_dwn(void)
{
    TIM1->CCR1 = motor1.pwm_duty;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

		TIM1->CCER &=~(0x1<<3);     //CC1NP=0;	
		TIM1->CCER |=0x1;           //CC1E=1
		TIM1->CCER &=~(0x1<<2);     //CC1NE=0,OC1N=CC1NP
	
		TIM1->CCER &=~(0x1<<7);     //CC2NP=0;
		TIM1->CCER &=~(0x1<<4);     //CC2E=0,输出禁止
		TIM1->CCER &=~(0x1<<6);     //CC2NE=0，输出禁止

		TIM1->CCER |=0x1<<11;     //CC3NP=1;	
		TIM1->CCER &=~(0x1<<8);     //CC3E=0,OC3=OC3P;
		TIM1->CCER |=0x1<<10;    //CC3NE=1,OC3N=OC3REF XOR CC3NP;
	
//    GPIO_WriteBit(M1_DWN_SIDE_W_PORT,M1_DWN_SIDE_W_PIN,Bit_SET);
//    GPIO_WriteBit(M1_DWN_SIDE_U_PORT,M1_DWN_SIDE_U_PIN,Bit_RESET);
//    GPIO_WriteBit(M1_DWN_SIDE_V_PORT,M1_DWN_SIDE_V_PIN,Bit_RESET);
}

/** @brief  V相上桥臂导通，W相下桥臂导通
 * @param  无
 * @retval 无
 */
void V_up_W_dwn(void)
{
    TIM1->CCR1=0;
    TIM1->CCR2 = motor1.pwm_duty;
    TIM1->CCR3=0;
	
		TIM1->CCER &=~(0x1<<3);     //CC1NP=0;	
		TIM1->CCER &=~(0x1);         //CC1E=0，输出禁止
		TIM1->CCER &=~(0x1<<2);     //CC1NE=0,输出禁止
		
		TIM1->CCER &=~(0x1<<7);     //CC2NP=0;
		TIM1->CCER |=0x1<<4;        //CC2E=1
		TIM1->CCER &=~(0x1<<6);     //CC2NE=0，OC2N=CC2NP
		
		TIM1->CCER |=0x1<<11;        //CC3NP=1;
		TIM1->CCER &=~(0x1<<8);     //CC3E=0,OC3=CC3P=0;
		TIM1->CCER |=0x1<<10;       //CC3NE=1,OC3N=OC3REF XOR CC3NP;
	
//    GPIO_WriteBit(M1_DWN_SIDE_W_PORT,M1_DWN_SIDE_W_PIN,Bit_SET);
//    GPIO_WriteBit(M1_DWN_SIDE_U_PORT,M1_DWN_SIDE_U_PIN,Bit_RESET);
//    GPIO_WriteBit(M1_DWN_SIDE_V_PORT,M1_DWN_SIDE_V_PIN,Bit_RESET);
}

/** @brief  V相上桥臂导通，U相下桥臂导通
 * @param  无
 * @retval 无
 */
void V_up_U_dwn(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = motor1.pwm_duty;
    TIM1->CCR3 = 0;
	
		TIM1->CCER |=0x1<<3;         //CC1NP=1;
		TIM1->CCER &=~(0x1);         //CC1E=0，输出禁止
		TIM1->CCER |=0x1<<2;         //CC1NE=1,OC1N=OC1REF XOR CC1NP;

		TIM1->CCER &=~(0x1<<7);      //CC2NP=0;	
		TIM1->CCER |=0x1<<4;         //CC2E=1
		TIM1->CCER &=~(0x1<<6);      //CC2NE=0，OC2N=CC2NP
	
		TIM1->CCER &=~(0x1<<11);     //CC3NP=0;
		TIM1->CCER &=~(0x1<<8);      //CC3E=0,OC3=OC3P=0;
		TIM1->CCER &=~(0x1<<10);     //CC3NE=0,OC3N=OC3NP(0)=0;
	
//    GPIO_WriteBit(M1_DWN_SIDE_U_PORT,M1_DWN_SIDE_U_PIN,Bit_SET);
//    GPIO_WriteBit(M1_DWN_SIDE_V_PORT,M1_DWN_SIDE_V_PIN,Bit_RESET);
//    GPIO_WriteBit(M1_DWN_SIDE_W_PORT,M1_DWN_SIDE_W_PIN,Bit_RESET);
}

/* @brief  W相上桥臂导通，U相下桥臂导通
 * @param  无
 * @retval 无
 */
void W_up_U_dwn(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = motor1.pwm_duty;
	
		TIM1->CCER |=0x1<<3;          //CC1NP=1;
		TIM1->CCER &=~(0x1);          //CC1E=0，输出禁止
		TIM1->CCER |=0x1<<2;          //CC1NE=1,OC1N=OC1REF XOR CC1NP;
	
		TIM1->CCER &=~(0x1<<7);       //CC1NP=0;
		TIM1->CCER &=~(0x1<<4);       //CC2E=0，输出禁止
		TIM1->CCER &=~(0x1<<6);       //CC2NE=0，输出禁止

		TIM1->CCER &=~(0x1<<11);      //CC3NP=0;	
		TIM1->CCER |=0x1<<8 ;         //CC3E=1,OC3=OC3REF XOR CC3P;
		TIM1->CCER &=~(0x1<<10);      //CC3NE=0,OC3N=CC3NP;
	
//    GPIO_WriteBit(M1_DWN_SIDE_U_PORT,M1_DWN_SIDE_U_PIN,Bit_SET);
//    GPIO_WriteBit(M1_DWN_SIDE_V_PORT,M1_DWN_SIDE_V_PIN,Bit_RESET);
//    GPIO_WriteBit(M1_DWN_SIDE_W_PORT,M1_DWN_SIDE_W_PIN,Bit_RESET);
}

/** @brief  W相上桥臂导通，V相下桥臂导通
 * @param  无
 * @retval 无
 */
void W_up_V_dwn(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = motor1.pwm_duty;	

/*下面3x3句TIM1的互补输出通道控制下臂MOS管的开闭，实现电机的换相，利用COM事件触发可实现同步换相
	此时TIM1初始化时，GPIO初始化为GPIO_Mode_Out_PP;且使能互补输出，即 
	TIM_OCInitStructure.TIM_OutputNState = Enable;*/	
	
		TIM1->CCER &=~(0x1<<3);      //CC1NP=0;
		TIM1->CCER &=~(0x1);         //CC1E=0，输出禁止
		TIM1->CCER &=~(0x1<<2);      //CC1NE=0,输出禁止;

		TIM1->CCER |=0x1<<7;     		//CC2NP=1;	
		TIM1->CCER &=~(0x1<<4);     //CC2E=0，
		TIM1->CCER |=0x1<<6;        //CC2NE=1，OC2N=OC2REF XOR CC2NP;
	
		TIM1->CCER &=~(0x1<<11);     //CC3NP=0;
		TIM1->CCER |=0x1<<8 ;         //CC3E=1,OC3=OC3REF XOR CC3P;
		TIM1->CCER &=~(0x1<<10);       //CC3NE=0,OC3N=CC3NP;

/*下面三句用软件控制下臂MOS管开闭，实现电机换相，但不能同步换相	如用软件控制下臂MOS管开闭，则TIM1初始化时，
   GPIO初始化为GPIO_Mode_Out_PP;且关闭互补输出，	TIM_OCInitStructure.TIM_OutputNState = Disable;*/
//    GPIO_WriteBit(M1_DWN_SIDE_V_PORT,M1_DWN_SIDE_V_PIN,Bit_SET);
//    GPIO_WriteBit(M1_DWN_SIDE_U_PORT,M1_DWN_SIDE_U_PIN,Bit_RESET);
//    GPIO_WriteBit(M1_DWN_SIDE_W_PORT,M1_DWN_SIDE_W_PIN,Bit_RESET);
}
