#ifndef _SVPWM_TIM_H_
#define _SVPWM_TIM_H_

#include "stm32f10x.h"
#include "stm32f10x_tim.h"

extern u16 PWM_RESOLUTION;   /* SVPWM占空比满量程(=arr), 定义于svpwm_tim.c */

/* 挡板电机: FOC/SVPWM PWM输出(TIM8)与电流采样 */
void SVPWM_TIM8_Init(int arr, int psc);
void ADC_Axis_Init(void);
void ADC_CurrentOffsetCalib(float *off_a, float *off_b);   /* 电流零点校准(输出关断时采样) */

/* 传送带电机: 六步换相 TIM1 PWM 与 TIM3 Hall检测 */
void TIM1_PwmoutCNF_OnLibFunc(int arr, int psc);
void Time3_HalldetectCNF(void);

/* 编码器接口: TIM4=传送带, TIM2=挡板 */
void Encoder_Init_TIM4(void);
void Encoder_Init_TIM2(void);

/* 10ms 系统节拍: 挡板速度环 + 分频100ms传送带速度PI */
void Init_TIM6(void);

void TIM2_NVIC_Config(void);
void TIM3_NVIC_Config(void);
void TIM4_NVIC_Config(void);
void TIM6_NVIC_Config(void);
void TIM8_NVIC_Config(void);

#endif
