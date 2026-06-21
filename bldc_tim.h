/*****************************************************************************************************
 * @file        bldc_tim.h
 * @author
 * @version     V1.1
 * @date        2025-12-28
 * @brief       定时器 驱动程序
 * @license     Copyright (c) 20260206, 湖南大学机电工程学院
 *
 * 实验平台:正点原子STM32F1精英开发板ATK-PD6010B电机驱动板
 *
 * V1.0 20260206
 * V1.1 20260604  新增 g_tim6_tick 用于主循环 PID 同步
 *
 ****************************************************************************************************/

#ifndef _BLDC_TIM_H_
#define _BLDC_TIM_H_

#include "stm32f10x.h"
#include "stm32f10x_tim.h"

void TIM1_PwmoutCNF_OnLibFunc(int arr,int psc);
void UVW_6_Step_Ponoff(void);
void Encoder_Init_TIM4(void);
void Init_TIM6(void);
void TIM6_NVIC_Config(void);
void Time3_HalldetectCNF(void);
void TIM1_NVIC_Config(void);
void TIM3_NVIC_Config(void);

extern volatile uint32_t g_tim6_tick;  /* TIM6 中断计数器，每 100ms 递增，供主循环 PID 同步 */

#endif
