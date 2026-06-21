/*****************************************************************************************************
 * @file        key.h
 * @author      
 * @version     V1.0
 * @date        20260206
 * @brief       按键输入 驱动代码
 * @license     Copyright (c) 20260206, 中南大学机电工程学院
 *
 * 实验平台:正点原子STM32F1精英版开发板和ATK-PD6010B电机驱动板
 *
 * V1.0 20260206
 *
 *****************************************************************************************************/

#ifndef __KEY_H_
#define __KEY_H_

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"

#define KEY0        GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)     /* 读取KEY0引脚 */
#define KEY1        GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3)     /* 读取KEY1引脚 */
#define KEY0_PRES    1              /* KEY0按下 */
#define KEY1_PRES    2              /* KEY1按下 */

void key_init(void);                /* 按键初始化函数 */
u8 key_scan(u8 mode);     /* 按键扫描函数 */

#endif


















