/*****************************************************************************************************
 * @file        key.c
 * @author      
 * @version     V1.0
 * @date        2026-02-06
 * @brief       按键输入 驱动代码
 * @license     Copyright (c) 20260206, 中南大学机电工程学院
 *
 * 实验平台:正点原子STM32F1精英版开发板和ATK-PD6010B电机驱动板
 * 
 * V1.0 20260206
 *
 ****************************************************************************************************
 */

#include "key.h"
#include "stm32f10x_gpio.h"
#include "delay.h"

/** @brief       按键初始化函数
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE); 
		GPIO_InitTypeDef gpio_init_struct;                     /* GPIO配置参数存储变量 */
    gpio_init_struct.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;     /* KEY0和KEY1引脚设置,上拉输入 */
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;            /* 输入 */
    gpio_init_struct.GPIO_Speed = GPIO_Speed_2MHz;         /* 高速 */
    GPIO_Init(GPIOE, &gpio_init_struct);                   /* KEY0引脚模式设置,上拉输入 */
}

/** @brief    按键扫描函数
 * @note      该函数有响应优先级(同时按下多个按键): WK_UP > KEY2 > KEY1 > KEY0!!
 * @param     mode:0 / 1, 具体含义如下:
 * @arg       0,  不支持连续按(当按键按下不放时, 只有第一次调用会返回键值,
 *                  必须松开以后, 再次按下才会返回其他键值)
 * @arg       1,  支持连续按(当按键按下不放时, 每次调用该函数都会返回键值)
 * @retval    键值, 定义如下:
 *            KEY0_PRES, 1, KEY0按下
 *            KEY1_PRES, 2, KEY1按下
 */
u8 key_scan(u8 mode)
{
    static u8 key_up = 1;  /* 按键按松开标志 */
    u8 keyval = 0;
    if (mode) key_up = 1;       /* 支持连按 */
    if (key_up && (KEY0 == 0 || KEY1 == 0 ))  /* 按键松开标志为1, 且有任意一个按键按下了 */
    {
      delay_ms(10);           /* 去抖动 */
      key_up = 0;
      if (KEY0 == 0)  keyval = KEY0_PRES;   //宏定义 KEY0读引脚状态函数 KEY0_PRES=1
      if (KEY1 == 0)  keyval = KEY1_PRES;    //宏定义 KEY0读引脚状态函数  KEY1_PRES=2 
    }
    else if (KEY0 == 1 && KEY1 == 1 )    key_up = 1;     /* 没有任何按键按下, 标记按键松开 */    
    return keyval;              /* 返回键值 */
}




















