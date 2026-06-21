/*****************************************************************************************************
 * @file        bldc.h
 * @author      中南大学机电工程学院
 * @version     V1.0
 * @date        2026-2-6
 * @brief       BLDC 驱动代码
 * @license     中南大学机电工程学院
 *
 * 实验平台:正点原子STM32F1精英版开发板和ATK-PD6010B电机驱动板
 *
 * V1.0 20260206
 *****************************************************************************************************/
 
#ifndef __BLDC_H_
#define __BLDC_H_

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"

/******************************** 电机运行状态与参数结构体 ********************************************/
typedef struct {
    __IO u8    run_flag;       /* 运行标志 */
    __IO u8    locked_rotor;   /* 堵转标记 */
    __IO u8    step_sta;       /* 本次霍尔状态 */
    __IO u8    hall_single_sta;/* 单个霍尔状态 */
    __IO u8    hall_sta_edge;  /* 单个霍尔状态跳变 */
    __IO u8    step_last;      /* 上次霍尔状态 */
    __IO u8    dir;            /* 电机旋转方向 */
    __IO int32_t    pos;            /* 电机位置 */
    __IO int16_t    speed;          /* 电机速度 */
    __IO int16_t    current;        /* 电机电流 */
    __IO uint16_t   pwm_duty;       /* 电机占空比 */
    __IO uint32_t   hall_keep_t;    /* 霍尔保持时间 */
    __IO uint32_t   hall_pul_num;   /* 霍尔传感器脉冲数 */
    __IO uint32_t   lock_time;      /* 电机堵转时间 */

} BLDC_Motor_State;

/************************************* 电机编号 **************************************************/

extern BLDC_Motor_State motor1;

#define Driver_Turnon                      GPIO_WriteBit(GPIOB,GPIO_Pin_11,Bit_SET);   /* 使能半桥芯片的SD引脚 */
#define Driver_Turnoff                     GPIO_WriteBit(GPIOB,GPIO_Pin_11,Bit_RESET); /* 失能半桥芯片的SD引脚 */

/****************************************** 电机相关系数 **************************************************/

#define MAX_PWM_DUTY (((10000) - 1)*0.96)        /* 最大占空比限制 */

#define UP_PWM_DWN_ON     //定义上桥PWM，下桥开工作模式

#define CCW           (1)                 /* 逆时针 */
#define CW            (2)                 /* 顺时针 */
#define HALL_ERROR    (0xF0)              /* 霍尔错误标志 */
#define RUN           (1)                 /* 电机运动标志 */
#define STOP          (0)                 /* 电机停机标志 */

/************************************* 函数声明 *****************************************/

void bldc_gpio_init(void);             /* BLDC初始化 */
void stop_motor1(void);                                 /* 停机 */
void start_motor1(void);                                /* 启动电机 */
u8 hallsensor_get_state(void);        /* 获取霍尔状态 */

typedef void(*pctr) (void);    //定义函数指针类型
extern pctr pfunclist[6];     // 六步换相函数指针数组

/*  六步换相函数 */
void U_up_V_dwn(void);
void U_up_W_dwn(void);
void V_up_W_dwn(void);
void V_up_U_dwn(void);
void W_up_U_dwn(void);
void W_up_V_dwn(void);

#endif
