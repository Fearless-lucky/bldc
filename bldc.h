#ifndef __BLDC_H_
#define __BLDC_H_

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"

/* 传送带电机(六步换相)状态与控制结构体 */
typedef struct {
    __IO u8    run_flag;       /* 运行标志 */
    __IO u8    locked_rotor;   /* 堵转检测 */
    __IO u8    step_sta;       /* 换相步状态 */
    __IO u8    hall_single_sta;/* 单霍尔状态 */
    __IO u8    hall_sta_edge;  /* 霍尔状态边沿 */
    __IO u8    step_last;      /* 上次步状态 */
    __IO u8    dir;            /* 电机转向 */
    __IO int32_t    pos;       /* 位置累计 */
    __IO int16_t    speed;     /* 当前转速 */
    __IO int16_t    current;   /* 相电流 */
    __IO uint16_t   pwm_duty;  /* PWM占空比 */
    __IO uint32_t   hall_keep_t;   /* 霍尔保持时间 */
    __IO uint32_t   hall_pul_num;  /* 霍尔脉冲计数 */
    __IO uint32_t   lock_time;     /* 堵转计时 */
} BLDC_Motor_State;

extern BLDC_Motor_State motor1;
extern float Kp, Ki, Kd;         /* PID 参数 */
extern float pid_target;         /* PID 目标转速(RPM) */

/* 传送带驱动器使能(SD)引脚: PB11 */
#define BLDC_DRIVER_SD_PORT   GPIOB
#define BLDC_DRIVER_SD_PIN    GPIO_Pin_11
#define BLDC_Driver_Turnon    GPIO_WriteBit(BLDC_DRIVER_SD_PORT, BLDC_DRIVER_SD_PIN, Bit_SET)
#define BLDC_Driver_Turnoff   GPIO_WriteBit(BLDC_DRIVER_SD_PORT, BLDC_DRIVER_SD_PIN, Bit_RESET)

/* 上桥PWM下桥常开换相模式 */
#define UP_PWM_DWN_ON

/* 与 svpwm.h 共用的方向/错误宏, 防重复定义 */
#ifndef CCW
#define CCW           (1)
#endif
#ifndef CW
#define CW            (2)
#endif
#ifndef HALL_ERROR
#define HALL_ERROR    (0xF0)
#endif

#define RUN           (1)
#define STOP          (0)

void bldc_gpio_init(void);             /* 六步换相电机GPIO初始化 */
void stop_motor1(void);                /* 停止电机 */
void start_motor1(void);               /* 启动电机 */
u8 hallsensor_get_state(void);         /* 读取霍尔状态 */
void UVW_6_Step_Ponoff(void);          /* 六步换相执行 */

typedef void(*pctr) (void);            /* 换相函数指针类型 */
extern pctr pfunclist[6];              /* 换相函数指针表 */

/* 六步换相函数 */
void U_up_V_dwn(void);
void U_up_W_dwn(void);
void V_up_W_dwn(void);
void V_up_U_dwn(void);
void W_up_U_dwn(void);
void W_up_V_dwn(void);

#endif
