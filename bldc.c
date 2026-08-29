/* bldc.c - 传送带电机 Hall 传感器六步换相
 * 硬件连接:
 *   Hall U/V/W: TIM3_CH1(PA6), TIM3_CH2(PA7), TIM3_CH3(PB0)
 *   PWM 上桥:   TIM1_CH1(PA8), TIM1_CH2(PA9), TIM1_CH3(PA10)
 *   PWM 下桥:   TIM1_CH1N(PB13), TIM1_CH2N(PB14), TIM1_CH3N(PB15)
 *   驱动器 SD:  PB11, 刹车输入 BKIN: PB12(TIM1_BKIN)
 */
#include "bldc.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

BLDC_Motor_State motor1 = {STOP,0,0,0,0,0,CCW,0,0,0,0,0,0,0};

float Kp = 2.0f, Ki = 0.2f, Kd = 0.0f;   /* PID 参数 */
float pid_target = 350;                   /* 目标转速 RPM */

/* Hall 传感器引脚定义 */
#define HALL_U_PIN       GPIO_Pin_6     /* U 相: PA6 */
#define HALL_U_GPIO      GPIOA
#define HALL_V_PIN       GPIO_Pin_7     /* V 相: PA7 */
#define HALL_V_GPIO      GPIOA
#define HALL_W_PIN       GPIO_Pin_0     /* W 相: PB0 */
#define HALL_W_GPIO      GPIOB

void bldc_gpio_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef gpio_init_struct;

    /* Hall 输入: PA6/PA7/PB0 上拉输入 */
    gpio_init_struct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init_struct);
    gpio_init_struct.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &gpio_init_struct);

    /* 驱动器 SD 使能: PB11 推挽输出 */
    gpio_init_struct.GPIO_Pin = GPIO_Pin_11;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &gpio_init_struct);

    /* TIM1 刹车输入: PB12 下拉输入(高有效刹车, 低电平不触发) */
    gpio_init_struct.GPIO_Pin = GPIO_Pin_12;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IPD;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &gpio_init_struct);

    /* PWM 上桥输出: PA8/PA9/PA10 复用推挽 */
    gpio_init_struct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio_init_struct);

    /* PWM 下桥输出: PB13/PB14/PB15 复用推挽(COM事件同步换相) */
    gpio_init_struct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &gpio_init_struct);
}

/* 读取三相 Hall 传感器状态, 编码为 3 位状态值 */
u8 hallsensor_get_state(void)
{
    u8 state = 0;
    if (GPIO_ReadInputDataBit(HALL_U_GPIO, HALL_U_PIN) != Bit_RESET) {
        state |= 0x01U;
    }
    if (GPIO_ReadInputDataBit(HALL_V_GPIO, HALL_V_PIN) != Bit_RESET) {
        state |= 0x02U;
    }
    if (GPIO_ReadInputDataBit(HALL_W_GPIO, HALL_W_PIN) != Bit_RESET) {
        state |= 0x04U;
    }
    return state;
}

/* 停止电机: 关驱动器并断开所有桥臂 */
void stop_motor1(void)
{
    BLDC_Driver_Turnoff;                 /* SD=0, 关闭驱动器 */
    TIM_CtrlPWMOutputs(TIM1, DISABLE);   /* MOE=0, 关闭PWM输出 */

    TIM1->CCR2 = 0;
    TIM1->CCR1 = 0;
    TIM1->CCR3 = 0;

    TIM1->CCER &= ~(0x1 << 2);           /* CC1NE=0 */
    TIM1->CCER &= ~(0x1 << 6);           /* CC2NE=0 */
    TIM1->CCER &= ~(0x1 << 10);          /* CC3NE=0 */
}

/* 启动电机: 使能驱动器与PWM输出 */
void start_motor1(void)
{
    BLDC_Driver_Turnon;                  /* SD=1, 使能驱动器 */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);    /* MOE=1 */
}

/* 六步换相执行: 按Hall状态查表选择导通桥臂
 * 由TIM3 Hall边沿中断调用(TIM3_IRQHandler), 也可手动调用于首次启动 */
void UVW_6_Step_Ponoff(void)
{
    if (motor1.run_flag == RUN) {
        if (motor1.dir == CCW) {
            /* 逆时针: Hall顺序 6,2,3,1,5,4 */
            motor1.step_sta = hallsensor_get_state();
        } else {
            /* 顺时针: 用7减使状态序与换相表顺序对应 */
            motor1.step_sta = 7 - hallsensor_get_state();
        }

        if (motor1.step_sta >= 1 && motor1.step_sta <= 6) {
            /* 按Hall状态码调用对应换相函数 */
            pfunclist[motor1.step_sta - 1]();
        } else {
            /* Hall状态异常: 断开驱动并停机 */
            stop_motor1();
            motor1.run_flag = STOP;
        }
    }
}

/* 六步换相函数指针表, 按 Hall 状态索引 */
pctr pfunclist[6] = {&U_up_W_dwn, &V_up_U_dwn, &V_up_W_dwn, &W_up_V_dwn, &U_up_V_dwn, &W_up_U_dwn};

/* U相上桥PWM导通, V相下桥导通 */
void U_up_V_dwn(void)
{
    TIM1->CCR1 = motor1.pwm_duty;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    TIM1->CCER &= ~(0x1 << 3);           /* CC1NP=0 */
    TIM1->CCER |=  0x1;                  /* CC1E=1 */
    TIM1->CCER &= ~(0x1 << 2);           /* CC1NE=0 */

    TIM1->CCER |=  (0x1 << 7);           /* CC2NP=1 */
    TIM1->CCER &= ~(0x1 << 4);           /* CC2E=0 */
    TIM1->CCER |=  (0x1 << 6);           /* CC2NE=1 */

    TIM1->CCER &= ~(0x1 << 11);          /* CC3NP=0 */
    TIM1->CCER &= ~(0x1 << 8);           /* CC3E=0 */
    TIM1->CCER &= ~(0x1 << 10);          /* CC3NE=0 */
}

/* U相上桥PWM导通, W相下桥导通 */
void U_up_W_dwn(void)
{
    TIM1->CCR1 = motor1.pwm_duty;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;

    TIM1->CCER &= ~(0x1 << 3);
    TIM1->CCER |=  0x1;
    TIM1->CCER &= ~(0x1 << 2);

    TIM1->CCER &= ~(0x1 << 7);
    TIM1->CCER &= ~(0x1 << 4);
    TIM1->CCER &= ~(0x1 << 6);

    TIM1->CCER |=  (0x1 << 11);          /* CC3NP=1 */
    TIM1->CCER &= ~(0x1 << 8);           /* CC3E=0 */
    TIM1->CCER |=  (0x1 << 10);          /* CC3NE=1 */
}

/* V相上桥PWM导通, W相下桥导通 */
void V_up_W_dwn(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = motor1.pwm_duty;
    TIM1->CCR3 = 0;

    TIM1->CCER &= ~(0x1 << 3);
    TIM1->CCER &= ~0x1;                  /* CC1E=0 */
    TIM1->CCER &= ~(0x1 << 2);

    TIM1->CCER &= ~(0x1 << 7);
    TIM1->CCER |=  (0x1 << 4);           /* CC2E=1 */
    TIM1->CCER &= ~(0x1 << 6);

    TIM1->CCER |=  (0x1 << 11);
    TIM1->CCER &= ~(0x1 << 8);
    TIM1->CCER |=  (0x1 << 10);
}

/* V相上桥PWM导通, U相下桥导通 */
void V_up_U_dwn(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = motor1.pwm_duty;
    TIM1->CCR3 = 0;

    TIM1->CCER |=  (0x1 << 3);           /* CC1NP=1 */
    TIM1->CCER &= ~0x1;                  /* CC1E=0 */
    TIM1->CCER |=  (0x1 << 2);           /* CC1NE=1 */

    TIM1->CCER &= ~(0x1 << 7);
    TIM1->CCER |=  (0x1 << 4);
    TIM1->CCER &= ~(0x1 << 6);

    TIM1->CCER &= ~(0x1 << 11);
    TIM1->CCER &= ~(0x1 << 8);
    TIM1->CCER &= ~(0x1 << 10);
}

/* W相上桥PWM导通, U相下桥导通 */
void W_up_U_dwn(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = motor1.pwm_duty;

    TIM1->CCER |=  (0x1 << 3);
    TIM1->CCER &= ~0x1;
    TIM1->CCER |=  (0x1 << 2);

    TIM1->CCER &= ~(0x1 << 7);
    TIM1->CCER &= ~(0x1 << 4);
    TIM1->CCER &= ~(0x1 << 6);

    TIM1->CCER &= ~(0x1 << 11);
    TIM1->CCER |=  (0x1 << 8);           /* CC3E=1 */
    TIM1->CCER &= ~(0x1 << 10);
}

/* W相上桥PWM导通, V相下桥导通 */
void W_up_V_dwn(void)
{
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = motor1.pwm_duty;

    TIM1->CCER &= ~(0x1 << 3);
    TIM1->CCER &= ~0x1;
    TIM1->CCER &= ~(0x1 << 2);

    TIM1->CCER |=  (0x1 << 7);           /* CC2NP=1 */
    TIM1->CCER &= ~(0x1 << 4);
    TIM1->CCER |=  (0x1 << 6);           /* CC2NE=1 */

    TIM1->CCER &= ~(0x1 << 11);
    TIM1->CCER |=  (0x1 << 8);           /* CC3E=1 */
    TIM1->CCER &= ~(0x1 << 10);
}
