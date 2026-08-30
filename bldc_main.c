/*****************************************************************************************************
 * @file        bldc_main.c
 * @version     V2.0
 * @date        2026-06-04
 * @brief       BLDC PID 速度闭环
 *              KEY0(PE4): 目标+100RPM / KEY1(PE3): 目标-100RPM
 ****************************************************************************************************/

#include "stm32f10x.h"
#include "bldc.h"
#include "key.h"
#include "delay.h"
#include "bldc_tim.h"

/* ---- PID 参数 ---- */
#define TS  0.1f     /* 控制周期 = 100ms，由 TIM6 中断周期决定 */

/* KP/KI/KD 已包含 RPM → PWM 占空比的隐式转换，因此 PID 输出可直接当 duty 用 */
#define KP  1.5f     /* 比例: 越大响应越快，太大振荡 */
#define KI  0.8f     /* 积分: 消静差，太大过冲 */
#define KD  0.15f    /* 微分: 防冲过头，太大抖动 */

u16 PWM_arr=7200, PWM_psc=1;    /* 5kHz */

int main(void)
{
  u8   key;
  int  target = 1000;                  /* 目标转速 */
  float integral  = 0, last_err = 0;  /* PID 状态 */
  uint32_t last_tick = 0;

  delay_init(168);
  Time3_HalldetectCNF();    TIM3_NVIC_Config();
  Encoder_Init_TIM4();      Init_TIM6();    TIM6_NVIC_Config();
  key_init();               bldc_gpio_init();
  TIM1_PwmoutCNF_OnLibFunc(PWM_arr-1, PWM_psc-1);

  stop_motor1();
  motor1.pwm_duty = 0;
  motor1.dir = CW;
  motor1.run_flag = RUN;
  start_motor1();
  UVW_6_Step_Ponoff();
  TIM_GenerateEvent(TIM1, TIM_EventSource_COM);

  while (1)
  {
    /* ---- 按键调目标 ---- */
    key = key_scan(0);
    if (key == KEY0_PRES) { target += 100; if (target > 3000) target = 3000;
                            integral=0; last_err=0; }   /* 换目标,清记忆 */
    if (key == KEY1_PRES) { target -= 100; if (target <    0) target =    0;
                            integral=0; last_err=0; }

    /* ---- PID 速度环 (100ms) ---- */
    if (g_tim6_tick != last_tick)
    {
      last_tick = g_tim6_tick;
      int speed = motor1.speed;    /* TIM6 ISR 已算好 */

      /* ① 误差 */
      float err = (float)(target - speed);

      /* ② 积分：∫e·dt ≈ Σ(e · Ts)
         乘 Ts 使量纲为 RPM·s，积分值不会随控制周期改变而跳变 */
      integral += err * TS;
      if (integral >  4800) integral =  4800;
      if (integral < -4800) integral = -4800;

      /* ③ 微分：de/dt ≈ (e[n] - e[n-1]) / Ts
         除 Ts 使量纲为 RPM/s，反映误差变化的快慢 */
      float diff = (err - last_err) / TS;
      last_err = err;

      /* ④ PID 合成为占空比
         KP/KI/KD 已内含 RPM → PWM 占空比的单位换算，
         因此直接相加即得 duty，无需额外转换因子 */
      int duty = (int)(KP*err + KI*integral + KD*diff);

      /* ⑤ 限幅 */
      if (duty > MAX_PWM_DUTY/2) duty = MAX_PWM_DUTY/2;
      if (duty < 0)              duty = 0;

      motor1.pwm_duty = duty;
    }

    UVW_6_Step_Ponoff();
  }
}
