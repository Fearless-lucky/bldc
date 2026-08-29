/* delay.c - SysTick 阻塞延时(非OS模式)
 * 提供 delay_init 初始化以及 delay_us / delay_ms 延时函数
 */
#include "misc.h"
#include "sys.h"
#include "delay.h"

static uint16_t g_fac_us = 0;      /* us延时因子 */

/* 初始化延时函数
 * sysclk: 系统时钟频率(HCLK)
 */
void delay_init(uint16_t sysclk)
{
  SysTick->CTRL = 0;                                          /* 复位Systick状态 */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);       /* SYSTICK使用内核时钟源8分频 */
  g_fac_us = sysclk / 8;                                      /* 1us的基础时基 */
}

/* 延时nus
 * nus: 要延时的us数(最大值受 2^24 / g_fac_us 限制)
 */
void delay_us(uint32_t nus)
{
  uint32_t temp;
  SysTick->LOAD = nus * g_fac_us; /* 时间加载 */
  SysTick->VAL = 0x00;            /* 清空计数器 */
  SysTick->CTRL |= 1 << 0;        /* 开始计数 */

  do
   {
     temp = SysTick->CTRL;
   } while ((temp & 0x01) && !(temp & (1 << 16))); /* 等待计数时间到 */

   SysTick->CTRL &= ~(1 << 0);     /* 关闭SYSTICK */
   SysTick->VAL = 0x00;            /* 清空计数器 */
}

/* 延时nms (0 < nms <= 65535) */
void delay_ms(uint16_t nms)
{
  uint32_t repeat = nms / 1000;   /* 超过1000ms分批处理 */
  uint32_t remain = nms % 1000;

  while (repeat)
   {
     delay_us(1000 * 1000);       /* 每次延时1000ms */
     repeat--;
   }

  if (remain)
   {
     delay_us(remain * 1000);
   }
}
