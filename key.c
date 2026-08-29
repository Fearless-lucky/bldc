/* key.c - 按键输入驱动(KEY0/KEY1) */
#include "key.h"
#include "stm32f10x_gpio.h"
#include "delay.h"

/* 按键初始化: PE3/PE4 上拉输入 */
void key_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    gpio_init_struct.GPIO_Mode = GPIO_Mode_IPU;
    gpio_init_struct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOE, &gpio_init_struct);
}

/* 按键扫描函数(带消抖, 不支持连按)
 * mode: 0 不支持连续按(松开后再次按下才返回), 1 支持连续按
 * 返回: 0 无按键, KEY0_PRES=1, KEY1_PRES=2
 */
u8 key_scan(u8 mode)
{
    static u8 key_up = 1;  /* 按键松开标志 */
    u8 keyval = 0;
    if (mode) key_up = 1;
    if (key_up && (KEY0 == 0 || KEY1 == 0))
    {
      delay_ms(10);                    /* 消抖 */
      key_up = 0;
      if (KEY0 == 0)  keyval = KEY0_PRES;
      if (KEY1 == 0)  keyval = KEY1_PRES;
    }
    else if (KEY0 == 1 && KEY1 == 1)  key_up = 1;
    return keyval;
}
