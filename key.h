#ifndef __KEY_H_
#define __KEY_H_

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"

#define KEY0        GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4)     /* 读取KEY0状态 */
#define KEY1        GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3)     /* 读取KEY1状态 */
#define KEY0_PRES   1              /* KEY0按下 */
#define KEY1_PRES   2              /* KEY1按下 */

void key_init(void);               /* 按键初始化 */
u8 key_scan(u8 mode);              /* 按键扫描函数 */

#endif
