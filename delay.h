#ifndef __DELAY_H_
#define __DELAY_H_

#include "sys.h"

void delay_init(uint16_t sysclk);       /* 初始化延时函数 */
void delay_ms(uint16_t nms);            /* 延时nms */
void delay_us(uint32_t nus);            /* 延时nus */

#endif
