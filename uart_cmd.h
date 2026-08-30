#ifndef __UART_CMD_H_
#define __UART_CMD_H_

#include "stm32f10x.h"

#define UART_BUF_SIZE   64
#define UART_BAUD       115200

void UART_Init(void);
void UART_SendString(char *str);
void uart_poll(void);
void cmd_guard_poll(void);               /* 命令层保护轮询(熔断+超时) */

/* 运行数据环形日志(定义于svpwm_main.c) */
void dlog_freeze(void);                  /* 故障时冻结采样保留现场 */
void dlog_dump(void);                    /* 导出日志(DUMP命令) */

/* 轻量格式化助手: 替代遥测中的sprintf, 约快5倍(无FPU的软件浮点环境) */
char *fmt_chr(char *p, char c);          /* 追加单字符, 返回新指针 */
char *fmt_int(char *p, int32_t v);       /* 追加有符号整数 */
char *fmt_uint(char *p, uint32_t v);     /* 追加无符号整数 */

#endif
