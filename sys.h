#ifndef __SYS_H
#define __SYS_H

#include "stm32f10x.h"

/* 是否支持OS: 0不支持, 1支持 */
#define SYS_SUPPORT_OS          0

/* ---- Flash参数存储(标定值/整定增益掉电保存) ---- */
#define PARAM_MAGIC   0x5A1BC0DEUL

typedef struct {
    uint32_t magic;          /* 参数有效标志 */
    float    theta_offset;   /* 挡板电角度零点(编码器计数) */
    float    i_offset_a;     /* 挡板A相电流零点(ADC码) */
    float    i_offset_b;     /* 挡板B相电流零点(ADC码) */
    float    conv_kp;        /* 传送带速度环P */
    float    conv_ki;        /* 传送带速度环I */
    float    conv_kd;        /* 传送带速度环D */
    float    adc_to_amp;     /* 电流标定系数(ADC码->安培) */
    int32_t  enc_dir;        /* 挡板编码器方向(+1/-1) */
    uint32_t sum;            /* 简单校验和 */
} ParamBlock_t;

uint8_t param_save(const ParamBlock_t *pb);   /* 保存参数到Flash末页, 1=成功 */
uint8_t param_load(ParamBlock_t *pb);         /* 从Flash加载并校验, 1=有效 */

void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset);   /* 设置中断向量表 */
void sys_standby(void);                                               /* 进入待机模式 */
void sys_soft_reset(void);                                            /* 系统软复位 */
uint8_t sys_clock_set(uint32_t plln);                                 /* 时钟设置函数 */
void Rcc_InitOnFuncLib(uint32_t pll);                                 /* 系统时钟初始化 */

/* 以下为内联函数 */
void sys_wfi_set(void);                        /* 执行WFI指令 */
void sys_intx_disable(void);                   /* 关闭所有中断 */
void sys_intx_enable(void);                    /* 开启所有中断 */
void sys_msr_msp(uint32_t addr);               /* 设置栈顶地址 */

#endif
