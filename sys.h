/**
 ****************************************************************************************************
 * @file        sys.h
 * @author      正点原子
 * @version     V1.0
 * @date        2026-02-08
 * @brief       系统全局头文件 - 包含STM32F10x标准头文件及常用宏定义
 * @license     Copyright (c) 2020-2032, 湖南大学机电工程学院
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子STM32F1精英开发板ATK-PD6010B电机驱动板
 *
 ****************************************************************************************************
 */

#ifndef __SYS_H_
#define __SYS_H_

#include "stm32f10x.h"

/********************************************************************************
 * 以下为位带操作,用于实现类似51单片机的GPIO位控制
 * 具体实现参考《CM3权威指南》第5章(87页~92页)
 ********************************************************************************/

/* IO口操作宏定义 */
#define BITBAND(addr, bitnum)  ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr)         *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum)  MEM_ADDR(BITBAND(addr, bitnum))

/* GPIO 输出寄存器地址映射 */
#define GPIOA_ODR_Addr  (GPIOA_BASE + 12) /* 0x4001080C */
#define GPIOB_ODR_Addr  (GPIOB_BASE + 12) /* 0x40010C0C */
#define GPIOC_ODR_Addr  (GPIOC_BASE + 12) /* 0x4001100C */
#define GPIOD_ODR_Addr  (GPIOD_BASE + 12) /* 0x4001140C */
#define GPIOE_ODR_Addr  (GPIOE_BASE + 12) /* 0x4001180C */
#define GPIOF_ODR_Addr  (GPIOF_BASE + 12) /* 0x40011C0C */
#define GPIOG_ODR_Addr  (GPIOG_BASE + 12) /* 0x4001200C */

/* GPIO 输入寄存器地址映射 */
#define GPIOA_IDR_Addr  (GPIOA_BASE + 8)  /* 0x40010808 */
#define GPIOB_IDR_Addr  (GPIOB_BASE + 8)  /* 0x40010C08 */
#define GPIOC_IDR_Addr  (GPIOC_BASE + 8)  /* 0x40011008 */
#define GPIOD_IDR_Addr  (GPIOD_BASE + 8)  /* 0x40011408 */
#define GPIOE_IDR_Addr  (GPIOE_BASE + 8)  /* 0x40011808 */
#define GPIOF_IDR_Addr  (GPIOF_BASE + 8)  /* 0x40011C08 */
#define GPIOG_IDR_Addr  (GPIOG_BASE + 8)  /* 0x40012008 */

/* IO口位带操作,实现单个IO口位操作（输出） */
#define PAout(n)  BIT_ADDR(GPIOA_ODR_Addr, n)  /* 输出 */
#define PBout(n)  BIT_ADDR(GPIOB_ODR_Addr, n)  /* 输出 */
#define PCout(n)  BIT_ADDR(GPIOC_ODR_Addr, n)  /* 输出 */
#define PDout(n)  BIT_ADDR(GPIOD_ODR_Addr, n)  /* 输出 */
#define PEout(n)  BIT_ADDR(GPIOE_ODR_Addr, n)  /* 输出 */
#define PFout(n)  BIT_ADDR(GPIOF_ODR_Addr, n)  /* 输出 */
#define PGout(n)  BIT_ADDR(GPIOG_ODR_Addr, n)  /* 输出 */

/* IO口位带操作,实现单个IO口位操作（输入） */
#define PAin(n)   BIT_ADDR(GPIOA_IDR_Addr, n)  /* 输入 */
#define PBin(n)   BIT_ADDR(GPIOB_IDR_Addr, n)  /* 输入 */
#define PCin(n)   BIT_ADDR(GPIOC_IDR_Addr, n)  /* 输入 */
#define PDin(n)   BIT_ADDR(GPIOD_IDR_Addr, n)  /* 输入 */
#define PEin(n)   BIT_ADDR(GPIOE_IDR_Addr, n)  /* 输入 */
#define PFin(n)   BIT_ADDR(GPIOF_IDR_Addr, n)  /* 输入 */
#define PGin(n)   BIT_ADDR(GPIOG_IDR_Addr, n)  /* 输入 */

#endif /* __SYS_H_ */
