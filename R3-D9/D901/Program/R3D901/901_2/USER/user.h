/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                           MASTER INCLUDES
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : user.h
* Version       : V1.00
* Programmer(s) : Y
*********************************************************************************************************
*/

#ifndef __USER_H
#define __USER_H	 
#include "stm32f4xx.h"
#include "delay.h"

void PVD_Config(void);															//低电压中断初始化
float battery_check(void);													//电压检测
void	Start_Succeed(void);													//程序自检、正常启动蜂鸣器关闭



#endif
