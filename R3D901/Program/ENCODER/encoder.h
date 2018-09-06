#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h" 
#include "stdio.h"
#include "stdbool.h"
#include "delay.h"

#define U16_MAX    ((u16)65535u)
#define U32_MAX    ((u32)4294967295uL)
#define ENCODER_LEFT_TIMER TIM3			//左电机码盘采集定时器 TIM3
#define ENCODER_RIGHT_TIMER TIM4		//右电机码盘采集定时器 TIM4

#define ICx_FILTER      (u8) 6 // 6<-> 670nsec   编码器模式设置参数

void ENC_Init_Left(void);//设置电机 TIM3编码器模式GPIOB6 GPIOB7 左电机
void ENC_Init_Right(void);//设置电机 TIM4编码器模式GPIOC6 GPIOC7 右电机

#endif 
