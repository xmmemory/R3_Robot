#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h" 
#include "stdio.h"
#include "stdbool.h"
#include "delay.h"

#define U16_MAX    ((u16)65535u)
#define U32_MAX    ((u32)4294967295uL)
#define ENCODER1_TIMER TIM3   // 电机B码盘采集定时器 TIM3
#define ENCODER_LEFT_TIMER TIM3   // 电机B码盘采集定时器 TIM3
#define ENCODER2_TIMER TIM4   // 电机A码盘采集定时器 TIM4
#define ENCODER_RIGHT_TIMER TIM4   // 右电机A码盘采集定时器 TIM4

#define ICx_FILTER      (u8) 6 // 6<-> 670nsec   编码器模式设置参数

void ENC_Init(void);//电机处理初始化

void ENC_Init1(void);//设置电机B TIM3编码器模式PA6 PA7 左电机
void ENC_Init2(void);//设置电机A TIM4编码器模式PB6 PB7 右电机

#endif 
