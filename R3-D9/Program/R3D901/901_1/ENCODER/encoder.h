#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h" 
#include "stdio.h"
#include "stdbool.h"
#include "delay.h"

#define U16_MAX    ((u16)65535u)
#define U32_MAX    ((u32)4294967295uL)
#define ENCODER_LEFT_TIMER TIM3			//�������̲ɼ���ʱ�� TIM3
#define ENCODER_RIGHT_TIMER TIM4		//�ҵ�����̲ɼ���ʱ�� TIM4

#define ICx_FILTER      (u8) 6 // 6<-> 670nsec   ������ģʽ���ò���

void ENC_Init_Left(void);//���õ�� TIM3������ģʽGPIOB6 GPIOB7 ����
void ENC_Init_Right(void);//���õ�� TIM4������ģʽGPIOC6 GPIOC7 �ҵ��

#endif 
