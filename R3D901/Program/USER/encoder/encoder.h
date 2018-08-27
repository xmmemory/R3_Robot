#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx.h" 
#include "stdio.h"
#include "stdbool.h"
#include "delay.h"

#define U16_MAX    ((u16)65535u)
#define U32_MAX    ((u32)4294967295uL)
#define ENCODER1_TIMER TIM3   // ���B���̲ɼ���ʱ�� TIM3
#define ENCODER_LEFT_TIMER TIM3   // ���B���̲ɼ���ʱ�� TIM3
#define ENCODER2_TIMER TIM4   // ���A���̲ɼ���ʱ�� TIM4
#define ENCODER_RIGHT_TIMER TIM4   // �ҵ��A���̲ɼ���ʱ�� TIM4

#define ICx_FILTER      (u8) 6 // 6<-> 670nsec   ������ģʽ���ò���

void ENC_Init(void);//��������ʼ��

void ENC_Init1(void);//���õ��B TIM3������ģʽPA6 PA7 ����
void ENC_Init2(void);//���õ��A TIM4������ģʽPB6 PB7 �ҵ��

#endif 
