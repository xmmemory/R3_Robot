#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
#define LED12 PBout(12)// PB12

#define LED_L PBin(0)// PB0
#define LED_M PBin(1)// PB1
#define LED_R PBin(2)// PB2

#define LED3 PBout(3)// PB3
//#define LED1 PEout(5)// PE5	

void LED_Init(void);//��ʼ��
void IF_Init(void);

		 				    
#endif
