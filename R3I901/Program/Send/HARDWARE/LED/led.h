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
#define LED11 PBout(11)// PB12

#define LED_L PBout(0)// PB0
#define LED_M PBout(1)// PB1
#define LED_R PBout(2)// PB2

#define LED_38K PBout(10)// PB10
//#define LED1 PEout(5)// PE5	

void LED_Init(void);//��ʼ��

		 				    
#endif
