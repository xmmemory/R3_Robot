#ifndef __AM2320_H
#define __AM2320_H 
#include "sys.h"   
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//AM2320������ʪ�ȴ�������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/12
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
//IO��������
#define AM2320_IO_IN()  {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=4<<24;}
#define AM2320_IO_OUT() {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=3<<24;}
////IO��������											   
#define	AM2320_DQ_OUT PBout(14) //���ݶ˿�	PA0 
#define	AM2320_DQ_IN  PBin(14)  //���ݶ˿�	PA0 


u8 AM2320_Init(void);//��ʼ��AM2320
u8 AM2320_Read_Data(void);//��ȡ��ʪ��
u8 AM2320_Read_Byte(void);//����һ���ֽ�
u8 AM2320_Read_Bit(void);//����һ��λ
u8 AM2320_Check(void);//����Ƿ����AM2320
void AM2320_Rst(void);//��λAM2320

extern u16 temperature_u16,humidity_u16;

#endif















