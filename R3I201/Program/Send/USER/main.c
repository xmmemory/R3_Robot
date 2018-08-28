#include "sys.h"
#include "delay.h"
//#include "usart.h"
//#include "usart2.h"
#include "TIM5_INT.h"
#include "stm32f10x_it.h"
#include "NVIC_CONFIG.h"
//#include "math.h"
#include "dog.h"
#include "led.h"
#include "timer.h"

void ir_send_bit(u8 pose,u8 sta);
void ir_send_start(u8 pose);
void ir_send_data(u8 pose,u8 data);
void ir_send_end(u8 pose);


int main()
{
//	int t=0,i;
	delay_init();
	Init_Tim2_Int();
//	TIM3_Int_Init(1999,199);//10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms
	Init_Nvic();
	IWDG_Init(4,625);   	//���Ƶ��Ϊ64,����ֵΪ625,���ʱ��Ϊ1s
	Ctr_Tim2(1);
	LED_Init();
	LED11 = 1;
	
	while(1)
	{
		IWDG_Feed();	//ι��		
		ir_send_data(0,0x1A);
		LED11 = ~LED11;
		delay_ms(100);			//��ʱ100ms
		IWDG_Feed();	//ι��
		ir_send_data(1,0xB2);
		LED11 = ~LED11;
		delay_ms(100) ;			//��ʱ100ms
		IWDG_Feed();	//ι��
		ir_send_data(2,0x3C);
		LED11 = ~LED11;
		delay_ms(100);			//��ʱ100ms
	}
}

/*
************************************************************************************************************************
*                                                   ir_send_data
* Description: �������ݷ���data;
* Arguments  : u8 pose,u8 data;
* Note(s)    : 1) Null��
************************************************************************************************************************
*/
void ir_send_data(u8 pose,u8 data)
{
	u8 i;	
	ir_send_start(pose);
	for(i=0;i<8;i++)
	{
		ir_send_bit(pose,data&0x80);
		data = data<<1;
	}
	ir_send_end(pose);
	LED_L = LED_M = LED_R = 0;
}	

/*
************************************************************************************************************************
*                                                   ir_send_bit
* Description: �������ݷ���bit;
* Arguments  : u8 pose,u8 sta;
* Note(s)    : 1) Null��
************************************************************************************************************************
*/
void ir_send_bit(u8 pose,u8 sta)
{
	if(sta)
	{
		switch(pose)
		{
			case 0:
				LED_L = 1;
				delay_us(560);
				LED_L = 0;
				delay_us(1690);
				break;
			case 1:
				LED_M = 1;
				delay_us(560);
				LED_M = 0;
				delay_us(1690);
				break;
			case 2:
				LED_R = 1;
				delay_us(560);
				LED_R = 0;
				delay_us(1690);
				break;
			default:
				break;
		}
	}
	else
	{
		switch(pose)
		{
			case 0:
				LED_L = 1;
				delay_us(560);
				LED_L = 0;
				delay_us(560);
				break;
			case 1:
				LED_M = 1;
				delay_us(560);
				LED_M = 0;
				delay_us(560);
				break;
			case 2:
				LED_R = 1;
				delay_us(560);
				LED_R = 0;
				delay_us(560);
				break;
			default:
				break;
		}
	}	
}	

/*
************************************************************************************************************************
*                                                   ir_send_start
* Description: �������ݷ��������ź�;
* Arguments  : u8 pose;
* Note(s)    : 1) Null��
************************************************************************************************************************
*/
void ir_send_start(u8 pose)
{
	switch(pose)
	{
		case 0:
			LED_L = 1;
			delay_us(9000);
			LED_L = 0;
			delay_us(4500);
			break;
		case 1:
			LED_M = 1;
			delay_us(9000);
			LED_M = 0;
			delay_us(4500);
			break;
		case 2:
			LED_R = 1;
			delay_us(9000);
			LED_R = 0;
			delay_us(4500);
			break;
		default:
			break;
	}	
}	

/*
************************************************************************************************************************
*                                                   ir_send_end
* Description: �������ݷ��������ź�;
* Arguments  : u8 pose;
* Note(s)    : 1) Null��
************************************************************************************************************************
*/
void ir_send_end(u8 pose)
{	
	switch(pose)
	{
		case 0:
			LED_L = 1;
			delay_us(9000);
			LED_L = 0;
			delay_us(2250);
			break;
		case 1:
			LED_M = 1;
			delay_us(9000);
			LED_M = 0;
			delay_us(2250);
			break;
		case 2:
			LED_R = 1;
			delay_us(9000);
			LED_R = 0;
			delay_us(2250);
			break;
		default:
			break;
	}	
}	





