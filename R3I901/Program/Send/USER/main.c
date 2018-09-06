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
//	TIM3_Int_Init(1999,199);//10Khz的计数频率，计数到5000为500ms
	Init_Nvic();
	IWDG_Init(4,625);   	//与分频数为64,重载值为625,溢出时间为1s
	Ctr_Tim2(1);
	LED_Init();
	LED11 = 1;
	
	while(1)
	{
		IWDG_Feed();	//喂狗		
		ir_send_data(0,0x1A);
		LED11 = ~LED11;
		delay_ms(100);			//延时100ms
		IWDG_Feed();	//喂狗
		ir_send_data(1,0xB2);
		LED11 = ~LED11;
		delay_ms(100) ;			//延时100ms
		IWDG_Feed();	//喂狗
		ir_send_data(2,0x3C);
		LED11 = ~LED11;
		delay_ms(100);			//延时100ms
	}
}

/*
************************************************************************************************************************
*                                                   ir_send_data
* Description: 红外数据发送data;
* Arguments  : u8 pose,u8 data;
* Note(s)    : 1) Null；
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
* Description: 红外数据发送bit;
* Arguments  : u8 pose,u8 sta;
* Note(s)    : 1) Null；
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
* Description: 红外数据发送引导信号;
* Arguments  : u8 pose;
* Note(s)    : 1) Null；
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
* Description: 红外数据发送引导信号;
* Arguments  : u8 pose;
* Note(s)    : 1) Null；
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





