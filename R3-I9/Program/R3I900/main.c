
/*------------------------------------------------------------------*/
/* --- STC MCU International Limited -------------------------------*/
/* --- STC 1T Series MCU RC Demo -----------------------------------*/
/* --- Mobile: (86)13922805190 -------------------------------------*/
/* --- Fax: 86-0513-55012956,55012947,55012969 ---------------------*/
/* --- Tel: 86-0513-55012928,55012929,55012966 ---------------------*/
/* --- Web: www.GXWMCU.com -----------------------------------------*/
/* --- QQ:  800003751 ----------------------------------------------*/
/* If you want to use the program or the program referenced in the  */
/* article, please specify in which data and procedures from STC    */
/*------------------------------------------------------------------*/

#include	"config.h"
#include	"timer.h"
#include	"delay.h"
#include	"GPIO.h"

/*************	功能说明	**************

******************************************/

/*************	本地常量声明	**************/


/*************	本地变量声明	**************/

u16 time_us_count;		//deley_us_count

/*************	本地函数声明	**************/
void ir_send_bit(u8 pose,u8 sta);
void ir_send_start(u8 pose);
void ir_send_data(u8 pose,u8 send_data);
void ir_send_end(u8 pose);
void delay_us(u16 i);
/*************  外部函数和变量声明 *****************/



/************************ 定时器配置 ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;					//结构定义
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_Polity    = PolityHigh;			//指定中断优先级, PolityHigh,PolityLow
	TIM_InitStructure.TIM_Interrupt = ENABLE;				//中断是否允许,   ENABLE或DISABLE
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;			//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / 100000UL);	//初值,————10us定时
	//65536UL - (MAIN_Fosc / 1000);			//初值——————1ms定时
	//65536UL - (MAIN_Fosc / (50*12));		//初值——————20ms定时
	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);				//初始化Timer0	  Timer0,Timer1,Timer2

//	TIM_InitTypeDef		TIM_InitStructure;					//结构定义
//	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
//	TIM_InitStructure.TIM_Polity    = PolityLow;			//指定中断优先级, PolityHigh,PolityLow
//	TIM_InitStructure.TIM_Interrupt = DISABLE;				//中断是否允许,   ENABLE或DISABLE
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;			//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = ENABLE;				//是否输出高速脉冲, ENABLE或DISABLE
//	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / 100000UL);		//初值,10us定时
//	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
//	Timer_Inilize(Timer0,&TIM_InitStructure);				//初始化Timer0	  Timer0,Timer1,Timer2

//	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
//	TIM_InitStructure.TIM_Polity    = PolityLow;			//指定中断优先级, PolityHigh,PolityLow
//	TIM_InitStructure.TIM_Interrupt = ENABLE;				//中断是否允许,   ENABLE或DISABLE
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;			//指定时钟源, TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = ENABLE;				//是否输出高速脉冲, ENABLE或DISABLE
//	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / 1000);		//初值,1ms定时
//	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
//	Timer_Inilize(Timer1,&TIM_InitStructure);				//初始化Timer1	  Timer0,Timer1,Timer2

//	TIM_InitStructure.TIM_Interrupt = ENABLE;				//中断是否允许,   ENABLE或DISABLE. (注意: Timer2固定为16位自动重装, 中断固定为低优先级)
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = ENABLE;				//是否输出高速脉冲, ENABLE或DISABLE
//	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / (50*12));		//初值20ms定时
//	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
//	Timer_Inilize(Timer2,&TIM_InitStructure);				//初始化Timer2	  Timer0,Timer1,Timer2
}


/******************** IO配置函数 **************************/
void	GPIO_config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;		//结构定义
	GPIO_InitStructure.Pin  = GPIO_Pin_7;		//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7, 或操作
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P2,&GPIO_InitStructure);	//初始化

	GPIO_InitStructure.Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;		//指定要初始化的IO, GPIO_Pin_0 |1|2, 或操作
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	//初始化
}


/******************** 主函数**************************/
void main(void)
{
	P_PWM = 0;
	P3M1 &= ~(1 << 5);	//P3.5 设置为推挽输出
	P3M0 |=  (1 << 5);

//	P1M1 &= ~(1 << 4);	//P1.4 设置为推挽输出	STC15W204S
//	P1M0 |=  (1 << 4);

	Timer_config();
	EA = 1;
	P35 = 0;		//对于STC15W408S系列，要输出低，或者设置成推挽或OD，否则不正常

//	WDT_CONTR = 0x04;       //看门狗定时器溢出时间计算公式: (12 * 32768 * PS) / FOSC (秒)
//                            //设置看门狗定时器分频数为32,溢出时间如下:
//                            //11.0592M : 1.14s
//                            //18.432M  : 0.68s
//                            //20M      : 0.63s
//    WDT_CONTR |= 0x20;      //启动看门狗
	GPIO_config();
	
	LED_Green = 1;	
	
//	if(WDT_CONTR | 0x80) 
//	LED_Green = 0;	
//	WDT_CONTR &= 0x7F;	//喂狗
	
	while (1)
	{	
//		WDT_CONTR |= 0x10;	//喂狗
		delay_ms(100);			//延时100ms
		ir_send_start(0);
		ir_send_data(0,0x00);
		ir_send_data(0,0xFF);
		ir_send_data(0,0x68);
		ir_send_data(0,0x97);
		ir_send_end(0);
		LED_L = LED_M = LED_R = 0;
		LED_Green = ~LED_Green;
		
		delay_ms(100);			//延时100ms
		ir_send_start(1);
		ir_send_data(1,0x00);
		ir_send_data(1,0xFF);
		ir_send_data(1,0x98);
		ir_send_data(1,0x67);
		ir_send_end(1);
		LED_L = LED_M = LED_R = 0;
		LED_Green = ~LED_Green;
		
		delay_ms(100);			//延时100ms
		ir_send_start(2);
		ir_send_data(2,0x00);
		ir_send_data(2,0xFF);
		ir_send_data(2,0xB0);
		ir_send_data(2,0x4F);
		ir_send_end(2);
		LED_L = LED_M = LED_R = 0;
		LED_Green = ~LED_Green;
	}
}

/*
************************************************************************************************************************
*                                                   ir_send_data
* Description: 红外数据发送data;
* Arguments  : u8 pose,u8 send_data;
* Note(s)    : 1) Null；
************************************************************************************************************************
*/
void ir_send_data(u8 pose,u8 send_data)
{
	u8 i;
	for(i=0;i<8;i++)
	{
		ir_send_bit(pose,send_data&0x80);
		send_data = send_data<<1;
	}
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
			delay_us(5000);
			LED_L = 0;
			delay_us(2250);
			break;
		case 1:
			LED_M = 1;
			delay_us(5000);
			LED_M = 0;
			delay_us(2250);
			break;
		case 2:
			LED_R = 1;
			delay_us(5000);
			LED_R = 0;
			delay_us(2250);
			break;
		default:
			break;
	}	
}	

void delay_us(unsigned int i)		//@12.000MHz
{	
	time_us_count = (i/10) + 1;
	while(time_us_count);
}


