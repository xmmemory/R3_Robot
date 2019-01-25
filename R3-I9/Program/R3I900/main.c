
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

/*************	����˵��	**************

******************************************/

/*************	���س�������	**************/


/*************	���ر�������	**************/

u16 time_us_count;		//deley_us_count

/*************	���غ�������	**************/
void ir_send_bit(u8 pose,u8 sta);
void ir_send_start(u8 pose);
void ir_send_data(u8 pose,u8 send_data);
void ir_send_end(u8 pose);
void delay_us(u16 i);
/*************  �ⲿ�����ͱ������� *****************/



/************************ ��ʱ������ ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;					//�ṹ����
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_Polity    = PolityHigh;			//ָ���ж����ȼ�, PolityHigh,PolityLow
	TIM_InitStructure.TIM_Interrupt = ENABLE;				//�ж��Ƿ�����,   ENABLE��DISABLE
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;			//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / 100000UL);	//��ֵ,��������10us��ʱ
	//65536UL - (MAIN_Fosc / 1000);			//��ֵ������������1ms��ʱ
	//65536UL - (MAIN_Fosc / (50*12));		//��ֵ������������20ms��ʱ
	TIM_InitStructure.TIM_Run       = ENABLE;				//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);				//��ʼ��Timer0	  Timer0,Timer1,Timer2

//	TIM_InitTypeDef		TIM_InitStructure;					//�ṹ����
//	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
//	TIM_InitStructure.TIM_Polity    = PolityLow;			//ָ���ж����ȼ�, PolityHigh,PolityLow
//	TIM_InitStructure.TIM_Interrupt = DISABLE;				//�ж��Ƿ�����,   ENABLE��DISABLE
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;			//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = ENABLE;				//�Ƿ������������, ENABLE��DISABLE
//	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / 100000UL);		//��ֵ,10us��ʱ
//	TIM_InitStructure.TIM_Run       = ENABLE;				//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
//	Timer_Inilize(Timer0,&TIM_InitStructure);				//��ʼ��Timer0	  Timer0,Timer1,Timer2

//	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
//	TIM_InitStructure.TIM_Polity    = PolityLow;			//ָ���ж����ȼ�, PolityHigh,PolityLow
//	TIM_InitStructure.TIM_Interrupt = ENABLE;				//�ж��Ƿ�����,   ENABLE��DISABLE
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_1T;			//ָ��ʱ��Դ, TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = ENABLE;				//�Ƿ������������, ENABLE��DISABLE
//	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / 1000);		//��ֵ,1ms��ʱ
//	TIM_InitStructure.TIM_Run       = ENABLE;				//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
//	Timer_Inilize(Timer1,&TIM_InitStructure);				//��ʼ��Timer1	  Timer0,Timer1,Timer2

//	TIM_InitStructure.TIM_Interrupt = ENABLE;				//�ж��Ƿ�����,   ENABLE��DISABLE. (ע��: Timer2�̶�Ϊ16λ�Զ���װ, �жϹ̶�Ϊ�����ȼ�)
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;		//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = ENABLE;				//�Ƿ������������, ENABLE��DISABLE
//	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / (50*12));		//��ֵ20ms��ʱ
//	TIM_InitStructure.TIM_Run       = ENABLE;				//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
//	Timer_Inilize(Timer2,&TIM_InitStructure);				//��ʼ��Timer2	  Timer0,Timer1,Timer2
}


/******************** IO���ú��� **************************/
void	GPIO_config(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;		//�ṹ����
	GPIO_InitStructure.Pin  = GPIO_Pin_7;		//ָ��Ҫ��ʼ����IO, GPIO_Pin_0 ~ GPIO_Pin_7, �����
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		//ָ��IO������������ʽ,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P2,&GPIO_InitStructure);	//��ʼ��

	GPIO_InitStructure.Pin  = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;		//ָ��Ҫ��ʼ����IO, GPIO_Pin_0 |1|2, �����
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		//ָ��IO������������ʽ,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	//��ʼ��
}


/******************** ������**************************/
void main(void)
{
	P_PWM = 0;
	P3M1 &= ~(1 << 5);	//P3.5 ����Ϊ�������
	P3M0 |=  (1 << 5);

//	P1M1 &= ~(1 << 4);	//P1.4 ����Ϊ�������	STC15W204S
//	P1M0 |=  (1 << 4);

	Timer_config();
	EA = 1;
	P35 = 0;		//����STC15W408Sϵ�У�Ҫ����ͣ��������ó������OD����������

//	WDT_CONTR = 0x04;       //���Ź���ʱ�����ʱ����㹫ʽ: (12 * 32768 * PS) / FOSC (��)
//                            //���ÿ��Ź���ʱ����Ƶ��Ϊ32,���ʱ������:
//                            //11.0592M : 1.14s
//                            //18.432M  : 0.68s
//                            //20M      : 0.63s
//    WDT_CONTR |= 0x20;      //�������Ź�
	GPIO_config();
	
	LED_Green = 1;	
	
//	if(WDT_CONTR | 0x80) 
//	LED_Green = 0;	
//	WDT_CONTR &= 0x7F;	//ι��
	
	while (1)
	{	
//		WDT_CONTR |= 0x10;	//ι��
		delay_ms(100);			//��ʱ100ms
		ir_send_start(0);
		ir_send_data(0,0x00);
		ir_send_data(0,0xFF);
		ir_send_data(0,0x68);
		ir_send_data(0,0x97);
		ir_send_end(0);
		LED_L = LED_M = LED_R = 0;
		LED_Green = ~LED_Green;
		
		delay_ms(100);			//��ʱ100ms
		ir_send_start(1);
		ir_send_data(1,0x00);
		ir_send_data(1,0xFF);
		ir_send_data(1,0x98);
		ir_send_data(1,0x67);
		ir_send_end(1);
		LED_L = LED_M = LED_R = 0;
		LED_Green = ~LED_Green;
		
		delay_ms(100);			//��ʱ100ms
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
* Description: �������ݷ���data;
* Arguments  : u8 pose,u8 send_data;
* Note(s)    : 1) Null��
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


