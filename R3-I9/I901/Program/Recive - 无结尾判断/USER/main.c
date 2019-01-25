#include "sys.h"
#include "delay.h"
#include "usart.h"
//#include "TIM5_INT.h"
#include "stm32f10x_it.h"
#include "NVIC_CONFIG.h"
#include "math.h"
#include "dog.h"
#include "led.h"
#include "string.h"
#include "timer.h"

#define SEND_BUF_SIZE 256	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
u8 SendBuff[SEND_BUF_SIZE];	//�������ݻ�����

/*******************************************************/
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length);
u8 wait_if_sta(u8 pose,u16 sustained_time,u16 wait_time,u8 sta);
u8 recive_bit(u8 pose);
/*******************************************************/
u8 Send_Ask = 1;
u8 temp=0;
/*******************************************************/
int main()
{
	int t=0,i,data_L,data_M,data_R;		
	u8 data_pose[3],error_id=0;
	
	delay_init();
	
	TIM3_Int_Init(4999,7199);//10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms
	
	Init_Nvic();
	uart_init(115200);
	
//	IWDG_Init(IWDG_Prescaler_256,625);   	//���Ƶ��Ϊ64,����ֵΪ625,���ʱ��Ϊ1s

//	DHT11_Init();
//	Adc_Init();		  		//ADC��ʼ��
	
	LED_Init();
	IF_Init();		//����������ų�ʼ��
	
	while(1)
	{
		IWDG_Feed();	//ι��
		
		//��
		if(!LED_M)
		{
			delay_us(500);		//��ʱ500usȥ����
			if(!LED_M)				//LED_M == 0
			{
//u8 wait_if_sta(u8 pose,u16 sustained_time,u16 wait_time,u8 sta)
				if(!wait_if_sta(1,450,900,1))	//Head,450*10us=4500us=4.5ms,900*10us=9000us=9ms,1-wait for high
				{
					error_id = wait_if_sta(1,200,600,0);
					if(!error_id)	//Tail,250*10us=2500us=2.5ms,500*10us=5000us=5ms,0-wait for low		//��������ճɹ�
					{
						data_M = 0;		//���data_M����
						for(i=0;i<8;i++)
						{
							t = recive_bit(1);
							if(t < 2)
							{
								data_M <<= 1;	//����һλ.							
								data_M |= t;
							}							
							else			//�����쳣���жϽ���
							{
								data_M = 0;
								continue;
							}
						}
						//8λ���ݽ������
						if(data_M == 0x1A)
							data_pose[1] = 1;
						else if(data_M == 0xB2)
							data_pose[1] = 2;
						else if(data_M == 0x3C)
							data_pose[1] = 3;					
					}
					else 
						error_id = error_id;
				}	
			}
		}
		
		//��
		else if(!LED_L)
		{
			delay_us(500);		//��ʱ500usȥ����
			if(!LED_L)
			{
				if(!wait_if_sta(0,450,900,1))	//HEAD,450*10us=4500us=4.5ms,900*10us=9000us=9ms,1-wait for high
				{
					if(!wait_if_sta(0,200,600,0))	//Left,250*10us=2500us=2.5ms,500*10us=5000us=5ms,0-wait for low		//��������ճɹ�
					{
						data_L = 0;		//���data_L����
						for(i=0;i<8;i++)
						{
							t = recive_bit(0);
//							data_temp[i] = t;		//���Ա���
							if(t < 2)
							{
								data_L <<= 1;	//����һλ.							
								data_L |= t;
							}							
							else			//�����쳣���жϽ���
							{
								data_L = 0;
								continue;
							}								
//							if(data_L == 0xF3)
//								data_L = 0;
//							if(i > 6)
//								data_L = 0;					
						}
						//8λ���ݽ������
						if(data_L == 0x1A)
							data_pose[0] = 1;
						else if(data_L == 0xB2)
							data_pose[0] = 2;
						else if(data_L == 0x3C)
							data_pose[0] = 3;
					}
				}	
			}
		}
		//��
		else if(!LED_R)
		{
			delay_us(500);		//��ʱ500usȥ����
			if(!LED_R)
			{
				if(!wait_if_sta(2,450,900,1))	//Left,450*10us=4500us=4.5ms,900*10us=9000us=9ms,1-wait for high
				{
					if(!wait_if_sta(2,200,600,0))	//Left,250*10us=2500us=2.5ms,500*10us=5000us=5ms,0-wait for low		//��������ճɹ�
					{
						data_R = 0;		//���data_R����
						for(i=0;i<8;i++)
						{
							t = recive_bit(2);
							if(t < 2)
							{
								data_R <<= 1;	//����һλ.							
								data_R |= t;
							}							
							else			//�����쳣���жϽ���
							{
								data_R = 0;
								continue;
							}											
						}
						//8λ���ݽ������
						if(data_R == 0x1A)
							data_pose[2] = 1;
						else if(data_R == 0xB2)
							data_pose[2] = 2;
						else if(data_R == 0x3C)
							data_pose[2] = 3;
					}
				}	
			}
		}
		
		
		//3��λ������-�ϼ�3���ֽ�---����-ID=0x03��������=0x01
		if(data_pose[0] != 0 || data_pose[1] != 0 || data_pose[2] != 0 || Send_Ask)
		{
			uart1_format_build_and_send(data_pose,0x03,0x01,3);		
			memset(data_pose,0,3);		//���ͳɹ����������
			Send_Ask = 0;			//Clear_Send_Ask
		}
		
	}

}

/*
************************************************************************************************************************
*                                                   UART1_SEND
* Description: �������ݸ�ʽ����Ȼ������Ϣ
* Arguments  : u8* arg,u8 length;
* Note(s)    : 1) Null��
************************************************************************************************************************
*/
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length)
{	
	u16 check_sum;
	u8 i,j,check_count;
	
	if(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		return 1;	
	
	if(length > 64)	return 2;
	//Header
	SendBuff[0] = 0xA6;
	SendBuff[1] = 0x59;
	//Length
	SendBuff[2] = length+2+2+3+3;
	//Device ID
	SendBuff[3] = device_id;
	//Command ID
	SendBuff[4] = commond_id;
	//Data
	for(j=0;j<length;j++)
	{
		SendBuff[j+5]=*(arg+j);
	}
	//Check_Sum_Cal
	for(check_count = 0;check_count < length+5;check_count++)
	check_sum += SendBuff[check_count];
	//Check_Sum
	SendBuff[((length++)+5)] = (check_sum >> 8);
	SendBuff[((length++)+5)] = check_sum;
	//Tail
	SendBuff[((length++)+5)]='Y';			//��ӽ�����
	SendBuff[((length++)+5)]='H';			//��ӽ�����
	SendBuff[((length++)+5)]='T';			//��ӽ�����

	for(i=0;i<length+5;i++)
	{
		USART_ClearFlag(USART1,USART_FLAG_TC);  //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����				
		USART_SendData(USART1,SendBuff[i]);//����һ���ֽڵ�����
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//�ȴ����ͽ���
	} 	
	return 0;
}

//pose:�ڴ�Ӧ���λ��
//sustained_time:����ʱ��(��λ:10us)
//wait_time:�ȴ�ʱ��(��λ:10us)
//sta:�ڴ��ĵõ�״̬
//����ֵ:0,�ɹ�(�õ����ڴ���Ӧ����)
//       1,�ȴ�����ʱ��ʱ
//       3,δ�ܴﵽ��ƽ����ʱ��
//       4,��ʱ
//       9,δ֪
u8 wait_if_sta(u8 pose,u16 sustained_time,u16 wait_time,u8 sta)
{
	u8 res=0; 
	
	switch(pose)
	{
		case 0:
			while(--wait_time)	//�ȴ�����ʱ
			{
				delay_us(10);
				if(sustained_time > 0) sustained_time--;
				if(LED_L == sta)
				{
					if(sustained_time > 0)
						res=3;			//δ�ܴﵽ��ƽ����ʱ��
					break;				//�õ���Ч����
				}		
			}
			if(wait_time==0)res=1; 
			break;
		case 1:
			while(--wait_time)	//�ȴ�����ʱ
			{
				delay_us(10);
				if(sustained_time > 0) sustained_time--;
				if(LED_M == sta)
				{
					if(sustained_time > 0)
						res=3;			//δ�ܴﵽ��ƽ����ʱ��
					break;				//�õ���Ч����
				}
					
			}
			if(wait_time==0)res=1; 
			break;
		case 2:
			while(--wait_time)	//�ȴ�����ʱ
			{
				delay_us(10);
				if(sustained_time > 0) sustained_time--;
				if(LED_R == sta)
				{
					if(sustained_time > 0)
						res=3;				//δ�ܴﵽ��ƽ����ʱ��
					break;					//�õ���Ч����
				}
					
			}
			if(wait_time==0)res=1; 
			break;
		default:
			res=9;
			break;
	}
	return res;

}
/*
************************************************************************************************************************
*                                                   recive_bit
* Description: ������Ϣ
* Arguments  : u8 pose;
* Note(s)    : 0,�ɹ�(�õ����ڴ���Ӧ����-0
//      			 1,�ɹ�(�õ����ڴ���Ӧ����-1
//      			 2,δ֪
//      			 3,��ʼ�͵�ƽ����
//      			 4,�����ߵ�ƽ��ʱ
************************************************************************************************************************
*/
u8 recive_bit(u8 pose)
{
	u8 count=0,error_id = 9;
	switch(pose)
	{
		case 0:
			error_id = wait_if_sta(0,40,80,1);		//560us�ĵ͵�ƽ�źš��͵�ƽ����ʱ�䲻����300us���ȴ��ߵ�ƽʱ�䲻����800us��
			if(error_id == 0)
			{
				while(count++ < 200)
				{
					delay_us(10);
					if(LED_L == 0)			//�ߵ�ƽ����ʱ���ж�	//560Ϊ��׼ֵ---0	//1680Ϊ��׼ֵ---1
					{
						if(count > 30 && count < 80)
							return 0;
						else if(count > 140 && count < 180)
							return 1;
					}
				}
				if(count >= 200) 
					error_id = 4;			
			}
			else
				error_id = error_id;
			break;
		case 1:
			error_id = wait_if_sta(1,40,80,1);		//560us�ĵ͵�ƽ�źš��͵�ƽ����ʱ�䲻����300us���ȴ��ߵ�ƽʱ�䲻����800us��
			if(error_id == 0)
			{
				while(count++ < 200)
				{
					delay_us(10);
					if(LED_M == 0) 
					{
						if(count > 30 && count < 80)
							return 0;
						else if(count > 140 && count < 180)
							return 1;
					}
				}
				if(count >= 200) 	error_id = 3;				
			}
			break;
		case 2:
			error_id = wait_if_sta(2,40,80,1);		//560us�ĵ͵�ƽ�źš��͵�ƽ����ʱ�䲻����300us���ȴ��ߵ�ƽʱ�䲻����800us��
			if(error_id == 0)
			{
				while(count++ < 200)
				{
					delay_us(10);
					if(LED_R == 0) 
					{
						if(count > 30 && count < 80)
							return 0;
						else if(count > 140 && count < 180)
							return 1;
					}
				}
				if(count >= 200) 	error_id = 3;				
			}
			break;
		default:
			break;
	}
	return error_id;
}


