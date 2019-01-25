#include "sys.h"
#include "delay.h"
#include "usart.h" 
#include "led.h"
#include "usb_lib.h"
#include "hw_config.h"
#include "usb_pwr.h"	 
#include "am2320.h"
#include "usart2.h"
#include "adc.h" 
#include "timer.h"
#include "math.h"

union recieveData							//����
{
float d;
unsigned char data[4];
}temperature,humidity,air_velocity,air_quality,voice,h2_value,pm1_0,pm2_5,pm10;					//����������

#define SEND_BUF_SIZE 128				//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
u8 SendBuff[SEND_BUF_SIZE];			//�������ݻ�����
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length);

/************************************************
ALIENTEKս��STM32������ʵ��48
USB���⴮�� ʵ�� 
����֧�֣�www.openedv.com
�Ա����̣�http://eboard.taobao.com 
��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
������������ӿƼ����޹�˾  
���ߣ�����ԭ�� @ALIENTEK
************************************************/


int main(void)
{	 
	u16 t;
	//	u16 len;	
	//	u16 times=0;    
	u8 usbstatus=0;	
	u16 h2_temp,Qu_temp,vo_temp;
	
	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	TIM3_Int_Init();//10Khz�ļ���Ƶ�ʣ�������5000Ϊ500ms
	TIM_Cmd(TIM3, ENABLE);  //ʹ��TIM3
	 
	uart_init(115200);	 	//���ڳ�ʼ��Ϊ115200
	uart2_init(9600);	 
	uart3_init(19200);	 
	 
	Adc_Init();		  		//ADC��ʼ��

	while(AM2320_Init())
	{
		LED1 = !LED1;
		
		delay_ms(100);
	}
		 
 	LED_Init();		  		//��ʼ����LED���ӵ�Ӳ���ӿ�
	// 	LCD_ShowString(30,130,200,16,16,"USB Connecting...");//��ʾUSB��ʼ����
	delay_ms(1800);
	USB_Port_Set(0); 	//USB�ȶϿ�
	delay_ms(700);
	USB_Port_Set(1);	//USB�ٴ�����
	Set_USBClock();   
	USB_Interrupts_Config();    
	USB_Init();
	
while(1)
{
	if(usbstatus!=bDeviceState)//USB����״̬�����˸ı�.
	{
		usbstatus=bDeviceState;//��¼�µ�״̬
		if(usbstatus==CONFIGURED)
		{
//				POINT_COLOR=BLUE;
//				LCD_ShowString(30,130,200,16,16,"USB Connected    ");//��ʾUSB���ӳɹ�
			LED1=0;//DS1��
		}else
		{
//				POINT_COLOR=RED;
//				LCD_ShowString(30,130,200,16,16,"USB disConnected ");//��ʾUSB�Ͽ�
			LED1=1;//DS1��
		}
	}
	
	if(t==1)
	{
		AM2320_Read_Data();	//��ȡ��ʪ��ֵ
		temperature.d = temperature_u16/10.0f;
		humidity.d = humidity_u16/10.0f;
	}
	if(t==2)
		air_velocity.d = GetWindSpeed()/10.0;
	if(t==4)
	{
		h2_temp = Get_Adc_Average(ADC_Channel_4,10);			
		h2_value.d = (float)h2_temp*(3.3/4096);
		//ʵ��ֵ-2019-1-25-���������		
		h2_value.d = h2_value.d*60.0;
		//Y���½���Ϊ����Сֵ�ﵽ0ppm
		if(h2_value.d < 2)
			h2_value.d = 0;
		else
			h2_value.d = h2_value.d - 2;
	}
	if(t==5)
	{
		vo_temp = Get_Adc_Average(ADC_Channel_5,10);
//			voice.d = (voice.d + (float)vo_temp*(3.3/4096))/2;
		voice.d = (float)vo_temp*(3.3/4096);
		//ʵ��ֵ-2018-11-2-���������
		voice.d = 6.34*pow(voice.d,3)-32.39*pow(voice.d,2)+59.65*pow(voice.d,1)+32.23;
	}
	if(t==6)
	{
		Qu_temp = Get_Adc_Average(ADC_Channel_6,10);
		air_quality.d=(float)(Qu_temp*(3.3/4096));
		//ʵ��ֵ-2019-1-25-���������
		air_quality.d = air_quality.d * 100.0;
		//Y���½���Ϊ����Сֵ�ﵽ0ppm
		if(air_quality.d < 2)
			air_quality.d = 0;
		else
			air_quality.d = air_quality.d - 2;
	}		
	if(t==7)
	{			
		pm1_0.d = (float)GetPM1_0();
		pm2_5.d = (float)GetPM2_5();
		pm10.d	= (float)GetPM10();
	}
	if(t==9)
	{
//			IWDG_Feed();	//ι��
		LED0=!LED0;//��˸LED,��ʾϵͳ��������.
		uart1_format_build_and_send(temperature.data,0x02,0x01,36);		//9������������ÿ��4�ֽںϼ�-36���ֽ�
	}
	
	delay_ms(200);			//ѭ��Ƶ��Ϊ����ʱʱ��*10��
	
	//�����л�	
	if((t++) > 10)
	t = 0;
	
//		if(USB_USART_RX_STA&0x8000)
//		{					   
//			len=USB_USART_RX_STA&0x3FFF;//�õ��˴ν��յ������ݳ���
//			usb_printf("\r\n�����͵���ϢΪ:%d\r\n\r\n",len);
//			for(t=0;t<len;t++)
//			{
//				USB_USART_SendData(USB_USART_RX_BUF[t]);//���ֽڷ�ʽ,���͸�USB 
//			}
//			usb_printf("\r\n\r\n");//���뻻��
//			USB_USART_RX_STA=0;
//		}
//		else
//		{
//			times++;
//			if(times%5000==0)
//			{
//				usb_printf("\r\nս��STM32������USB���⴮��ʵ��\r\n");
//				usb_printf("����ԭ��@ALIENTEK\r\n\r\n");
//			}
//			if(times%200==0)usb_printf("����������,�Իس�������\r\n");  
//			if(times%30==0)LED0=!LED0;//��˸LED,��ʾϵͳ��������.
//			delay_ms(10);   
//		}
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
u8 j,check_count,i;

//	if(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
//		return 1;	

if(length > 128)	return 2;
//Header
SendBuff[0] = 0xA6;
SendBuff[1] = 0x59;
//Length
SendBuff[2] = length+2+2+3+4;		//���ݳ���+֡ͷ+�ۼӺ�+֡β+��Length+Device ID+Command ID+register_address��;
//Device ID
SendBuff[3] = device_id;
//Command ID
SendBuff[4] = commond_id;
//Register Address
SendBuff[5] = 0x01;				//register_address;
//Data
for(j=0;j<length;j++)
{
	SendBuff[j+6]=*(arg+j);
}
//Check_Sum_Cal
for(check_count = 0;check_count < length+6;check_count++)		//��length+6�����ݳ���+��ʼ6���ֽ�
check_sum += SendBuff[check_count];
//Check_Sum
SendBuff[((length++)+6)] = (check_sum >> 8);
SendBuff[((length++)+6)] = check_sum;
//Tail
SendBuff[((length++)+6)]='Y';			//��ӽ�����
SendBuff[((length++)+6)]='H';			//��ӽ�����
SendBuff[((length++)+6)]='T';			//��ӽ�����

for(i=0;i<length+6;i++)
{
	USB_USART_SendData(SendBuff[i]);//���ֽڷ�ʽ,���͸�USB
	
//		USART_ClearFlag(USART1,USART_FLAG_TC);  //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����				
//		USART_SendData(USART1,SendBuff[i]);//����һ���ֽڵ�����
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//�ȴ����ͽ���
} 

//	if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)	//�ж�ͨ��4�������
//	{
//		DMA_ClearFlag(DMA1_FLAG_TC4);//���ͨ��4������ɱ�־
//		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //ʹ�ܴ���1��DMA����
//		MYDMA_Enable(DMA1_Channel4,length+6);//��ʼһ��DMA���䣡
//		return 0;
//	}
//	else
//		return 3;
	return 0;
}


