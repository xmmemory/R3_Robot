#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "usart2.h"
#include "TIM5_INT.h"
#include "stm32f10x_it.h"
#include "NVIC_CONFIG.h"
#include "math.h"
#include "dog.h"
#include "dht11.h"
#include "led.h"
#include "adc.h"
#include "dma.h"
#include "am2305_dev.h"
#include "ds18b20.h" 

#define SEND_BUF_SIZE 128				//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.
u8 SendBuff[SEND_BUF_SIZE];			//�������ݻ�����

u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length);

/*******************************************UNION VARIABLES**************************************************************/
union recieveData							//���յ�������
{
	float d;
	unsigned char data[4];
}temperature,humidity,air_velocity,air_quality,voice,h2_value,pm1_0,pm2_5,pm10,JiaQuan,WenDu,ShiDu;					//���յĴ���������

//static u8 PM_INIT[] 		= 	{0x42, 0x4d, 0xe4, 0x00, 0x00, 0x01, 0x73};
//static u8 PM_GET[] 			= 	{0x42, 0x4d, 0xe2, 0x00, 0x00, 0x01, 0x71};
/************************************************************************************************************************/

int main()
{
	//u16 time_ms100=0,time_ms150=0,time_ms200=0,time_ms250=0,time_ms300=0,time_ms400=0;
	//int temp;
	
	int t=0;
	u16 h2_temp,Qu_temp,vo_temp;
//	static u8 Sensor_data[60];			//���������ݻ���
	
	delay_init();
	Init_Tim5_Int();
	Init_Nvic();
	
	uart_init(115200);
	MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA1ͨ��4,����Ϊ����1,�洢��ΪSendBuff,����SEND_BUF_SIZE.
	
	uart3_init(9600);
	uart2_init(19200);
	IWDG_Init(4,0x0FFE);   	//���Ƶ��Ϊ64,����ֵΪ625,���ʱ��Ϊns
	Ctr_Tim5(1);
	
//	DS18B20_Init();					//DS18B20��ʼ��
	DHT11_Init();
//	AM2305_Init();
	
	LED_Init();
	Adc_Init();		  		//ADC��ʼ��
		
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //ʹ�ܴ���1��DMA����
	MYDMA_Enable(DMA1_Channel4,1);//��ʼһ��DMA���䣡

	while(1)
	{
		IWDG_Feed();	//ι��
		
		if(t==1)
			LED0=!LED0;
		if(t==2)			//DHT11����ÿ1000ms��ȡһ��
		{
			DHT11_Read_Data();	//��ȡ��ʪ��ֵ		
			temperature.d = temp;
			humidity.d = humi;
		}
//		if(t==2)			//AM2305����ÿ1000ms��ȡһ��
//		{
//			AM2305_UpdateData();	//��ȡ��ʪ��ֵ		
		
//			temperature.d = (float)Temperature/10.0;
//			humidity.d = (float)Humidity/10.0;
//		}
//		if(t==2)			//DS18B20����ÿ1000ms��ȡһ��
//		{	
//			temperature.d = DS18B20_Get_Temp()/10.0;
//			humidity.d = 22;
//		}
		if(t==3)
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
			IWDG_Feed();	//ι��
			uart1_format_build_and_send(temperature.data,0x02,0x01,36);		//9������������ÿ��4�ֽںϼ�-36���ֽ�
		}
		
		IWDG_Feed();			//ι��
		delay_ms(10);			//ѭ��Ƶ��Ϊ����ʱʱ��*10��
		
		//�����л�	
		if((t++) > 10)		
		t = 0;
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
	u8 j,check_count;
	
	if(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		return 1;	
	
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
	
	if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)	//�ж�ͨ��4�������
	{
		DMA_ClearFlag(DMA1_FLAG_TC4);//���ͨ��4������ɱ�־
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //ʹ�ܴ���1��DMA����
		MYDMA_Enable(DMA1_Channel4,length+6);//��ʼһ��DMA���䣡
		return 0;
	}
	else
		return 3;
//	for(i=0;i<length+5;i++)
//	{
//		USART_ClearFlag(USART1,USART_FLAG_TC);  //�ڷ��͵�һ������ǰ�Ӵ˾䣬�����һ�����ݲ����������͵�����				
//		USART_SendData(USART1,SendBuff[i]);//����һ���ֽڵ�����
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//�ȴ����ͽ���
//	} 		
}

