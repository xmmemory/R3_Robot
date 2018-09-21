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

#define SEND_BUF_SIZE 48	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
u8 SendBuff[SEND_BUF_SIZE];	//发送数据缓冲区

u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length);

/*******************************************UNION VARIABLES**************************************************************/
union recieveData							//接收到的数据
{
	float d;
	unsigned char data[4];
}humidity,temperature,air_velocity,air_quality,voice,h2_value,pm1_0,pm2_5,pm10;					//接收的传感器数据

//static u8 PM_INIT[] 		= 	{0x42, 0x4d, 0xe4, 0x00, 0x00, 0x01, 0x73};
//static u8 PM_GET[] 			= 	{0x42, 0x4d, 0xe2, 0x00, 0x00, 0x01, 0x71};
/************************************************************************************************************************/

int main()
{
	//u16 time_ms100=0,time_ms150=0,time_ms200=0,time_ms250=0,time_ms300=0,time_ms400=0;
	//int temp;
	
	int t=0,i;
	u16 h2_temp,Qu_temp,vo_temp;
	static u8 Sensor_data[60];			//传感器数据缓存
	
	delay_init();
	Init_Tim5_Int();
	Init_Nvic();
	
	uart_init(115200);
	MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA1通道4,外设为串口1,存储器为SendBuff,长度SEND_BUF_SIZE.
	
	uart3_init(9600);
	uart2_init(19200);
	IWDG_Init(4,625);   	//与分频数为64,重载值为625,溢出时间为1s
	Ctr_Tim5(1);
	DHT11_Init();
	LED_Init();
	Adc_Init();		  		//ADC初始化
		
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //使能串口1的DMA发送
	MYDMA_Enable(DMA1_Channel4);//开始一次DMA传输！

	while(1)
	{
		IWDG_Feed();	//喂狗
		
		if(t==1)
			LED0=!LED0;
		if(t==2)			//每1000ms读取一次
		{
			DHT11_Read_Data();	//读取温湿度值		
			temperature.d = Temp();
			humidity.d = Humi();
		}
		if(t==3)
			air_velocity.d = (GetWindSpeed()/10.0);
		if(t==4)
		{
			h2_temp = Get_Adc_Average(ADC_Channel_4,10);			
			h2_value.d = (float)h2_temp*(3.3/4096);		
		}		
		if(t==5)
		{
			vo_temp = Get_Adc_Average(ADC_Channel_5,10);
//			voice.d = (voice.d + (float)vo_temp*(3.3/4096))/2;
			voice.d = (float)vo_temp*(3.3/4096);
		}
		if(t==6)
		{
			Qu_temp = Get_Adc_Average(ADC_Channel_6,10);
			air_quality.d=(float)(Qu_temp*(3.3/4096));
//			air_quality.d = air_quality.d * 100;
		}
		if(t==7)
		{			
			pm1_0.d = (float)GetPM1_0();
			pm2_5.d = (float)GetPM2_5();
			pm10.d	= (float)GetPM10();
		}
		if(t==9)
		{
			IWDG_Feed();	//喂狗
			for(i=0;i<4;i++)
			{
				Sensor_data[i] = temperature.data[i];
				Sensor_data[i+4] = humidity.data[i];
				Sensor_data[i+8] = air_velocity.data[i];	
				Sensor_data[i+12] = air_quality.data[i];
				Sensor_data[i+16] = voice.data[i];
				Sensor_data[i+20] = h2_value.data[i];
				Sensor_data[i+24] = pm1_0.data[i];
				Sensor_data[i+28] = pm2_5.data[i];
				Sensor_data[i+32] = pm10.data[i];
			}
			i = 36;
			//数据推送

			uart1_format_build_and_send(Sensor_data,0x02,0x01,36);		//9个传感器数据每个4字节合计-36个字节。
//		printf("%f,%f,%f,%f,%f,%f,%f,%f,%f",pm1_0,pm2_5,pm10,air_velocity,f_temperature,f_humidity,vo_value,h2_value,Qu_value);
		}
		
		IWDG_Feed();			//喂狗
		delay_ms(10);			//循环频率为（延时时间*10）
		
		//任务切换	
		if((t++) > 10)		
		t = 0;
	}
	
}

/*
************************************************************************************************************************
*                                                   UART1_SEND
* Description: 串口数据格式整理、然后发送消息
* Arguments  : u8* arg,u8 length;
* Note(s)    : 1) Null；
************************************************************************************************************************
*/
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length)
{	
	u16 check_sum;
	u8 j,check_count;
	
	if(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		return 1;	
	
	if(length > 64)	return 2;
	//Header
	SendBuff[0] = 0xA6;
	SendBuff[1] = 0x59;
	//Length
	SendBuff[2] = length+2;
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
	for(check_count = 0;check_count < length;check_count++)
	check_sum = SendBuff[check_count];
	//Check_Sum
	SendBuff[((length++)+5)] = (check_sum >> 8);
	SendBuff[((length++)+5)] = check_sum;
	//Tail
	SendBuff[((length++)+5)]='\n';		//添加结束符
	SendBuff[((length++)+5)]='\r';		//添加结束符
	SendBuff[((length++)+5)]='Y';			//添加结束符
	SendBuff[((length++)+5)]='H';			//添加结束符
	SendBuff[((length++)+5)]='T';			//添加结束符
	
	if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)	//判断通道4传输完成
	{
		DMA_ClearFlag(DMA1_FLAG_TC4);//清除通道4传输完成标志
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //使能串口1的DMA发送
		MYDMA_Enable(DMA1_Channel4);//开始一次DMA传输！
		return 0;
	}
	else
		return 3;
//	for(i=0;i<length+5;i++)
//	{
//		USART_ClearFlag(USART1,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题				
//		USART_SendData(USART1,SendBuff[i]);//发送一个字节到串口
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//等待发送结束
//	} 		
}

