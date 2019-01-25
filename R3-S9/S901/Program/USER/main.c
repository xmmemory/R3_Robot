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

#define SEND_BUF_SIZE 128				//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
u8 SendBuff[SEND_BUF_SIZE];			//发送数据缓冲区

u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length);

/*******************************************UNION VARIABLES**************************************************************/
union recieveData							//接收到的数据
{
	float d;
	unsigned char data[4];
}temperature,humidity,air_velocity,air_quality,voice,h2_value,pm1_0,pm2_5,pm10,JiaQuan,WenDu,ShiDu;					//接收的传感器数据

//static u8 PM_INIT[] 		= 	{0x42, 0x4d, 0xe4, 0x00, 0x00, 0x01, 0x73};
//static u8 PM_GET[] 			= 	{0x42, 0x4d, 0xe2, 0x00, 0x00, 0x01, 0x71};
/************************************************************************************************************************/

int main()
{
	//u16 time_ms100=0,time_ms150=0,time_ms200=0,time_ms250=0,time_ms300=0,time_ms400=0;
	//int temp;
	
	int t=0;
	u16 h2_temp,Qu_temp,vo_temp;
//	static u8 Sensor_data[60];			//传感器数据缓存
	
	delay_init();
	Init_Tim5_Int();
	Init_Nvic();
	
	uart_init(115200);
	MYDMA_Config(DMA1_Channel4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA1通道4,外设为串口1,存储器为SendBuff,长度SEND_BUF_SIZE.
	
	uart3_init(9600);
	uart2_init(19200);
	IWDG_Init(4,0x0FFE);   	//与分频数为64,重载值为625,溢出时间为ns
	Ctr_Tim5(1);
	
//	DS18B20_Init();					//DS18B20初始化
	DHT11_Init();
//	AM2305_Init();
	
	LED_Init();
	Adc_Init();		  		//ADC初始化
		
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //使能串口1的DMA发送
	MYDMA_Enable(DMA1_Channel4,1);//开始一次DMA传输！

	while(1)
	{
		IWDG_Feed();	//喂狗
		
		if(t==1)
			LED0=!LED0;
		if(t==2)			//DHT11——每1000ms读取一次
		{
			DHT11_Read_Data();	//读取温湿度值		
			temperature.d = temp;
			humidity.d = humi;
		}
//		if(t==2)			//AM2305——每1000ms读取一次
//		{
//			AM2305_UpdateData();	//读取温湿度值		
		
//			temperature.d = (float)Temperature/10.0;
//			humidity.d = (float)Humidity/10.0;
//		}
//		if(t==2)			//DS18B20——每1000ms读取一次
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
			//实际值-2019-1-25-张鹏测算得		
			h2_value.d = h2_value.d*60.0;
			//Y轴下降、为了最小值达到0ppm
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
			//实际值-2018-11-2-张鹏测算得
			voice.d = 6.34*pow(voice.d,3)-32.39*pow(voice.d,2)+59.65*pow(voice.d,1)+32.23;
		}
		if(t==6)
		{
			Qu_temp = Get_Adc_Average(ADC_Channel_6,10);
			air_quality.d=(float)(Qu_temp*(3.3/4096));
			//实际值-2019-1-25-张鹏测算得
			air_quality.d = air_quality.d * 100.0;
			//Y轴下降、为了最小值达到0ppm
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
			IWDG_Feed();	//喂狗
			uart1_format_build_and_send(temperature.data,0x02,0x01,36);		//9个传感器数据每个4字节合计-36个字节
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
	
	if(length > 128)	return 2;
	//Header
	SendBuff[0] = 0xA6;
	SendBuff[1] = 0x59;
	//Length
	SendBuff[2] = length+2+2+3+4;		//数据长度+帧头+累加和+帧尾+（Length+Device ID+Command ID+register_address）;
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
	for(check_count = 0;check_count < length+6;check_count++)		//“length+6”数据长度+起始6个字节
	check_sum += SendBuff[check_count];
	//Check_Sum
	SendBuff[((length++)+6)] = (check_sum >> 8);
	SendBuff[((length++)+6)] = check_sum;
	//Tail
	SendBuff[((length++)+6)]='Y';			//添加结束符
	SendBuff[((length++)+6)]='H';			//添加结束符
	SendBuff[((length++)+6)]='T';			//添加结束符
	
	if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)	//判断通道4传输完成
	{
		DMA_ClearFlag(DMA1_FLAG_TC4);//清除通道4传输完成标志
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //使能串口1的DMA发送
		MYDMA_Enable(DMA1_Channel4,length+6);//开始一次DMA传输！
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

