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

union recieveData							//数据
{
float d;
unsigned char data[4];
}temperature,humidity,air_velocity,air_quality,voice,h2_value,pm1_0,pm2_5,pm10;					//传感器数据

#define SEND_BUF_SIZE 128				//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
u8 SendBuff[SEND_BUF_SIZE];			//发送数据缓冲区
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length);

/************************************************
ALIENTEK战舰STM32开发板实验48
USB虚拟串口 实验 
技术支持：www.openedv.com
淘宝店铺：http://eboard.taobao.com 
关注微信公众平台微信号："正点原子"，免费获取STM32资料。
广州市星翼电子科技有限公司  
作者：正点原子 @ALIENTEK
************************************************/


int main(void)
{	 
	u16 t;
	//	u16 len;	
	//	u16 times=0;    
	u8 usbstatus=0;	
	u16 h2_temp,Qu_temp,vo_temp;
	
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	TIM3_Int_Init();//10Khz的计数频率，计数到5000为500ms
	TIM_Cmd(TIM3, ENABLE);  //使能TIM3
	 
	uart_init(115200);	 	//串口初始化为115200
	uart2_init(9600);	 
	uart3_init(19200);	 
	 
	Adc_Init();		  		//ADC初始化

	while(AM2320_Init())
	{
		LED1 = !LED1;
		
		delay_ms(100);
	}
		 
 	LED_Init();		  		//初始化与LED连接的硬件接口
	// 	LCD_ShowString(30,130,200,16,16,"USB Connecting...");//提示USB开始连接
	delay_ms(1800);
	USB_Port_Set(0); 	//USB先断开
	delay_ms(700);
	USB_Port_Set(1);	//USB再次连接
	Set_USBClock();   
	USB_Interrupts_Config();    
	USB_Init();
	
while(1)
{
	if(usbstatus!=bDeviceState)//USB连接状态发生了改变.
	{
		usbstatus=bDeviceState;//记录新的状态
		if(usbstatus==CONFIGURED)
		{
//				POINT_COLOR=BLUE;
//				LCD_ShowString(30,130,200,16,16,"USB Connected    ");//提示USB连接成功
			LED1=0;//DS1亮
		}else
		{
//				POINT_COLOR=RED;
//				LCD_ShowString(30,130,200,16,16,"USB disConnected ");//提示USB断开
			LED1=1;//DS1灭
		}
	}
	
	if(t==1)
	{
		AM2320_Read_Data();	//读取温湿度值
		temperature.d = temperature_u16/10.0f;
		humidity.d = humidity_u16/10.0f;
	}
	if(t==2)
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
//			IWDG_Feed();	//喂狗
		LED0=!LED0;//闪烁LED,提示系统正在运行.
		uart1_format_build_and_send(temperature.data,0x02,0x01,36);		//9个传感器数据每个4字节合计-36个字节
	}
	
	delay_ms(200);			//循环频率为（延时时间*10）
	
	//任务切换	
	if((t++) > 10)
	t = 0;
	
//		if(USB_USART_RX_STA&0x8000)
//		{					   
//			len=USB_USART_RX_STA&0x3FFF;//得到此次接收到的数据长度
//			usb_printf("\r\n您发送的消息为:%d\r\n\r\n",len);
//			for(t=0;t<len;t++)
//			{
//				USB_USART_SendData(USB_USART_RX_BUF[t]);//以字节方式,发送给USB 
//			}
//			usb_printf("\r\n\r\n");//插入换行
//			USB_USART_RX_STA=0;
//		}
//		else
//		{
//			times++;
//			if(times%5000==0)
//			{
//				usb_printf("\r\n战舰STM32开发板USB虚拟串口实验\r\n");
//				usb_printf("正点原子@ALIENTEK\r\n\r\n");
//			}
//			if(times%200==0)usb_printf("请输入数据,以回车键结束\r\n");  
//			if(times%30==0)LED0=!LED0;//闪烁LED,提示系统正在运行.
//			delay_ms(10);   
//		}
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
u8 j,check_count,i;

//	if(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
//		return 1;	

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

for(i=0;i<length+6;i++)
{
	USB_USART_SendData(SendBuff[i]);//以字节方式,发送给USB
	
//		USART_ClearFlag(USART1,USART_FLAG_TC);  //在发送第一个数据前加此句，解决第一个数据不能正常发送的问题				
//		USART_SendData(USART1,SendBuff[i]);//发送一个字节到串口
//		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);	//等待发送结束
} 

//	if(DMA_GetFlagStatus(DMA1_FLAG_TC4)!=RESET)	//判断通道4传输完成
//	{
//		DMA_ClearFlag(DMA1_FLAG_TC4);//清除通道4传输完成标志
//		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE); //使能串口1的DMA发送
//		MYDMA_Enable(DMA1_Channel4,length+6);//开始一次DMA传输！
//		return 0;
//	}
//	else
//		return 3;
	return 0;
}


