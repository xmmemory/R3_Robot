#include "am2320.h"
#include "delay.h"

 //////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//AM2320数字温湿度传感器驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/12
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
      
//复位AM2320
void AM2320_Rst(void)	   
{
	AM2320_IO_OUT(); 	//SET OUTPUT
  AM2320_DQ_OUT=0; 	//拉低DQ
  delay_us(900);    	//拉低至少800us
  AM2320_DQ_OUT=1; 	//DQ=1
	delay_us(25);     //主机拉高20~200us
}
//等待AM2320的回应
//返回1:未检测到AM2320的存在
//返回0:存在
u8 AM2320_Check(void) 	   
{
	AM2320_IO_IN();		//SET INPUT
	TIM3->CNT = 0;
	while(TIM3->CNT < 72*5);		//5us后判断响应信号
	if(AM2320_DQ_IN == 1)				//如果未获取到低电平的响应信号、则未检测到AM2320的存在
		return 1;
	//检测到低电平后
	TIM3->CNT = 0;
	while(AM2320_DQ_IN == 0 && TIM3->CNT < 72*100);		//电平跳变到高电平或者时间超时到100us	
	if(TIM3->CNT > 72*100)															//超时
		return 1;
	TIM3->CNT = 0;
	//检测到高电平后
	while(AM2320_DQ_IN == 1 && TIM3->CNT < 72*100);		//电平跳变到低电平或者时间超时到100us
	if(TIM3->CNT < 72*75 && TIM3->CNT > 72*100)				//高电平持续时间75us到100us
		return 1;
	else
		return 0;
}
//从AM2320读取一个位
//返回值：1/0
u8 AM2320_Read_Bit(void) 			 
{
	TIM3->CNT = 0;
	while(AM2320_DQ_IN == 0 && TIM3->CNT < 72*80);		//电平跳变到高电平或者时间超时到48~55us		//示波器显示51us、但是程序在第二个字节接收开始便不通过；
			//实际程序测试发现、需求值为63us
	if(TIM3->CNT > 72*80)															//超时
	{
//		temperature = TIM3->CNT / 72;
		return 0; 
	}	
//	if(TIM3->CNT > 72*55)
//	{
//		temperature = TIM3->CNT / 72;
//	}
	TIM3->CNT = 0;
	while(AM2320_DQ_IN == 1 && TIM3->CNT < 72*100);		//电平跳变到低电平或者时间超时到100us	
	if(TIM3->CNT > 72*40)												//信号“1”高电平时间
	{
		return 1;
	}
	else
		return 0; 
//	else if(TIM3->CNT < 72*30)									//信号“0”高电平时间
//	{
//		return 0; 
//	} 

}

u8 iii;

//从AM2320读取一个字节
//返回值：读到的数据
u8 AM2320_Read_Byte(void)    
{        
   u8 j,dat;
   dat=0;
	
	for (j=0;j<8;j++) 
	{
   		dat<<=1; 
	    dat|=AM2320_Read_Bit();
   }
   return dat;
}
//从AM2320读取一次数据
//temp:温度值(范围:0~50°)
//humi:湿度值(范围:20%~90%)
//返回值：0,正常;1,读取失败
u16 temperature_u16=0,humidity_u16=0;

u8 AM2320_Read_Data(void)    
{        
 	u8 buf[5];

	AM2320_Rst();
	if(AM2320_Check()==0)
	{
//		for(iii=0;iii<5;iii++)//读取40位数据
//		{
			buf[0]=AM2320_Read_Byte();
			buf[1]=AM2320_Read_Byte();
			buf[2]=AM2320_Read_Byte();
			buf[3]=AM2320_Read_Byte();
			buf[4]=AM2320_Read_Byte();
//		}
		if((u8)(buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
		{
			humidity_u16 = (buf[0] << 8)+ buf[1];
			temperature_u16 = (buf[2] << 8)+ buf[3];
		}
	}
	else 
	return 1;
	return 0;	    
}
//初始化AM2320的IO口 DQ 同时检测AM2320的存在
//返回1:不存在
//返回0:存在    	 
u8 AM2320_Init(void)
{	 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PG端口时钟
	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;				 //PG11端口配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);				 //初始化IO口
 	GPIO_SetBits(GPIOB,GPIO_Pin_14);						 //PG11 输出高
			    
	AM2320_Rst();  //复位AM2320
	return AM2320_Check();//等待AM2320的回应
} 


