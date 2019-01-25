#ifndef _AM2305_DEV_C
#define _AM2305_DEV_C
#include "am2305_dev.h"
#include "delay.h"

short  Temperature = 0;
short  Humidity = 0;

static u8 AM2305_Check(void);
static u8 AM2305_Read_Bit(void);
static u8 AM2305_Read_Byte(void);
static void AM2305_RST(void);

u8 AM2305_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PG端口时钟
	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 //PB13端口配置
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);				 //初始化IO口
 	GPIO_SetBits(GPIOB,GPIO_Pin_13);						 //PG11 输出高
			    
	AM2305_RST();
	
	return AM2305_Check(); 
}

void AM2305_RST(void)
{
	/*复位AM2305*/
	AM2305_GPIO_OUT(); 	//SET OUTPUT
	AM2305_DATA_OUT=0; 	//拉低DQ
	delay_ms(20);    	//拉低至少18ms
	AM2305_DATA_OUT=1; 	//DQ=1 
	delay_us(30);     	//主机拉高20~40us
}
//等待AM2305的回应
//返回1:未检测到AM2305的存在
//返回0:存在
u8 AM2305_Check(void) 	   
{   
	u8 retry=0;
	AM2305_GPIO_IN();//SET INPUT	 
  while (AM2305_DATA_IN&&retry<100)//DHT11会拉低40~80us
	{
		retry++;
		delay_us(1);
	};	 
	if(retry>=100)return 1;
	else retry=0;
  while (!AM2305_DATA_IN&&retry<100)//DHT11拉低后会再次拉高40~80us
	{
		retry++;
		delay_us(1);
	};
	if(retry>=100)return 1;	    
	return 0;
}
//从AM2305读取一个位
//返回值：1/0
u8 AM2305_Read_Bit(void) 			 
{
 	u8 retry=0;
	while(AM2305_DATA_IN&&retry<120)//等待变为低电平
	{
		retry++;
		delay_us(1);
	}
	retry=0;
	while(!AM2305_DATA_IN&&retry<78)//等待变高电平
	{
		retry++;
		delay_us(1);
	}
	delay_us(40);//等待40us
	if(AM2305_DATA_IN)return 1;
	else return 0;		   
}
//从AM2305读取一个字节
//返回值：读到的数据
u8 AM2305_Read_Byte(void)    
{        
    u8 i,dat;
    dat=0;
	for (i=0;i<8;i++) 
	{
   		dat<<=1; 
	    dat|=AM2305_Read_Bit();
    }						    
    return dat;
}

void AM2305_UpdateData(void)
{
	u8 buf[5];
	u8 i; 
	
	AM2305_RST();
	if (AM2305_Check() == 0)
	{
		for (i = 0; i < 5; i++)
		{
			buf[i] = AM2305_Read_Byte();
		}		
		Humidity 	= ((buf[0] * 256) + (buf[1]));	   
		Temperature = ((buf[2] * 256) + (buf[3]));
	}
}

#endif
