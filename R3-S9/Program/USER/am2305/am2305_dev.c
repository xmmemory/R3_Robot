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
 	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PG�˿�ʱ��
	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;				 //PB13�˿�����
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);				 //��ʼ��IO��
 	GPIO_SetBits(GPIOB,GPIO_Pin_13);						 //PG11 �����
			    
	AM2305_RST();
	
	return AM2305_Check(); 
}

void AM2305_RST(void)
{
	/*��λAM2305*/
	AM2305_GPIO_OUT(); 	//SET OUTPUT
	AM2305_DATA_OUT=0; 	//����DQ
	delay_ms(20);    	//��������18ms
	AM2305_DATA_OUT=1; 	//DQ=1 
	delay_us(30);     	//��������20~40us
}
//�ȴ�AM2305�Ļ�Ӧ
//����1:δ��⵽AM2305�Ĵ���
//����0:����
u8 AM2305_Check(void) 	   
{   
	u8 retry=0;
	AM2305_GPIO_IN();//SET INPUT	 
  while (AM2305_DATA_IN&&retry<100)//DHT11������40~80us
	{
		retry++;
		delay_us(1);
	};	 
	if(retry>=100)return 1;
	else retry=0;
  while (!AM2305_DATA_IN&&retry<100)//DHT11���ͺ���ٴ�����40~80us
	{
		retry++;
		delay_us(1);
	};
	if(retry>=100)return 1;	    
	return 0;
}
//��AM2305��ȡһ��λ
//����ֵ��1/0
u8 AM2305_Read_Bit(void) 			 
{
 	u8 retry=0;
	while(AM2305_DATA_IN&&retry<120)//�ȴ���Ϊ�͵�ƽ
	{
		retry++;
		delay_us(1);
	}
	retry=0;
	while(!AM2305_DATA_IN&&retry<78)//�ȴ���ߵ�ƽ
	{
		retry++;
		delay_us(1);
	}
	delay_us(40);//�ȴ�40us
	if(AM2305_DATA_IN)return 1;
	else return 0;		   
}
//��AM2305��ȡһ���ֽ�
//����ֵ������������
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
