#include "am2320.h"
#include "delay.h"

 //////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//AM2320������ʪ�ȴ�������������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/12
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
      
//��λAM2320
void AM2320_Rst(void)	   
{
	AM2320_IO_OUT(); 	//SET OUTPUT
  AM2320_DQ_OUT=0; 	//����DQ
  delay_us(900);    	//��������800us
  AM2320_DQ_OUT=1; 	//DQ=1
	delay_us(25);     //��������20~200us
}
//�ȴ�AM2320�Ļ�Ӧ
//����1:δ��⵽AM2320�Ĵ���
//����0:����
u8 AM2320_Check(void) 	   
{
	AM2320_IO_IN();		//SET INPUT
	TIM3->CNT = 0;
	while(TIM3->CNT < 72*5);		//5us���ж���Ӧ�ź�
	if(AM2320_DQ_IN == 1)				//���δ��ȡ���͵�ƽ����Ӧ�źš���δ��⵽AM2320�Ĵ���
		return 1;
	//��⵽�͵�ƽ��
	TIM3->CNT = 0;
	while(AM2320_DQ_IN == 0 && TIM3->CNT < 72*100);		//��ƽ���䵽�ߵ�ƽ����ʱ�䳬ʱ��100us	
	if(TIM3->CNT > 72*100)															//��ʱ
		return 1;
	TIM3->CNT = 0;
	//��⵽�ߵ�ƽ��
	while(AM2320_DQ_IN == 1 && TIM3->CNT < 72*100);		//��ƽ���䵽�͵�ƽ����ʱ�䳬ʱ��100us
	if(TIM3->CNT < 72*75 && TIM3->CNT > 72*100)				//�ߵ�ƽ����ʱ��75us��100us
		return 1;
	else
		return 0;
}
//��AM2320��ȡһ��λ
//����ֵ��1/0
u8 AM2320_Read_Bit(void) 			 
{
	TIM3->CNT = 0;
	while(AM2320_DQ_IN == 0 && TIM3->CNT < 72*80);		//��ƽ���䵽�ߵ�ƽ����ʱ�䳬ʱ��48~55us		//ʾ������ʾ51us�����ǳ����ڵڶ����ֽڽ��տ�ʼ�㲻ͨ����
			//ʵ�ʳ�����Է��֡�����ֵΪ63us
	if(TIM3->CNT > 72*80)															//��ʱ
	{
//		temperature = TIM3->CNT / 72;
		return 0; 
	}	
//	if(TIM3->CNT > 72*55)
//	{
//		temperature = TIM3->CNT / 72;
//	}
	TIM3->CNT = 0;
	while(AM2320_DQ_IN == 1 && TIM3->CNT < 72*100);		//��ƽ���䵽�͵�ƽ����ʱ�䳬ʱ��100us	
	if(TIM3->CNT > 72*40)												//�źš�1���ߵ�ƽʱ��
	{
		return 1;
	}
	else
		return 0; 
//	else if(TIM3->CNT < 72*30)									//�źš�0���ߵ�ƽʱ��
//	{
//		return 0; 
//	} 

}

u8 iii;

//��AM2320��ȡһ���ֽ�
//����ֵ������������
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
//��AM2320��ȡһ������
//temp:�¶�ֵ(��Χ:0~50��)
//humi:ʪ��ֵ(��Χ:20%~90%)
//����ֵ��0,����;1,��ȡʧ��
u16 temperature_u16=0,humidity_u16=0;

u8 AM2320_Read_Data(void)    
{        
 	u8 buf[5];

	AM2320_Rst();
	if(AM2320_Check()==0)
	{
//		for(iii=0;iii<5;iii++)//��ȡ40λ����
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
//��ʼ��AM2320��IO�� DQ ͬʱ���AM2320�Ĵ���
//����1:������
//����0:����    	 
u8 AM2320_Init(void)
{	 
 	GPIO_InitTypeDef  GPIO_InitStructure;
 	
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //ʹ��PG�˿�ʱ��
	
 	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;				 //PG11�˿�����
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);				 //��ʼ��IO��
 	GPIO_SetBits(GPIOB,GPIO_Pin_14);						 //PG11 �����
			    
	AM2320_Rst();  //��λAM2320
	return AM2320_Check();//�ȴ�AM2320�Ļ�Ӧ
} 


