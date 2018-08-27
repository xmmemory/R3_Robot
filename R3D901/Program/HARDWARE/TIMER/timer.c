#include "timer.h"
//#include "led.h"
#include "encoder.h"
//#include "odometry.h"
//////////////////////////////////////////////////////////////////////////////////	 
//��ʱ����ʼ�����ж�		
//********************************************************************************
//�޸�����:2017/12/31
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ��������(����)���ܻ����˿Ƽ����޹�˾ 
//All rights reserved
//********************************************************************************

//ͨ�ö�ʱ��5�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��5!
void TIM5_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  ///ʹ��TIM5ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);//��ʼ��TIM5
	
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE); //����ʱ��5�����ж�
	TIM_Cmd(TIM5,ENABLE); //ʹ�ܶ�ʱ��5
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM5_IRQn; //��ʱ��5�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//ͨ�ö�ʱ��2�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��2!
void TIM2_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM2ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM2
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��2�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��2
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��ʱ��2�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x02; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x02; //�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
/***************************************************************************************************/
//��ʱ��5�жϷ�����---5ms
void TIM5_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)==SET) //����ж�
	{
		if(odom_cal_status)		//��һ�ε���̼����ݻ�δ��ȡ
		{
			TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //����жϱ�־λ
			return;
		}			
		//ִ����̼Ʋ�����
		PULSE_RIGHT = ENCODER_RIGHT_TIMER->CNT - 32768;	//B ��ȡ�ı�����
		ENCODER_RIGHT_TIMER->CNT = 32768;
		PULSE_LEFT = ENCODER_LEFT_TIMER->CNT - 32768;	//A ��ȡ�ı�����
		ENCODER_LEFT_TIMER->CNT = 32768;

		odom_cal_status = 1;//ִ�м�����̼����ݲ���        	
	}
	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);  //����жϱ�־λ
}

//��ʱ��2�жϷ�����---20ms
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //����ж�
	{
		;
	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ
}

