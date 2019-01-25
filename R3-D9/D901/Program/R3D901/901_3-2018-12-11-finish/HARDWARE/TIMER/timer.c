//********************************************************************************
//定时器初始化、中断		
//修改日期:2017/12/31
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 光线量子(北京)智能机器人科技有限公司 
//All rights reserved
//******************************************************************************** 
#include "timer.h"
#include "led.h"
#include "odometry.h"
#include "encoder.h"
//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//
//
//
void TIM6_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);  							//使能TIM7时钟
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  										//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 		//向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period=arr;   											//自动重装载值
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); 													//允许定时器6更新中断
	TIM_Cmd(TIM6,ENABLE); 																						//使能定时器6
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn; 								//定时器6中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; 				//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x02; 							//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//
//
//
void TIM7_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  							//使能TIM7时钟
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  										//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; 		//向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period=arr;   											//自动重装载值
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); 													//允许定时器7更新中断
	TIM_Cmd(TIM7,ENABLE); 																						//使能定时器7
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; 										//定时器7中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; 				//抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; 							//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//
//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		;
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}





