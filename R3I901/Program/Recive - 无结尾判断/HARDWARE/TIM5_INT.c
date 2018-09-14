#include"TIM5_INT.h"

void Init_Tim2_Int(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	TIM_DeInit(TIM2);

	TIM_TimeBaseStructure.TIM_Period = 1000;	//1000us中断一次
	
	TIM_TimeBaseStructure.TIM_Prescaler=72;//72分频
	 
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; //or TIM_CKD_DIV2 or TIM_CKD_DIV4
	
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
}
//控制TIM5的开启和关闭 
//0 关闭
//1 开启
void Ctr_Tim2(u8 sta)
{
	if(sta)
		TIM_Cmd(TIM2, ENABLE);
	else
		TIM_Cmd(TIM2, DISABLE);
}
