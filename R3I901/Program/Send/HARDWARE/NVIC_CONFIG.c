//���ļ������ж�Ƕ������
#include "NVIC_CONFIG.h"
void Init_Nvic(void)//
{
	  NVIC_InitTypeDef NVIC_InitStructure;
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	  NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;//TIM5ȫ�ֱ����жϵ�����
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	
//	  	  NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn ;//pwm���ж����ȼ�����
//	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//�����ȼ�
//	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//�����ȼ�
//	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	  NVIC_Init(&NVIC_InitStructure);
	
		  NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn ;//pwm���ж����ȼ�����
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//�����ȼ�
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//�����ȼ�
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	
	  NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn ;//���벶����ж����ȼ�����
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//�����ȼ�
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//�����ȼ�
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
	
	  NVIC_InitStructure.NVIC_IRQChannel=TIM8_CC_IRQn ;//���벶����ж����ȼ�����
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//�����ȼ�
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//�����ȼ�
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}

