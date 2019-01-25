#include "user.h"
#include "adc.h"
#include "beep.h" 
//********************************************************************************
//�û�����	 
//�޸�����:2018-10-22
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ��������(����)���ܻ����˿Ƽ����޹�˾ 
//All rights reserved
//********************************************************************************

/************************************************************************************************************************
*                                                   battery_check
* Description: ��ص�ѹ���
* Arguments  : Null.
* Note(s)    : 1) Null��
************************************************************************************************************************/	
//��ѹ���
float battery_check(void)
{
	u16 battery_adc = 0;
	float battery_vol = 0;
	battery_adc=Get_Adc_Average(ADC_Channel_5,20);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
	battery_vol = (float)((battery_adc*3.3f)/4096);			//��ȡ�����Ĵ�С����ʵ�ʵ�ѹֵ������3.1111
	battery_vol = battery_vol*11.0f;			//�Ŵ�11�����õ���ص�ѹֵ
	return battery_vol;
}
/************************************************************************************************************************
*                                                   Start_Succeed
* Description: �����Լ졢���������������ر�
* Arguments  : Null.
* Note(s)    : 1) Null��
************************************************************************************************************************/	
u8 reset_pro = 0;
void	Start_Succeed(void)
{
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET)								//�ǿ��Ź���λ����
	{
		RCC_ClearFlag();													//���RCC_CSR�������λ
		while(1)
		{
			BEEP = 1;
			delay_ms(200);													//�ϵ�ȴ��������ϵ�---part1
			BEEP = 0;
			delay_ms(200);													//�ϵ�ȴ��������ϵ�---part1
			reset_pro = 1;
		}
	}
	else if(RCC_GetFlagStatus(RCC_FLAG_PORRST) == SET)						//�͵�ѹ��λ
	{
		RCC_ClearFlag();													//���RCC_CSR�������λ
		while(1)
		{
			BEEP = 1;
			delay_ms(1000);													//�ϵ�ȴ��������ϵ�---part1
			BEEP = 0;
			delay_ms(1000);													//�ϵ�ȴ��������ϵ�---part1	
			reset_pro = 1;
		}
	}
	else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) == SET)						//������λ
	{
//		reset_pro = 1;
	}
	else
	{
		BEEP = 1;
		delay_ms(750);													//�ϵ�ȴ��������ϵ�---part1
		BEEP = 0;
	}
	
}
/************************************************************************** 
* Function Name  : PVD_Config  
* Description    : This function PVD Configure PVD. 
* Input          : None 
* Output         : None 
* Return         : None  
***************************************************************************/ 
void PVD_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line16;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;       

	EXTI_Init(&EXTI_InitStructure);
	
	PWR_PVDLevelConfig(PWR_PVDLevel_7);
	PWR_PVDCmd(ENABLE);
}
/************************************************************************** 
* Function Name  : PVD_IRQHandler  
* Description    : This function handles PVD interrupt request. 
* Input          : None
* Output         : None
* Return         : None
***************************************************************************/ 
void PVD_IRQHandler(void)
{   
	if (PWR_GetFlagStatus(PWR_FLAG_PVDO))
		
	while(1)
	{
//		USART_Cmd(USART2, DISABLE);  //���ô���2 
//		USART_Cmd(USART3, DISABLE);  //���ô���3
			BEEP = 1;
		//PWR_EnterSTANDBYMode();			//����ģʽ,1.8V �ں˵�Դ�ر�-�˹�����ͣ����ʹ����2uA����
//		PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);				//ͣ��ģʽ,����ʱ�Ӷ�ֹͣ-�˹��Ľϵͣ����ʹ����20uA����
	}
}

//#define PWR_Regulator_ON               //��Դ�����͹��� ���ѻ���û�ӳ�
//#define PWR_Regulator_LowPower         //��Դ��ȥ�͹��� ��������������һ���ӳ�    

//#define PWR_STOPEntry_WFI              //�жϻ���
//#define PWR_STOPEntry_WFE              //�¼�����


//void main_task(void *p_arg)
//{
//	OS_ERR err;
//	u8 robot_task = 0;		//check_count
//	while(0)
//	{
//		//���͸����ڵ���̼���������---Ԥ����2���ĳ��ȷ�ֹ�쳣�����Ӧ���ò��ϣ�
//		u8 odometry_data[64];   	
//		//Odom_Get
//		x_data.odoemtry_float=position_x;//��λmm
//		y_data.odoemtry_float=position_y;//��λmm
//		theta_data.odoemtry_float=oriention;//��λrad
//		vel_linear.odoemtry_float=velocity_linear;//��λmm/s
//		vel_angular.odoemtry_float=velocity_angular;//��λrad/s
//		//��������̼����ݴ浽Ҫ���͵�����
//		for(u8 j=0;j<4;j++)
//		{
//			odometry_data[j]=x_data.odometry_char[j];
//			odometry_data[j+4]=y_data.odometry_char[j];
//			odometry_data[j+8]=theta_data.odometry_char[j];
//			odometry_data[j+12]=vel_linear.odometry_char[j];
//			odometry_data[j+16]=vel_angular.odometry_char[j];			
//		}
//		robot_task = odometry_data[0];
//		odometry_data[0] = robot_task;
//		/**********�����ٽ���**********/
//		CPU_SR_ALLOC();
//		OS_CRITICAL_ENTER();
//		//
//		OS_CRITICAL_EXIT();
//		/**********�˳��ٽ���*********/		
//		OSTimeDlyHMSM(0,0,0,Odom_Rate_Seconds,OS_OPT_TIME_PERIODIC,&err);   //Ĭ����ʱ100ms--10Hz��
//	}
//}
