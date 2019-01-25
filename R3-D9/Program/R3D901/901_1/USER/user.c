#include "user.h"
#include "adc.h"
#include "beep.h" 
//********************************************************************************
//用户代码	 
//修改日期:2018-10-22
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 光线量子(北京)智能机器人科技有限公司 
//All rights reserved
//********************************************************************************

/************************************************************************************************************************
*                                                   battery_check
* Description: 电池电压检测
* Arguments  : Null.
* Note(s)    : 1) Null；
************************************************************************************************************************/	
//电压检测
float battery_check(void)
{
	u16 battery_adc = 0;
	float battery_vol = 0;
	battery_adc=Get_Adc_Average(ADC_Channel_5,20);//获取通道5的转换值，20次取平均
	battery_vol = (float)((battery_adc*3.3f)/4096);			//获取计算后的带小数的实际电压值，比如3.1111
	battery_vol = battery_vol*11.0f;			//放大11倍、得到电池电压值
	return battery_vol;
}
/************************************************************************************************************************
*                                                   Start_Succeed
* Description: 程序自检、正常启动蜂鸣器关闭
* Arguments  : Null.
* Note(s)    : 1) Null；
************************************************************************************************************************/	
u8 reset_pro = 0;
void	Start_Succeed(void)
{
	if(RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET)								//是看门狗复位启动
	{
		RCC_ClearFlag();													//清空RCC_CSR由软件复位
		while(1)
		{
			BEEP = 1;
			delay_ms(200);													//上电等待驱动器上电---part1
			BEEP = 0;
			delay_ms(200);													//上电等待驱动器上电---part1
			reset_pro = 1;
		}
	}
	else if(RCC_GetFlagStatus(RCC_FLAG_PORRST) == SET)						//低电压复位
	{
		RCC_ClearFlag();													//清空RCC_CSR由软件复位
		while(1)
		{
			BEEP = 1;
			delay_ms(1000);													//上电等待驱动器上电---part1
			BEEP = 0;
			delay_ms(1000);													//上电等待驱动器上电---part1	
			reset_pro = 1;
		}
	}
	else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) == SET)						//按键复位
	{
//		reset_pro = 1;
	}
	else
	{
		BEEP = 1;
		delay_ms(750);													//上电等待驱动器上电---part1
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
//		USART_Cmd(USART2, DISABLE);  //禁用串口2 
//		USART_Cmd(USART3, DISABLE);  //禁用串口3
			BEEP = 1;
		//PWR_EnterSTANDBYMode();			//待机模式,1.8V 内核电源关闭-此功耗最低，典型大概在2uA左右
//		PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);				//停机模式,所有时钟都停止-此功耗较低，典型大概在20uA左右
	}
}

//#define PWR_Regulator_ON               //电源不进低功耗 唤醒基本没延迟
//#define PWR_Regulator_LowPower         //电源进去低功耗 不过唤醒启动有一点延迟    

//#define PWR_STOPEntry_WFI              //中断唤醒
//#define PWR_STOPEntry_WFE              //事件唤醒


//void main_task(void *p_arg)
//{
//	OS_ERR err;
//	u8 robot_task = 0;		//check_count
//	while(0)
//	{
//		//发送给串口的里程计数据数组---预留出2倍的长度防止异常溢出（应该用不上）
//		u8 odometry_data[64];   	
//		//Odom_Get
//		x_data.odoemtry_float=position_x;//单位mm
//		y_data.odoemtry_float=position_y;//单位mm
//		theta_data.odoemtry_float=oriention;//单位rad
//		vel_linear.odoemtry_float=velocity_linear;//单位mm/s
//		vel_angular.odoemtry_float=velocity_angular;//单位rad/s
//		//将所有里程计数据存到要发送的数组
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
//		/**********进入临界区**********/
//		CPU_SR_ALLOC();
//		OS_CRITICAL_ENTER();
//		//
//		OS_CRITICAL_EXIT();
//		/**********退出临界区*********/		
//		OSTimeDlyHMSM(0,0,0,Odom_Rate_Seconds,OS_OPT_TIME_PERIODIC,&err);   //默认延时100ms--10Hz、
//	}
//}
