#include "motor.h"
//********************************************************************************
//电机驱动 代码	 
//修改日期:2018-10-8
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 光线量子(北京)智能机器人科技有限公司 
//All rights reserved
//********************************************************************************
/************************************************************************************************************************
*                                                   Stop_Moving
* Description: 驱动器速度归零;
* Arguments  : NULL
* Note(s)    : NULL 
************************************************************************************************************************/   
void Stop_Moving(void)
{
	car_control(0,0);	 //将接收到的左右轮速度赋给小车	
}
/************************************************************************************************************************
*                                                   Robot_Moving
* Description: n*10ms运行;
* Arguments  : L_speed---左轮速度，R_speed---右轮速度，time---运动时间*10ms
* Note(s)    :      
************************************************************************************************************************/   
void Robot_Moving(float L_speed,float R_speed,u8 time)
{	
	while(time--)
	{
		car_control(R_speed,L_speed);	 //将接收到的左右轮速度赋给小车	
		delay_ms(10);
	}
}
/************************************************************************************************************************
*                                                   motor_init_task
* Description: 左右轮方向和速度初始化;
* Arguments  : Null;
* Note(s)    : Null;
************************************************************************************************************************/ 
void motor_init_task(void)
{
//	unsigned char i;	
	//MOTOR_STOP_CMD
	CAN1_Send_Msg(CAN_ID1,MOTOR_STOP_CMD,8);//发送8个字节
	CAN1_Send_Msg(CAN_ID2,MOTOR_STOP_CMD,8);//发送8个字节
	delay_ms(500);
	//MOTOR_SPEED_MODE
	CAN1_Send_Msg(CAN_ID1,MOTOR_SPEED_MODE_CMD,8);//发送8个字节
	CAN1_Send_Msg(CAN_ID2,MOTOR_SPEED_MODE_CMD,8);//发送8个字节
	delay_ms(500);
//	//MOTOR_RESET_SPEED
//	CAN1_Send_Msg(CAN_ID1,MOTOR_RESET_SPEED_CMD,8);//发送8个字节
//	CAN1_Send_Msg(CAN_ID2,MOTOR_RESET_SPEED_CMD,8);//发送8个字节
//	delay_ms(500);
	//MOTOR_START_CMD
	CAN1_Send_Msg(CAN_ID1,MOTOR_START_CMD,8);//发送8个字节
	CAN1_Send_Msg(CAN_ID2,MOTOR_START_CMD,8);//发送8个字节
	delay_ms(500);
}






