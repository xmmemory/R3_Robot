#include "motor.h"

//********************************************************************************
//������� ����	 
//�޸�����:2018-10-8
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ��������(����)���ܻ����˿Ƽ����޹�˾ 
//All rights reserved
//********************************************************************************
///*
//************************************************************************************************************************
//*                                                   Stop_Moving
//*
//* Description: �������ٶȹ���;
//* Arguments  : NULL
//* Note(s)    : NULL 
//*
//************************************************************************************************************************
//*/   
//void Stop_Moving(void)
//{
//	//MOTOR_RESET_SPEED
//	CAN1_Send_Msg(CAN_ID1,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�
//	CAN1_Send_Msg(CAN_ID2,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�;
//}

/*
************************************************************************************************************************
*                                                   motor_init_task
* Description: �����ַ�����ٶȳ�ʼ��;
* Arguments  : Null;
* Note(s)    : Null;
************************************************************************************************************************
*/ 
void motor_init_task(void)
{
//	unsigned char i;	
	//MOTOR_STOP_CMD
	CAN1_Send_Msg(CAN_ID1,MOTOR_STOP_CMD,8);//����8���ֽ�
	CAN1_Send_Msg(CAN_ID2,MOTOR_STOP_CMD,8);//����8���ֽ�
	delay_ms(100);
	//MOTOR_SPEED_MODE
	CAN1_Send_Msg(CAN_ID1,MOTOR_SPEED_MODE_CMD,8);//����8���ֽ�
	CAN1_Send_Msg(CAN_ID2,MOTOR_SPEED_MODE_CMD,8);//����8���ֽ�
	delay_ms(100);
	//MOTOR_RESET_SPEED
	CAN1_Send_Msg(CAN_ID1,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�
	CAN1_Send_Msg(CAN_ID2,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�
	delay_ms(100);
	//MOTOR_START_CMD
	CAN1_Send_Msg(CAN_ID1,MOTOR_START_CMD,8);//����8���ֽ�
	CAN1_Send_Msg(CAN_ID2,MOTOR_START_CMD,8);//����8���ֽ�
	delay_ms(100);
}






