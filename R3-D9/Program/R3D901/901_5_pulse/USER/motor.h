#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h" 
#include "stdio.h"
#include "delay.h"
#include "can.h"
#include "contact.h"


void Stop_Moving(void);																			//������ֹ����
void motor_init_task(void);																	//��������ʼ��
void Robot_Moving(float L_speed,float R_speed,u8 time);			//n*50ms����;
	
#endif 





