#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx.h" 
#include "stdio.h"
#include "delay.h"
#include "can.h"
#include "contact.h"


void Stop_Moving(void);																			//轮子锁止程序
void motor_init_task(void);																	//驱动器初始化
void Robot_Moving(float L_speed,float R_speed,u8 time);			//n*50ms运行;
	
#endif 





