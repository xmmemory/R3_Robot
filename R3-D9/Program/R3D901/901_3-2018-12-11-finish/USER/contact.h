#ifndef __CONTACT_H
#define __CONTACT_H

#include "stm32f4xx.h" 

//#include "encoder.h"

#include "math.h"
#include <stdio.h>
#include "cstring"
#include "usart.h"
#include "led.h"

#define	wheel_diameter	(170.0F)	//轮子的直径0.17m=170mm
#define	pi		(3.14159F)

//#define		RPM_TO_NUM(x)		(x/6000) * 16384		//7000RPM-16384

static u8 MOTOR_START_CMD[] 					= 	{0x00, 0xFA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x1F};
static u8 MOTOR_STOP_CMD[]						= 	{0x00, 0xFA, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0F};
static u8 MOTOR_SPEED_MODE_CMD[] 			= 	{0x00, 0xFA, 0x00, 0x19, 0x00, 0x00, 0x00, 0x2F};
//static const char MOTOR_SPEED_PROPORTION_MODIFY[] 		= 	{0x40, 0x13, 0x88, 0xDB};
//static const char MOTOR_SPEED_INTERGRATION_MODIFY[] 		= 	{0x41, 0x01, 0xF4, 0x36};
//static const char MOTOR_SPEED_DIFFERENTIAL_MODIFY[] 		= 	{0x42, 0x00, 0x00, 0x42};
//static const char MOTOR_CLEAT_FAULT_CMD[]	= 	{0x4a, 0x00, 0x00, 0x4a};
static u8 MOTOR_RESET_SPEED_CMD[] 		= 	{0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00};
static u8 MOTOR_CLEAR_FAULT[] 				= 	{0x00, 0xFA, 0x00, 0x15, 0x00, 0x00, 0x00, 0x7F};
static u8 MOTOR_RESET_SPEED_TEST1[] 	= 	{0x00, 0xFA, 0x00, 0x11, 0x00, 0x00, 0x00, 0x88};
static u8 MOTOR_RESET_SPEED_TEST2[] 	= 	{0x00, 0xFA, 0x00, 0x11, 0xFF, 0xFF, 0xFF, 0x78};
void LeftMovingSpeedW(long int val);//左轮方向和速度控制函数
void RightMovingSpeedW(long int val2);//右轮方向和速度控制函数

void car_control(float rightspeed,float leftspeed);//小车速度转化和控制函数

void Contact_Init(void);	//左右轮方向和速度初始化
void Get_Pulse(void);	//获取里程计数据

extern void delay_ms(u16 nms);

#endif  
