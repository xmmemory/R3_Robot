#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "stm32f4xx.h" 
#include <math.h>
#include <stdlib.h>

//#define ODOMETRY_LENGTH 12;

//要发布的里程计数据，分别为：X，Y方向移动的距离，当前角度，线速度，角速度
extern float position_x,position_y,oriention,velocity_linear,velocity_angular;
extern float wheel_interval;
extern float wheel_diameter;
extern float pi;

void odometry_cal(int right,int left);		//里程计计算函数
void odometry_clear(void);								//里程计清空函数	


#endif
