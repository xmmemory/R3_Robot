#ifndef __ODOMETRY_H
#define __ODOMETRY_H

#include "stm32f4xx.h" 
#include <math.h>
#include <stdlib.h>

//#define ODOMETRY_LENGTH 12;

//Ҫ��������̼����ݣ��ֱ�Ϊ��X��Y�����ƶ��ľ��룬��ǰ�Ƕȣ����ٶȣ����ٶ�
extern float position_x,position_y,oriention,velocity_linear,velocity_angular;

void odometry_cal(int right,int left);//��̼Ƽ��㺯��

#endif
