//********************************************************************************
//用户程序 代码	 
//修改日期:2018-10-22
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 光线量子(北京)智能机器人科技有限公司 
//All rights reserved.
//********************************************************************************
#ifndef __USER_H
#define __USER_H

#include "sys.h"

void system_init_step1(void);			//内部初始化
void system_init_step2(void);			//运动部分初始化
void system_init_step3(void);			//通讯初始化
void system_init_step4(void);			//参数初始化
void system_init_step5(void);			//充电初始化

void uart1_deal(void);							//串口数据处理

void odom_send_task(void);					//里程计数据整理任务
void auto_charge_task(void);				//自动充电任务
void exit_charge_task(void);				//充电脱离任务
u8 battery_check(void);							//电压检测任务
u8 remote_get(void);								//红外



#endif

