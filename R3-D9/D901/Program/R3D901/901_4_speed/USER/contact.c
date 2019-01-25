#include "contact.h"
#include "can.h"
#include "odometry.h"

//********************************************************************************
//速度转换驱动 代码	 
//修改日期:2018-10-8
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 光线量子(北京)智能机器人科技有限公司 
//All rights reserved
//********************************************************************************

/***********************************************  输出  *****************************************************************/

/***********************************************  输入  *****************************************************************/

//extern struct PID Control_left;//左轮PID参数，适于新电机4096
//extern struct PID Control_right;//右轮PID参数，适于新电机4096
/***********************************************  变量  *****************************************************************/

/*******************************************************************************************************************/
union Can_Buff								//里程计数据共用体
{
	long int d;
	unsigned char data[4];
}speed_array,speed_array2;

u8 Can_Speed_Head[4] = {0x00, 0xFA, 0x00, 0x11};



void LeftMovingSpeedW(long int val)//左轮方向和速度控制函数
{     
	u8 Can_buff[8];
	speed_array.d= val;
		
	memcpy(Can_buff,Can_Speed_Head,4);
	
	Can_buff[7] = speed_array.data[0];
	Can_buff[6] = speed_array.data[1];
	Can_buff[5] = speed_array.data[2];
	Can_buff[4] = speed_array.data[3];
	
//	CAN1_Send_Msg(CAN_ID1,Can_buff,8);//发送8个字节	
	if(CAN1_Send_Msg(CAN_ID1,Can_buff,8))
		GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP引脚拉高， 等同BEEP=1;
}

void RightMovingSpeedW(long int val2)//右轮方向和速度控制函数
{   
	u8 Can_buff2[8];
	speed_array2.d= val2;
	
	memcpy(Can_buff2,Can_Speed_Head,4);
	
	Can_buff2[7] = speed_array2.data[0];
	Can_buff2[6] = speed_array2.data[1];
	Can_buff2[5] = speed_array2.data[2];
	Can_buff2[4] = speed_array2.data[3];

//	CAN1_Send_Msg(CAN_ID2,Can_buff2,8);//发送8个字节
	if(CAN1_Send_Msg(CAN_ID2,Can_buff2,8))
		GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP引脚拉高， 等同BEEP=1;
}

void car_control(float rightspeed,float leftspeed)//小车速度转化和控制函数
{
    float k2=1.00;										//速度转换比例,转/分钟
		int left_circle,right_circle;			//线速度,
		//闪烁LED,提示系统正在运行.
		LED0=!LED0;
	
		rightspeed = -rightspeed;
	
    //将从串口接收到的速度(mm/s)转换成实际控制小车的速度(RPM)？还是PWM？
		right_circle = (int)((k2*rightspeed*60)/(pi * wheel_diameter));    
		left_circle = (int)((k2*leftspeed*60)/(pi * wheel_diameter));
	
		right_circle = (long int)(right_circle * (16384.00 / 6000.00));
		left_circle	= (long int)(left_circle * (16384.00 / 6000.00));
		
    RightMovingSpeedW(right_circle);
    LeftMovingSpeedW(left_circle);
}

void speed_convert(float linear_vel_t,float angular_vel_t)					//小车速度转化和控制函数单位mm/s
{
	float right_speed,left_speed;
	//将线速度(mm/s)角速度(rad/s)转换成两轮速度(mm/s)
	left_speed = linear_vel_t - 0.5f*angular_vel_t*wheel_interval;									//mm/s
	right_speed = linear_vel_t + 0.5f*angular_vel_t*wheel_interval;					//右轮逆时针旋转

	car_control(right_speed,left_speed);
}



