#include "odometry.h"

/***********************************************  输出  *****************************************************************/

float position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;

/***********************************************  变量  *****************************************************************/

float Milemeter_L_Motor,Milemeter_R_Motor;     //dt时间内的左右轮(脉冲)速度,用于里程计计算

float wheel_interval= 502.0000f;		//轮距   
//float wheel_interval=276.089f;    //轴距校正值=原轴距/0.987

float wheel_diameter=170.0f;     //轮子直径，单位mm
//float dt=0.005f;             //采样时间间隔5ms
float dt=0.01f;             //采样时间间隔10ms
//int dt_x=200;		//放大200倍
int dt_x=100;		//放大100倍
float oriention_interval=0;  //dt时间内方向变化值
float line_number=4096.0f;       //码盘线数
float pi_1_2=1.570796f;			 //π/2
float pi=3.141593f;              //π
float pi_3_2=4.712389f;			 //π*3/2
float pi_2_1=6.283186f;			 //π*2

float delta_distance=0,delta_oriention=0;   //采样时间间隔内运动的距离

float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0;

float oriention_1=0;

float sin_=0;        //角度计算值
float cos_=0;

unsigned char once=1;

/****************************************************************************************************************/

//里程计计算函数
void odometry_cal(int right,int left)
{
	if((right == 0) && (left == 0))	return;
	
	if(once)  //常数仅计算一次
	{
		const_frame=wheel_diameter*pi/line_number;
		const_angle=const_frame/wheel_interval;
		once=0;
	}
	
	distance_sum = (float)(0.5f*(right+left));//在很短的时间内，小车行驶的路程为两轮速度和
	distance_diff = right-left;//在很短的时间内，小车行驶的角度为两轮速度差

	delta_distance = distance_sum;
	delta_oriention = distance_diff;	
	
	oriention_interval = (float)(delta_oriention * const_angle);//采样时间内走的角度
	oriention = oriention + oriention_interval;//计算出里程计方向角
	oriention_1 = (float)(oriention + 0.5f * oriention_interval);//里程计方向角数据位数变化，用于三角函数计算
	
	sin_ = (float)sin(oriention_1);//计算出采样时间内y坐标
	cos_ = (float)cos(oriention_1);//计算出采样时间内x坐标
	
  position_x = position_x + (float)(delta_distance * cos_ * const_frame);//计算出里程计x坐标
	position_y = position_y + (float)(delta_distance * sin_ * const_frame);//计算出里程计y坐标    
	velocity_linear =(float)(delta_distance*const_frame * dt_x);//计算出里程计线速度
	velocity_angular = (float)(oriention_interval * dt_x);//计算出里程计角速度	
	 //方向角角度纠正
	if(oriention >= pi)
	{
		oriention -= pi_2_1;
	}
	else
	{
		if(oriention < -pi)
		{
			oriention += pi_2_1;
		}
	}
}
/****************************************************************************************************************/
//	Milemeter_L_Motor= ((left / 4096.0) * pi * wheel_diameter); //储存脉冲数
//	Milemeter_R_Motor= ((right / 4096.0) * pi * wheel_diameter);
	
//	if ( abs(Milemeter_R_Motor - Milemeter_L_Motor) < 50.00)
//	{
//		velocity_angular = (float)(Milemeter_R_Motor - Milemeter_L_Motor) / wheel_interval;
//		oriention += (float)velocity_angular;		
//	}
//	
//	if ( (Milemeter_R_Motor != 0) || (Milemeter_L_Motor != 0))
//	{
//		velocity_linear = (float)((Milemeter_R_Motor + Milemeter_L_Motor) * 0.5);
//		position_x += (velocity_linear * (float)cos(oriention + (velocity_angular / 2)));
//		position_y += (velocity_linear * (float)sin(oriention + (velocity_angular / 2)));		
//	}	
/****************************************************************************************************************/	
	

