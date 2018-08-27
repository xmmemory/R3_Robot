#include "odometry.h"

/***********************************************  ���  *****************************************************************/

float position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;

/***********************************************  ����  *****************************************************************/

float Milemeter_L_Motor,Milemeter_R_Motor;     //dtʱ���ڵ�������(����)�ٶ�,������̼Ƽ���

float wheel_interval= 502.0000f;		//�־�   
//float wheel_interval=276.089f;    //���У��ֵ=ԭ���/0.987

float wheel_diameter=170.0f;     //����ֱ������λmm
//float dt=0.005f;             //����ʱ����5ms
float dt=0.01f;             //����ʱ����10ms
//int dt_x=200;		//�Ŵ�200��
int dt_x=100;		//�Ŵ�100��
float oriention_interval=0;  //dtʱ���ڷ���仯ֵ
float line_number=4096.0f;       //��������
float pi_1_2=1.570796f;			 //��/2
float pi=3.141593f;              //��
float pi_3_2=4.712389f;			 //��*3/2
float pi_2_1=6.283186f;			 //��*2

float delta_distance=0,delta_oriention=0;   //����ʱ�������˶��ľ���

float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0;

float oriention_1=0;

float sin_=0;        //�Ƕȼ���ֵ
float cos_=0;

unsigned char once=1;

/****************************************************************************************************************/

//��̼Ƽ��㺯��
void odometry_cal(int right,int left)
{
	if((right == 0) && (left == 0))	return;
	
	if(once)  //����������һ��
	{
		const_frame=wheel_diameter*pi/line_number;
		const_angle=const_frame/wheel_interval;
		once=0;
	}
	
	distance_sum = (float)(0.5f*(right+left));//�ں̵ܶ�ʱ���ڣ�С����ʻ��·��Ϊ�����ٶȺ�
	distance_diff = right-left;//�ں̵ܶ�ʱ���ڣ�С����ʻ�ĽǶ�Ϊ�����ٶȲ�

	delta_distance = distance_sum;
	delta_oriention = distance_diff;	
	
	oriention_interval = (float)(delta_oriention * const_angle);//����ʱ�����ߵĽǶ�
	oriention = oriention + oriention_interval;//�������̼Ʒ����
	oriention_1 = (float)(oriention + 0.5f * oriention_interval);//��̼Ʒ��������λ���仯���������Ǻ�������
	
	sin_ = (float)sin(oriention_1);//���������ʱ����y����
	cos_ = (float)cos(oriention_1);//���������ʱ����x����
	
  position_x = position_x + (float)(delta_distance * cos_ * const_frame);//�������̼�x����
	position_y = position_y + (float)(delta_distance * sin_ * const_frame);//�������̼�y����    
	velocity_linear =(float)(delta_distance*const_frame * dt_x);//�������̼����ٶ�
	velocity_angular = (float)(oriention_interval * dt_x);//�������̼ƽ��ٶ�	
	 //����ǽǶȾ���
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
//	Milemeter_L_Motor= ((left / 4096.0) * pi * wheel_diameter); //����������
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
	

