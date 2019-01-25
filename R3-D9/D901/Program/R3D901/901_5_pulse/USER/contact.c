#include "contact.h"
#include "can.h"
#include "odometry.h"
#include "timer.h"
#include "delay.h"
#include "math.h"

//********************************************************************************
//�ٶ�ת������ ����	 
//�޸�����:2018-10-8
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ��������(����)���ܻ����˿Ƽ����޹�˾ 
//All rights reserved
//********************************************************************************

/***********************************************  ���  *****************************************************************/

/***********************************************  ����  *****************************************************************/

//extern struct PID Control_left;//����PID�����������µ��4096
//extern struct PID Control_right;//����PID�����������µ��4096
/***********************************************  ����  *****************************************************************/

/*******************************************************************************************************************/
union Can_Buff								//��̼����ݹ�����
{
	long int d;
	unsigned char data[4];
}speed_array,speed_array2;

u8 Can_Speed_Head[4] = {0x00, 0xFA, 0x00, 0x11};



void LeftMovingSpeedW(long int val)//���ַ�����ٶȿ��ƺ���
{     
	u8 Can_buff[8];
	speed_array.d= val;
		
	memcpy(Can_buff,Can_Speed_Head,4);
	
	Can_buff[7] = speed_array.data[0];
	Can_buff[6] = speed_array.data[1];
	Can_buff[5] = speed_array.data[2];
	Can_buff[4] = speed_array.data[3];
	
//	CAN1_Send_Msg(CAN_ID1,Can_buff,8);//����8���ֽ�	
	if(CAN1_Send_Msg(CAN_ID1,Can_buff,8))
		GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP�������ߣ� ��ͬBEEP=1;
}

void RightMovingSpeedW(long int val2)//���ַ�����ٶȿ��ƺ���
{   
	u8 Can_buff2[8];
	speed_array2.d= val2;
	
	memcpy(Can_buff2,Can_Speed_Head,4);
	
	Can_buff2[7] = speed_array2.data[0];
	Can_buff2[6] = speed_array2.data[1];
	Can_buff2[5] = speed_array2.data[2];
	Can_buff2[4] = speed_array2.data[3];

//	CAN1_Send_Msg(CAN_ID2,Can_buff2,8);//����8���ֽ�
	if(CAN1_Send_Msg(CAN_ID2,Can_buff2,8))
		GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP�������ߣ� ��ͬBEEP=1;
}


float rightspeed_temp,leftspeed_temp;

void car_control(float rightspeed,float leftspeed)//С���ٶ�ת���Ϳ��ƺ���
{
	unsigned short int i1,i2;
	float k2=1.00;										//�ٶ�ת������,ת/����
	int left_circle,right_circle;			//���ٶ�,
	//��˸LED,��ʾϵͳ��������.
	LED0=!LED0;
	
	//���ٶ�δ�����������·��ٶ�
	if(rightspeed_temp == rightspeed && leftspeed_temp == leftspeed)
		return;

	if(rightspeed > 0)
		RIGHT_DIR = 1;		//������ǰ�������С������Ƿ�ת״̬,�ߵ�ƽ��ת���͵�ƽ��ת;
	else
		RIGHT_DIR = 0;
	if(leftspeed > 0)
		LEFT_DIR = 0;			//������ǰ�������С���������ת״̬,�ߵ�ƽ��ת���͵�ƽ��ת;
	else
		LEFT_DIR = 1;
	
	delay_us(10);				//�����ź�Ӧ���������ź����� 5us;
	
	if(leftspeed < 0)
		leftspeed = -leftspeed;
	if(rightspeed < 0)
		rightspeed = -rightspeed;
	
	//�����յ����ٶ�(mm/s)ת����ʵ�ʿ���С�����ٶ�(RPM��������/min)������PWM��
	right_circle = (int)((k2*rightspeed*60)/(pi * wheel_diameter));
	left_circle = (int)((k2*leftspeed*60)/(pi * wheel_diameter));
	
	right_circle = right_circle*4096/60;		//����/�롪��һȦ4096������
	left_circle = left_circle*4096/60;
	
	TIM_Cmd(TIM13, DISABLE);  //ʹ��TIM13
	TIM_Cmd(TIM14, DISABLE);  //ʹ��TIM14
	
	i1 = (1000000/left_circle)+1;
	i2 = (1000000/right_circle)+1;
	
	TIM13_PWM_Init(i1-1,84-1);	//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ1000������PWMƵ��Ϊ 1M/1000=1khz.����PF8
	TIM14_PWM_Init(i2-1,84-1);	//84M/84=1Mhz�ļ���Ƶ��,��װ��ֵ1000������PWMƵ��Ϊ 1M/1000=1khz.����PF9
	
	TIM_Cmd(TIM13, ENABLE);  //ʹ��TIM13
	TIM_Cmd(TIM14, ENABLE);  //ʹ��TIM14
	
	//�ٶ��ݴ�
	rightspeed_temp = rightspeed;
	leftspeed_temp = leftspeed;
}

void speed_convert(float linear_vel_t,float angular_vel_t)					//С���ٶ�ת���Ϳ��ƺ�����λmm/s
{
	float right_speed,left_speed;
	//�����ٶ�(mm/r)���ٶ�(rad/s)ת���������ٶ�(mm/s)
	left_speed = linear_vel_t - 0.5f*angular_vel_t*wheel_interval;									//mm/s
	right_speed = linear_vel_t + 0.5f*angular_vel_t*wheel_interval;					//������ʱ����ת

	car_control(right_speed,left_speed);
}



