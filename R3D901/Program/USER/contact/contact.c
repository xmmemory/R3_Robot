#include "contact.h"
#include "can.h"

/***********************************************  ���  *****************************************************************/

/***********************************************  ����  *****************************************************************/

//extern struct PID Control_left;//����PID�����������µ��4096
//extern struct PID Control_right;//����PID�����������µ��4096
int right_speed;	//���ٶ�
int left_speed;
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
	
	CAN1_Send_Msg(CAN_ID1,Can_buff,8);//����8���ֽ�	
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

	CAN1_Send_Msg(CAN_ID2,Can_buff2,8);//����8���ֽ�
}

void car_control(float rightspeed,float leftspeed)//С���ٶ�ת���Ϳ��ƺ���
{
    float k2=1.00;         //�ٶ�ת������,ת/����	

		LED0=!LED0;//��˸LED,��ʾϵͳ��������.
	
		rightspeed = -rightspeed;
	
    //���Ӵ��ڽ��յ����ٶ�(mm/s)ת����ʵ�ʿ���С�����ٶ�(RPM)������PWM��
		right_speed = (int)((k2*rightspeed*60)/(pi * wheel_diameter));    
		left_speed = (int)((k2*leftspeed*60)/(pi * wheel_diameter));
	
		right_speed = (long int)(right_speed * (16384.00 / 6000.00));
		left_speed	= (long int)(left_speed * (16384.00 / 6000.00));
		
    RightMovingSpeedW(right_speed);
    LeftMovingSpeedW(left_speed);
}


