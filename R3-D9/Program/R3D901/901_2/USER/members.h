/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                           MASTER INCLUDES
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : r3driver.h
* Version       : V1.00
* Programmer(s) : Y
*********************************************************************************************************
*/

#ifndef  INCLUDES_MEMBERS
#define  INCLUDES_MEMBERS

/*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************/
#define GNUC_PACKED __attribute__((packed))

#define TASK_Q_NUM	3							//�������ڽ���Ϣ���еĳ���
#define SPEED_TRANSFORM_Q_NUM	2		//�ٶ��·������ڽ���Ϣ���еĳ���---2
#define UART1_ANALYZE_Q_NUM	2			//����1���ݽ����ڽ���Ϣ���еĳ���---2
#define SPEED_TRANSFORM_TIMEOUT 200u	//n*5ms�ĵȴ�ʱ��--1s

#define	DEVICE_ID_D9		0X01			//�豸ID-0x01-���̿�����
#define	SEND_COMMOND		0X01			//����ID-0x01-����ָ��
#define	READ_COMMOND		0X02			//����ID-0x02-��ȡָ��
#define	WRITE_COMMOND		0X03			//����ID-0x03-д��ָ��

#define MAX_Circle_Moving_Count 1000		//��תѰ���ź�����������Դ���

/*
*********************************************************************************************************
*                                              REGISTER
*********************************************************************************************************
*/
#define	REG_STATUS						0x01		//�Ĵ���-0x01-����״̬
#define REG_ODOMETRY					0x10		//�Ĵ���-0x10-��̼�
#define REG_BATTERY_VOLTAGE		0x11		//�Ĵ���-0x11-������
#define REG_CHARGING_TIME			0x20		//�Ĵ���-0x20-���ʱ��
#define REG_MOVING_SPEED			0x21		//�Ĵ���-0x21-�˶��ٶ�
#define REG_ERROR							0xF1		//�Ĵ���-0xF1-������
/*********************************************************************************************************/
#define REG_FLASH_ENABLE			0xF5		//�Ĵ���-0xF5-0x5xд��Ȩ��ʹ��
//#define REG_ERROR							0xF1		//�Ĵ���-0xF1-������
//#define REG_ERROR							0xF1		//�Ĵ���-0xF1-������
//#define REG_ERROR							0xF1		//�Ĵ���-0xF1-������
/*����������������������Ҫ0x04��д��ָ����д�롪������������������*/
#define REG_WHEEL_INTERVAL		0x51		//�Ĵ���-0x51-���ּ��
#define REG_WHEEL_DIAMETER		0x52		//�Ĵ���-0x52-����ֱ��
#define REG_LEFT_WHEEL_RATIO	0x53		//�Ĵ���-0x53-�����˶�����ϵ��
#define REG_RIGHT_WHEEL_RATIO	0x54		//�Ĵ���-0x54-�����˶�����ϵ��
//#define REG_WHEEL_INTERVAL		0x51		//�Ĵ���-0x51-���ּ��

/**********************************************Robot_Status***************************************************/
#define	ERROT_STATUS					0x00		//״̬-0x00-�쳣״̬
#define	NAVI_STATUS						0x01		//״̬-0x01-����״̬
#define	CHARGING_STATUS				0x02		//״̬-0x02-���״̬
#define	FINDING_DOCK_STATUS		0x03		//״̬-0x03-�ҵ�״̬
/**********************************************Robot_Error***************************************************/
#define	UULL_ERROT						0x00		//״̬-0x00-���쳣
#define	CHARGE_FALSE					0x01		//״̬-0x01-���ʧ��
//#define	CHARGING_STATUS				0x02		//״̬-0x02-���״̬
//#define	FINDING_DOCK_STATUS		0x03		//״̬-0x03-�ҵ�״̬



//Ҫд�뵽STM32 FLASH���ַ�������
const u8 TEXT_Buffer[]={"STM32 FLASH TEST"};
#define TEXT_LENTH sizeof(TEXT_Buffer)	 		  	//���鳤��	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)

#define FLASH_SAVE_ADDR  0X0800C004 	//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.
										//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.

/*********************************************************************************************************
*                                              	NUMBERS
*********************************************************************************************************/
u16 Moving_Seconds;								//�������ʱ��---n*50ms
u8 temp_fr_control = 0;
u8 IR_L=0,IR_M=0,IR_R=0;					//�������У���.
u16 Reset_Robot_Count = 65535;

//odom
u8 odometry_data[64];		
//float_4byte
union float_4byte							
{
	float d;
	unsigned char data[4];
}battery_v,							//������ѹ���,
leftdata,								//��������
rightdata;							//��������
//int_2byte
union	 int_2byte							
{	
	u16 d;
	u8 data[2];
}Robot_Status,					//�����˹���ģʽ��������ģʽ+��������
Robot_Error,						//�������쳣�롪���쳣����+�쳣����
Charge_Exit_Seconds;		//���ʱ��
//��̼����ݹ�����
union odometry								
{
	float odoemtry_float;
	u8 odometry_char[4];
}x_data,
y_data,
theta_data,
vel_linear,
vel_angular;
//�ֽڳ��ȶ���
enum {_0byte = 0, _1byte, _2byte, _3byte, _4byte, _5byte, _6byte, _7byte, _8byte, _9byte, _10byte, _11byte, _12byte} Data_Length;
//TRUE_FALSE����
enum {FALSE = 0, TRUE = !FALSE} Boolean;

#endif
