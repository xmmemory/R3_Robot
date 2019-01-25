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

#define	DEVICE_ID_D9					0X01		//�豸ID-0x01-���̿�����
#define	COM_COMMOND						0X01		//����ID-0x01-ͨѶָ��
#define	READ_COMMOND					0X02		//����ID-0x02-��ȡָ��
#define	WRITE_COMMOND					0X03		//����ID-0x03-д��ָ��

#define MAX_Circle_Moving_Count 85*1		//��תѰ���ź�����������Դ���-85/Circle
/*********************************************************************************************************
*                                              REGISTER
*********************************************************************************************************/
#define	REG_STATUS						0x01		//�Ĵ���-0x01-����״̬
#define REG_ODOMETRY					0x10		//�Ĵ���-0x10-��̼�
#define REG_BATTERY_VOLTAGE		0x11		//�Ĵ���-0x11-������
#define REG_CHARGING_TIME			0x20		//�Ĵ���-0x20-���ʱ��
#define REG_MOVING_SPEED			0x21		//�Ĵ���-0x21-�˶��ٶ�
#define REG_SET_POSOTION			0x31		//�Ĵ���-0x31-λ������
#define REG_NAVI_STATUS				0x32		//�Ĵ���-0x32-NAVI״̬
#define REG_REAL_ODOM					0x33		//�Ĵ���-0x33-ʵ��ODOM
#define REG_SERVER_SEND				0x60		//�Ĵ���-0x60-͸������
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
const unsigned char TEXT_Buffer[]={"STM32 FLASH TEST"};
#define TEXT_LENTH sizeof(TEXT_Buffer)	 		  	//���鳤��	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)

#define FLASH_SAVE_ADDR  0X0800C004 	//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.
										//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.

/*********************************************************************************************************
*                                              	NUMBERS
*********************************************************************************************************/
unsigned char main_sta = 0;									//������Ⱥ���
unsigned char Moving_Seconds;						//�������ʱ��---n*10ms
unsigned char temp_fr_control = 0;						//TCP����Ƶ������
unsigned char Robot_Status;									//�����˹���ģʽ
unsigned char Robot_Error;										//�������쳣��
unsigned int Circle_Moving_Count = 0;				//�����ת���Դ�������
unsigned char IR_No_Find_Count = 0;					//�������źż���
//odom
unsigned char odometry_data[64];		
//float_4byte
union float_4byte							
{
	float d;
	unsigned char data[4];
}leftdata,								//��������
rightdata;								//��������
//int_2byte
union	 int_2byte							
{	
	unsigned short int d;
	unsigned char data[2];
}Charge_Exit_Seconds;			//���ʱ��
//��̼����ݹ�����
union odometry								
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,
y_data,
theta_data,
vel_linear,
vel_angular;
//�ֽڳ��ȶ���
enum {_0byte = 0, _1byte, _2byte, _3byte, _4byte, _5byte, _6byte, _7byte, _8byte, _9byte, _10byte, _11byte, _12byte} Data_Length;
//������
enum {IR_LEFT = 1, IR_MIDDLE, IR_RIGHT} IR_POSITION;
//TRUE_FALSE����
enum {FALSE = 0, TRUE = !FALSE} Boolean;

#endif
