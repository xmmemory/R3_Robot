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

//�ֽڳ��ȶ���
enum {_0byte = 0, _1byte, _2byte, _3byte, _4byte, _5byte, _6byte, _7byte, _8byte, _9byte, _10byte, _11byte, _12byte} Data_Length;
//������
enum {IR_LEFT = 1, IR_MIDDLE, IR_RIGHT} IR_POSITION;
//TRUE_FALSE����
enum {FALSE = 0, TRUE = !FALSE} Boolean;

#define	DEVICE_ID_D9					0X01		//�豸ID-0x01-���̿�����
#define	COM_COMMOND						0X01		//����ID-0x01-ͨѶָ��
#define	READ_COMMOND					0X02		//����ID-0x02-��ȡָ��
#define	WRITE_COMMOND					0X03		//����ID-0x03-д��ָ��
#define	REBOOT_COMMOND				0XFF		//����ID-0xFF-����ָ��

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
/*********************************************************************************************************
*                                              REBOOT
*********************************************************************************************************/
u8 RebootBuff1[3] = {0xBC,
0x34,0x56};														//PC��������ָ����Ĵ���-0x0A(ռλ)
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
/**********************************************Check_Enable***************************************************/
#define EN_Check	 			0			//ʹ�ܣ�1��/��ֹ��0��
#define EN_Check_1 			1			//ʹ�ܣ�1��/��ֹ��0��
#define EN_Check_2 			1			//ʹ�ܣ�1��/��ֹ��0��
#define EN_Check_3 			1			//ʹ�ܣ�1��/��ֹ��0��
#define EN_Check_4 			0			//ʹ�ܣ�1��/��ֹ��0��
#define EN_Check_5 			0			//ʹ�ܣ�1��/��ֹ��0��


/**********************************************Type_Define***************************************************/
typedef union
{
	struct
	{
		uint32_t ser_num;				//�ɼ�ָ����ˮ��             
		uint16_t channel;				//ͨ������
		uint8_t battery;				//�����ٷֱ�
		double pose_x;					//X����
		double pose_y;					//Y����
		double pose_theta;			//THETA��̬���������
		uint8_t infred;					//����
		uint32_t 
		charing_time_surplus;		//PM0.3�������ʣ��ʱ��
		uint32_t pm1_0;					//PM1.0
		uint32_t pm2_5;					//PM2.5
		uint32_t pm10;					//PM10
		float air_quality;			//��������
		uint16_t voice1;				//����left
		uint16_t voice2;				//����right
		float hydr;							//����
		double wind_speed;			//����
		float temp;							//�¶�
		uint8_t humi;						//ʪ��
		float co;								//һ����̼
		float is_Charge;				//���������Ƿ���	
	}GNUC_PACKED d;
	uint8_t data[80];
}SensorToServer_TypeDef;

//Ҫд�뵽STM32 FLASH���ַ�������
const unsigned char TEXT_Buffer[]={"STM32 FLASH TEST"};
#define TEXT_LENTH sizeof(TEXT_Buffer)	 		  	//���鳤��	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)

#define FLASH_SAVE_ADDR  0X0800C004 	//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.
										//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.

/*********************************************************************************************************
*                                              	NUMBERS
*********************************************************************************************************/
uint8_t main_sta = 0;												//������Ⱥ���
uint8_t Moving_Seconds = 250;								//�������ʱ��---n*10ms
uint8_t temp_fr_control = 0;								//TCP����Ƶ������
uint8_t Robot_Status;												//�����˹���ģʽ
uint8_t Robot_Error;												//�������쳣��
unsigned int Circle_Moving_Count = 0;			//�����ת���Դ�������
uint8_t IR_No_Find_Count = 0;								//�������źż���
uint8_t Server_Developer_Is_Fool = TRUE;		//д���������ǲ���ɵ��
uint8_t _One_Second_Count = 0;							//1s��������
uint8_t Speed_Clear_Count = 0;							//�ٶ���ռ���
//odom
unsigned char odometry_data[64];		
//float_4byte
union float_4byte							
{
	float d;
	unsigned char data[4];
}linear_vel,
angular_vel;

//int_2byte
typedef union	 							
{	
	unsigned short int d;
	unsigned char data[2];
}int_2byte;
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


#endif
