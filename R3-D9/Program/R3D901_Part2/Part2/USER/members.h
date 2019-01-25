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

#define	DEVICE_ID_D9					0X01		//设备ID-0x01-底盘控制器
#define	COM_COMMOND						0X01		//命令ID-0x01-通讯指令
#define	READ_COMMOND					0X02		//命令ID-0x02-读取指令
#define	WRITE_COMMOND					0X03		//命令ID-0x03-写入指令

#define MAX_Circle_Moving_Count 85*1		//旋转寻找信号最大连续尝试次数-85/Circle
/*********************************************************************************************************
*                                              REGISTER
*********************************************************************************************************/
#define	REG_STATUS						0x01		//寄存器-0x01-运行状态
#define REG_ODOMETRY					0x10		//寄存器-0x10-里程计
#define REG_BATTERY_VOLTAGE		0x11		//寄存器-0x11-电量计
#define REG_CHARGING_TIME			0x20		//寄存器-0x20-充电时长
#define REG_MOVING_SPEED			0x21		//寄存器-0x21-运动速度
#define REG_SET_POSOTION			0x31		//寄存器-0x31-位置坐标
#define REG_NAVI_STATUS				0x32		//寄存器-0x32-NAVI状态
#define REG_REAL_ODOM					0x33		//寄存器-0x33-实际ODOM
#define REG_SERVER_SEND				0x60		//寄存器-0x60-透传服务
#define REG_ERROR							0xF1		//寄存器-0xF1-错误码

/*********************************************************************************************************/
#define REG_FLASH_ENABLE			0xF5		//寄存器-0xF5-0x5x写入权限使能
//#define REG_ERROR							0xF1		//寄存器-0xF1-错误码
//#define REG_ERROR							0xF1		//寄存器-0xF1-错误码
//#define REG_ERROR							0xF1		//寄存器-0xF1-错误码
/*——————————需要0x04的写入指令来写入——————————*/
#define REG_WHEEL_INTERVAL		0x51		//寄存器-0x51-两轮间距
#define REG_WHEEL_DIAMETER		0x52		//寄存器-0x52-轮子直径
#define REG_LEFT_WHEEL_RATIO	0x53		//寄存器-0x53-左轮运动比例系数
#define REG_RIGHT_WHEEL_RATIO	0x54		//寄存器-0x54-右轮运动比例系数
//#define REG_WHEEL_INTERVAL		0x51		//寄存器-0x51-两轮间距
/**********************************************Robot_Status***************************************************/
#define	ERROT_STATUS					0x00		//状态-0x00-异常状态
#define	NAVI_STATUS						0x01		//状态-0x01-导航状态
#define	CHARGING_STATUS				0x02		//状态-0x02-充电状态
#define	FINDING_DOCK_STATUS		0x03		//状态-0x03-找电状态
/**********************************************Robot_Error***************************************************/
#define	UULL_ERROT						0x00		//状态-0x00-无异常
#define	CHARGE_FALSE					0x01		//状态-0x01-充电失败
//#define	CHARGING_STATUS				0x02		//状态-0x02-充电状态
//#define	FINDING_DOCK_STATUS		0x03		//状态-0x03-找电状态



//要写入到STM32 FLASH的字符串数组
const unsigned char TEXT_Buffer[]={"STM32 FLASH TEST"};
#define TEXT_LENTH sizeof(TEXT_Buffer)	 		  	//数组长度	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)

#define FLASH_SAVE_ADDR  0X0800C004 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.
										//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.

/*********************************************************************************************************
*                                              	NUMBERS
*********************************************************************************************************/
unsigned char main_sta = 0;									//任务调度函数
unsigned char Moving_Seconds;						//充电脱离时长---n*10ms
unsigned char temp_fr_control = 0;						//TCP发送频率限制
unsigned char Robot_Status;									//机器人工作模式
unsigned char Robot_Error;										//机器人异常码
unsigned int Circle_Moving_Count = 0;				//充电旋转尝试次数计数
unsigned char IR_No_Find_Count = 0;					//红外无信号计数
//odom
unsigned char odometry_data[64];		
//float_4byte
union float_4byte							
{
	float d;
	unsigned char data[4];
}leftdata,								//左轮数据
rightdata;								//右轮数据
//int_2byte
union	 int_2byte							
{	
	unsigned short int d;
	unsigned char data[2];
}Charge_Exit_Seconds;			//充电时长
//里程计数据共用体
union odometry								
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,
y_data,
theta_data,
vel_linear,
vel_angular;
//字节长度定义
enum {_0byte = 0, _1byte, _2byte, _3byte, _4byte, _5byte, _6byte, _7byte, _8byte, _9byte, _10byte, _11byte, _12byte} Data_Length;
//方向定义
enum {IR_LEFT = 1, IR_MIDDLE, IR_RIGHT} IR_POSITION;
//TRUE_FALSE定义
enum {FALSE = 0, TRUE = !FALSE} Boolean;

#endif
