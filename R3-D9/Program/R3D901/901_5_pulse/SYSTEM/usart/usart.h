/*
************************************************************************************************************************
*                                                      USART
*                               Universal Synchronous/Asynchronous Receiver/Transmitter
*                                  				(c) Copyright 2009-2012; YHT
*                           All rights reserved.  Protected by international copyright laws.
*
* File    : USART.H
* By      : YHT
* Version : V1.03.00
*
* LICENSING TERMS:
* ---------------
*           uC/OS-III is provided in source form for FREE short-term evaluation, for educational use or 
*           for peaceful research.  If you plan or intend to use uC/OS-III in a commercial application/
*           product then, you need to contact Micrium to properly license uC/OS-III for its use in your 
*           application/product.   We provide ALL the source code for your convenience and to help you 
*           experience uC/OS-III.  The fact that the source is provided does NOT mean that you can use 
*           it commercially without paying a licensing fee.
*
*           Knowledge of the source code may NOT be used to develop a similar product.
*
*           Please help us continue to provide the embedded community with the finest software available.
*           Your honesty is greatly appreciated.
*
*           You can contact us at www.micrium.com, or by phone at +1 (954) 217-2036.
************************************************************************************************************************
* Note(s) : (1) Assumes the following versions (or more recent) of software modules are included in the project build:
*
*               (a) uC/LIB V1.36.01
*               (b) uC/CPU V1.29.00
************************************************************************************************************************
*/

#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h"
#include "dma.h"	

#define EN_USART1 			1			//使能（1）/禁止（0）串口1
#define EN_USART2 			1			//使能（1）/禁止（0）串口2
#define EN_USART3 			1			//使能（1）/禁止（0）串口3
#define EN_USART4 			0			//使能（1）/禁止（0）串口4
#define EN_USART5 			0			//使能（1）/禁止（0）串口5

#define EN_USART1_RX 			1			//使能（1）/禁止（0）串口1接收
#define EN_USART2_RX 			0			//使能（1）/禁止（0）串口2接收
#define EN_USART3_RX 			1			//使能（1）/禁止（0）串口3接收
#define EN_USART4_RX 			0			//使能（1）/禁止（0）串口4接收
#define EN_USART5_RX 			0			//使能（1）/禁止（0）串口5接收

#define SEND_BUF1_SIZE		128		//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.
#define SEND_BUF3_SIZE		128		//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.

//如果想串口中断接收，请不要注释以下宏定义
void uart_init(void);
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void uart4_init(u32 bound);
void uart5_init(u32 bound);

extern u8 UART1_RX_BUF_RECIVE;													//接收数组计数
extern unsigned char UART1_RX_BUFF[128];     					//接收缓存区
extern u16 USART3_RX_STA;																//接收状态标记

extern uint8_t get_uart1_data;
typedef union							//接收到的数据
{
	float d;
	unsigned char data[4];
}sensor_rec;

extern sensor_rec temperature,humidity,air_velocity,air_quality,voice,h2_value,pm1_0,pm2_5,pm10;					//接收的传感器数据
/*******************************************UNION VARIABLES**************************************************************/

	
//extern u8 SendBuff3[SEND_BUF3_SIZE];										//发送缓存区

u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 register_address,u8 length);		//串口1数据整理及发送函数
u8 uart3_format_build_and_send(u8 *arg,u8 device_id,u8 data_id,u8 length);			//串口3数据发送函数

#endif


