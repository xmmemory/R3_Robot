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
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//����1��ʼ��		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.csom
//�޸�����:2011/6/14
//�汾��V1.4
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
//********************************************************************************
//V1.3�޸�˵�� 
//֧����Ӧ��ͬƵ���µĴ��ڲ���������.
//�����˶�printf��֧��
//�����˴��ڽ��������.
//������printf��һ���ַ���ʧ��bug
//V1.4�޸�˵��
//1,�޸Ĵ��ڳ�ʼ��IO��bug
//2,�޸���USART_RX_STA,ʹ�ô����������ֽ���Ϊ2��14�η�
//3,������USART_REC_LEN,���ڶ��崮�����������յ��ֽ���(������2��14�η�)
//4,�޸���EN_USART1_RX��ʹ�ܷ�ʽ
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			100  		//�����������ֽ��� 200
#define UART3_REC_LEN  			100  		//�����������ֽ��� 100
#define UART4_REC_LEN  			100  		//�����������ֽ��� 100
#define UART5_REC_LEN  			100  		//�����������ֽ��� 100
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8  UART3_RX_BUF[UART3_REC_LEN]; //���ջ���,���USART4_REC_LEN���ֽ�.
extern u8  UART4_RX_BUF[UART4_REC_LEN]; //���ջ���,���USART4_REC_LEN���ֽ�.
extern u8  UART5_RX_BUF[UART5_REC_LEN]; //���ջ���,���USART5_REC_LEN���ֽ�.
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(void);
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void uart4_init(u32 bound);

#endif


