//********************************************************************************
//�û����� ����	 
//�޸�����:2018-10-22
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ��������(����)���ܻ����˿Ƽ����޹�˾ 
//All rights reserved.
//********************************************************************************
#ifndef __USER_H
#define __USER_H

#include "sys.h"

void system_init_step1(void);			//�ڲ���ʼ��
void system_init_step2(void);			//�˶����ֳ�ʼ��
void system_init_step3(void);			//ͨѶ��ʼ��
void system_init_step4(void);			//������ʼ��
void system_init_step5(void);			//����ʼ��

void uart1_deal(void);							//�������ݴ���

void odom_send_task(void);					//��̼�������������
void auto_charge_task(void);				//�Զ��������
void exit_charge_task(void);				//�����������
u8 battery_check(void);							//��ѹ�������
u8 remote_get(void);								//����



#endif

