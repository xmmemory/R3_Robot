
#ifndef  INCLUDES_U_TASK
#define  INCLUDES_U_TASK

#include "r3driver.h"

////////////////////////��ʱ��////////////////////////////////
u8 tmr1sta=0; 											//��Ƕ�ʱ���Ĺ���״̬
OS_TMR	Sencond_1_tmr1;							//����һ����ʱ��
OS_SEM	MY_SEM;											//����һ���ź��������ڿ���ODOM�ļ���
OS_TMR	Odom_Calculator_tmr2;				//��ʱ��2---��̼�pwm����ص�����---10ms--100hz

//UCOSIII���������ȼ��û�������ʹ�ã�RAY_ROBOT
//����Щ���ȼ��������UCOSIII��5��ϵͳ�ڲ�����
//���ȼ�0���жϷ������������� OS_IntQTask()
//���ȼ�1��ʱ�ӽ������� OS_TickTask()
//���ȼ�2����ʱ���� OS_TmrTask()
//���ȼ�OS_CFG_PRIO_MAX-2��ͳ������ OS_StatTask()
//���ȼ�OS_CFG_PRIO_MAX-1���������� OS_IdleTask()

//�������ȼ�
#define START_TASK_PRIO   3
//�����ջ��С
#define START_STK_SIZE 		128
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *p_arg);

//�������ȼ�
#define MAIN_TASK_PRIO   4
//�����ջ��С	
#define MAIN_STK_SIZE 		128
//������ƿ�
OS_TCB Main_TaskTCB;
//�����ջ	
CPU_STK MAIN_TASK_STK[MAIN_STK_SIZE];
void main_task(void *p_arg);

//�������ȼ�
#define MSGDIS_TASK_PRIO   5
//�����ջ
#define MSGDIS_STK_SIZE		128
//������ƿ�
OS_TCB	Msgdis_TaskTCB;
//�����ջ
CPU_STK	MSGDIS_TASK_STK[MSGDIS_STK_SIZE];
//������
void msgdis_task(void *p_arg);

////�������ȼ�
//#define ODOM_SEND_TASK_PRIO   10
////�����ջ
//#define ODOM_SEND_STK_SIZE		128
////������ƿ�
//OS_TCB	ODOM_SEND_TaskTCB;
////�����ջ
//CPU_STK	ODOM_SEND_TASK_STK[ODOM_SEND_STK_SIZE];
////������
//void odom_send_task(void *p_arg);

//�������ȼ�
#define SPEED_TRANSFORM_TASK_PRIO   11
//�����ջ
#define SPEED_TRANSFORM_STK_SIZE		128
//������ƿ�
OS_TCB	SPEED_TRANSFORM_TaskTCB;
//�����ջ
CPU_STK	SPEED_TRANSFORM_TASK_STK[SPEED_TRANSFORM_STK_SIZE];
//������
void speed_transform_task(void *p_arg);

//�������ȼ�
#define UART1_ANALYZE_TASK_PRIO   12
//�����ջ
#define UART1_ANALYZE_STK_SIZE		128
//������ƿ�
OS_TCB	UART1_ANALYZE_TaskTCB;
//�����ջ
CPU_STK	UART1_ANALYZE_TASK_STK[UART1_ANALYZE_STK_SIZE];
//������
void uart1_analyze_task(void *p_arg);

////�������ȼ�
//#define DOOR_OPEN_TASK_PRIO   14
////�����ջ
//#define DOOR_OPEN_STK_SIZE		128
////������ƿ�
//OS_TCB	DOOR_OPEN_TaskTCB;
////�����ջ
//CPU_STK	DOOR_OPEN_TASK_STK[DOOR_OPEN_STK_SIZE];
////������
//void door_open_task(void *p_arg);

//�������ȼ�
#define AUTO_CHARGE_TASK_PRIO   16
//�����ջ
#define AUTO_CHARGE_STK_SIZE		128
//������ƿ�
OS_TCB	AUTO_CHARGE_TaskTCB;
//�����ջ
CPU_STK	AUTO_CHARGE_TASK_STK[AUTO_CHARGE_STK_SIZE];
//������
void auto_charge_task(void *p_arg);

//�������ȼ�
#define REMOTE_CONTROL_TASK_PRIO   17
//�����ջ
#define REMOTE_CONTROL_STK_SIZE		128
//������ƿ�
OS_TCB	REMOTE_CONTROL_TaskTCB;
//�����ջ
CPU_STK	REMOTE_CONTROL_TASK_STK[REMOTE_CONTROL_STK_SIZE];
//������
void remote_control_task(void *p_arg);

#endif
