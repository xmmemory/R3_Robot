
#ifndef  INCLUDES_U_TASK
#define  INCLUDES_U_TASK

#include "r3driver.h"

////////////////////////定时器////////////////////////////////
u8 tmr1sta=0; 											//标记定时器的工作状态
OS_TMR	Sencond_1_tmr1;							//定义一个定时器
OS_SEM	MY_SEM;											//定义一个信号量，用于控制ODOM的计算
OS_TMR	Odom_Calculator_tmr2;				//定时器2---里程计pwm捕获回调函数---10ms--100hz

//UCOSIII中以下优先级用户程序不能使用，RAY_ROBOT
//将这些优先级分配给了UCOSIII的5个系统内部任务
//优先级0：中断服务服务管理任务 OS_IntQTask()
//优先级1：时钟节拍任务 OS_TickTask()
//优先级2：定时任务 OS_TmrTask()
//优先级OS_CFG_PRIO_MAX-2：统计任务 OS_StatTask()
//优先级OS_CFG_PRIO_MAX-1：空闲任务 OS_IdleTask()

//任务优先级
#define START_TASK_PRIO   3
//任务堆栈大小
#define START_STK_SIZE 		128
//任务控制块
OS_TCB StartTaskTCB;
//任务堆栈	
CPU_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *p_arg);

//任务优先级
#define MAIN_TASK_PRIO   4
//任务堆栈大小	
#define MAIN_STK_SIZE 		128
//任务控制块
OS_TCB Main_TaskTCB;
//任务堆栈	
CPU_STK MAIN_TASK_STK[MAIN_STK_SIZE];
void main_task(void *p_arg);

//任务优先级
#define MSGDIS_TASK_PRIO   5
//任务堆栈
#define MSGDIS_STK_SIZE		128
//任务控制块
OS_TCB	Msgdis_TaskTCB;
//任务堆栈
CPU_STK	MSGDIS_TASK_STK[MSGDIS_STK_SIZE];
//任务函数
void msgdis_task(void *p_arg);

////任务优先级
//#define ODOM_SEND_TASK_PRIO   10
////任务堆栈
//#define ODOM_SEND_STK_SIZE		128
////任务控制块
//OS_TCB	ODOM_SEND_TaskTCB;
////任务堆栈
//CPU_STK	ODOM_SEND_TASK_STK[ODOM_SEND_STK_SIZE];
////任务函数
//void odom_send_task(void *p_arg);

//任务优先级
#define SPEED_TRANSFORM_TASK_PRIO   11
//任务堆栈
#define SPEED_TRANSFORM_STK_SIZE		128
//任务控制块
OS_TCB	SPEED_TRANSFORM_TaskTCB;
//任务堆栈
CPU_STK	SPEED_TRANSFORM_TASK_STK[SPEED_TRANSFORM_STK_SIZE];
//任务函数
void speed_transform_task(void *p_arg);

//任务优先级
#define UART1_ANALYZE_TASK_PRIO   12
//任务堆栈
#define UART1_ANALYZE_STK_SIZE		128
//任务控制块
OS_TCB	UART1_ANALYZE_TaskTCB;
//任务堆栈
CPU_STK	UART1_ANALYZE_TASK_STK[UART1_ANALYZE_STK_SIZE];
//任务函数
void uart1_analyze_task(void *p_arg);

////任务优先级
//#define DOOR_OPEN_TASK_PRIO   14
////任务堆栈
//#define DOOR_OPEN_STK_SIZE		128
////任务控制块
//OS_TCB	DOOR_OPEN_TaskTCB;
////任务堆栈
//CPU_STK	DOOR_OPEN_TASK_STK[DOOR_OPEN_STK_SIZE];
////任务函数
//void door_open_task(void *p_arg);

//任务优先级
#define AUTO_CHARGE_TASK_PRIO   16
//任务堆栈
#define AUTO_CHARGE_STK_SIZE		128
//任务控制块
OS_TCB	AUTO_CHARGE_TaskTCB;
//任务堆栈
CPU_STK	AUTO_CHARGE_TASK_STK[AUTO_CHARGE_STK_SIZE];
//任务函数
void auto_charge_task(void *p_arg);

//任务优先级
#define REMOTE_CONTROL_TASK_PRIO   17
//任务堆栈
#define REMOTE_CONTROL_STK_SIZE		128
//任务控制块
OS_TCB	REMOTE_CONTROL_TaskTCB;
//任务堆栈
CPU_STK	REMOTE_CONTROL_TASK_STK[REMOTE_CONTROL_STK_SIZE];
//任务函数
void remote_control_task(void *p_arg);

#endif
