/***********************************************  说明  *****************************************************************
*   1.串口接收
*    （1）内容：小车左右轮速度,单位:mm/s（所有数据都为float型，float型占4字节）
*    （2）格式：10字节 [起始符"0xFF"][右轮速度4字节][左轮速度4字节][门序号][结束符"\n"]
*   2.串口发送
*    （1）内容：里程计（x,y坐标、线速度、角速度和方向角，单位依次为：mm,mm,mm/s,rad/s,rad，所有数据都为float型，float型占4字节）
*    （2）格式：21字节 [x坐标4字节][y坐标4字节][方向角4字节][线速度4字节][角速度4字节][结束符"\nYHT"4字节]
************************************************************************************************************************/
#include "includes.h"
/*
*********************************************************************************************************
*                                               DECLARED
*********************************************************************************************************
*/
u8 Format_Check(u8 *p_arg,u8 length);							//数据格式校验
void tmr1_callback(void *p_tmr,void *p_arg); 		//定时器1回调函数
void Odom_Calculator(void *p_tmr, void *p_arg); 	//定时器2---里程计odom计算函数---20ms--50hz
void PVD_Config(void);														//低电压中断初始化
void Stop_Moving(void);													//机器人停止子程序
void Robot_Moving(float L_speed,float R_speed,u8 time);					//机器人移动
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length);		//串口1数据整理及发送函数
/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/
#define GNUC_PACKED __attribute__((packed))

#define TASK_Q_NUM	3							//发任务内建消息队列的长度
#define SPEED_TRANSFORM_Q_NUM	2		//速度下发任务内建消息队列的长度---2
#define UART1_SEND_Q_NUM	3				//串口1发任务内建消息队列的长度---3
#define UART1_ANALYZE_Q_NUM	2			//串口1数据解析内建消息队列的长度---2
#define SPEED_TRANSFORM_TIMEOUT 200u	//n*5ms的等待时长--1s
#define SENSOR_POST_TIMEOUT     2000u	//n*5ms的等待时长--10s

//要写入到STM32 FLASH的字符串数组
const u8 TEXT_Buffer[]={"STM32 FLASH TEST"};
#define TEXT_LENTH sizeof(TEXT_Buffer)	 		  	//数组长度	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)

#define FLASH_SAVE_ADDR  0X0800C004 	//设置FLASH 保存地址(必须为偶数，且所在扇区,要大于本代码所占用到的扇区.
										//否则,写操作的时候,可能会导致擦除整个扇区,从而引起部分程序丢失.引起死机.

/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/
extern u8 odom_cal_status;//主函数步骤执行标志位
extern int PULSE_RIGHT,PULSE_LEFT;
int PULSE_RIGHT_HIS,PULSE_LEFT_HIS;
u8 Charge_Status = 0;						//控制权限切换——1为本地控制、0为外部控制

//上位机可修改的参数
u8 charge_status_push = 0;		//充电状态上传频率
u8 Exit_Charge_Seconds = 20;				//充电脱离时长---n*100ms
u32 Odom_Rate_Seconds = 100;				//odom上传间隔、100ms 
/**************************************************************************************************************************/
#define SEND_BUF_SIZE 256	//发送数据长度,最好等于sizeof(TEXT_TO_SEND)+2的整数倍.

u8 SendBuff[SEND_BUF_SIZE];	//发送数据缓冲区
u8 Battery_Status_Temp = 0;
////////////////////////定时器////////////////////////////////
u8 tmr1sta=0; 											//标记定时器的工作状态
OS_TMR	battery_status_tmr1;				//定义一个定时器
OS_SEM	MY_SEM;											//定义一个信号量，用于控制ODOM的计算
OS_TMR	Odom_Calculator_tmr2;				//定时器2---里程计pwm捕获回调函数---10ms--100hz
/**************************************************************************************************************************/
union recieveData							//接收到的数据
{
	float d;
	unsigned char data[4];
}leftdata,rightdata;					//接收的左右轮数据
union sendData								//接收到的数据
{
	float d;
	unsigned char data[4];
}battery_v;					//电量电压监测
union odometry								//里程计数据共用体
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;     
//要发布的里程计数据，分别为：X，Y方向移动的距离，当前角度，线速度，角速度
extern float position_x,position_y,oriention,velocity_linear,velocity_angular;         //计算得到的里程计数值
/************************************************************************************************************************/

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
#define MOTOR_INIT_TASK_PRIO   5
//任务堆栈
#define MOTOR_INIT_STK_SIZE		128
//任务控制块
OS_TCB	MOTOR_INIT_TaskTCB;
//任务堆栈
CPU_STK	MOTOR_INIT_TASK_STK[MOTOR_INIT_STK_SIZE];
//任务函数
void motor_init_task(void *p_arg);

//任务优先级
#define MSGDIS_TASK_PRIO   6
//任务堆栈
#define MSGDIS_STK_SIZE		128
//任务控制块
OS_TCB	Msgdis_TaskTCB;
//任务堆栈
CPU_STK	MSGDIS_TASK_STK[MSGDIS_STK_SIZE];
//任务函数
void msgdis_task(void *p_arg);

//任务优先级
#define ODOM_SEND_TASK_PRIO   10
//任务堆栈
#define ODOM_SEND_STK_SIZE		128
//任务控制块
OS_TCB	ODOM_SEND_TaskTCB;
//任务堆栈
CPU_STK	ODOM_SEND_TASK_STK[ODOM_SEND_STK_SIZE];
//任务函数
void odom_send_task(void *p_arg);

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

//任务优先级
#define DOOR_OPEN_TASK_PRIO   14
//任务堆栈
#define DOOR_OPEN_STK_SIZE		128
//任务控制块
OS_TCB	DOOR_OPEN_TaskTCB;
//任务堆栈
CPU_STK	DOOR_OPEN_TASK_STK[DOOR_OPEN_STK_SIZE];
//任务函数
void door_open_task(void *p_arg);

//任务优先级
#define BATTERY_SEND_TASK_PRIO   15
//任务堆栈
#define BATTERY_SEND_STK_SIZE		128
//任务控制块
OS_TCB	BATTERY_SEND_TaskTCB;
//任务堆栈
CPU_STK	BATTERY_SEND_TASK_STK[BATTERY_SEND_STK_SIZE];
//任务函数
void battery_send_task(void *p_arg);

//任务优先级
#define REMOTE_CONTROL_TASK_PRIO   16
//任务堆栈
#define REMOTE_CONTROL_STK_SIZE		128
//任务控制块
OS_TCB	REMOTE_CONTROL_TaskTCB;
//任务堆栈
CPU_STK	REMOTE_CONTROL_TASK_STK[REMOTE_CONTROL_STK_SIZE];
//任务函数
void remote_control_task(void *p_arg);

//主函数
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	delay_init(168);  		//时钟初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组配置
//	Remote_Init();				//红外接收初始化
	uart_init();   				//串口初始化
	LED_Init();       	  //LED初始化
	KEY_Init();						//按键_充电状态判断引脚初始化
	BEEP_Init();					//初始化蜂鸣器
	Adc_Init();         	//初始化ADC
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN初始化普通模式,波特率500Kbps    
	
	ENC_Init_Left();					//设置电机D TIM4编码器模式PB6 PB7 左电机 
	ENC_Init_Right();					//设置电机A TIM3编码器模式PC6 PC7 右电机
	
	PVD_Config();					//低电压中断初始化
	
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	
	delay_ms(1000);				//上电等待驱动器上电---part1
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
	MYDMA_Enable(DMA2_Stream7,1);     //开始一次DMA传输
	
	
//	//里程计算定时器初始化
//	TIM5_Int_Init(100-1,8400-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms(500次为50ms)(50次为5ms)(5_0.5ms_2Khz)	

	GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP引脚拉高， 等同BEEP=1;	
	
	my_mem_init(SRAMIN);//初始化内部RAM
	OSInit(&err);		    //初始化UCOSIII
	OS_CRITICAL_ENTER();	//进入临界区			 
	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 		//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,					//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,					//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,					//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,					//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区
	OSStart(&err);      //开启UCOSIII
}
/*
************************************************************************************************************************
*                                                   start_task
*
* Description: Null；
*
* Arguments  : Null
*
* Note(s)    : 1) Null；
************************************************************************************************************************
*/
//开始任务函数
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	
	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//统计任务                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//如果使能了测量中断关闭时间
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //当使用时间片轮转的时候
	 //使能时间片轮转调度功能,时间片长度为1个系统时钟节拍，既1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif	
	
	//进入临界区	
	OS_CRITICAL_ENTER();
//	//创建一个信号量
//	OSSemCreate ((OS_SEM*	)&ODOM_SEND,
//                 (CPU_CHAR*	)"ODOM_SEND",
//                 (OS_SEM_CTR)1,		
//                 (OS_ERR*	)&err);
	//创建一个信号量
	OSSemCreate ((OS_SEM*	)&MY_SEM,
                 (CPU_CHAR*	)"MY_SEM",
                 (OS_SEM_CTR)1,		
                 (OS_ERR*	)&err);
//	//创建一个事件标志组
//	OSFlagCreate((OS_FLAG_GRP*)&Sensor_Send_Flags,		//指向事件标志组
//                 (CPU_CHAR*	  )"Sensor Send Flags",	//名字
//                 (OS_FLAGS	  )0,	//事件标志组初始值
//                 (OS_ERR*  	  )&err);			//错误码	
	//创建定时器1
	OSTmrCreate((OS_TMR		*)&battery_status_tmr1,		//定时器1
                (CPU_CHAR	*)"battery status tmr1",		//定时器名字
                (OS_TICK	 )0,			//0ms
                (OS_TICK	 )10,          //10*10=100ms
                (OS_OPT		 )OS_OPT_TMR_PERIODIC, //周期模式
                (OS_TMR_CALLBACK_PTR)tmr1_callback,//定时器1回调函数
                (void	    *)0,			//参数为0
                (OS_ERR	    *)&err);		//返回的错误码
	//创建定时器2
	OSTmrCreate((OS_TMR		*)&Odom_Calculator_tmr2,		
                (CPU_CHAR	*)"Odom Calculator tmr2",		
                (OS_TICK	 )0,
                (OS_TICK	 )1,   	//1*10=10ms				
                (OS_OPT		 )OS_OPT_TMR_PERIODIC, 	//周期模式
                (OS_TMR_CALLBACK_PTR)Odom_Calculator,	//定时器2回调函数---里程计pwm捕获回调函数---10ms--100hz
                (void	    *)0,			
                (OS_ERR	    *)&err);									
	//创建MOTOR_INIT任务
	OSTaskCreate((OS_TCB 	* )&MOTOR_INIT_TaskTCB,	
								 (CPU_CHAR	* )"motor init task", 		
								 (OS_TASK_PTR )motor_init_task, 			
								 (void		* )0,
								 (OS_PRIO	  )MOTOR_INIT_TASK_PRIO,     
								 (CPU_STK   * )&MOTOR_INIT_TASK_STK[0],	
								 (CPU_STK_SIZE)MOTOR_INIT_STK_SIZE/10,
								 (CPU_STK_SIZE)MOTOR_INIT_STK_SIZE,		
								 (OS_MSG_QTY  )0,		//MOTOR_INIT_TaskTCB不需要使用内建消息队列		
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);
	//创建ODOM_SEND任务
	OSTaskCreate((OS_TCB 	* )&ODOM_SEND_TaskTCB,	
								 (CPU_CHAR	* )"odom send task", 		
								 (OS_TASK_PTR )odom_send_task, 			
								 (void		* )0,
								 (OS_PRIO	  )ODOM_SEND_TASK_PRIO,     
								 (CPU_STK   * )&ODOM_SEND_TASK_STK[0],	
								 (CPU_STK_SIZE)ODOM_SEND_STK_SIZE/10,	
								 (CPU_STK_SIZE)ODOM_SEND_STK_SIZE,		
								 (OS_MSG_QTY  )0,		//ODOM_SEND_TaskTCB不需要使用内建消息队列		
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);
	//创建SPEED_TRANSFORM任务
	OSTaskCreate((OS_TCB 	* )&SPEED_TRANSFORM_TaskTCB,		
				 (CPU_CHAR	* )"speed transform task", 		
								 (OS_TASK_PTR )speed_transform_task, 			
								 (void		* )0,
								 (OS_PRIO	  )SPEED_TRANSFORM_TASK_PRIO,     
								 (CPU_STK   * )&SPEED_TRANSFORM_TASK_STK[0],	
								 (CPU_STK_SIZE)SPEED_TRANSFORM_STK_SIZE/10,	
								 (CPU_STK_SIZE)SPEED_TRANSFORM_STK_SIZE,		
								 (OS_MSG_QTY  )SPEED_TRANSFORM_Q_NUM,		//任务speed_transform_task需要使用内建消息队列，消息队列长度为3					
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);
	//创建UART1_ANALYZE任务
	OSTaskCreate((OS_TCB 	* )&UART1_ANALYZE_TaskTCB,	
								 (CPU_CHAR	* )"uart1 analyze task", 		
								 (OS_TASK_PTR )uart1_analyze_task, 			
								 (void		* )0,
								 (OS_PRIO	  )UART1_ANALYZE_TASK_PRIO,     
								 (CPU_STK   * )&UART1_ANALYZE_TASK_STK[0],	
								 (CPU_STK_SIZE)UART1_ANALYZE_STK_SIZE/10,	
								 (CPU_STK_SIZE)UART1_ANALYZE_STK_SIZE,		
								 (OS_MSG_QTY  )UART1_ANALYZE_Q_NUM,		//任务uart1_analyze_task需要使用内建消息队列，消息队列长度为1					
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);
	//创建battery_Collect&Send任务
	OSTaskCreate((OS_TCB 	* )&BATTERY_SEND_TaskTCB,	
								 (CPU_CHAR	* )"Battery send task", 		
								 (OS_TASK_PTR )battery_send_task, 			
								 (void		* )0,
								 (OS_PRIO	  )BATTERY_SEND_TASK_PRIO,     
								 (CPU_STK   * )&BATTERY_SEND_TASK_STK[0],	
								 (CPU_STK_SIZE)BATTERY_SEND_STK_SIZE/10,	
								 (CPU_STK_SIZE)BATTERY_SEND_STK_SIZE,		
								 (OS_MSG_QTY  )0,		//不需要使用内建消息队列			
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);	
//	//创建REMOTE_CONTROL任务
//	OSTaskCreate((OS_TCB 	* )&REMOTE_CONTROL_TaskTCB,	
//								 (CPU_CHAR	* )"remote control task", 		
//								 (OS_TASK_PTR )remote_control_task, 			
//								 (void		* )0,
//								 (OS_PRIO	  )REMOTE_CONTROL_TASK_PRIO,     
//								 (CPU_STK   * )&REMOTE_CONTROL_TASK_STK[0],	
//								 (CPU_STK_SIZE)REMOTE_CONTROL_STK_SIZE/10,	
//								 (CPU_STK_SIZE)REMOTE_CONTROL_STK_SIZE,		
//								 (OS_MSG_QTY  )0,		//不需要使用内建消息队列			
//								 (OS_TICK	  )0,  					
//								 (void   	* )0,					
//								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//								 (OS_ERR 	* )&err);									 
	OSTmrStart(&battery_status_tmr1,&err);			//启动定时器1						 								 
	OSTmrStart(&Odom_Calculator_tmr2,&err);			//启动定时器2
	//退出临界区						 
	OS_CRITICAL_EXIT();
	OSTaskDel((OS_TCB*)0,&err);	//删除start_task任务自身
}
/*
************************************************************************************************************************
*                                                   tmr1_callback
* Description: Null；
* Arguments  : Null
* Note(s)    : 1) Null；
************************************************************************************************************************
*/
//定时器1的回调函数
void tmr1_callback(void *p_tmr,void *p_arg)
{
//	OS_ERR err;		
	u8 arry[1] = {0x01};	
	
	if(Battery_Status)
	{
		Stop_Moving();
		Charge_Status = 1;		//切换为本地控制
//		GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP引脚拉高， 等同BEEP=1;
				
		uart1_format_build_and_send(arry,0x01,0x03,0x01);		//发送数据到串口---id = 0x01,commond = 0x02;		
	}
	else
	{
//		GPIO_ResetBits(GPIOF,GPIO_Pin_8); //BEEP引脚拉低， 等同BEEP=0;
		Charge_Status = 0;		//切换为上位机控制
	}

//	OSTmrStop(&battery_status_tmr1,OS_OPT_TMR_NONE,0,&err); //停止定时器1
}

/*
************************************************************************************************************************
*                                                   Odom_Calculator
* Description: 定时器1的回调函数---里程计计算发送消息给UART1_SEND_TaskTCB---20ms--50hz;
* Arguments  : Null
* Note(s)    : 1) Null；
************************************************************************************************************************
*/	
//定时器2---里程计计算---回调函数
void Odom_Calculator(void *p_tmr, void *p_arg)
{
	PULSE_LEFT = ENCODER_LEFT_TIMER->CNT - 32768;	//左电机码盘采集定时器 TIM3---TIM3编码器模式GPIOB6 GPIOB7 左电机
	ENCODER_LEFT_TIMER->CNT = 32768;
	PULSE_RIGHT = ENCODER_RIGHT_TIMER->CNT - 32768;	//右电机码盘采集定时器 TIM4---TIM4编码器模式GPIOC6 GPIOC7 右电机
	ENCODER_RIGHT_TIMER->CNT = 32768;
	
	PULSE_LEFT_HIS += PULSE_LEFT;
	PULSE_RIGHT_HIS += PULSE_RIGHT;
	//计算里程计
	odometry_cal(PULSE_LEFT,PULSE_RIGHT);
}

/*
************************************************************************************************************************
*                                                   UART1_SEND
* Description: 串口数据格式整理、然后发送消息
* Arguments  : u8* arg,u8 length;
* Note(s)    : 1) Null；
************************************************************************************************************************
*/
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length)
{
	if(length > 64)	return 1;
//	u8 uart_data[128];
//	u8 check_count;
	u16 check_sum;
	
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
	{		
		//Header
		SendBuff[0] = 0xA6;
		SendBuff[1] = 0x59;
		//Length
		SendBuff[2] = length+2+2+5+3;
		//Device ID
		SendBuff[3] = device_id;
		//Command ID
		SendBuff[4] = commond_id;
		//data
		for(u8 j=0;j<length;j++)
		{
			SendBuff[j+5]=*(arg+j);
		}		
		//Check_Sum_Cal
		for(u8 check_count = 0;check_count < length+5;check_count++)
		check_sum += SendBuff[check_count];		
		//Check_Sum
		SendBuff[((length++)+5)] = (check_sum >> 8);
		SendBuff[((length++)+5)] = check_sum;
		//Tail
		SendBuff[((length++)+5)]='\n';//添加结束符
		SendBuff[((length++)+5)]='\r';//添加结束符
		SendBuff[((length++)+5)]='Y';//添加结束符
		SendBuff[((length++)+5)]='H';//添加结束符
		SendBuff[((length++)+5)]='T';//添加结束符
		//DMA_Send
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		//清除DMA2_Steam7传输完成标志
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送     
		MYDMA_Enable(DMA2_Stream7,(length+5));     		//开始一次DMA传输！	  
		//发送成功
		return 0;		
	}	\
	else
		return 2;
}

/*
************************************************************************************************************************
*                                                   Odom_Send
* Description: 里程计数据整理、然后发送消息给UART1_SEND_TaskTCB---100ms--10hz；
* Arguments  : Null
* Note(s)    : 1) Null；
************************************************************************************************************************
*/
void odom_send_task(void *p_arg)
{
	OS_ERR err;
	u8 length;//,check_count;
	//u16 check_sum;
	while(1)
	{	
		//发送给串口的里程计数据数组---预留出2倍的长度防止异常溢出（应该用不上）
		u8 odometry_data[64];   	
		//Odom_Get
		x_data.odoemtry_float=position_x;//单位mm		
		y_data.odoemtry_float=position_y;//单位mm	
		theta_data.odoemtry_float=oriention;//单位rad
		vel_linear.odoemtry_float=velocity_linear;//单位mm/s
		vel_angular.odoemtry_float=velocity_angular;//单位rad/s
/*
		vel_linear.odoemtry_float=PULSE_LEFT_HIS;//单位mm/s
		vel_angular.odoemtry_float=PULSE_RIGHT_HIS;//单位rad/s
*/	
		//将所有里程计数据存到要发送的数组
		for(u8 j=0;j<4;j++)
		{
			odometry_data[j]=x_data.odometry_char[j];
			odometry_data[j+4]=y_data.odometry_char[j];
			odometry_data[j+8]=theta_data.odometry_char[j];
			odometry_data[j+12]=vel_linear.odometry_char[j];
			odometry_data[j+16]=vel_angular.odometry_char[j];			
		}
		length = 20;
		/**********进入临界区**********/
		CPU_SR_ALLOC();
		OS_CRITICAL_ENTER();
		uart1_format_build_and_send(odometry_data,0x01,0x01,length);		//发送数据到串口---id = 0x01,commond = 0x01;
		OS_CRITICAL_EXIT();
		/**********退出临界区*********/		
		OSTimeDlyHMSM(0,0,0,Odom_Rate_Seconds,OS_OPT_TIME_PERIODIC,&err);   //默认延时100ms--10Hz、
	}
}
/*
************************************************************************************************************************
*                                                   msgdis_task
* Description: Null;
* Arguments  : Null;
* Note(s)    : 1) Null；
************************************************************************************************************************
*/
//显示消息队列中的消息
void msgdis_task(void *p_arg)
{
	u8 *p;
	OS_MSG_SIZE size;
	OS_ERR err; 
	while(1)
	{
		//请求消息
		p=OSTaskQPend((OS_TICK		)0,
                      (OS_OPT		)OS_OPT_PEND_BLOCKING,
                      (OS_MSG_SIZE*	)&size,
                      (CPU_TS*		)0,
                      (OS_ERR*      )&err );
		LCD_ShowString(40,270,100,16,16,p);
		myfree(SRAMIN,p);	//释放内存
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err); //延时1s
	}
}
/*
************************************************************************************************************************
*                                                   uart1_analyze_task
* Description: 接收USART1_IRQHandler 推送的数据加以解析处理、分发给不同的消息队列或者推送给其他函数；
* Arguments  : NULL
* Note(s)    : 1) 推送速度数据给SPEED_TRANSFORM_TaskTCB、8个字节；
************************************************************************************************************************
*/	
//串口1接收到的数据解析
void uart1_analyze_task(void *p_arg)
{
	u8 *p;
	OS_MSG_SIZE size;
	OS_ERR err; 
	while(1)
	{
		//申请内存
		p = mymalloc(SRAMIN,20);
		//请求消息
		p=OSTaskQPend((OS_TICK		)2000,		//n*5ms的超时等待时长
                      (OS_OPT		)OS_OPT_PEND_BLOCKING,	//阻塞任务
                      (OS_MSG_SIZE*	)&size,		//获取的字节长度
                      (CPU_TS*		)0,	//时间戳、NULL--表示不记录
                      (OS_ERR*      )&err );		//保存错误码
		//若是在timeout时间内未收到速度、则可进行机器人短距离后退指令
		if(err == OS_ERR_NONE)
		{
			if(!Format_Check(p,size))	//若检验通过---Format_Check()返回值为0;
			{
				switch(*(p+4))
				{
					//执行指令---命令字0x01
					case 0x01:		
					{
						//电机启动指令---指令地址0x01
						if(*(p+5) == 0x01)		
						{
							//MOTOR_START_CMD
							CAN1_Send_Msg(CAN_ID1,MOTOR_START_CMD,8);//发送8个字节
							CAN1_Send_Msg(CAN_ID2,MOTOR_START_CMD,8);//发送8个字节
						}
						//电机停止指令---指令地址0x02					
						else if(*(p+5) == 0x02)
						{
							//MOTOR_STOP_CMD
							Stop_Moving();
						}
						//上位机请求控制权	
						else if(*(p+5) == 0x03)					//充电脱离指令
						{
							Robot_Moving(100.00,100.00,Exit_Charge_Seconds);		//机器人向前弹出一段距离
						}
						else if(*(p+5) == 0x04)					//允许控制权限切换
						{
//							Charge_Status |= 0x02;		
						}
						else if(*(p+5) == 0xF1)					//驱动器清障指令
						{
							//MOTOR_CLEAR_FAULT
							CAN1_Send_Msg(CAN_ID1,MOTOR_CLEAR_FAULT,8);//发送8个字节
							CAN1_Send_Msg(CAN_ID2,MOTOR_CLEAR_FAULT,8);//发送8个字节
						}
					}break;
					//读取指令---命令字0x02
					case 0x02:		
					{
						
					}break;
					//写入指令---命令字0x03
					case 0x03:		
					{
						switch(*(p+5))
						{							
							case 0x01:	//两轮速度写入---指令地址0x01
							{
								if((Charge_Status&0x01) == 0)		//确认上位机是否有控制权限&&Moving状态
								{
									OSTaskQPost((OS_TCB*)&SPEED_TRANSFORM_TaskTCB,	//向任务SPEED_TRANSFORM发送消息
												(void*		)(p+6),
												(OS_MSG_SIZE)8,
												(OS_OPT		)OS_OPT_POST_FIFO,
												(OS_ERR*	)&err);
//									GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP引脚拉高， 等同BEEP=1;
								}
							}break;
							case 0x02:
								break;
							case 0x05:		//修改上传频率
								if((*(p+6)<=100) && (*(p+6)>1))		Odom_Rate_Seconds = 1000 / *(p+6);
								break;	
							case 0x06:		//修改脱离时长
								if((*(p+6)<=50) && (*(p+6)>9))		Exit_Charge_Seconds = *(p+6);
								break;	
							case 0x08:		//开门指令---指令地址0x08
							{
							}break;
						}						
					}break;
					//未知指令
					default:
						break;
				}
			}
//			else
//				GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP引脚拉高， 等同BEEP=1;
		}
		//释放内存	
		myfree(SRAMIN,p);
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_PERIODIC,&err); //延时10ms
	}
}
/*
************************************************************************************************************************
*                                                   speed_transform_task
*
* Description: 接收uart1_analyze_task解析后提取的速度数据进行转换并调用下发子程序；
*
* Arguments  : SPEED_TRANSFORM_TIMEOUT —— n*5ms的等待时长
*
* Note(s)    : 1) 速度下发的最下间隔取决于speed_transform_task的延时时间，过短会覆盖之前的数据；
*              2) 速度下发的最大间隔取决于参数--SPEED_TRANSFORM_TIMEOUT、过长会下发停止命令；
************************************************************************************************************************
*/	
//速度转换下发任务
void speed_transform_task(void *p_arg)
{
	u8 *p;
	OS_MSG_SIZE size;
	OS_ERR err;
	//申请内存,使用频率较高，不释放避免反复申请释放降低效率
	p = mymalloc(SRAMIN,10);
	while(1)
	{
		//请求消息
		p=OSTaskQPend((OS_TICK		)SPEED_TRANSFORM_TIMEOUT,//n*5ms的超时等待时长
                      (OS_OPT		)OS_OPT_PEND_BLOCKING,	//阻塞任务
                      (OS_MSG_SIZE*	)&size,		//获取的字节长度
                      (CPU_TS*		)0,	//时间戳、NULL--表示不记录
                      (OS_ERR*      )&err );		//保存错误码
		//正常接受到速度则进行转换
		if(err == OS_ERR_NONE)
		{
			u8 t;
			for(t=0;t<4;t++)	//将串口接受到的速度存储在结构体中;
			{
				rightdata.data[t]=*(p+t);
				leftdata.data[t]=*(p+t+4);
			}			
			car_control(rightdata.d,leftdata.d);	 //将接收到的左右轮速度赋给小车
		}
		//若是在timeout时间内未收到速度、则下发速度归零指令
		else if(err == OS_ERR_TIMEOUT)		
		{
			//MOTOR_RESET_SPEED
			CAN1_Send_Msg(CAN_ID1,MOTOR_RESET_SPEED_CMD,8);//发送8个字节
			CAN1_Send_Msg(CAN_ID2,MOTOR_RESET_SPEED_CMD,8);//发送8个字节			
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_PERIODIC,&err); //延时50ms——速度下发的小于间隔应当小于50ms
	}
}
/*
************************************************************************************************************************
*                                                   motor_init_task
* Description: 左右轮方向和速度初始化;
* Arguments  : Null;
* Note(s)    : Null;
************************************************************************************************************************
*/ 
void motor_init_task(void *p_arg)
{
	unsigned char i;
	//u8 canbuf[8];
	OS_ERR err;
	delay_ms(500);				//上电等待驱动器上电---part2
	
	//MOTOR_STOP_CMD
	i = CAN1_Send_Msg(CAN_ID1,MOTOR_STOP_CMD,8);//发送8个字节
	i = CAN1_Send_Msg(CAN_ID2,MOTOR_STOP_CMD,8);//发送8个字节
	delay_ms(100);
	//MOTOR_SPEED_MODE
	i = CAN1_Send_Msg(CAN_ID1,MOTOR_SPEED_MODE_CMD,8);//发送8个字节
	i = CAN1_Send_Msg(CAN_ID2,MOTOR_SPEED_MODE_CMD,8);//发送8个字节
	delay_ms(100);
	//MOTOR_RESET_SPEED
	i = CAN1_Send_Msg(CAN_ID1,MOTOR_RESET_SPEED_CMD,8);//发送8个字节
	i = CAN1_Send_Msg(CAN_ID2,MOTOR_RESET_SPEED_CMD,8);//发送8个字节
	delay_ms(100);
	//MOTOR_START_CMD
	i = CAN1_Send_Msg(CAN_ID1,MOTOR_START_CMD,8);//发送8个字节
	i = CAN1_Send_Msg(CAN_ID2,MOTOR_START_CMD,8);//发送8个字节
	delay_ms(100);
	
	i = i;
	GPIO_ResetBits(GPIOF,GPIO_Pin_8); //BEEP引脚拉低， 等同BEEP=0;	

	OSTaskDel((OS_TCB*)0,&err);	//删除motor_init_task任务自身
}
/************************************************************************** 
* Function Name  : battery_send_task  
* Description    : battery_send. 
***************************************************************************/
void battery_send_task(void *p_arg)
{
	OS_ERR err;
	while(1)
	{
		static u16 battery_adc;
		battery_adc=Get_Adc_Average(ADC_Channel_5,20);//获取通道5的转换值，20次取平均
		battery_v.d = (float)((battery_adc*3.3f)/4096);			//获取计算后的带小数的实际电压值，比如3.1111
		battery_v.d = battery_v.d*11.0f;			//放大11倍、得到电池电压值

		while(uart1_format_build_and_send(battery_v.data,0x01,0x02,0x04));		//发送数据到串口---id = 0x01,commond = 0x02; return 0---表示发送成功

		/**********退出临界区*********/		
		OSTimeDlyHMSM(0,0,10,0,OS_OPT_TIME_PERIODIC,&err);   //延时10s--0.1Hz
	}
}
/************************************************************************** 
* Function Name  : remote_control_task  
* Description    : Remote Control the Robot. 
***************************************************************************/
void remote_control_task(void *p_arg)
{
	OS_ERR err; 
	u8 key;
	u8 k=1;
	
	while(1)
	{
		key=Remote_Scan();	
		if(key)
		{	 		  
			switch(key)
			{
					case 98:		//str="UP"
						car_control(100*k,100*k);	 //将接收到的左右轮速度赋给小车	
						break;				
					case 168:		//str="DOWN"
						car_control(-100*k,-100*k);	 //将接收到的左右轮速度赋给小车	
					break;		  
					case 194:		//str="RIGHT"
						car_control(-100*k,100*k);	 //将接收到的左右轮速度赋给小车	
						break;					
					case 34:		//str="LEFT"
						car_control(100*k,-100*k);	 //将接收到的左右轮速度赋给小车	
						break;
					case 2:		//str="PLAY"
						//清空速度、锁定机器人
						CAN1_Send_Msg(CAN_ID1,MOTOR_RESET_SPEED_CMD,8);//发送8个字节
						CAN1_Send_Msg(CAN_ID2,MOTOR_RESET_SPEED_CMD,8);//发送8个字节
						break;
//				case 0:str="ERROR";break;			   
//				case 162:str="POWER";break;	    
//				case 98:str="UP";break;	    
//				case 2:str="PLAY";break;		 
//				case 226:str="ALIENTEK";break;		  
//				case 194:str="RIGHT";break;	   
//				case 34:str="LEFT";break;		  
//				case 224:str="VOL-";break;		  
//				case 168:str="DOWN";break;		   
//				case 144:str="VOL+";break;		    
					case 104:		//str="1";
						k = 1;
						break;		  
					case 152:		//str="2";
						k = 2;
						break;  
					case 176:		//str="3";
						k = 3;
						break;   
					case 48:		//str="4";
						k = 4;
						break;    
					case 24:		//str="5";
						k = 5;
						break;	    
					case 122:		//str="6";
						k = 6;
						break;	  
					case 16:		//str="7";
						k = 7;
						break;		   					
					case 56:		//str="8";
						k = 8;
						break; 
					case 90:		//str="9";
						k = 9;
						break;
//				case 66:str="0";break;
//				case 82:str="DELETE";break;	
			}
		}else delay_ms(10);
		
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_PERIODIC,&err); //延时50ms
	}
}
/************************************************************************** 
* Function Name  : PVD_Config  
* Description    : This function PVD Configure PVD. 
* Input          : None 
* Output         : None 
* Return         : None  
***************************************************************************/ 
void PVD_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
	NVIC_Init(&NVIC_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line16;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;       

	EXTI_Init(&EXTI_InitStructure);
	
	PWR_PVDLevelConfig(PWR_PVDLevel_7);
	PWR_PVDCmd(ENABLE);
}
/************************************************************************** 
* Function Name  : PVD_IRQHandler  
* Description    : This function handles PVD interrupt request. 
* Input          : None
* Output         : None
* Return         : None
***************************************************************************/ 
void PVD_IRQHandler(void)
{   
	if (PWR_GetFlagStatus(PWR_FLAG_PVDO))
		
	while(1)
	{
		USART_Cmd(USART2, DISABLE);  //禁用串口2 
		USART_Cmd(USART3, DISABLE);  //禁用串口3
		GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP引脚拉高， 等同BEEP=1;
		//PWR_EnterSTANDBYMode();			//待机模式,1.8V 内核电源关闭-此功耗最低，典型大概在2uA左右
//		PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);				//停机模式,所有时钟都停止-此功耗较低，典型大概在20uA左右
	}
}

//#define PWR_Regulator_ON               //电源不进低功耗 唤醒基本没延迟
//#define PWR_Regulator_LowPower         //电源进去低功耗 不过唤醒启动有一点延迟    

//#define PWR_STOPEntry_WFI              //中断唤醒
//#define PWR_STOPEntry_WFE              //事件唤醒

/*
************************************************************************************************************************
*                                                   Format_Check
*
* Description: 接收n个字节,并进行数据校验;
*
* Arguments  : u8 *p_arg---数据指针,u8 length--数据长度
*
* Note(s)    : 1) 校验通过返回-0;
*				       2) 数据头错误返回-1;
*				       3) 数据尾错误返回-2;
*				       4) 长度错误返回-3;
*				       4) 累计和错误返回-4;
************************************************************************************************************************
  
u8 Format_Check(u8 *p_arg,u8 length)
{
	u16 check_sum= 0;
	for(int i=0;i<(length-4);i++)
	{
		check_sum += *(p_arg+i);
	}
	if((*p_arg != 0xA6) || (*(p_arg+1) != 0x59))
		return 1;		//数据头错误返回-1;
	else if((*(p_arg+length-2) != 0x0D) || (*(p_arg+length-1) != 0x0A))
		return 2;		//数据尾错误返回-2;
	else if(*(p_arg+2) != length)
		return 3;		//长度错误返回-3;
	else if((*(p_arg+length-3) != check_sum) && (*(p_arg+length-4) != (check_sum >> 8)))
		return 4;		//累加和错误返回-4;
	else if(*(p_arg+3) != 0x01)
		return 5;		//指令不是发送给底盘控制器的返回-5;
	else
		return 0;		//校验通过返回-0;
}
*/

/*
************************************************************************************************************************
*                                                   Format_Check
*
* Description: 接收n个字节,并进行数据校验;
*
* Arguments  : u8 *p_arg---数据指针,u8 length--数据长度
*
* Note(s)    : 1) 校验通过返回-0;
*				       2) 数据头错误返回-1;
*				       3) 数据尾错误返回-2;
*				       4) 长度错误返回-3;
*				       4) 累计和错误返回-4;
************************************************************************************************************************
*/
u8 Format_Check(u8 *p_arg,u8 length)
{
	u16 check_sum= 0;
	for(int i=0;i<(length-7);i++)
	{
		check_sum += *(p_arg+i);
	}
	if((*p_arg != 0xA6) || (*(p_arg+1) != 0x59))
		return 1;		//数据头错误返回-1;
	else if((*(p_arg+length-5) != 0x0D) || (*(p_arg+length-4) != 0x0A) || (*(p_arg+length-3) != 0x59) || (*(p_arg+length-2) != 0x48) || (*(p_arg+length-1) != 0x54))
		return 2;		//数据尾错误返回-2;
	else if(*(p_arg+2) != length)
		return 3;		//长度错误返回-3;
	else if((*(p_arg+length-6) != check_sum) && (*(p_arg+length-7) != (check_sum >> 8)))
		return 4;		//累加和错误返回-4;
	else if(*(p_arg+3) != 0x01)
		return 5;		//指令不是发送给底盘控制器的返回-5;
	else
		return 0;		//校验通过返回-0;
}


/*
************************************************************************************************************************
*                                                   Stop_Moving
*
* Description: 驱动器速度归零;
*
* Arguments  : NULL
*
* Note(s)    : NULL 
*
************************************************************************************************************************
*/   
void Stop_Moving(void)
{
	//MOTOR_RESET_SPEED
	CAN1_Send_Msg(CAN_ID1,MOTOR_RESET_SPEED_CMD,8);//发送8个字节
	CAN1_Send_Msg(CAN_ID2,MOTOR_RESET_SPEED_CMD,8);//发送8个字节;
}	
/*
************************************************************************************************************************
*                                                   Robot_Moving
*
* Description: 接收n个字节,并进行数据校验;
*
* Arguments  : L_speed---左轮速度，R_speed---右轮速度，time---运动时间*100ms
*
* Note(s)    :
*				      
************************************************************************************************************************
*/   
void Robot_Moving(float L_speed,float R_speed,u8 time)
{
	u8 p[8],t;
	OS_ERR err; 
	
	rightdata.d = R_speed;
	leftdata.d = L_speed;
	
	for(t=0;t<4;t++)	//将串口接受到的速度存储在结构体中;
	{
		*(p+t) = rightdata.data[t];
		*(p+t+4) = leftdata.data[t];
	}
	
	while(time--)
	{
		OSTaskQPost((OS_TCB*)&SPEED_TRANSFORM_TaskTCB,	//向任务SPEED_TRANSFORM发送消息
												(void*		)p,
												(OS_MSG_SIZE)8,
												(OS_OPT		)OS_OPT_POST_FIFO,
												(OS_ERR*	)&err);
//		car_control(L_speed,R_speed);	 //将接收到的左右轮速度赋给小车	
		delay_ms(100);
	}
}

//CRC 多项式: CRC-16 x16+x15+x2+1 8005->A001 按位颠倒
/*******************************************
* 函数名称: CRC16(uchar *Data , ushort Len )
* 函数功能: CRC 校验
* 入口参数: *Data：需要校验的数据
Len： 需要校验数据的长度
* 出口参数: 返回校验值
********************************************/
u16 CRC16(unsigned char *Data , unsigned short Len )
{
	unsigned int DataCRC = 0xffff;
	unsigned char i;
	while(Len != 0)
	{
		DataCRC = DataCRC^(*Data);
		for(i=0;i<8;i++)
		{
			if((DataCRC & 0x0001) == 0)
				DataCRC = DataCRC >> 1;
			else
			{
				DataCRC = DataCRC >> 1;
				DataCRC ^= 0xa001;
			}
		}
		Len -= 1;
		Data++;
	}
	return DataCRC;
}


