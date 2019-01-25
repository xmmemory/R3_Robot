//********************************************************************************
//主程序逻辑 代码	 
//修改日期:2018-10-22
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 光线量子(北京)智能机器人科技有限公司 
//All rights reserved
//********************************************************************************

#include "r3driver.h"
#include "members.h"
#include "u_task.h"
/*********************************************************************************************************
*                                               DECLARED
*********************************************************************************************************/
u8 Format_Check(u8 *p_arg,u8 length);													//数据格式校验
void tmr1_callback(void *p_tmr,void *p_arg); 								//定时器1回调函数
void Odom_Calculator(void *p_tmr, void *p_arg); 							//定时器2---里程计odom计算函数---20ms--50hz
void odom_send_task(void);																		//里程计算
void Robot_Moving(float L_speed,float R_speed,u8 time);			//机器人移动
/*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************/
extern int PULSE_RIGHT,PULSE_LEFT;
extern u8 reset_pro;
int PULSE_RIGHT_HIS,PULSE_LEFT_HIS;

extern u8 SendBuff2[SEND_BUF2_SIZE];	//发送数据缓冲区

//要发布的里程计数据，分别为：X，Y方向移动的距离，当前角度，线速度，角速度
extern float position_x,position_y,oriention,velocity_linear,velocity_angular;
//计算得到的里程计数值
/************************************************************************************************************************/

/************************************************************************************************************************
*                                                   Main
* Description: Null
* Arguments  : Null
* Note(s)    : 1) Null
************************************************************************************************************************/
//主函数
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	delay_init(168);  				//时钟初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//中断分组配置
	IWDG_Init(IWDG_Prescaler_64,2000);		 //看门狗启动，分频数为64,重载值为2000,溢出时间为4s	
	Remote_Init();						//红外接收初始化
	LED_Init();								//LED初始化
	KEY_Init();								//按键_充电状态判断引脚初始化
	BEEP_Init();							//初始化蜂鸣器
	Adc_Init();								//初始化ADC
	uart_init();   						//串口初始化	
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN初始化普通模式,波特率500Kbps	
	ENC_Init_Left();					//设置电机D TIM4编码器模式PB6 PB7 左电机 
	ENC_Init_Right();					//设置电机A TIM3编码器模式PC6 PC7 右电机	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//使能PWR时钟
	PWR_BackupAccessCmd(ENABLE);	//使能后备寄存器访问
	/*系统正常运行的保护措施*/
	PVD_Config();							//低电压中断初始化

	IWDG_Feed();																							//喂狗
	delay_ms(3000);																						//等待上电稳定
	IWDG_Feed();																							//喂狗				

	Start_Succeed();					//程序自检、正常启动蜂鸣器关闭			
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送
	MYDMA_Enable(DMA2_Stream7,1);     //开始一次DMA传输
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送
	MYDMA_Enable(DMA1_Stream4,39);     //开始一次DMA传输	
	
	Robot_Status.data[1] = FINDING_DOCK_STATUS;		//机器人开机默认为寻电模式	
	Charge_Exit_Seconds.d = 20;										//20s的充电时长
	Moving_Seconds = 60;													//充电脱离时长---n*50ms
//	//里程计算定时器初始化
//	TIM5_Int_Init(100-1,8400-1);	//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms(100次为10ms)(50次为5ms)(5_0.5ms_2Khz)	
	
	my_mem_init(SRAMIN);//初始化内部RAM
	OSInit(&err);		    //初始化UCOSIII
	OS_CRITICAL_ENTER();	//进入临界区
	
	IWDG_Feed();																							//喂狗
	//创建开始任务
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//任务控制块
				 (CPU_CHAR	* )"start task", 				//任务名字
                 (OS_TASK_PTR )start_task, 			//任务函数
                 (void		* )0,							//传递给任务函数的参数
                 (OS_PRIO	  )START_TASK_PRIO,     //任务优先级
                 (CPU_STK   * )&START_TASK_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)START_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,						//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,							//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,							//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);				//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区
	OSStart(&err);      //开启UCOSIII
}
/************************************************************************************************************************
*                                                   main_task
* Description: Null
* Arguments  : Null
* Note(s)    : 1) Null
************************************************************************************************************************/
void main_task(void *p_arg)
{
	u8 *p;
	OS_MSG_SIZE size;
	OS_ERR err;  
	while(1)
	{
		//申请内存
		p = mymalloc(SRAMIN,20);
		//请求消息
		p=OSTaskQPend((OS_TICK		)0,			//n*5ms的超时等待时长
                      (OS_OPT		)OS_OPT_PEND_BLOCKING,
                      (OS_MSG_SIZE*	)&size,
                      (CPU_TS*		)0,
                      (OS_ERR*      )&err );
		
		LCD_ShowString(40,270,100,16,16,p);
		myfree(SRAMIN,p);	//释放内存
		
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err); //延时1s
	}
}
/************************************************************************************************************************
*                                                   start_task
* Description: Null；
* Arguments  : Null
* Note(s)    : 1) Null；
************************************************************************************************************************/
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
	OSTmrCreate((OS_TMR		*)&Sencond_1_tmr1,		//定时器1
                (CPU_CHAR	*)"Sencond_1_tmr1",		//定时器名字
                (OS_TICK	 )0,			//0ms
                (OS_TICK	 )100,          //100*10=1s
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
//	//创建MAIN_TASK任务
//	OSTaskCreate((OS_TCB 	* )&Main_TaskTCB,
//								 (CPU_CHAR	* )"main task",
//								 (OS_TASK_PTR )main_task,
//								 (void		* )0,
//								 (OS_PRIO	  )MAIN_TASK_PRIO,
//								 (CPU_STK   * )&MAIN_TASK_STK[0],
//								 (CPU_STK_SIZE)MAIN_STK_SIZE/10,
//								 (CPU_STK_SIZE)MAIN_STK_SIZE,
//								 (OS_MSG_QTY  )TASK_Q_NUM,		//消息队列长度为3
//								 (OS_TICK	  )0,
//								 (void   	* )0,
//								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//								 (OS_ERR 	* )&err);
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
								 (OS_MSG_QTY  )UART1_ANALYZE_Q_NUM,		//任务uart1_analyze_task需要使用内建消息队列，消息队列长度为2					
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);
	//创建auto_charge&Send任务
	OSTaskCreate((OS_TCB 	* )&AUTO_CHARGE_TaskTCB,	
								 (CPU_CHAR	* )"Auto charge task", 		
								 (OS_TASK_PTR )auto_charge_task, 			
								 (void		* )0,
								 (OS_PRIO	  )AUTO_CHARGE_TASK_PRIO,     
								 (CPU_STK   * )&AUTO_CHARGE_TASK_STK[0],	
								 (CPU_STK_SIZE)AUTO_CHARGE_STK_SIZE/10,	
								 (CPU_STK_SIZE)AUTO_CHARGE_STK_SIZE,		
								 (OS_MSG_QTY  )0,		//不需要使用内建消息队列			
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);	
	//创建REMOTE_CONTROL任务
	OSTaskCreate((OS_TCB 	* )&REMOTE_CONTROL_TaskTCB,	
								 (CPU_CHAR	* )"remote control task", 		
								 (OS_TASK_PTR )remote_control_task, 			
								 (void		* )0,
								 (OS_PRIO	  )REMOTE_CONTROL_TASK_PRIO,     
								 (CPU_STK   * )&REMOTE_CONTROL_TASK_STK[0],	
								 (CPU_STK_SIZE)REMOTE_CONTROL_STK_SIZE/10,	
								 (CPU_STK_SIZE)REMOTE_CONTROL_STK_SIZE,		
								 (OS_MSG_QTY  )0,		//不需要使用内建消息队列			
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);									 
	OSTmrStart(&Sencond_1_tmr1,&err);			//启动定时器1						 								 
	OSTmrStart(&Odom_Calculator_tmr2,&err);			//启动定时器2
	//退出临界区						 
	OS_CRITICAL_EXIT();
	OSTaskDel((OS_TCB*)0,&err);	//删除start_task任务自身
}
/************************************************************************************************************************
*                                                   tmr1_callback
* Description: Null；
* Arguments  : Null
* Note(s)    : 1) Null；
************************************************************************************************************************/
//定时器1的回调函数——1s
void tmr1_callback(void *p_tmr,void *p_arg)
{
	if(Charge_Exit_Seconds.d && Robot_Status.data[1]==CHARGING_STATUS)
	{
		if(Charge_Exit_Seconds.d)
			Charge_Exit_Seconds.d--;			//充电时间-1s
		if(Reset_Robot_Count)
			Reset_Robot_Count--;					//重启倒计时
		else
		{
			__disable_fault_irq();				//关中断
			NVIC_SystemReset();						//重启
		}			
		
	}
	temp_fr_control = 1;			//控制tcp发送频率
	//LED闪烁
	LED1 = !LED1;
}

/************************************************************************************************************************
*                                                   Odom_Calculator
* Description: 定时器1的回调函数---里程计计算发送消息给UART1_SEND_TaskTCB---20ms--50hz;
* Arguments  : Null
* Note(s)    : 1) Null；
************************************************************************************************************************/	
//定时器2---里程计计算---回调函数
void Odom_Calculator(void *p_tmr, void *p_arg)
{
	PULSE_LEFT = ENCODER_LEFT_TIMER->CNT - 32768;	//左电机码盘采集定时器 TIM3---TIM3编码器模式GPIOB6 GPIOB7 左电机
	ENCODER_LEFT_TIMER->CNT = 32768;
	PULSE_RIGHT = ENCODER_RIGHT_TIMER->CNT - 32768;	//右电机码盘采集定时器 TIM4---TIM4编码器模式GPIOC6 GPIOC7 右电机
	ENCODER_RIGHT_TIMER->CNT = 32768;	
//	PULSE_LEFT_HIS += PULSE_LEFT;
//	PULSE_RIGHT_HIS += PULSE_RIGHT;
	//计算里程计
	odometry_cal(PULSE_LEFT,PULSE_RIGHT);
}

/************************************************************************************************************************
*                                                   uart1_analyze_task
* Description: 接收USART1_IRQHandler 推送的数据加以解析处理、分发给不同的消息队列或者推送给其他函数；
* Arguments  : NULL
* Note(s)    : 1) 推送速度数据给SPEED_TRANSFORM_TaskTCB、8个字节；
************************************************************************************************************************/	
//串口1接收到的数据解析
void uart1_analyze_task(void *p_arg)
{
	u8 *p;
	OS_MSG_SIZE size;
	OS_ERR err; 
	while(1)
	{
		//申请内存
		p = mymalloc(SRAMIN,200);
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
				switch(*(p+4))		//指令类型判断-(执行-读取-写入)
				{
//					case 0x01:		//执行指令---命令字0x01
//					{
//						switch(*(p+5))		//命令字判断
//						{
//							case 0x01:		//电机启动指令---指令地址0x01
//							{	
//								//MOTOR_CLEAR_FAULT
//								CAN1_Send_Msg(CAN_ID1,MOTOR_CLEAR_FAULT,8);//发送8个字节
//								CAN1_Send_Msg(CAN_ID2,MOTOR_CLEAR_FAULT,8);//发送8个字节								
//							}break;
//							default:			//未知指令
//								break;
//						}
//					}break;
					//读取指令---命令字0x02
					case 0x02:
					{
						switch(*(p+5))
						{
							case REG_STATUS:	//寄存器-0x01-运行状态
							{
								while(uart1_format_build_and_send(Robot_Status.data,DEVICE_ID_D9,READ_COMMOND,REG_STATUS,_2byte));		//发送Robot_Status，2个字节							
							}break;
							case REG_ODOMETRY:	//寄存器-0x10-里程计
							{
								odom_send_task();
								uart1_format_build_and_send(odometry_data,DEVICE_ID_D9,READ_COMMOND,REG_ODOMETRY,31);	
							}break;	
							case REG_BATTERY_VOLTAGE:	//寄存器-0x11-电量计
							{
								battery_v.d = battery_check();
								while(uart1_format_build_and_send(battery_v.data,DEVICE_ID_D9,READ_COMMOND,REG_BATTERY_VOLTAGE,_4byte));
							}break;	
							case REG_CHARGING_TIME:	//寄存器-0x20-充电时长
							{
								while(uart1_format_build_and_send(Charge_Exit_Seconds.data,DEVICE_ID_D9,READ_COMMOND,REG_CHARGING_TIME,_2byte));
							}break;
							case REG_ERROR:	//寄存器-0xF1-错误码
							{
								while(uart1_format_build_and_send(Robot_Error.data,DEVICE_ID_D9,READ_COMMOND,REG_ERROR,_2byte));		//发送Robot_Error，2个字节	
							}break;	
							default:			//未知指令
								break;
						}
					}break;
					//写入指令---命令字0x03
					case 0x03:
					{
						switch(*(p+5))
						{
							case REG_CHARGING_TIME:		//寄存器-0x20-充电时长
							{
								Charge_Exit_Seconds.data[1] = *(p+7);
								Charge_Exit_Seconds.data[0] = *(p+6);
								if((Charge_Exit_Seconds.d > 5) && (Charge_Exit_Seconds.d < 65535))
								Robot_Status.data[1] = FINDING_DOCK_STATUS;				//运行状态-切换为-找电状态
								Reset_Robot_Count = Charge_Exit_Seconds.d - 20;		//机器人重启倒计时
								//回显
								while(uart1_format_build_and_send(Charge_Exit_Seconds.data,DEVICE_ID_D9,WRITE_COMMOND,REG_CHARGING_TIME,_2byte));
							}break;								
							case REG_MOVING_SPEED:	//寄存器-0x21-运动速度
							{
								if(Robot_Status.data[1] == NAVI_STATUS)		//确认运行状态==导航状态
								{
									OSTaskQPost((OS_TCB*)&SPEED_TRANSFORM_TaskTCB,	//向任务SPEED_TRANSFORM发送消息
												(void*		)(p+6),
												(OS_MSG_SIZE)8,
												(OS_OPT		)OS_OPT_POST_FIFO,
												(OS_ERR*	)&err);
									//回显
									while(uart1_format_build_and_send((p+6),DEVICE_ID_D9,WRITE_COMMOND,REG_MOVING_SPEED,_8byte));
								}
							}break;
							case 0x60:
								if(temp_fr_control)
								{
									memcpy(SendBuff2,(p+6),(size-11));
									if(size < 200)
									{
										uart3_send(size);
										temp_fr_control = 0;
									}
								}
								break;		
							case 0x06:
								break;	
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
/************************************************************************************************************************
*                                                   speed_transform_task
* Description: 接收uart1_analyze_task解析后提取的速度数据进行转换并调用下发子程序；
* Arguments  : SPEED_TRANSFORM_TIMEOUT —— n*5ms的等待时长
* Note(s)    : 1) 速度下发的最下间隔取决于speed_transform_task的延时时间，过短会覆盖之前的数据；
*              2) 速度下发的最大间隔取决于参数--SPEED_TRANSFORM_TIMEOUT、过长会下发停止命令；
************************************************************************************************************************/	
//速度转换下发任务
void speed_transform_task(void *p_arg)
{
	u8 *p;
	OS_MSG_SIZE size;
	OS_ERR err;
	//申请内存,使用频率较高，不释放避免反复申请释放降低效率
	while(1)
	{
		p = mymalloc(SRAMIN,10);
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
			if(rightdata.d < 1000 && leftdata.d < 1000)		//对速度加以限制
				car_control(rightdata.d,leftdata.d);	 			//将接收到的左右轮速度赋给小车
		}
		//若是在timeout时间内未收到速度、则下发速度归零指令
		else if(err == OS_ERR_TIMEOUT)
		{
			car_control(0,0);						//停止
		}
		IWDG_Feed();																							//喂狗
		myfree(SRAMIN,p);	//释放内存
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_PERIODIC,&err); 				//延时50ms——速度下发的小于间隔应当小于50ms
	}
}

/************************************************************************************************************************
*                                                   Format_Check
* Description: 接收n个字节,并进行数据校验;
* Arguments  : u8 *p_arg---数据指针,u8 length--数据长度
* Note(s)    : 1) 校验通过返回-0;
*				       2) 数据头错误返回-1;
*				       3) 数据尾错误返回-2;
*				       4) 长度错误返回-3;
*				       4) 累计和错误返回-4;
************************************************************************************************************************/
u8 Format_Check(u8 *p_arg,u8 length)
{
	u16 check_sum= 0;
	for(int i=0;i<(length-5);i++)
	{
		check_sum += *(p_arg+i);
	}
	if((*p_arg != 0x59) || (*(p_arg+1) != 0xA6))
		return 1;		//数据头错误返回-1;
	else if((*(p_arg+length-3) != 0x59) || (*(p_arg+length-2) != 0x48) || (*(p_arg+length-1) != 0x54))
		return 2;		//数据尾错误返回-2;
	else if(*(p_arg+2) != length)
		return 3;		//长度错误返回-3;
	else if((*(p_arg+length-4) != check_sum) && (*(p_arg+length-5) != (check_sum >> 8)))
		return 4;		//累加和错误返回-4;
	else if(*(p_arg+3) != 0x01)
		return 5;		//指令不是发送给底盘控制器的返回-5;
	else
		return 0;		//校验通过返回-0;
}
/************************************************************************************************************************
*                                                   Robot_Moving
* Description: 接收n个字节,并进行数据校验;
* Arguments  : L_speed---左轮速度，R_speed---右轮速度，time---运动时间*50ms
* Note(s)    :      
************************************************************************************************************************/   
void Robot_Moving(float L_speed,float R_speed,u8 time)
{
	u8 p[8],t;
	OS_ERR err;
	
	leftdata.d = L_speed;
	rightdata.d = R_speed;
	
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
		delay_ms(50);
	}
}
/************************************************************************************************************************
*                                                   remote_control_task
* Description: Remote Control the Robot.
* Arguments  : NULL
* Note(s)    :	      
************************************************************************************************************************/ 
void remote_control_task(void *p_arg)
{
	OS_ERR err; 
	u8 key;
	while(1)
	{
			
		if((Robot_Status.data[1] == FINDING_DOCK_STATUS) || (Robot_Status.data[1] == CHARGING_STATUS))		//运行状态-为-找电状态||运行状态-为-充电状态
		{			
			key=Remote_Scan();	
			if(key)
			{
				switch(key)
				{
						case 98:				//str="UP"
							Robot_Moving(100.00,100.00,1);		//机器人向前弹出一段距离
//						GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP引脚拉高， 等同BEEP=1;
							break;				
						case 168:				//str="DOWN"
							Robot_Moving(-100.00,-100.00,1);		//机器人向前弹出一段距离
						break;		  
						case 194:				//str="RIGHT"
							Robot_Moving(100.00,-100.00,1);		//机器人向前弹出一段距离
							break;					
						case 34:				//str="LEFT"
							Robot_Moving(-100.00,100.00,1);		//机器人向前弹出一段距离
							break;
						case 2:					//str="PLAY"
							//清空速度、锁定机器人
							Robot_Moving(0,0,1);		//停止
							break;
//						case 144:				//str="VOL+"
//							k++;
//							break;	
//						case 224:				//str="VOL-"
//							if(k)
//								k--;
//							break;
						case 104:		//str="1";
							IR_M = 1;						break;		  
						case 152:		//str="2";
							IR_M = 2;						break;  
						case 176:		//str="3";
							IR_M = 3;						break;   
				}
			}else
			OSTimeDlyHMSM(0,0,0,30,OS_OPT_TIME_PERIODIC,&err); //延时50ms
		}
		else
			OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err); //延时1s
		
	}
}
/************************************************************************************************************************
*                                                   auto_charge_task
* Description: 自动充电任务;
* Arguments  : NULL
* Note(s)    :
*				      
************************************************************************************************************************/ 
void auto_charge_task(void *p_arg)
{
	OS_ERR err;
	u8 k = 1,IR_No_Find_Count = 0;
	u16 Circle_Moving_Count = 0;
	
	while(1)
	{
		if((Robot_Status.data[1] == FINDING_DOCK_STATUS) || (Robot_Status.data[1] == CHARGING_STATUS))		//运行状态-为-找电状态||运行状态-为-充电状态
		{
			if(_If_Charging == TRUE)				//充上电了
			{
				Robot_Moving(0,0,1);					//停止
				delay_ms(1000);								//延时1s，等机器人稳定
				if(_If_Charging == TRUE)			//充上电了
				{					
					Robot_Status.data[1] = CHARGING_STATUS;		//运行状态-切换为-充电状态			
				}
			}
			else
			{
				if(Robot_Status.data[1] == FINDING_DOCK_STATUS && _If_Charging==FALSE)		//寻电状态&&未充上电
				{
					if(IR_M==NULL)			//无红外信号
					{
						IR_No_Find_Count++;												//红外无信号计数
						if(IR_No_Find_Count >= 30)								//连续无红外信号计数 > 30
						{
							IR_No_Find_Count = 0;										//清空红外无信号计数
							Circle_Moving_Count++;									//原地旋转计数计数
							Robot_Moving(-20.00*k,20.00*k,1);				//机器人逆时针旋转
						}
					}
					else
					{
						switch(IR_M)
						{
							case 0x01:				
								Robot_Moving(-40.00*k,-20.00*k,1);		//机器人向后退一段距离-右轮快-顺时针								
							break;
							case 0x02:
								Robot_Moving(-20.00*k,-20.00*k,1);		//机器人向后退一段距离
							break;
							case 0x03:
								Robot_Moving(-20.00*k,-40.00*k,1);		//机器人向后退一段距离-左轮快-逆时针
							break;
							default:															
								break;
						}
						IR_M = NULL;															//复位IR_M
						Circle_Moving_Count = NULL;								//清空原地旋转次数计数
						IR_No_Find_Count = NULL;									//清空原地旋转次数计数
					}
					/**********旋转寻找信号超过设定值*********/
					if(Circle_Moving_Count >= MAX_Circle_Moving_Count)	//原地旋转计数 > 旋转寻找信号最大连续尝试次数
					{
						Circle_Moving_Count = NULL;												//清空原地旋转次数计数
						Charge_Exit_Seconds.d = NULL;											//清空充电时间						
						Robot_Status.data[1] = ERROT_STATUS;							//运行状态-切换为-异常状态
						Robot_Error.data[1] = CHARGE_FALSE;								//异常报警-错误类型-CHARGE_FALSE											
					}
				}				
			}
			/**********充电计时结束，贴近充电桩、运行弹射程序*********/
			if(Charge_Exit_Seconds.d==NULL && Robot_Status.data[1]==CHARGING_STATUS)		//充电时间耗尽&&充电状态
			{
				Robot_Moving(100.00,100.00,Moving_Seconds);		//机器人向前弹出一段距离
				Robot_Status.data[1] = NAVI_STATUS;						//运行状态-切换为-导航状态
			}
			/**********延时*********/
			OSTimeDlyHMSM(0,0,0,60,OS_OPT_TIME_PERIODIC,&err);   //延时20ms
		}
		else		//未有充电任务，延时10s
		{
			OSTimeDlyHMSM(0,0,10,0,OS_OPT_TIME_PERIODIC,&err);   //延时10s
		}
		/**********退出临界区*********/		
	}
}

/*************************************************************************************************************************
*                                                   Odom_Send
* Description: 里程计数据整理存入odometry_data
* Arguments  : Null
* Note(s)    : 1) Null；
************************************************************************************************************************/
void odom_send_task(void)
{
		//发送给串口的里程计数据数组---预留出2倍的长度防止异常溢出（应该用不上）		//Odom_Get
	
		x_data.odoemtry_float=position_x;//单位mm		
		y_data.odoemtry_float=position_y;//单位mm
		theta_data.odoemtry_float=oriention;//单位rad
		vel_linear.odoemtry_float=velocity_linear;//单位mm/s
		vel_angular.odoemtry_float=velocity_angular;//单位rad/s
		//将所有里程计数据存到要发送的数组
		for(u8 j=0;j<4;j++)
		{
			odometry_data[j]=x_data.odometry_char[j];
			odometry_data[j+4]=y_data.odometry_char[j];
			odometry_data[j+8]=theta_data.odometry_char[j];
			odometry_data[j+12]=vel_linear.odometry_char[j];
			odometry_data[j+16]=vel_angular.odometry_char[j];			
		}
}
/************************************************************************************************************************
*                                                   msgdis_task
* Description: Null;
* Arguments  : Null;
* Note(s)    : 1) Null；
************************************************************************************************************************/
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
//CRC 多项式: CRC-16 x16+x15+x2+1 8005->A001 按位颠倒
/*******************************************
* 函数名称: CRC16(uchar *Data , ushort Len )
* 函数功能: CRC 校验
* 入口参数: *Data：需要校验的数据
Len： 需要校验数据的长度
* 出口参数: 返回校验值
********************************************/
u16 CRC16(unsigned char *Data , unsigned short Len)
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
/************************************************************************************************************************
*                                                   Other
************************************************************************************************************************/


