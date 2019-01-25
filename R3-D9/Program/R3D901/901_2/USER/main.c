//********************************************************************************
//�������߼� ����	 
//�޸�����:2018-10-22
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ��������(����)���ܻ����˿Ƽ����޹�˾ 
//All rights reserved
//********************************************************************************

#include "r3driver.h"
#include "members.h"
#include "u_task.h"
/*********************************************************************************************************
*                                               DECLARED
*********************************************************************************************************/
u8 Format_Check(u8 *p_arg,u8 length);													//���ݸ�ʽУ��
void tmr1_callback(void *p_tmr,void *p_arg); 								//��ʱ��1�ص�����
void Odom_Calculator(void *p_tmr, void *p_arg); 							//��ʱ��2---��̼�odom���㺯��---20ms--50hz
void odom_send_task(void);																		//��̼���
void Robot_Moving(float L_speed,float R_speed,u8 time);			//�������ƶ�
/*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************/
extern int PULSE_RIGHT,PULSE_LEFT;
extern u8 reset_pro;
int PULSE_RIGHT_HIS,PULSE_LEFT_HIS;

extern u8 SendBuff2[SEND_BUF2_SIZE];	//�������ݻ�����

//Ҫ��������̼����ݣ��ֱ�Ϊ��X��Y�����ƶ��ľ��룬��ǰ�Ƕȣ����ٶȣ����ٶ�
extern float position_x,position_y,oriention,velocity_linear,velocity_angular;
//����õ�����̼���ֵ
/************************************************************************************************************************/

/************************************************************************************************************************
*                                                   Main
* Description: Null
* Arguments  : Null
* Note(s)    : 1) Null
************************************************************************************************************************/
//������
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	delay_init(168);  				//ʱ�ӳ�ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�жϷ�������
	IWDG_Init(IWDG_Prescaler_64,2000);		 //���Ź���������Ƶ��Ϊ64,����ֵΪ2000,���ʱ��Ϊ4s	
	Remote_Init();						//������ճ�ʼ��
	LED_Init();								//LED��ʼ��
	KEY_Init();								//����_���״̬�ж����ų�ʼ��
	BEEP_Init();							//��ʼ��������
	Adc_Init();								//��ʼ��ADC
	uart_init();   						//���ڳ�ʼ��	
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN��ʼ����ͨģʽ,������500Kbps	
	ENC_Init_Left();					//���õ��D TIM4������ģʽPB6 PB7 ���� 
	ENC_Init_Right();					//���õ��A TIM3������ģʽPC6 PC7 �ҵ��	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);//ʹ��PWRʱ��
	PWR_BackupAccessCmd(ENABLE);	//ʹ�ܺ󱸼Ĵ�������
	/*ϵͳ�������еı�����ʩ*/
	PVD_Config();							//�͵�ѹ�жϳ�ʼ��

	IWDG_Feed();																							//ι��
	delay_ms(3000);																						//�ȴ��ϵ��ȶ�
	IWDG_Feed();																							//ι��				

	Start_Succeed();					//�����Լ졢���������������ر�			
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����
	MYDMA_Enable(DMA2_Stream7,1);     //��ʼһ��DMA����
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA����
	MYDMA_Enable(DMA1_Stream4,39);     //��ʼһ��DMA����	
	
	Robot_Status.data[1] = FINDING_DOCK_STATUS;		//�����˿���Ĭ��ΪѰ��ģʽ	
	Charge_Exit_Seconds.d = 20;										//20s�ĳ��ʱ��
	Moving_Seconds = 60;													//�������ʱ��---n*50ms
//	//��̼��㶨ʱ����ʼ��
//	TIM5_Int_Init(100-1,8400-1);	//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms(100��Ϊ10ms)(50��Ϊ5ms)(5_0.5ms_2Khz)	
	
	my_mem_init(SRAMIN);//��ʼ���ڲ�RAM
	OSInit(&err);		    //��ʼ��UCOSIII
	OS_CRITICAL_ENTER();	//�����ٽ���
	
	IWDG_Feed();																							//ι��
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 				//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,							//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,						//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,							//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,							//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���
	OSStart(&err);      //����UCOSIII
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
		//�����ڴ�
		p = mymalloc(SRAMIN,20);
		//������Ϣ
		p=OSTaskQPend((OS_TICK		)0,			//n*5ms�ĳ�ʱ�ȴ�ʱ��
                      (OS_OPT		)OS_OPT_PEND_BLOCKING,
                      (OS_MSG_SIZE*	)&size,
                      (CPU_TS*		)0,
                      (OS_ERR*      )&err );
		
		LCD_ShowString(40,270,100,16,16,p);
		myfree(SRAMIN,p);	//�ͷ��ڴ�
		
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err); //��ʱ1s
	}
}
/************************************************************************************************************************
*                                                   start_task
* Description: Null��
* Arguments  : Null
* Note(s)    : 1) Null��
************************************************************************************************************************/
//��ʼ������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;
	
	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif
	
#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,ʱ��Ƭ����Ϊ1��ϵͳʱ�ӽ��ģ���1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED,1,&err);  
#endif	
	
	//�����ٽ���	
	OS_CRITICAL_ENTER();
//	//����һ���ź���
//	OSSemCreate ((OS_SEM*	)&ODOM_SEND,
//                 (CPU_CHAR*	)"ODOM_SEND",
//                 (OS_SEM_CTR)1,		
//                 (OS_ERR*	)&err);
	//����һ���ź���
	OSSemCreate ((OS_SEM*	)&MY_SEM,
                 (CPU_CHAR*	)"MY_SEM",
                 (OS_SEM_CTR)1,		
                 (OS_ERR*	)&err);
//	//����һ���¼���־��
//	OSFlagCreate((OS_FLAG_GRP*)&Sensor_Send_Flags,		//ָ���¼���־��
//                 (CPU_CHAR*	  )"Sensor Send Flags",	//����
//                 (OS_FLAGS	  )0,	//�¼���־���ʼֵ
//                 (OS_ERR*  	  )&err);			//������	
	//������ʱ��1
	OSTmrCreate((OS_TMR		*)&Sencond_1_tmr1,		//��ʱ��1
                (CPU_CHAR	*)"Sencond_1_tmr1",		//��ʱ������
                (OS_TICK	 )0,			//0ms
                (OS_TICK	 )100,          //100*10=1s
                (OS_OPT		 )OS_OPT_TMR_PERIODIC, //����ģʽ
                (OS_TMR_CALLBACK_PTR)tmr1_callback,//��ʱ��1�ص�����
                (void	    *)0,			//����Ϊ0
                (OS_ERR	    *)&err);		//���صĴ�����
	//������ʱ��2
	OSTmrCreate((OS_TMR		*)&Odom_Calculator_tmr2,		
                (CPU_CHAR	*)"Odom Calculator tmr2",		
                (OS_TICK	 )0,
                (OS_TICK	 )1,   	//1*10=10ms				
                (OS_OPT		 )OS_OPT_TMR_PERIODIC, 	//����ģʽ
                (OS_TMR_CALLBACK_PTR)Odom_Calculator,	//��ʱ��2�ص�����---��̼�pwm����ص�����---10ms--100hz
                (void	    *)0,			
                (OS_ERR	    *)&err);
//	//����MAIN_TASK����
//	OSTaskCreate((OS_TCB 	* )&Main_TaskTCB,
//								 (CPU_CHAR	* )"main task",
//								 (OS_TASK_PTR )main_task,
//								 (void		* )0,
//								 (OS_PRIO	  )MAIN_TASK_PRIO,
//								 (CPU_STK   * )&MAIN_TASK_STK[0],
//								 (CPU_STK_SIZE)MAIN_STK_SIZE/10,
//								 (CPU_STK_SIZE)MAIN_STK_SIZE,
//								 (OS_MSG_QTY  )TASK_Q_NUM,		//��Ϣ���г���Ϊ3
//								 (OS_TICK	  )0,
//								 (void   	* )0,
//								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//								 (OS_ERR 	* )&err);
	//����SPEED_TRANSFORM����
	OSTaskCreate((OS_TCB 	* )&SPEED_TRANSFORM_TaskTCB,
				 (CPU_CHAR	* )"speed transform task",
								 (OS_TASK_PTR )speed_transform_task,
								 (void		* )0,
								 (OS_PRIO	  )SPEED_TRANSFORM_TASK_PRIO,     
								 (CPU_STK   * )&SPEED_TRANSFORM_TASK_STK[0],
								 (CPU_STK_SIZE)SPEED_TRANSFORM_STK_SIZE/10,	
								 (CPU_STK_SIZE)SPEED_TRANSFORM_STK_SIZE,		
								 (OS_MSG_QTY  )SPEED_TRANSFORM_Q_NUM,		//����speed_transform_task��Ҫʹ���ڽ���Ϣ���У���Ϣ���г���Ϊ3					
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);
	//����UART1_ANALYZE����
	OSTaskCreate((OS_TCB 	* )&UART1_ANALYZE_TaskTCB,	
								 (CPU_CHAR	* )"uart1 analyze task", 		
								 (OS_TASK_PTR )uart1_analyze_task, 			
								 (void		* )0,
								 (OS_PRIO	  )UART1_ANALYZE_TASK_PRIO,     
								 (CPU_STK   * )&UART1_ANALYZE_TASK_STK[0],	
								 (CPU_STK_SIZE)UART1_ANALYZE_STK_SIZE/10,	
								 (CPU_STK_SIZE)UART1_ANALYZE_STK_SIZE,		
								 (OS_MSG_QTY  )UART1_ANALYZE_Q_NUM,		//����uart1_analyze_task��Ҫʹ���ڽ���Ϣ���У���Ϣ���г���Ϊ2					
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);
	//����auto_charge&Send����
	OSTaskCreate((OS_TCB 	* )&AUTO_CHARGE_TaskTCB,	
								 (CPU_CHAR	* )"Auto charge task", 		
								 (OS_TASK_PTR )auto_charge_task, 			
								 (void		* )0,
								 (OS_PRIO	  )AUTO_CHARGE_TASK_PRIO,     
								 (CPU_STK   * )&AUTO_CHARGE_TASK_STK[0],	
								 (CPU_STK_SIZE)AUTO_CHARGE_STK_SIZE/10,	
								 (CPU_STK_SIZE)AUTO_CHARGE_STK_SIZE,		
								 (OS_MSG_QTY  )0,		//����Ҫʹ���ڽ���Ϣ����			
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);	
	//����REMOTE_CONTROL����
	OSTaskCreate((OS_TCB 	* )&REMOTE_CONTROL_TaskTCB,	
								 (CPU_CHAR	* )"remote control task", 		
								 (OS_TASK_PTR )remote_control_task, 			
								 (void		* )0,
								 (OS_PRIO	  )REMOTE_CONTROL_TASK_PRIO,     
								 (CPU_STK   * )&REMOTE_CONTROL_TASK_STK[0],	
								 (CPU_STK_SIZE)REMOTE_CONTROL_STK_SIZE/10,	
								 (CPU_STK_SIZE)REMOTE_CONTROL_STK_SIZE,		
								 (OS_MSG_QTY  )0,		//����Ҫʹ���ڽ���Ϣ����			
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);									 
	OSTmrStart(&Sencond_1_tmr1,&err);			//������ʱ��1						 								 
	OSTmrStart(&Odom_Calculator_tmr2,&err);			//������ʱ��2
	//�˳��ٽ���						 
	OS_CRITICAL_EXIT();
	OSTaskDel((OS_TCB*)0,&err);	//ɾ��start_task��������
}
/************************************************************************************************************************
*                                                   tmr1_callback
* Description: Null��
* Arguments  : Null
* Note(s)    : 1) Null��
************************************************************************************************************************/
//��ʱ��1�Ļص���������1s
void tmr1_callback(void *p_tmr,void *p_arg)
{
	if(Charge_Exit_Seconds.d && Robot_Status.data[1]==CHARGING_STATUS)
	{
		if(Charge_Exit_Seconds.d)
			Charge_Exit_Seconds.d--;			//���ʱ��-1s
		if(Reset_Robot_Count)
			Reset_Robot_Count--;					//��������ʱ
		else
		{
			__disable_fault_irq();				//���ж�
			NVIC_SystemReset();						//����
		}			
		
	}
	temp_fr_control = 1;			//����tcp����Ƶ��
	//LED��˸
	LED1 = !LED1;
}

/************************************************************************************************************************
*                                                   Odom_Calculator
* Description: ��ʱ��1�Ļص�����---��̼Ƽ��㷢����Ϣ��UART1_SEND_TaskTCB---20ms--50hz;
* Arguments  : Null
* Note(s)    : 1) Null��
************************************************************************************************************************/	
//��ʱ��2---��̼Ƽ���---�ص�����
void Odom_Calculator(void *p_tmr, void *p_arg)
{
	PULSE_LEFT = ENCODER_LEFT_TIMER->CNT - 32768;	//�������̲ɼ���ʱ�� TIM3---TIM3������ģʽGPIOB6 GPIOB7 ����
	ENCODER_LEFT_TIMER->CNT = 32768;
	PULSE_RIGHT = ENCODER_RIGHT_TIMER->CNT - 32768;	//�ҵ�����̲ɼ���ʱ�� TIM4---TIM4������ģʽGPIOC6 GPIOC7 �ҵ��
	ENCODER_RIGHT_TIMER->CNT = 32768;	
//	PULSE_LEFT_HIS += PULSE_LEFT;
//	PULSE_RIGHT_HIS += PULSE_RIGHT;
	//������̼�
	odometry_cal(PULSE_LEFT,PULSE_RIGHT);
}

/************************************************************************************************************************
*                                                   uart1_analyze_task
* Description: ����USART1_IRQHandler ���͵����ݼ��Խ��������ַ�����ͬ����Ϣ���л������͸�����������
* Arguments  : NULL
* Note(s)    : 1) �����ٶ����ݸ�SPEED_TRANSFORM_TaskTCB��8���ֽڣ�
************************************************************************************************************************/	
//����1���յ������ݽ���
void uart1_analyze_task(void *p_arg)
{
	u8 *p;
	OS_MSG_SIZE size;
	OS_ERR err; 
	while(1)
	{
		//�����ڴ�
		p = mymalloc(SRAMIN,200);
		//������Ϣ
		p=OSTaskQPend((OS_TICK		)2000,		//n*5ms�ĳ�ʱ�ȴ�ʱ��
                      (OS_OPT		)OS_OPT_PEND_BLOCKING,	//��������
                      (OS_MSG_SIZE*	)&size,		//��ȡ���ֽڳ���
                      (CPU_TS*		)0,	//ʱ�����NULL--��ʾ����¼
                      (OS_ERR*      )&err );		//���������
		//������timeoutʱ����δ�յ��ٶȡ���ɽ��л����˶̾������ָ��
		if(err == OS_ERR_NONE)
		{
			if(!Format_Check(p,size))	//������ͨ��---Format_Check()����ֵΪ0;
			{
				switch(*(p+4))		//ָ�������ж�-(ִ��-��ȡ-д��)
				{
//					case 0x01:		//ִ��ָ��---������0x01
//					{
//						switch(*(p+5))		//�������ж�
//						{
//							case 0x01:		//�������ָ��---ָ���ַ0x01
//							{	
//								//MOTOR_CLEAR_FAULT
//								CAN1_Send_Msg(CAN_ID1,MOTOR_CLEAR_FAULT,8);//����8���ֽ�
//								CAN1_Send_Msg(CAN_ID2,MOTOR_CLEAR_FAULT,8);//����8���ֽ�								
//							}break;
//							default:			//δָ֪��
//								break;
//						}
//					}break;
					//��ȡָ��---������0x02
					case 0x02:
					{
						switch(*(p+5))
						{
							case REG_STATUS:	//�Ĵ���-0x01-����״̬
							{
								while(uart1_format_build_and_send(Robot_Status.data,DEVICE_ID_D9,READ_COMMOND,REG_STATUS,_2byte));		//����Robot_Status��2���ֽ�							
							}break;
							case REG_ODOMETRY:	//�Ĵ���-0x10-��̼�
							{
								odom_send_task();
								uart1_format_build_and_send(odometry_data,DEVICE_ID_D9,READ_COMMOND,REG_ODOMETRY,31);	
							}break;	
							case REG_BATTERY_VOLTAGE:	//�Ĵ���-0x11-������
							{
								battery_v.d = battery_check();
								while(uart1_format_build_and_send(battery_v.data,DEVICE_ID_D9,READ_COMMOND,REG_BATTERY_VOLTAGE,_4byte));
							}break;	
							case REG_CHARGING_TIME:	//�Ĵ���-0x20-���ʱ��
							{
								while(uart1_format_build_and_send(Charge_Exit_Seconds.data,DEVICE_ID_D9,READ_COMMOND,REG_CHARGING_TIME,_2byte));
							}break;
							case REG_ERROR:	//�Ĵ���-0xF1-������
							{
								while(uart1_format_build_and_send(Robot_Error.data,DEVICE_ID_D9,READ_COMMOND,REG_ERROR,_2byte));		//����Robot_Error��2���ֽ�	
							}break;	
							default:			//δָ֪��
								break;
						}
					}break;
					//д��ָ��---������0x03
					case 0x03:
					{
						switch(*(p+5))
						{
							case REG_CHARGING_TIME:		//�Ĵ���-0x20-���ʱ��
							{
								Charge_Exit_Seconds.data[1] = *(p+7);
								Charge_Exit_Seconds.data[0] = *(p+6);
								if((Charge_Exit_Seconds.d > 5) && (Charge_Exit_Seconds.d < 65535))
								Robot_Status.data[1] = FINDING_DOCK_STATUS;				//����״̬-�л�Ϊ-�ҵ�״̬
								Reset_Robot_Count = Charge_Exit_Seconds.d - 20;		//��������������ʱ
								//����
								while(uart1_format_build_and_send(Charge_Exit_Seconds.data,DEVICE_ID_D9,WRITE_COMMOND,REG_CHARGING_TIME,_2byte));
							}break;								
							case REG_MOVING_SPEED:	//�Ĵ���-0x21-�˶��ٶ�
							{
								if(Robot_Status.data[1] == NAVI_STATUS)		//ȷ������״̬==����״̬
								{
									OSTaskQPost((OS_TCB*)&SPEED_TRANSFORM_TaskTCB,	//������SPEED_TRANSFORM������Ϣ
												(void*		)(p+6),
												(OS_MSG_SIZE)8,
												(OS_OPT		)OS_OPT_POST_FIFO,
												(OS_ERR*	)&err);
									//����
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
					//δָ֪��
					default:
						break;
				}
			}
//			else
//				GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP�������ߣ� ��ͬBEEP=1;
		}
		//�ͷ��ڴ�	
		myfree(SRAMIN,p);
		OSTimeDlyHMSM(0,0,0,10,OS_OPT_TIME_PERIODIC,&err); //��ʱ10ms
	}
}
/************************************************************************************************************************
*                                                   speed_transform_task
* Description: ����uart1_analyze_task��������ȡ���ٶ����ݽ���ת���������·��ӳ���
* Arguments  : SPEED_TRANSFORM_TIMEOUT ���� n*5ms�ĵȴ�ʱ��
* Note(s)    : 1) �ٶ��·������¼��ȡ����speed_transform_task����ʱʱ�䣬���̻Ḳ��֮ǰ�����ݣ�
*              2) �ٶ��·��������ȡ���ڲ���--SPEED_TRANSFORM_TIMEOUT���������·�ֹͣ���
************************************************************************************************************************/	
//�ٶ�ת���·�����
void speed_transform_task(void *p_arg)
{
	u8 *p;
	OS_MSG_SIZE size;
	OS_ERR err;
	//�����ڴ�,ʹ��Ƶ�ʽϸߣ����ͷű��ⷴ�������ͷŽ���Ч��
	while(1)
	{
		p = mymalloc(SRAMIN,10);
		//������Ϣ
		p=OSTaskQPend((OS_TICK		)SPEED_TRANSFORM_TIMEOUT,//n*5ms�ĳ�ʱ�ȴ�ʱ��
                      (OS_OPT		)OS_OPT_PEND_BLOCKING,	//��������
                      (OS_MSG_SIZE*	)&size,		//��ȡ���ֽڳ���
                      (CPU_TS*		)0,	//ʱ�����NULL--��ʾ����¼
                      (OS_ERR*      )&err );		//���������
		//�������ܵ��ٶ������ת��
		if(err == OS_ERR_NONE)
		{
			u8 t;
			for(t=0;t<4;t++)	//�����ڽ��ܵ����ٶȴ洢�ڽṹ����;
			{
				rightdata.data[t]=*(p+t);
				leftdata.data[t]=*(p+t+4);
			}
			if(rightdata.d < 1000 && leftdata.d < 1000)		//���ٶȼ�������
				car_control(rightdata.d,leftdata.d);	 			//�����յ����������ٶȸ���С��
		}
		//������timeoutʱ����δ�յ��ٶȡ����·��ٶȹ���ָ��
		else if(err == OS_ERR_TIMEOUT)
		{
			car_control(0,0);						//ֹͣ
		}
		IWDG_Feed();																							//ι��
		myfree(SRAMIN,p);	//�ͷ��ڴ�
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_PERIODIC,&err); 				//��ʱ50ms�����ٶ��·���С�ڼ��Ӧ��С��50ms
	}
}

/************************************************************************************************************************
*                                                   Format_Check
* Description: ����n���ֽ�,����������У��;
* Arguments  : u8 *p_arg---����ָ��,u8 length--���ݳ���
* Note(s)    : 1) У��ͨ������-0;
*				       2) ����ͷ���󷵻�-1;
*				       3) ����β���󷵻�-2;
*				       4) ���ȴ��󷵻�-3;
*				       4) �ۼƺʹ��󷵻�-4;
************************************************************************************************************************/
u8 Format_Check(u8 *p_arg,u8 length)
{
	u16 check_sum= 0;
	for(int i=0;i<(length-5);i++)
	{
		check_sum += *(p_arg+i);
	}
	if((*p_arg != 0x59) || (*(p_arg+1) != 0xA6))
		return 1;		//����ͷ���󷵻�-1;
	else if((*(p_arg+length-3) != 0x59) || (*(p_arg+length-2) != 0x48) || (*(p_arg+length-1) != 0x54))
		return 2;		//����β���󷵻�-2;
	else if(*(p_arg+2) != length)
		return 3;		//���ȴ��󷵻�-3;
	else if((*(p_arg+length-4) != check_sum) && (*(p_arg+length-5) != (check_sum >> 8)))
		return 4;		//�ۼӺʹ��󷵻�-4;
	else if(*(p_arg+3) != 0x01)
		return 5;		//ָ��Ƿ��͸����̿������ķ���-5;
	else
		return 0;		//У��ͨ������-0;
}
/************************************************************************************************************************
*                                                   Robot_Moving
* Description: ����n���ֽ�,����������У��;
* Arguments  : L_speed---�����ٶȣ�R_speed---�����ٶȣ�time---�˶�ʱ��*50ms
* Note(s)    :      
************************************************************************************************************************/   
void Robot_Moving(float L_speed,float R_speed,u8 time)
{
	u8 p[8],t;
	OS_ERR err;
	
	leftdata.d = L_speed;
	rightdata.d = R_speed;
	
	for(t=0;t<4;t++)	//�����ڽ��ܵ����ٶȴ洢�ڽṹ����;
	{
		*(p+t) = rightdata.data[t];
		*(p+t+4) = leftdata.data[t];
	}
	
	while(time--)
	{
		OSTaskQPost((OS_TCB*)&SPEED_TRANSFORM_TaskTCB,	//������SPEED_TRANSFORM������Ϣ
												(void*		)p,
												(OS_MSG_SIZE)8,
												(OS_OPT		)OS_OPT_POST_FIFO,
												(OS_ERR*	)&err);
//		car_control(L_speed,R_speed);	 //�����յ����������ٶȸ���С��	
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
			
		if((Robot_Status.data[1] == FINDING_DOCK_STATUS) || (Robot_Status.data[1] == CHARGING_STATUS))		//����״̬-Ϊ-�ҵ�״̬||����״̬-Ϊ-���״̬
		{			
			key=Remote_Scan();	
			if(key)
			{
				switch(key)
				{
						case 98:				//str="UP"
							Robot_Moving(100.00,100.00,1);		//��������ǰ����һ�ξ���
//						GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP�������ߣ� ��ͬBEEP=1;
							break;				
						case 168:				//str="DOWN"
							Robot_Moving(-100.00,-100.00,1);		//��������ǰ����һ�ξ���
						break;		  
						case 194:				//str="RIGHT"
							Robot_Moving(100.00,-100.00,1);		//��������ǰ����һ�ξ���
							break;					
						case 34:				//str="LEFT"
							Robot_Moving(-100.00,100.00,1);		//��������ǰ����һ�ξ���
							break;
						case 2:					//str="PLAY"
							//����ٶȡ�����������
							Robot_Moving(0,0,1);		//ֹͣ
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
			OSTimeDlyHMSM(0,0,0,30,OS_OPT_TIME_PERIODIC,&err); //��ʱ50ms
		}
		else
			OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err); //��ʱ1s
		
	}
}
/************************************************************************************************************************
*                                                   auto_charge_task
* Description: �Զ��������;
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
		if((Robot_Status.data[1] == FINDING_DOCK_STATUS) || (Robot_Status.data[1] == CHARGING_STATUS))		//����״̬-Ϊ-�ҵ�״̬||����״̬-Ϊ-���״̬
		{
			if(_If_Charging == TRUE)				//���ϵ���
			{
				Robot_Moving(0,0,1);					//ֹͣ
				delay_ms(1000);								//��ʱ1s���Ȼ������ȶ�
				if(_If_Charging == TRUE)			//���ϵ���
				{					
					Robot_Status.data[1] = CHARGING_STATUS;		//����״̬-�л�Ϊ-���״̬			
				}
			}
			else
			{
				if(Robot_Status.data[1] == FINDING_DOCK_STATUS && _If_Charging==FALSE)		//Ѱ��״̬&&δ���ϵ�
				{
					if(IR_M==NULL)			//�޺����ź�
					{
						IR_No_Find_Count++;												//�������źż���
						if(IR_No_Find_Count >= 30)								//�����޺����źż��� > 30
						{
							IR_No_Find_Count = 0;										//��պ������źż���
							Circle_Moving_Count++;									//ԭ����ת��������
							Robot_Moving(-20.00*k,20.00*k,1);				//��������ʱ����ת
						}
					}
					else
					{
						switch(IR_M)
						{
							case 0x01:				
								Robot_Moving(-40.00*k,-20.00*k,1);		//�����������һ�ξ���-���ֿ�-˳ʱ��								
							break;
							case 0x02:
								Robot_Moving(-20.00*k,-20.00*k,1);		//�����������һ�ξ���
							break;
							case 0x03:
								Robot_Moving(-20.00*k,-40.00*k,1);		//�����������һ�ξ���-���ֿ�-��ʱ��
							break;
							default:															
								break;
						}
						IR_M = NULL;															//��λIR_M
						Circle_Moving_Count = NULL;								//���ԭ����ת��������
						IR_No_Find_Count = NULL;									//���ԭ����ת��������
					}
					/**********��תѰ���źų����趨ֵ*********/
					if(Circle_Moving_Count >= MAX_Circle_Moving_Count)	//ԭ����ת���� > ��תѰ���ź�����������Դ���
					{
						Circle_Moving_Count = NULL;												//���ԭ����ת��������
						Charge_Exit_Seconds.d = NULL;											//��ճ��ʱ��						
						Robot_Status.data[1] = ERROT_STATUS;							//����״̬-�л�Ϊ-�쳣״̬
						Robot_Error.data[1] = CHARGE_FALSE;								//�쳣����-��������-CHARGE_FALSE											
					}
				}				
			}
			/**********����ʱ�������������׮�����е������*********/
			if(Charge_Exit_Seconds.d==NULL && Robot_Status.data[1]==CHARGING_STATUS)		//���ʱ��ľ�&&���״̬
			{
				Robot_Moving(100.00,100.00,Moving_Seconds);		//��������ǰ����һ�ξ���
				Robot_Status.data[1] = NAVI_STATUS;						//����״̬-�л�Ϊ-����״̬
			}
			/**********��ʱ*********/
			OSTimeDlyHMSM(0,0,0,60,OS_OPT_TIME_PERIODIC,&err);   //��ʱ20ms
		}
		else		//δ�г��������ʱ10s
		{
			OSTimeDlyHMSM(0,0,10,0,OS_OPT_TIME_PERIODIC,&err);   //��ʱ10s
		}
		/**********�˳��ٽ���*********/		
	}
}

/*************************************************************************************************************************
*                                                   Odom_Send
* Description: ��̼������������odometry_data
* Arguments  : Null
* Note(s)    : 1) Null��
************************************************************************************************************************/
void odom_send_task(void)
{
		//���͸����ڵ���̼���������---Ԥ����2���ĳ��ȷ�ֹ�쳣�����Ӧ���ò��ϣ�		//Odom_Get
	
		x_data.odoemtry_float=position_x;//��λmm		
		y_data.odoemtry_float=position_y;//��λmm
		theta_data.odoemtry_float=oriention;//��λrad
		vel_linear.odoemtry_float=velocity_linear;//��λmm/s
		vel_angular.odoemtry_float=velocity_angular;//��λrad/s
		//��������̼����ݴ浽Ҫ���͵�����
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
* Note(s)    : 1) Null��
************************************************************************************************************************/
//��ʾ��Ϣ�����е���Ϣ
void msgdis_task(void *p_arg)
{
	u8 *p;
	OS_MSG_SIZE size;
	OS_ERR err; 
	while(1)
	{
		//������Ϣ
		p=OSTaskQPend((OS_TICK		)0,
                      (OS_OPT		)OS_OPT_PEND_BLOCKING,
                      (OS_MSG_SIZE*	)&size,
                      (CPU_TS*		)0,
                      (OS_ERR*      )&err );
		LCD_ShowString(40,270,100,16,16,p);
		myfree(SRAMIN,p);	//�ͷ��ڴ�
		OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_PERIODIC,&err); //��ʱ1s
	}
}
//CRC ����ʽ: CRC-16 x16+x15+x2+1 8005->A001 ��λ�ߵ�
/*******************************************
* ��������: CRC16(uchar *Data , ushort Len )
* ��������: CRC У��
* ��ڲ���: *Data����ҪУ�������
Len�� ��ҪУ�����ݵĳ���
* ���ڲ���: ����У��ֵ
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


