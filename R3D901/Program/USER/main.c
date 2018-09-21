/***********************************************  ˵��  *****************************************************************
*   1.���ڽ���
*    ��1�����ݣ�С���������ٶ�,��λ:mm/s���������ݶ�Ϊfloat�ͣ�float��ռ4�ֽڣ�
*    ��2����ʽ��10�ֽ� [��ʼ��"0xFF"][�����ٶ�4�ֽ�][�����ٶ�4�ֽ�][�����][������"\n"]
*   2.���ڷ���
*    ��1�����ݣ���̼ƣ�x,y���ꡢ���ٶȡ����ٶȺͷ���ǣ���λ����Ϊ��mm,mm,mm/s,rad/s,rad���������ݶ�Ϊfloat�ͣ�float��ռ4�ֽڣ�
*    ��2����ʽ��21�ֽ� [x����4�ֽ�][y����4�ֽ�][�����4�ֽ�][���ٶ�4�ֽ�][���ٶ�4�ֽ�][������"\nYHT"4�ֽ�]
************************************************************************************************************************/
#include "includes.h"
/*
*********************************************************************************************************
*                                               DECLARED
*********************************************************************************************************
*/
u8 Format_Check(u8 *p_arg,u8 length);							//���ݸ�ʽУ��
void tmr1_callback(void *p_tmr,void *p_arg); 		//��ʱ��1�ص�����
void Odom_Calculator(void *p_tmr, void *p_arg); 	//��ʱ��2---��̼�odom���㺯��---20ms--50hz
void PVD_Config(void);														//�͵�ѹ�жϳ�ʼ��
void Stop_Moving(void);													//������ֹͣ�ӳ���
void Robot_Moving(float L_speed,float R_speed,u8 time);					//�������ƶ�
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length);		//����1�����������ͺ���
/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/
#define GNUC_PACKED __attribute__((packed))

#define TASK_Q_NUM	3							//�������ڽ���Ϣ���еĳ���
#define SPEED_TRANSFORM_Q_NUM	2		//�ٶ��·������ڽ���Ϣ���еĳ���---2
#define UART1_SEND_Q_NUM	3				//����1�������ڽ���Ϣ���еĳ���---3
#define UART1_ANALYZE_Q_NUM	2			//����1���ݽ����ڽ���Ϣ���еĳ���---2
#define SPEED_TRANSFORM_TIMEOUT 200u	//n*5ms�ĵȴ�ʱ��--1s
#define SENSOR_POST_TIMEOUT     2000u	//n*5ms�ĵȴ�ʱ��--10s

//Ҫд�뵽STM32 FLASH���ַ�������
const u8 TEXT_Buffer[]={"STM32 FLASH TEST"};
#define TEXT_LENTH sizeof(TEXT_Buffer)	 		  	//���鳤��	
#define SIZE TEXT_LENTH/4+((TEXT_LENTH%4)?1:0)

#define FLASH_SAVE_ADDR  0X0800C004 	//����FLASH �����ַ(����Ϊż��������������,Ҫ���ڱ�������ռ�õ�������.
										//����,д������ʱ��,���ܻᵼ�²�����������,�Ӷ����𲿷ֳ���ʧ.��������.

/*
*********************************************************************************************************
*                                          GLOBAL VARIABLES
*********************************************************************************************************
*/
extern u8 odom_cal_status;//����������ִ�б�־λ
extern int PULSE_RIGHT,PULSE_LEFT;
int PULSE_RIGHT_HIS,PULSE_LEFT_HIS;
u8 Charge_Status = 0;						//����Ȩ���л�����1Ϊ���ؿ��ơ�0Ϊ�ⲿ����

//��λ�����޸ĵĲ���
u8 charge_status_push = 0;		//���״̬�ϴ�Ƶ��
u8 Exit_Charge_Seconds = 20;				//�������ʱ��---n*100ms
u32 Odom_Rate_Seconds = 100;				//odom�ϴ������100ms 
/**************************************************************************************************************************/
#define SEND_BUF_SIZE 256	//�������ݳ���,��õ���sizeof(TEXT_TO_SEND)+2��������.

u8 SendBuff[SEND_BUF_SIZE];	//�������ݻ�����
u8 Battery_Status_Temp = 0;
////////////////////////��ʱ��////////////////////////////////
u8 tmr1sta=0; 											//��Ƕ�ʱ���Ĺ���״̬
OS_TMR	battery_status_tmr1;				//����һ����ʱ��
OS_SEM	MY_SEM;											//����һ���ź��������ڿ���ODOM�ļ���
OS_TMR	Odom_Calculator_tmr2;				//��ʱ��2---��̼�pwm����ص�����---10ms--100hz
/**************************************************************************************************************************/
union recieveData							//���յ�������
{
	float d;
	unsigned char data[4];
}leftdata,rightdata;					//���յ�����������
union sendData								//���յ�������
{
	float d;
	unsigned char data[4];
}battery_v;					//������ѹ���
union odometry								//��̼����ݹ�����
{
	float odoemtry_float;
	unsigned char odometry_char[4];
}x_data,y_data,theta_data,vel_linear,vel_angular;     
//Ҫ��������̼����ݣ��ֱ�Ϊ��X��Y�����ƶ��ľ��룬��ǰ�Ƕȣ����ٶȣ����ٶ�
extern float position_x,position_y,oriention,velocity_linear,velocity_angular;         //����õ�����̼���ֵ
/************************************************************************************************************************/

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
#define MOTOR_INIT_TASK_PRIO   5
//�����ջ
#define MOTOR_INIT_STK_SIZE		128
//������ƿ�
OS_TCB	MOTOR_INIT_TaskTCB;
//�����ջ
CPU_STK	MOTOR_INIT_TASK_STK[MOTOR_INIT_STK_SIZE];
//������
void motor_init_task(void *p_arg);

//�������ȼ�
#define MSGDIS_TASK_PRIO   6
//�����ջ
#define MSGDIS_STK_SIZE		128
//������ƿ�
OS_TCB	Msgdis_TaskTCB;
//�����ջ
CPU_STK	MSGDIS_TASK_STK[MSGDIS_STK_SIZE];
//������
void msgdis_task(void *p_arg);

//�������ȼ�
#define ODOM_SEND_TASK_PRIO   10
//�����ջ
#define ODOM_SEND_STK_SIZE		128
//������ƿ�
OS_TCB	ODOM_SEND_TaskTCB;
//�����ջ
CPU_STK	ODOM_SEND_TASK_STK[ODOM_SEND_STK_SIZE];
//������
void odom_send_task(void *p_arg);

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

//�������ȼ�
#define DOOR_OPEN_TASK_PRIO   14
//�����ջ
#define DOOR_OPEN_STK_SIZE		128
//������ƿ�
OS_TCB	DOOR_OPEN_TaskTCB;
//�����ջ
CPU_STK	DOOR_OPEN_TASK_STK[DOOR_OPEN_STK_SIZE];
//������
void door_open_task(void *p_arg);

//�������ȼ�
#define BATTERY_SEND_TASK_PRIO   15
//�����ջ
#define BATTERY_SEND_STK_SIZE		128
//������ƿ�
OS_TCB	BATTERY_SEND_TaskTCB;
//�����ջ
CPU_STK	BATTERY_SEND_TASK_STK[BATTERY_SEND_STK_SIZE];
//������
void battery_send_task(void *p_arg);

//�������ȼ�
#define REMOTE_CONTROL_TASK_PRIO   16
//�����ջ
#define REMOTE_CONTROL_STK_SIZE		128
//������ƿ�
OS_TCB	REMOTE_CONTROL_TaskTCB;
//�����ջ
CPU_STK	REMOTE_CONTROL_TASK_STK[REMOTE_CONTROL_STK_SIZE];
//������
void remote_control_task(void *p_arg);

//������
int main(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	delay_init(168);  		//ʱ�ӳ�ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�жϷ�������
//	Remote_Init();				//������ճ�ʼ��
	uart_init();   				//���ڳ�ʼ��
	LED_Init();       	  //LED��ʼ��
	KEY_Init();						//����_���״̬�ж����ų�ʼ��
	BEEP_Init();					//��ʼ��������
	Adc_Init();         	//��ʼ��ADC
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN��ʼ����ͨģʽ,������500Kbps    
	
	ENC_Init_Left();					//���õ��D TIM4������ģʽPB6 PB7 ���� 
	ENC_Init_Right();					//���õ��A TIM3������ģʽPC6 PC7 �ҵ��
	
	PVD_Config();					//�͵�ѹ�жϳ�ʼ��
	
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	
	delay_ms(1000);				//�ϵ�ȴ��������ϵ�---part1
	
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
	MYDMA_Enable(DMA2_Stream7,1);     //��ʼһ��DMA����
	
	
//	//��̼��㶨ʱ����ʼ��
//	TIM5_Int_Init(100-1,8400-1);	//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms(500��Ϊ50ms)(50��Ϊ5ms)(5_0.5ms_2Khz)	

	GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP�������ߣ� ��ͬBEEP=1;	
	
	my_mem_init(SRAMIN);//��ʼ���ڲ�RAM
	OSInit(&err);		    //��ʼ��UCOSIII
	OS_CRITICAL_ENTER();	//�����ٽ���			 
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
				 (CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���
	OSStart(&err);      //����UCOSIII
}
/*
************************************************************************************************************************
*                                                   start_task
*
* Description: Null��
*
* Arguments  : Null
*
* Note(s)    : 1) Null��
************************************************************************************************************************
*/
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
	OSTmrCreate((OS_TMR		*)&battery_status_tmr1,		//��ʱ��1
                (CPU_CHAR	*)"battery status tmr1",		//��ʱ������
                (OS_TICK	 )0,			//0ms
                (OS_TICK	 )10,          //10*10=100ms
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
	//����MOTOR_INIT����
	OSTaskCreate((OS_TCB 	* )&MOTOR_INIT_TaskTCB,	
								 (CPU_CHAR	* )"motor init task", 		
								 (OS_TASK_PTR )motor_init_task, 			
								 (void		* )0,
								 (OS_PRIO	  )MOTOR_INIT_TASK_PRIO,     
								 (CPU_STK   * )&MOTOR_INIT_TASK_STK[0],	
								 (CPU_STK_SIZE)MOTOR_INIT_STK_SIZE/10,
								 (CPU_STK_SIZE)MOTOR_INIT_STK_SIZE,		
								 (OS_MSG_QTY  )0,		//MOTOR_INIT_TaskTCB����Ҫʹ���ڽ���Ϣ����		
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);
	//����ODOM_SEND����
	OSTaskCreate((OS_TCB 	* )&ODOM_SEND_TaskTCB,	
								 (CPU_CHAR	* )"odom send task", 		
								 (OS_TASK_PTR )odom_send_task, 			
								 (void		* )0,
								 (OS_PRIO	  )ODOM_SEND_TASK_PRIO,     
								 (CPU_STK   * )&ODOM_SEND_TASK_STK[0],	
								 (CPU_STK_SIZE)ODOM_SEND_STK_SIZE/10,	
								 (CPU_STK_SIZE)ODOM_SEND_STK_SIZE,		
								 (OS_MSG_QTY  )0,		//ODOM_SEND_TaskTCB����Ҫʹ���ڽ���Ϣ����		
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);
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
								 (OS_MSG_QTY  )UART1_ANALYZE_Q_NUM,		//����uart1_analyze_task��Ҫʹ���ڽ���Ϣ���У���Ϣ���г���Ϊ1					
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);
	//����battery_Collect&Send����
	OSTaskCreate((OS_TCB 	* )&BATTERY_SEND_TaskTCB,	
								 (CPU_CHAR	* )"Battery send task", 		
								 (OS_TASK_PTR )battery_send_task, 			
								 (void		* )0,
								 (OS_PRIO	  )BATTERY_SEND_TASK_PRIO,     
								 (CPU_STK   * )&BATTERY_SEND_TASK_STK[0],	
								 (CPU_STK_SIZE)BATTERY_SEND_STK_SIZE/10,	
								 (CPU_STK_SIZE)BATTERY_SEND_STK_SIZE,		
								 (OS_MSG_QTY  )0,		//����Ҫʹ���ڽ���Ϣ����			
								 (OS_TICK	  )0,  					
								 (void   	* )0,					
								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
								 (OS_ERR 	* )&err);	
//	//����REMOTE_CONTROL����
//	OSTaskCreate((OS_TCB 	* )&REMOTE_CONTROL_TaskTCB,	
//								 (CPU_CHAR	* )"remote control task", 		
//								 (OS_TASK_PTR )remote_control_task, 			
//								 (void		* )0,
//								 (OS_PRIO	  )REMOTE_CONTROL_TASK_PRIO,     
//								 (CPU_STK   * )&REMOTE_CONTROL_TASK_STK[0],	
//								 (CPU_STK_SIZE)REMOTE_CONTROL_STK_SIZE/10,	
//								 (CPU_STK_SIZE)REMOTE_CONTROL_STK_SIZE,		
//								 (OS_MSG_QTY  )0,		//����Ҫʹ���ڽ���Ϣ����			
//								 (OS_TICK	  )0,  					
//								 (void   	* )0,					
//								 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR,
//								 (OS_ERR 	* )&err);									 
	OSTmrStart(&battery_status_tmr1,&err);			//������ʱ��1						 								 
	OSTmrStart(&Odom_Calculator_tmr2,&err);			//������ʱ��2
	//�˳��ٽ���						 
	OS_CRITICAL_EXIT();
	OSTaskDel((OS_TCB*)0,&err);	//ɾ��start_task��������
}
/*
************************************************************************************************************************
*                                                   tmr1_callback
* Description: Null��
* Arguments  : Null
* Note(s)    : 1) Null��
************************************************************************************************************************
*/
//��ʱ��1�Ļص�����
void tmr1_callback(void *p_tmr,void *p_arg)
{
//	OS_ERR err;		
	u8 arry[1] = {0x01};	
	
	if(Battery_Status)
	{
		Stop_Moving();
		Charge_Status = 1;		//�л�Ϊ���ؿ���
//		GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP�������ߣ� ��ͬBEEP=1;
				
		uart1_format_build_and_send(arry,0x01,0x03,0x01);		//�������ݵ�����---id = 0x01,commond = 0x02;		
	}
	else
	{
//		GPIO_ResetBits(GPIOF,GPIO_Pin_8); //BEEP�������ͣ� ��ͬBEEP=0;
		Charge_Status = 0;		//�л�Ϊ��λ������
	}

//	OSTmrStop(&battery_status_tmr1,OS_OPT_TMR_NONE,0,&err); //ֹͣ��ʱ��1
}

/*
************************************************************************************************************************
*                                                   Odom_Calculator
* Description: ��ʱ��1�Ļص�����---��̼Ƽ��㷢����Ϣ��UART1_SEND_TaskTCB---20ms--50hz;
* Arguments  : Null
* Note(s)    : 1) Null��
************************************************************************************************************************
*/	
//��ʱ��2---��̼Ƽ���---�ص�����
void Odom_Calculator(void *p_tmr, void *p_arg)
{
	PULSE_LEFT = ENCODER_LEFT_TIMER->CNT - 32768;	//�������̲ɼ���ʱ�� TIM3---TIM3������ģʽGPIOB6 GPIOB7 ����
	ENCODER_LEFT_TIMER->CNT = 32768;
	PULSE_RIGHT = ENCODER_RIGHT_TIMER->CNT - 32768;	//�ҵ�����̲ɼ���ʱ�� TIM4---TIM4������ģʽGPIOC6 GPIOC7 �ҵ��
	ENCODER_RIGHT_TIMER->CNT = 32768;
	
	PULSE_LEFT_HIS += PULSE_LEFT;
	PULSE_RIGHT_HIS += PULSE_RIGHT;
	//������̼�
	odometry_cal(PULSE_LEFT,PULSE_RIGHT);
}

/*
************************************************************************************************************************
*                                                   UART1_SEND
* Description: �������ݸ�ʽ����Ȼ������Ϣ
* Arguments  : u8* arg,u8 length;
* Note(s)    : 1) Null��
************************************************************************************************************************
*/
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 length)
{
	if(length > 64)	return 1;
//	u8 uart_data[128];
//	u8 check_count;
	u16 check_sum;
	
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������
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
		SendBuff[((length++)+5)]='\n';//��ӽ�����
		SendBuff[((length++)+5)]='\r';//��ӽ�����
		SendBuff[((length++)+5)]='Y';//��ӽ�����
		SendBuff[((length++)+5)]='H';//��ӽ�����
		SendBuff[((length++)+5)]='T';//��ӽ�����
		//DMA_Send
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		//���DMA2_Steam7������ɱ�־
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����     
		MYDMA_Enable(DMA2_Stream7,(length+5));     		//��ʼһ��DMA���䣡	  
		//���ͳɹ�
		return 0;		
	}	\
	else
		return 2;
}

/*
************************************************************************************************************************
*                                                   Odom_Send
* Description: ��̼���������Ȼ������Ϣ��UART1_SEND_TaskTCB---100ms--10hz��
* Arguments  : Null
* Note(s)    : 1) Null��
************************************************************************************************************************
*/
void odom_send_task(void *p_arg)
{
	OS_ERR err;
	u8 length;//,check_count;
	//u16 check_sum;
	while(1)
	{	
		//���͸����ڵ���̼���������---Ԥ����2���ĳ��ȷ�ֹ�쳣�����Ӧ���ò��ϣ�
		u8 odometry_data[64];   	
		//Odom_Get
		x_data.odoemtry_float=position_x;//��λmm		
		y_data.odoemtry_float=position_y;//��λmm	
		theta_data.odoemtry_float=oriention;//��λrad
		vel_linear.odoemtry_float=velocity_linear;//��λmm/s
		vel_angular.odoemtry_float=velocity_angular;//��λrad/s
/*
		vel_linear.odoemtry_float=PULSE_LEFT_HIS;//��λmm/s
		vel_angular.odoemtry_float=PULSE_RIGHT_HIS;//��λrad/s
*/	
		//��������̼����ݴ浽Ҫ���͵�����
		for(u8 j=0;j<4;j++)
		{
			odometry_data[j]=x_data.odometry_char[j];
			odometry_data[j+4]=y_data.odometry_char[j];
			odometry_data[j+8]=theta_data.odometry_char[j];
			odometry_data[j+12]=vel_linear.odometry_char[j];
			odometry_data[j+16]=vel_angular.odometry_char[j];			
		}
		length = 20;
		/**********�����ٽ���**********/
		CPU_SR_ALLOC();
		OS_CRITICAL_ENTER();
		uart1_format_build_and_send(odometry_data,0x01,0x01,length);		//�������ݵ�����---id = 0x01,commond = 0x01;
		OS_CRITICAL_EXIT();
		/**********�˳��ٽ���*********/		
		OSTimeDlyHMSM(0,0,0,Odom_Rate_Seconds,OS_OPT_TIME_PERIODIC,&err);   //Ĭ����ʱ100ms--10Hz��
	}
}
/*
************************************************************************************************************************
*                                                   msgdis_task
* Description: Null;
* Arguments  : Null;
* Note(s)    : 1) Null��
************************************************************************************************************************
*/
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
/*
************************************************************************************************************************
*                                                   uart1_analyze_task
* Description: ����USART1_IRQHandler ���͵����ݼ��Խ��������ַ�����ͬ����Ϣ���л������͸�����������
* Arguments  : NULL
* Note(s)    : 1) �����ٶ����ݸ�SPEED_TRANSFORM_TaskTCB��8���ֽڣ�
************************************************************************************************************************
*/	
//����1���յ������ݽ���
void uart1_analyze_task(void *p_arg)
{
	u8 *p;
	OS_MSG_SIZE size;
	OS_ERR err; 
	while(1)
	{
		//�����ڴ�
		p = mymalloc(SRAMIN,20);
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
				switch(*(p+4))
				{
					//ִ��ָ��---������0x01
					case 0x01:		
					{
						//�������ָ��---ָ���ַ0x01
						if(*(p+5) == 0x01)		
						{
							//MOTOR_START_CMD
							CAN1_Send_Msg(CAN_ID1,MOTOR_START_CMD,8);//����8���ֽ�
							CAN1_Send_Msg(CAN_ID2,MOTOR_START_CMD,8);//����8���ֽ�
						}
						//���ָֹͣ��---ָ���ַ0x02					
						else if(*(p+5) == 0x02)
						{
							//MOTOR_STOP_CMD
							Stop_Moving();
						}
						//��λ���������Ȩ	
						else if(*(p+5) == 0x03)					//�������ָ��
						{
							Robot_Moving(100.00,100.00,Exit_Charge_Seconds);		//��������ǰ����һ�ξ���
						}
						else if(*(p+5) == 0x04)					//�������Ȩ���л�
						{
//							Charge_Status |= 0x02;		
						}
						else if(*(p+5) == 0xF1)					//����������ָ��
						{
							//MOTOR_CLEAR_FAULT
							CAN1_Send_Msg(CAN_ID1,MOTOR_CLEAR_FAULT,8);//����8���ֽ�
							CAN1_Send_Msg(CAN_ID2,MOTOR_CLEAR_FAULT,8);//����8���ֽ�
						}
					}break;
					//��ȡָ��---������0x02
					case 0x02:		
					{
						
					}break;
					//д��ָ��---������0x03
					case 0x03:		
					{
						switch(*(p+5))
						{							
							case 0x01:	//�����ٶ�д��---ָ���ַ0x01
							{
								if((Charge_Status&0x01) == 0)		//ȷ����λ���Ƿ��п���Ȩ��&&Moving״̬
								{
									OSTaskQPost((OS_TCB*)&SPEED_TRANSFORM_TaskTCB,	//������SPEED_TRANSFORM������Ϣ
												(void*		)(p+6),
												(OS_MSG_SIZE)8,
												(OS_OPT		)OS_OPT_POST_FIFO,
												(OS_ERR*	)&err);
//									GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP�������ߣ� ��ͬBEEP=1;
								}
							}break;
							case 0x02:
								break;
							case 0x05:		//�޸��ϴ�Ƶ��
								if((*(p+6)<=100) && (*(p+6)>1))		Odom_Rate_Seconds = 1000 / *(p+6);
								break;	
							case 0x06:		//�޸�����ʱ��
								if((*(p+6)<=50) && (*(p+6)>9))		Exit_Charge_Seconds = *(p+6);
								break;	
							case 0x08:		//����ָ��---ָ���ַ0x08
							{
							}break;
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
/*
************************************************************************************************************************
*                                                   speed_transform_task
*
* Description: ����uart1_analyze_task��������ȡ���ٶ����ݽ���ת���������·��ӳ���
*
* Arguments  : SPEED_TRANSFORM_TIMEOUT ���� n*5ms�ĵȴ�ʱ��
*
* Note(s)    : 1) �ٶ��·������¼��ȡ����speed_transform_task����ʱʱ�䣬���̻Ḳ��֮ǰ�����ݣ�
*              2) �ٶ��·��������ȡ���ڲ���--SPEED_TRANSFORM_TIMEOUT���������·�ֹͣ���
************************************************************************************************************************
*/	
//�ٶ�ת���·�����
void speed_transform_task(void *p_arg)
{
	u8 *p;
	OS_MSG_SIZE size;
	OS_ERR err;
	//�����ڴ�,ʹ��Ƶ�ʽϸߣ����ͷű��ⷴ�������ͷŽ���Ч��
	p = mymalloc(SRAMIN,10);
	while(1)
	{
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
			car_control(rightdata.d,leftdata.d);	 //�����յ����������ٶȸ���С��
		}
		//������timeoutʱ����δ�յ��ٶȡ����·��ٶȹ���ָ��
		else if(err == OS_ERR_TIMEOUT)		
		{
			//MOTOR_RESET_SPEED
			CAN1_Send_Msg(CAN_ID1,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�
			CAN1_Send_Msg(CAN_ID2,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�			
		}
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_PERIODIC,&err); //��ʱ50ms�����ٶ��·���С�ڼ��Ӧ��С��50ms
	}
}
/*
************************************************************************************************************************
*                                                   motor_init_task
* Description: �����ַ�����ٶȳ�ʼ��;
* Arguments  : Null;
* Note(s)    : Null;
************************************************************************************************************************
*/ 
void motor_init_task(void *p_arg)
{
	unsigned char i;
	//u8 canbuf[8];
	OS_ERR err;
	delay_ms(500);				//�ϵ�ȴ��������ϵ�---part2
	
	//MOTOR_STOP_CMD
	i = CAN1_Send_Msg(CAN_ID1,MOTOR_STOP_CMD,8);//����8���ֽ�
	i = CAN1_Send_Msg(CAN_ID2,MOTOR_STOP_CMD,8);//����8���ֽ�
	delay_ms(100);
	//MOTOR_SPEED_MODE
	i = CAN1_Send_Msg(CAN_ID1,MOTOR_SPEED_MODE_CMD,8);//����8���ֽ�
	i = CAN1_Send_Msg(CAN_ID2,MOTOR_SPEED_MODE_CMD,8);//����8���ֽ�
	delay_ms(100);
	//MOTOR_RESET_SPEED
	i = CAN1_Send_Msg(CAN_ID1,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�
	i = CAN1_Send_Msg(CAN_ID2,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�
	delay_ms(100);
	//MOTOR_START_CMD
	i = CAN1_Send_Msg(CAN_ID1,MOTOR_START_CMD,8);//����8���ֽ�
	i = CAN1_Send_Msg(CAN_ID2,MOTOR_START_CMD,8);//����8���ֽ�
	delay_ms(100);
	
	i = i;
	GPIO_ResetBits(GPIOF,GPIO_Pin_8); //BEEP�������ͣ� ��ͬBEEP=0;	

	OSTaskDel((OS_TCB*)0,&err);	//ɾ��motor_init_task��������
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
		battery_adc=Get_Adc_Average(ADC_Channel_5,20);//��ȡͨ��5��ת��ֵ��20��ȡƽ��
		battery_v.d = (float)((battery_adc*3.3f)/4096);			//��ȡ�����Ĵ�С����ʵ�ʵ�ѹֵ������3.1111
		battery_v.d = battery_v.d*11.0f;			//�Ŵ�11�����õ���ص�ѹֵ

		while(uart1_format_build_and_send(battery_v.data,0x01,0x02,0x04));		//�������ݵ�����---id = 0x01,commond = 0x02; return 0---��ʾ���ͳɹ�

		/**********�˳��ٽ���*********/		
		OSTimeDlyHMSM(0,0,10,0,OS_OPT_TIME_PERIODIC,&err);   //��ʱ10s--0.1Hz
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
						car_control(100*k,100*k);	 //�����յ����������ٶȸ���С��	
						break;				
					case 168:		//str="DOWN"
						car_control(-100*k,-100*k);	 //�����յ����������ٶȸ���С��	
					break;		  
					case 194:		//str="RIGHT"
						car_control(-100*k,100*k);	 //�����յ����������ٶȸ���С��	
						break;					
					case 34:		//str="LEFT"
						car_control(100*k,-100*k);	 //�����յ����������ٶȸ���С��	
						break;
					case 2:		//str="PLAY"
						//����ٶȡ�����������
						CAN1_Send_Msg(CAN_ID1,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�
						CAN1_Send_Msg(CAN_ID2,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�
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
		
		OSTimeDlyHMSM(0,0,0,50,OS_OPT_TIME_PERIODIC,&err); //��ʱ50ms
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
		USART_Cmd(USART2, DISABLE);  //���ô���2 
		USART_Cmd(USART3, DISABLE);  //���ô���3
		GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP�������ߣ� ��ͬBEEP=1;
		//PWR_EnterSTANDBYMode();			//����ģʽ,1.8V �ں˵�Դ�ر�-�˹�����ͣ����ʹ����2uA����
//		PWR_EnterSTOPMode(PWR_Regulator_LowPower,PWR_STOPEntry_WFI);				//ͣ��ģʽ,����ʱ�Ӷ�ֹͣ-�˹��Ľϵͣ����ʹ����20uA����
	}
}

//#define PWR_Regulator_ON               //��Դ�����͹��� ���ѻ���û�ӳ�
//#define PWR_Regulator_LowPower         //��Դ��ȥ�͹��� ��������������һ���ӳ�    

//#define PWR_STOPEntry_WFI              //�жϻ���
//#define PWR_STOPEntry_WFE              //�¼�����

/*
************************************************************************************************************************
*                                                   Format_Check
*
* Description: ����n���ֽ�,����������У��;
*
* Arguments  : u8 *p_arg---����ָ��,u8 length--���ݳ���
*
* Note(s)    : 1) У��ͨ������-0;
*				       2) ����ͷ���󷵻�-1;
*				       3) ����β���󷵻�-2;
*				       4) ���ȴ��󷵻�-3;
*				       4) �ۼƺʹ��󷵻�-4;
************************************************************************************************************************
  
u8 Format_Check(u8 *p_arg,u8 length)
{
	u16 check_sum= 0;
	for(int i=0;i<(length-4);i++)
	{
		check_sum += *(p_arg+i);
	}
	if((*p_arg != 0xA6) || (*(p_arg+1) != 0x59))
		return 1;		//����ͷ���󷵻�-1;
	else if((*(p_arg+length-2) != 0x0D) || (*(p_arg+length-1) != 0x0A))
		return 2;		//����β���󷵻�-2;
	else if(*(p_arg+2) != length)
		return 3;		//���ȴ��󷵻�-3;
	else if((*(p_arg+length-3) != check_sum) && (*(p_arg+length-4) != (check_sum >> 8)))
		return 4;		//�ۼӺʹ��󷵻�-4;
	else if(*(p_arg+3) != 0x01)
		return 5;		//ָ��Ƿ��͸����̿������ķ���-5;
	else
		return 0;		//У��ͨ������-0;
}
*/

/*
************************************************************************************************************************
*                                                   Format_Check
*
* Description: ����n���ֽ�,����������У��;
*
* Arguments  : u8 *p_arg---����ָ��,u8 length--���ݳ���
*
* Note(s)    : 1) У��ͨ������-0;
*				       2) ����ͷ���󷵻�-1;
*				       3) ����β���󷵻�-2;
*				       4) ���ȴ��󷵻�-3;
*				       4) �ۼƺʹ��󷵻�-4;
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
		return 1;		//����ͷ���󷵻�-1;
	else if((*(p_arg+length-5) != 0x0D) || (*(p_arg+length-4) != 0x0A) || (*(p_arg+length-3) != 0x59) || (*(p_arg+length-2) != 0x48) || (*(p_arg+length-1) != 0x54))
		return 2;		//����β���󷵻�-2;
	else if(*(p_arg+2) != length)
		return 3;		//���ȴ��󷵻�-3;
	else if((*(p_arg+length-6) != check_sum) && (*(p_arg+length-7) != (check_sum >> 8)))
		return 4;		//�ۼӺʹ��󷵻�-4;
	else if(*(p_arg+3) != 0x01)
		return 5;		//ָ��Ƿ��͸����̿������ķ���-5;
	else
		return 0;		//У��ͨ������-0;
}


/*
************************************************************************************************************************
*                                                   Stop_Moving
*
* Description: �������ٶȹ���;
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
	CAN1_Send_Msg(CAN_ID1,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�
	CAN1_Send_Msg(CAN_ID2,MOTOR_RESET_SPEED_CMD,8);//����8���ֽ�;
}	
/*
************************************************************************************************************************
*                                                   Robot_Moving
*
* Description: ����n���ֽ�,����������У��;
*
* Arguments  : L_speed---�����ٶȣ�R_speed---�����ٶȣ�time---�˶�ʱ��*100ms
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
		delay_ms(100);
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


