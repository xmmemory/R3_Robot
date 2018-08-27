/*
************************************************************************************************************************
*                                                      USART
*                               Universal Synchronous/Asynchronous Receiver/Transmitter
*
*                                  (c) Copyright 2009-2012; YHT
*                           All rights reserved.  Protected by international copyright laws.
*
*                                                   USART MANAGEMENT
*
* File    : USART.C
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
*/

#include "sys.h"
#include "usart.h"
///////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
/////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

//��ʼ��IO ����
void uart_init(void)
{
	uart1_init(115200);	//����1��ʼ��������Ϊ115200
//	uart2_init(57600);	//����2��ʼ��������Ϊ57600
//	uart3_init(38400);	//����3��ʼ��������Ϊ57600
//	uart4_init(38400);	//����4��ʼ��������Ϊ19200
//	uart5_init(9600);	//����4��ʼ��������Ϊ9600
}
/*
************************************************************************************************************************
*                                                   usart1_init
* Description: Usart1_init.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
//��ʼ��IO ����1 
//bound:������
void uart1_init(u32 bound){
   //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ClearFlag(USART1, USART_FLAG_IDLE);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//��������ж�
	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����

#endif
	
}
/*
************************************************************************************************************************
*                                                   usart2_init
* Description: Usart2_init.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
//��ʼ��IO ����2 
//bound:������
void uart2_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART2ʱ��
 
	//����2��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2����ΪUSART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3����ΪUSART2
	
	//USART2�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2��GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA2��PA3

   //USART2 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Tx;	//��ģʽ
  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	
  USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);
}
/*
************************************************************************************************************************
*                                                   usart3_init
* Description: Usart3_init.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
//��ʼ��IO ����3 
//bound:������
void uart3_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ��
 
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11����ΪUSART3
	
	//USART3�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA11��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PA10��PA11

   //USART3 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//��ģʽ
  USART_Init(USART3, &USART_InitStructure); //��ʼ������3
	
  USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3 
	
	USART_ClearFlag(USART3, USART_FLAG_TC);
}
/*
************************************************************************************************************************
*                                                   uart4_init
* Description: Uart4_init.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
//��ʼ��IO ����4 
//bound:������
void uart4_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	//ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);	//ʹ��UART4ʱ��
 
	//����4��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10����ΪUART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11����ΪUART4
	
	//USART3�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC11��GPIOC10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC10��PC11

   //USART4 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//��ģʽ
  USART_Init(UART4, &USART_InitStructure); //��ʼ������4
	
  USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���4 
	
	//USART_ClearFlag(UART4, USART_FLAG_TC);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);//��������ж�
	
	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//����4�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =4;		//�����ȼ�4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}
/*
************************************************************************************************************************
*                                                   uart5_init
* Description: Uart5_init.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
//��ʼ��IO ����5
//bound:������
void uart5_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	//ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);	//ʹ��UART4ʱ��
 
	//����4��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10����ΪUART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11����ΪUART4
	
	//USART3�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC11��GPIOC10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC10��PC11

   //USART4 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//��ģʽ
  USART_Init(UART4, &USART_InitStructure); //��ʼ������4
	
  USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���4 
	
	//USART_ClearFlag(UART4, USART_FLAG_TC);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);//��������ж�
	
	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//����4�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =4;		//�����ȼ�4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}
/*
************************************************************************************************************************
*                                                   USART1_IRQHandler
*
* Description: ���մ������ݡ����ڴ�������֡�ж�ʱ������post��SPEED_TRANSFORM_TaskTCB.
*
* Arguments  : 1.USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
*              2.USART_RX_LEN=0;       						//��¼���ճ���	
*              
* Note(s)    : 1) Null.
************************************************************************************************************************
*/ 	
unsigned char USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 USART_RX_LEN=0;       											//��¼���ճ���

extern OS_TCB	UART1_ANALYZE_TaskTCB;						//���մ������ݵ�UART1_ANALYZE_TaskTCB����

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	unsigned char Res;
#if SYSTEM_SUPPORT_OS  //ʹ��UCOS����ϵͳ
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //���յ�һ���ֽ�
	{
		if(USART_RX_LEN > (USART_REC_LEN-1))	USART_RX_LEN = 0;	//��ֹ���������һ֡�������Ϊ254�ֽ�.
		USART_RX_BUF[USART_RX_LEN++]=USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
	}	
	else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)//���յ�һ֡���ݣ����յ�����֡��
	{
		//Res = USART2->SR;	//��SR���DR���IDLE��־λ��
		OS_ERR err;
		Res = USART1->DR;
		//������Ϣ
		OSTaskQPost((OS_TCB*	)&UART1_ANALYZE_TaskTCB,	//������ART1_ANALYZE������Ϣ
                    (void*		)USART_RX_BUF,
                    (OS_MSG_SIZE)USART_RX_LEN,
                    (OS_OPT		)OS_OPT_POST_FIFO,
					(OS_ERR*	)&err);
		if(err == OS_ERR_NONE)
			USART_RX_LEN=0;		//��Ϣpost��ɣ����������������¿�ʼ����
		//����Ϣpost fail---	
		else USART_RX_LEN=0;			//��ʱδ�����ô����
	}
	else if(USART_GetITStatus(USART1, USART_IT_ORE_RX) != RESET)   //ORE_RX���� 
	{
		Res = USART1->DR;
		USART_RX_LEN=0;		//����ORE_RX�������������������¿�ʼ����
		Res = Res;
	} 	
 #if SYSTEM_SUPPORT_OS  
	OSIntExit();    	//�˳��ж�
#endif
} 
/*
************************************************************************************************************************
*                                                   USART4_IRQHandler
* Description: ���մ��������ݡ����ڴ�������֡�ж�ʱ�����ݱ�����Wind_Speed��.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
extern OS_TCB	SENSOR_SEND_TaskTCB;							//���մ���4���ݵ�SENSOR_SEND_TaskTCB����
unsigned char UART4_RX_BUF[UART4_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 UART4_RX_LEN=0;       												//��¼���ճ���

void UART4_IRQHandler(void)                	//����4�жϷ������.
{
	unsigned char Res;
#if SYSTEM_SUPPORT_OS  //ʹ��UCOS����ϵͳ
	OSIntEnter();
#endif
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //���յ�һ���ֽ�
	{
		if(UART4_RX_LEN > (UART4_REC_LEN-1))	UART4_RX_LEN = 0;	//��ֹ���������һ֡�������Ϊ254�ֽ�.
		UART4_RX_BUF[UART4_RX_LEN++]=USART_ReceiveData(UART4);//(UART4->DR);	//��ȡ���յ�������
	}	
	else if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)//���յ�һ֡���ݣ����յ�����֡��
	{
		Res = UART4->DR;
		if((UART4_RX_LEN == 41) && (UART4_RX_BUF[40] == 'T') && (UART4_RX_BUF[39] == 'H') &&
			(UART4_RX_BUF[38] == 'Y') && (UART4_RX_BUF[37] == '\r') && (UART4_RX_BUF[36] == '\n'))//У��ͨ��
		{
			OS_ERR err;
			//������Ϣ
			OSTaskQPost((OS_TCB*	)&SENSOR_SEND_TaskTCB,	//������SENSOR_SEND_TaskTCB������Ϣ
											(void*		)UART4_RX_BUF,
											(OS_MSG_SIZE)UART4_RX_LEN,
											(OS_OPT		)OS_OPT_POST_FIFO,
						(OS_ERR*	)&err);
			if(err == OS_ERR_NONE)
			UART4_RX_LEN=0;		//��Ϣpost��ɣ����������������¿�ʼ����
			//����Ϣpost fail---	
			else USART_RX_LEN=0;			//��ʱδ�����ô����
		}
		UART4_RX_LEN = 0;
	}
	else if(USART_GetITStatus(UART4, USART_IT_ORE_RX) != RESET)   //ORE_RX���� 
	{
		Res = UART4->DR;
		UART4_RX_LEN=0;		//����ORE_RX�������������������¿�ʼ����
		Res = Res;
	} 	
 #if SYSTEM_SUPPORT_OS  
	OSIntExit();    	//�˳��ж�
#endif
} 

/*
************************************************************************************************************************
*                                                   USART3_IRQHandler
* Description: ���մ��������ݡ����ڴ�������֡�ж�ʱ�����ݱ�����Wind_Speed��.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
extern OS_TCB	SENSOR_SEND_TaskTCB;							//���մ���3���ݵ�SENSOR_SEND_TaskTCB����
unsigned char UART3_RX_BUF[UART3_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 UART3_RX_LEN=0;       												//��¼���ճ���

void UART3_IRQHandler(void)                	//����4�жϷ������.
{
	unsigned char Res;
#if SYSTEM_SUPPORT_OS  //ʹ��UCOS����ϵͳ
	OSIntEnter();
#endif
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //���յ�һ���ֽ�
	{
		if(UART3_RX_LEN > (UART3_REC_LEN-1))	UART3_RX_LEN = 0;	//��ֹ���������һ֡�������Ϊ254�ֽ�.
		UART3_RX_BUF[UART3_RX_LEN++]=USART_ReceiveData(USART3);//(UART4->DR);	//��ȡ���յ�������
	}	
	else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//���յ�һ֡���ݣ����յ�����֡��
	{
		Res = USART3->DR;
		if((UART3_RX_LEN == 41) && (UART3_RX_BUF[40] == 'T') && (UART3_RX_BUF[39] == 'H') &&
			(UART3_RX_BUF[38] == 'Y') && (UART3_RX_BUF[37] == '\r') && (UART3_RX_BUF[36] == '\n'))//У��ͨ��
		{
			OS_ERR err;
			//������Ϣ
			OSTaskQPost((OS_TCB*	)&SENSOR_SEND_TaskTCB,	//������SENSOR_SEND_TaskTCB������Ϣ
											(void*		)UART3_RX_BUF,
											(OS_MSG_SIZE)UART3_RX_LEN,
											(OS_OPT		)OS_OPT_POST_FIFO,
						(OS_ERR*	)&err);
			if(err == OS_ERR_NONE)
			UART3_RX_LEN=0;		//��Ϣpost��ɣ����������������¿�ʼ����
			//����Ϣpost fail---	
			else UART3_RX_LEN=0;			//��ʱδ�����ô����
		}
		UART3_RX_LEN = 0;
	}
	else if(USART_GetITStatus(USART3, USART_IT_ORE_RX) != RESET)   //ORE_RX���� 
	{
		Res = USART3->DR;
		UART3_RX_LEN=0;		//����ORE_RX�������������������¿�ʼ����
		Res = Res;
	} 	
 #if SYSTEM_SUPPORT_OS  
	OSIntExit();    	//�˳��ж�
#endif
} 




