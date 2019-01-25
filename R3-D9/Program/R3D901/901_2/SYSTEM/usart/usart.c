//********************************************************************************
//�������� ����	 
//�޸�����:2018-10-22
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ��������(����)���ܻ����˿Ƽ����޹�˾ 
//All rights reserved
//********************************************************************************

#include "sys.h"
#include "usart.h"

u8 SendBuff[SEND_BUF_SIZE];	//�������ݻ�����
u8 SendBuff2[SEND_BUF2_SIZE] = {0x05,0x00,0x00,0x00,0x27,0x00,0x01,0x00,0x0B,0x20,0x20,0x10,0x23,0x16,0x03,
0x59,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0xFF,0x07};	//�������ݻ�����

/////////////////////////////////////////////////////////////////
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "r3driver.h"					//ucos ʹ��
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
//	uart2_init(115200);	//����2��ʼ��������Ϊ115200
	uart3_init(115200);	//����3��ʼ��������Ϊ115200
//	uart4_init(38400);	//����4��ʼ��������Ϊ19200
//	uart5_init(9600);	//����4��ʼ��������Ϊ9600
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff,SEND_BUF_SIZE);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	MYDMA_Config(DMA1_Stream4,DMA_Channel_7,(u32)&USART3->DR,(u32)SendBuff2,SEND_BUF2_SIZE);//DMA2,STEAM7,CH4,����Ϊ����3,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
}

/************************************************************************************************************************
*                                             		uart1_init
* Description: uart1_init_interrupt.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
#if EN_USART1		  															//���ʹ���˴���1
#define USART_REC_LEN  			200  								//�����������ֽ��� 100
unsigned char USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
u16 USART_RX_LEN=0;       											//��¼���ճ���
extern OS_TCB	UART1_ANALYZE_TaskTCB;						//���մ������ݵ�UART1_ANALYZE_TaskTCB����
//��ʼ��IO ����1����bound:������
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
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}
/************************************************************************************************************************
*                                                   USART1_IRQHandler
*
* Description: ���մ������ݡ����ڴ�������֡�ж�ʱ������post��SPEED_TRANSFORM_TaskTCB.
* Arguments  : 1.USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
*              2.USART_RX_LEN=0;       						//��¼���ճ���	             
* Note(s)    : 1) Null.
************************************************************************************************************************/
//���ս�����־λ
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
#endif			//���ʹ���˴���1����

#endif			//���ʹ���˴���1
/************************************************************************************************************************
*                                             	uart2_init
* Description: uart4_init_interrupt.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
#if EN_USART2		   																//���ʹ���˴���2
#define UART2_REC_LEN  			50  									//�����������ֽ��� 200
unsigned char UART2_RX_BUF[UART2_REC_LEN];     	//���ջ���,���USART_REC_LEN���ֽ�.
u16 UART2_RX_LEN=0;       												//��¼���ճ���
//��ʼ��IO ����2����bound:������
void uart2_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
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
	
  USART_Cmd(USART2, ENABLE);											//ʹ�ܴ���2 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);					//��λ���ͱ�־λ
	USART_ClearFlag(USART2, USART_FLAG_IDLE);				//��λ���б�־λ
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	//��������ж�
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);	//��������ж�
	
	//Usart2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����2�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//�����ȼ�4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
}

/***********************************************************************************************************************
*                                                   USART2_IRQHandler
* Description: ���մ��������ݡ����ڴ�������֡�ж�ʱ�����ݱ�����IR_M��.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
void USART2_IRQHandler(void)                	//����2�жϷ������.
{
	unsigned char Res;
#if SYSTEM_SUPPORT_OS  //ʹ��UCOS����ϵͳ
	OSIntEnter();
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //���յ�һ���ֽ�
	{
		if(UART2_RX_LEN > (UART2_REC_LEN-1))	UART2_RX_LEN = 0;	//��ֹ���������һ֡�������Ϊ254�ֽ�.
		UART2_RX_BUF[UART2_RX_LEN++]=USART_ReceiveData(USART2);//(UART4->DR);	//��ȡ���յ�������
	}	
	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)//���յ�һ֡���ݣ����յ�����֡��
	{
		Res = USART2->DR;
		if(UART2_RX_LEN >= 13)
		{
			if((UART2_RX_BUF[0] == 0xA6) && (UART2_RX_BUF[1] == 0x59) &&(UART2_RX_BUF[2] == 13) && (UART2_RX_BUF[12] == 0x54))//У��ͨ��
			{
//				IR_M = UART2_RX_BUF[6];
				UART2_RX_LEN=0;		//��Ϣpost��ɣ����������������¿�ʼ����
			}
			//����Ϣpost fail---	
			else UART2_RX_LEN=0;			//��ʱδ�����ô����
		}
	}
	else if(USART_GetITStatus(USART2, USART_IT_ORE_RX) != RESET)   //ORE_RX���� 
	{
		Res = USART2->DR;
		UART2_RX_LEN=0;		//����ORE_RX�������������������¿�ʼ����
		Res = Res;
	} 	
 #if SYSTEM_SUPPORT_OS  
	OSIntExit();    	//�˳��ж�
#endif
}
#endif				//ʹ���˴���2

/************************************************************************************************************************
*                                             usart3_init_Interrupt
* Description: usart3_init_Interrupt.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
#if EN_USART3 																		//���ʹ���˴���3
#define UART3_REC_LEN				200  									//�����������ֽ��� 200
unsigned char UART3_RX_BUF[UART3_REC_LEN];     	//���ջ���,���USART_REC_LEN���ֽ�.
u16 UART3_RX_LEN=0;       												//��¼���ճ���
//��ʼ��IO ����3����bound:������
void uart3_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//ʹ��USART3ʱ�� 
	//����3��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10����ΪUSART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11����ΪUSART3	
	//USART3�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB11��GPIOB10
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
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure); //��ʼ������3
	
  USART_Cmd(USART3, ENABLE);  										//ʹ�ܴ���3 	
	USART_ClearFlag(USART3, USART_FLAG_TC);					//��λ���ͱ�־λ
	USART_ClearFlag(USART3, USART_FLAG_IDLE);				//��λ���б�־λ
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	//��������ж�
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);	//��������ж�	
	//Uart3 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//����3�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�5
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
}
/***********************************************************************************************************************
*                                                   USART3_IRQHandler
* Description: ���մ��������ݡ����ڴ�������֡�ж�ʱ�����ݱ��淢�͸�pc��.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
//����3�жϷ������.
void USART3_IRQHandler(void)                	
{
	unsigned char Res;
#if SYSTEM_SUPPORT_OS  //ʹ��UCOS����ϵͳ
	OSIntEnter();
#endif
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)								//���յ�һ���ֽ�
	{
		if(UART3_RX_LEN > (UART3_REC_LEN-1))	UART3_RX_LEN = 0;						//��ֹ���������һ֡�������Ϊ254�ֽ�.
		UART3_RX_BUF[UART3_RX_LEN++]=USART_ReceiveData(USART3);						//(UART3->DR);	//��ȡ���յ�������
	}
	else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)					//���յ�һ֡���ݣ����յ�����֡��
	{
		Res = USART3->DR;
		if(UART3_RX_LEN >= 10)
		{
			if((UART3_RX_BUF[0] == 0x05) && (UART3_RX_BUF[UART3_RX_LEN-1] == 0x07))							//У��ͨ��
			{
				while(uart1_format_build_and_send(UART3_RX_BUF,0x01,0x01,0x01,UART3_RX_LEN));			//͸��
				UART3_RX_LEN=0;																																		//��Ϣpost��ɣ����������������¿�ʼ����
			}
			//����Ϣpost fail---	
			else UART3_RX_LEN=0;					//��ʱδ�����ô����
		}
	}
	else if(USART_GetITStatus(USART3, USART_IT_ORE_RX) != RESET)   //ORE_RX���� 
	{
		Res = USART3->DR;
		UART3_RX_LEN=0;																								//����ORE_RX�������������������¿�ʼ����
		Res = Res;
	}
 #if SYSTEM_SUPPORT_OS  
	OSIntExit();    	//�˳��ж�
#endif
}
#endif				//ʹ���˴���3

/************************************************************************************************************************
*                                             uart4_init_interrupt
* Description: uart4_init_interrupt.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
#if EN_USART4
#define UART4_REC_LEN  			100  		//�����������ֽ��� 100
u8  UART4_RX_BUF[UART4_REC_LEN]; //���ջ���,���USART4_REC_LEN���ֽ�.
//��ʼ��IO ����4����bound:������
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
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =6;		//�����ȼ�4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}

#endif				//ʹ���˴���4


/*************************************************************************************************************************
*                                                   uart5_init
* Description: Uart5_init.
* Arguments  : Null
* Note(s)    : 1) Null.
*************************************************************************************************************************/
#if EN_USART5
#define UART5_REC_LEN  			100  		//�����������ֽ��� 100
u8  UART5_RX_BUF[UART5_REC_LEN]; //���ջ���,���USART5_REC_LEN���ֽ�.
//��ʼ��IO ����5����bound:������
void uart5_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	//ʹ��GPIOCʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);	//ʹ��UART4ʱ��
 
	//����5��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10����ΪUART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11����ΪUART4
	
	//USART5�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC11��GPIOC10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PC10��PC11

   //USART5 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//��ģʽ
  USART_Init(UART4, &USART_InitStructure); //��ʼ������4
	
  USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���5 
	
	//USART_ClearFlag(UART4, USART_FLAG_TC);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);//��������ж�
	
	//Usart5 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//����5�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =4;		//�����ȼ�4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}

#endif				//ʹ���˴���5
/*************************************************************************************************************************
*                                                   UART1_SEND
* Description: �������ݸ�ʽ����Ȼ������Ϣ
* Arguments  : u8* arg,u8 length;
* Note(s)    : 1) Null��
************************************************************************************************************************/
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 register_address,u8 length)
{
	if(length > 64)	return 1;
//	u8 uart_data[128];
//	u8 check_count;
	u16 check_sum = 0;
	
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������
	{
		//Header
		SendBuff[0] = 0xA6;
		SendBuff[1] = 0x59;
		//Length
		SendBuff[2] = length+2+2+3+4;		//���ݳ���+֡ͷ+�ۼӺ�+֡β+��Length+Device ID+Command ID+register_address��
		//Device ID
		SendBuff[3] = device_id;
		//Command ID
		SendBuff[4] = commond_id;
		//Register Address
		SendBuff[5] = register_address;
		//data
		for(u8 j=0;j<length;j++)
		{
			SendBuff[j+6]=*(arg+j);
		}
		//Check_Sum_Cal
		for(u8 check_count = 0;check_count < length+6;check_count++)		//��length+6�����ݳ���+��ʼ6���ֽ�
		check_sum += SendBuff[check_count];		
		//Check_Sum
		SendBuff[((length++)+6)] = (check_sum >> 8);
		SendBuff[((length++)+6)] = check_sum;
		//Tail
		SendBuff[((length++)+6)]='Y';//��ӽ�����
		SendBuff[((length++)+6)]='H';//��ӽ�����
		SendBuff[((length++)+6)]='T';//��ӽ�����
		//DMA_Send
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		//���DMA2_Steam7������ɱ�־
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA����
		MYDMA_Enable(DMA2_Stream7,(length+6));     		//��ʼһ��DMA���䣡
		//���ͳɹ�
		return 0;	
	}
	else
		return 2;
}

/*************************************************************************************************************************
*                                                   UART3_SEND
* Description: 
* Arguments  : u8* arg,u8 length;
* Note(s)    : 1) Null��
************************************************************************************************************************/
//u8 uart3_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 register_address,u8 length)
//{
u8 uart3_send(u8 length)
{
	if(length > 198)	return 1;
	
	if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//�ȴ�DMA1_Steam4�������
	{
		//DMA_Send
		DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);		//���DMA1_Stream4������ɱ�־
		USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA����
		MYDMA_Enable(DMA1_Stream4,length);  					//��ʼһ��DMA���䣡
		//���ͳɹ�
		return 0;	
	}
	else
		return 2;
}

/************************************************************************************************************************
*                                                   USART_END
************************************************************************************************************************/
