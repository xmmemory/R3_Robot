#include "sys.h"
#include "usart2.h"
#include "usart.h"
#include "stm32f10x_rcc.h"

//����3�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART3_RX_BUF[USART3_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART3_RX_STA=0;       //����״̬���	


//��ʼ��IO ����1 
//bound:������
void uart3_init(u32 bound){
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//ʹ��USART1��GPIOAʱ��
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	USART_DeInit(USART3);  //��λ����1
	//USART3_TX   PB.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	//USART3_RX	  PB.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);  //��ʼ��PB.11
	

	//Usart3 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
//	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//��ģʽ
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;

	USART_Init(USART3, &USART_InitStructure); //��ʼ������
	USART_ClockInit(USART3, &USART_ClockInitStructure);
//	USART_ITConfig(USART3, USART_IT_TC, ENABLE);//�����ж�
	USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ��� 
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//��������ж�	

}

/*
************************************************************************************************************************
*                                                   uart2_init
* Description: Uart2_init.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
//��ʼ��IO ����1 
//bound:������
void uart2_init(u32 bound){
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_DeInit(USART2);  //��λ����1
//	//USART2_TX   PA.2
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
//	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	//USART2_RX	  PA.3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA.3

	//Usart3 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//��ģʽ
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;

	USART_Init(USART2, &USART_InitStructure); 				//��ʼ������
	USART_ClockInit(USART2, &USART_ClockInitStructure);
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	//�����ж�
	USART_Cmd(USART2, ENABLE);                  			//ʹ�ܴ��� 
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//��������ж�
}
/*
************************************************************************************************************************
*                                                   USART2_IRQHandler
* Description: ���մ��������ݡ����ڴ�������֡�ж�ʱ�����ݱ�����Wind_Speed��.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
static short Wind_Speed = 0;
u16 UART2_RX_LEN=0;       											//��¼���ճ���
#define UART2_REC_LEN  			100  								//�����������ֽ��� 100
unsigned char UART2_RX_BUF[UART2_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.

void USART2_IRQHandler(void)                	//����3�жϷ������
{		
	unsigned char Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //���յ�һ���ֽ�
	{
		if(UART2_RX_LEN > (UART2_REC_LEN-1))	UART2_RX_LEN = 0;	//��ֹ���������һ֡�������Ϊ254�ֽ�.
		UART2_RX_BUF[UART2_RX_LEN++]=USART_ReceiveData(USART2);//(UART2->DR);	//��ȡ���յ�������
	}
	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)//���յ�һ֡���ݣ����յ�����֡��
	{
		Res = USART2->DR;Res = Res;
		if((UART2_RX_LEN == 8) && ((u8)(UART2_RX_BUF[0]+UART2_RX_BUF[1]+UART2_RX_BUF[2]+
			UART2_RX_BUF[3]+UART2_RX_BUF[4]+UART2_RX_BUF[5]+UART2_RX_BUF[6]) == UART2_RX_BUF[7]))		//У��ͨ��
		{
			Wind_Speed = ((UART2_RX_BUF[2]<<8)+ UART2_RX_BUF[3]);
		}
		UART2_RX_LEN = 0;
	}
	
	else if(USART_GetITStatus(USART2, USART_IT_ORE_RX) != RESET)   //ORE_RX���� 
	{
		Res = USART2->DR;
		UART2_RX_LEN=0;		//����ORE_RX�������������������¿�ʼ����
		Res = Res;
	}	
}
/*
************************************************************************************************************************
*                                                   GetWindSpeed
* Description: Return Wind_Speed.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
short GetWindSpeed(void)
{
	return Wind_Speed;
}
/*
************************************************************************************************************************
*                                                   USART3_IRQHandler
* Description: ���մ��������ݡ����ڴ�������֡�ж�ʱ�����ݱ�����PM_Value��.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
static short Pm1_0 = 0;
static short Pm2_5 = 0;
static short Pm10 = 0;

u16 UART3_RX_LEN=0;       											//��¼���ճ���
#define UART3_REC_LEN  			100  								//�����������ֽ��� 100
unsigned char UART3_RX_BUF[UART3_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.

void USART3_IRQHandler(void)                	//����3�жϷ������
{
	unsigned char Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //���յ�һ���ֽ�
	{
		if(UART3_RX_LEN > (UART3_REC_LEN-1))	UART3_RX_LEN = 0;	//��ֹ���������һ֡�������Ϊ254�ֽ�.
		UART3_RX_BUF[UART3_RX_LEN++]=USART_ReceiveData(USART3);//(UART5->DR);	//��ȡ���յ�������
	}
	else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//���յ�һ֡���ݣ����յ�����֡��
	{
		Res = USART3->DR;Res = Res;
		if((UART3_RX_LEN == 32) && (UART3_RX_BUF[0] == 0x42) && (UART3_RX_BUF[1] == 0x4d))		//У��ͨ��
		{			
			Pm1_0 = ((UART3_RX_BUF[4]<<8)+ UART3_RX_BUF[5]);
			Pm2_5 = ((UART3_RX_BUF[6]<<8)+ UART3_RX_BUF[7]);
			Pm10 = ((UART3_RX_BUF[8]<<8)+ UART3_RX_BUF[9]);
		}
		UART3_RX_LEN = 0;
		
	}
	else if(USART_GetITStatus(USART3, USART_IT_ORE_RX) != RESET)   //ORE_RX���� 
	{
		Res = USART3->DR;
		UART3_RX_LEN=0;		//����ORE_RX�������������������¿�ʼ����
		Res = Res;
	}	
}

/*
************************************************************************************************************************
*                                                   GetPMValue
* Description: Return PM_Value.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/

short GetPM1_0(void)
{
	return Pm1_0;
}

short GetPM2_5(void)
{
	return Pm2_5;

}
short GetPM10(void)
{
	return Pm10;
}

