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
#include "beep.h"
#include <string.h>

u8 SendBuff1[SEND_BUF1_SIZE] = {0xA6,0x59,0x0C,0x01,0x01,0x01,0x01,0x01,0x0E,0x59,0x48,0x54};																//�������ݻ�����
u8 SendBuff3[SEND_BUF3_SIZE] = {0x05,0x00,0x00,0x00,0x27,0x00,0x01,0x00,0x0B,0x20,0x20,0x10,0x23,0x16,0x03,
0x59,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0xFF,0x07};		//�������ݻ�����

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

/************************************************************************************************************************
*                                                   A6_Format_Check
* Description: ����n���ֽ�,����������У��;
* Arguments  : u8 *p_arg---����ָ��,u8 length--���ݳ���
* Note(s)    : 1) У��ͨ������-0;
*				       2) ����ͷ���󷵻�-1;
*				       3) ����β���󷵻�-2;
*				       4) ���ȴ��󷵻�-3;
*				       4) �ۼƺʹ��󷵻�-4;
							 5) �豸id���󷵻�-4;
************************************************************************************************************************/
u8 A6_Format_Check(u8 *p_arg,u8 length)
{
	u16 check_sum= 0;
	for(int i=0;i<(length-5);i++)
	{
		check_sum += *(p_arg+i);
	}
	if(*(p_arg+3) != 0x01)
		return 5;		//ָ��Ƿ��͸����̿������ķ���-5;
	else if((*p_arg != 0x59) || (*(p_arg+1) != 0xA6))
		return 1;		//����ͷ���󷵻�-1;
	else if((*(p_arg+length-3) != 0x59) || (*(p_arg+length-2) != 0x48) || (*(p_arg+length-1) != 0x54))
		return 2;		//����β���󷵻�-2;
	else if(*(p_arg+2) != length)
		return 3;		//���ȴ��󷵻�-3;
	else if((*(p_arg+length-4) != check_sum) && (*(p_arg+length-5) != (check_sum >> 8)))
		return 4;		//�ۼӺʹ��󷵻�-4;	
	else
		return 0;		//У��ͨ������-0;
}
/************************************************************************************************************************
*                                                   _05_Format_Check
* Description: ����n���ֽ�,����������У��;
* Arguments  : u8 *p_arg---����ָ��,u8 length--���ݳ���
* Note(s)    : 1) У��ͨ������-0;
*				       2) ����ͷ���󷵻�-1;
*				       3) ����β���󷵻�-2;
*				       4) ���ȴ��󷵻�-3;
*				       4) �ۼƺʹ��󷵻�-4;
							 5) �豸id���󷵻�-4;
************************************************************************************************************************/
u8 _05_Format_Check(u8 *p_arg,u8 length)
{
	u16 check_sum= 0;
	for(int i=0;i<(length-5);i++)
	{
		check_sum += *(p_arg+i);
	}
	if(*p_arg != 0x05)
		return 1;		//����ͷ���󷵻�-1;
	else if(*(p_arg+length-1) != 0x07)
		return 2;		//����β���󷵻�-2;
//	else if(*(p_arg+2) != length)
//		return 3;		//���ȴ��󷵻�-3;
//	else if((*(p_arg+length-4) != check_sum) && (*(p_arg+length-5) != (check_sum >> 8)))
//		return 4;		//�ۼӺʹ��󷵻�-4;	
//	else if(*(p_arg+3) != 0x01)
//		return 5;		//ָ��Ƿ��͸����̿������ķ���-5;
	else
		return 0;		//У��ͨ������-0;
}

//��ʼ��IO ����
void uart_init(void)
{	
	uart1_init(115200);	//����1��ʼ��������Ϊ115200
//	uart2_init(115200);	//����2��ʼ��������Ϊ115200
	uart3_init(115200);	//����3��ʼ��������Ϊ115200
//	uart4_init(38400);	//����4��ʼ��������Ϊ19200
//	uart5_init(9600);	//����4��ʼ��������Ϊ9600
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF1_SIZE);//DMA2,STEAM7,CH4,����Ϊ����1,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
	MYDMA_Config(DMA1_Stream4,DMA_Channel_7,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF3_SIZE);//DMA2,STEAM7,CH4,����Ϊ����3,�洢��ΪSendBuff,����Ϊ:SEND_BUF_SIZE.
}

#if EN_USART1		  															//���ʹ���˴���1
#define USART_REC_LEN  			256  								//�����������ֽ��� 100
unsigned char UART1_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
unsigned char UART1_RX_BUFF[3][128];     			//���ջ�����
unsigned char UART1_RX_BUF_RECIVE=0;     			//���ջ������
u8 USART1_RX_STA=0;       //����״̬���
/*************************************************************************************************************************
*                                                   UART1_SEND
* Description: �������ݸ�ʽ����Ȼ������Ϣ
* Arguments  : u8* arg,u8 length;
* Note(s)    : 1) Null��
************************************************************************************************************************/
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 register_address,u8 length)
{
	if(length > 128)	return 1;
//	u8 uart_data[128];
//	u8 check_count;
	u16 check_sum = 0;
	
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������
	{
		//Header
		SendBuff1[0] = 0xA6;
		SendBuff1[1] = 0x59;
		//Length
		SendBuff1[2] = length+2+2+3+4;		//���ݳ���+֡ͷ+�ۼӺ�+֡β+��Length+Device ID+Command ID+register_address��
		//Device ID
		SendBuff1[3] = device_id;
		//Command ID
		SendBuff1[4] = commond_id;
		//Register Address
		SendBuff1[5] = register_address;
		//data
		for(u8 j=0;j<length;j++)
		{
			SendBuff1[j+6]=*(arg+j);
		}
		//Check_Sum_Cal
		for(u8 check_count = 0;check_count < length+6;check_count++)		//��length+6�����ݳ���+��ʼ6���ֽ�
		check_sum += SendBuff1[check_count];		
		//Check_Sum
		SendBuff1[((length++)+6)] = (check_sum >> 8);
		SendBuff1[((length++)+6)] = check_sum;
		//Tail
		SendBuff1[((length++)+6)]='Y';//��ӽ�����
		SendBuff1[((length++)+6)]='H';//��ӽ�����
		SendBuff1[((length++)+6)]='T';//��ӽ�����
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
/************************************************************************************************************************
*                                             		uart1_init
* Description: uart1_init_interrupt.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
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
* Arguments  : 1.UART1_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
*              2.USART_RX_LEN=0;       						//��¼���ճ���	             
* Note(s)    : 1) Null.
************************************************************************************************************************/
//���ս�����־λ
void USART1_IRQHandler(void)                										//����1�жϷ������
{
	unsigned char Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  				//���յ�һ���ֽ�
	{
		if(USART1_RX_STA > (USART_REC_LEN-1))	USART1_RX_STA = 0;		//��ֹ�������||�жϽ����Ƿ���ɡ�һ֡�������Ϊ254�ֽ�.
		UART1_RX_BUF[USART1_RX_STA++]=USART_ReceiveData(USART1);		//(USART1->DR);	//��ȡ���յ�������
	}	
	else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)		//���յ�һ֡���ݣ����յ�����֡��
	{
		Res = USART1->DR;
		if(!A6_Format_Check(UART1_RX_BUF,USART1_RX_STA))			//У��ͨ��
		{
			if(UART1_RX_BUF_RECIVE < 3)				//3�����ջ��������п���
			memcpy(UART1_RX_BUFF[UART1_RX_BUF_RECIVE++],UART1_RX_BUF+4,USART1_RX_STA-4-5);			
		}
		USART1_RX_STA=0;					//���������������¿�ʼ����
//		else												//����Ϣfail---
//			USART1_RX_STA=0;					//���������������¿�ʼ����
	}
	else if(USART_GetITStatus(USART1, USART_IT_ORE_RX) != RESET)   //ORE_RX���� 
	{
		Res = USART1->DR;
		USART1_RX_STA=0;		//����ORE_RX�������������������¿�ʼ����
		Res = Res;
	} 	 
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
u16 USART3_RX_STA=0;       												//��¼���ճ���
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
#if EN_USART3_RX	
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
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //���յ�һ���ֽ�
	{
		if(USART3_RX_STA > (UART3_REC_LEN-1))	USART3_RX_STA = 0;	//��ֹ�������||�жϽ����Ƿ���ɡ�һ֡�������Ϊ254�ֽ�.
		UART3_RX_BUF[USART3_RX_STA++]=USART_ReceiveData(USART3);//(USART3->DR);	//��ȡ���յ�������
	}	
	else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//���յ�һ֡���ݣ����յ�����֡��
	{
		Res = USART3->DR;
		if(!_05_Format_Check(UART3_RX_BUF,USART3_RX_STA&0x7FFF))		//У��ͨ��
			USART3_RX_STA|=0x8000;		//���������-bit15��	������ɱ�־
		else												//����Ϣfail---	
			USART3_RX_STA=0;					//���������������¿�ʼ����
	}
	else if(USART_GetITStatus(USART3, USART_IT_ORE_RX) != RESET)   //ORE_RX���� 
	{
		Res = USART3->DR;
		USART3_RX_STA=0;		//����ORE_RX�������������������¿�ʼ����
		Res = Res;
	} 	 
}
#endif			//���ʹ���˴���3����
#endif			//���ʹ���˴���3

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
*                                                   UART3_SEND
* Description: 
* Arguments  : u8* arg,u8 length;
* Note(s)    : 1) Null��
************************************************************************************************************************/
u8 uart3_format_build_and_send(u8 *arg,u8 device_id,u8 data_id,u8 length)
{
	if(length > 198)	return 1;
	u16 check_sum = 0;
	
	if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//�ȴ�DMA1_Steam4�������
	{
		//Header
		SendBuff3[0] = 0x05;
		//Length
		SendBuff3[0+4] = length+1+1+2+4+2+2+7;		//���ݳ���+֡ͷ+֡β+У��+��Length+Device ID+Data ID+Time��
		//Device ID
		SendBuff3[0+4+2] = device_id;
		//Command ID
		SendBuff3[0+4+2+2] = data_id;
		//Register Address
		SendBuff3[0+4+2+2+7] = 0;
		//data
		for(u8 j=0;j<length;j++)
		{
			SendBuff3[j+6]=*(arg+j);
		}
		//Check_Sum_Cal
		for(u8 check_count = 0;check_count < length+6;check_count++)		//��length+6�����ݳ���+��ʼ6���ֽ�
		check_sum += SendBuff3[check_count];		
		//Check_Sum
		SendBuff3[((length++)+6)] = (check_sum >> 8);
		SendBuff3[((length++)+6)] = check_sum;
		//Tail
		SendBuff3[((length++)+6)]='Y';//��ӽ�����
		SendBuff3[((length++)+6)]='H';//��ӽ�����
		SendBuff3[((length++)+6)]='T';//��ӽ�����			
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
