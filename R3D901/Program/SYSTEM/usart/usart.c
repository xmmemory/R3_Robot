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
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
/////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif

//初始化IO 串口
void uart_init(void)
{
	uart1_init(115200);	//串口1初始化波特率为115200
//	uart2_init(57600);	//串口2初始化波特率为57600
//	uart3_init(38400);	//串口3初始化波特率为57600
//	uart4_init(38400);	//串口4初始化波特率为19200
//	uart5_init(9600);	//串口4初始化波特率为9600
}
/*
************************************************************************************************************************
*                                                   usart1_init
* Description: Usart1_init.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
//初始化IO 串口1 
//bound:波特率
void uart1_init(u32 bound){
   //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
	USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ClearFlag(USART1, USART_FLAG_IDLE);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启相关中断
	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、

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
//初始化IO 串口2 
//bound:波特率
void uart2_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART2时钟
 
	//串口2对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA2复用为USART2
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA3复用为USART2
	
	//USART2端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA2与GPIOA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA2，PA3

   //USART2 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Tx;	//发模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口2
	
  USART_Cmd(USART2, ENABLE);  //使能串口2 
	
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
//初始化IO 串口3 
//bound:波特率
void uart3_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟
 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3
	
	//USART3端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA11与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA10，PA11

   //USART3 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口3
	
  USART_Cmd(USART3, ENABLE);  //使能串口3 
	
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
//初始化IO 串口4 
//bound:波特率
void uart4_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	//使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);	//使能UART4时钟
 
	//串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10复用为UART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11复用为UART4
	
	//USART3端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC11与GPIOC10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC10，PC11

   //USART4 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口4
	
  USART_Cmd(UART4, ENABLE);  //使能串口4 
	
	//USART_ClearFlag(UART4, USART_FLAG_TC);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);//开启相关中断
	
	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口4中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =4;		//子优先级4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}
/*
************************************************************************************************************************
*                                                   uart5_init
* Description: Uart5_init.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
//初始化IO 串口5
//bound:波特率
void uart5_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	//使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);	//使能UART4时钟
 
	//串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10复用为UART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11复用为UART4
	
	//USART3端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC11与GPIOC10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC10，PC11

   //USART4 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口4
	
  USART_Cmd(UART4, ENABLE);  //使能串口4 
	
	//USART_ClearFlag(UART4, USART_FLAG_TC);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);//开启相关中断
	
	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口4中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =4;		//子优先级4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}
/*
************************************************************************************************************************
*                                                   USART1_IRQHandler
*
* Description: 接收串口数据、并在触发空闲帧中断时将数据post给SPEED_TRANSFORM_TaskTCB.
*
* Arguments  : 1.USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
*              2.USART_RX_LEN=0;       						//记录接收长度	
*              
* Note(s)    : 1) Null.
************************************************************************************************************************
*/ 	
unsigned char USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART_RX_LEN=0;       											//记录接收长度

extern OS_TCB	UART1_ANALYZE_TaskTCB;						//接收串口数据的UART1_ANALYZE_TaskTCB任务

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	unsigned char Res;
#if SYSTEM_SUPPORT_OS  //使用UCOS操作系统
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收到一个字节
	{
		if(USART_RX_LEN > (USART_REC_LEN-1))	USART_RX_LEN = 0;	//防止数组溢出、一帧数据最大为254字节.
		USART_RX_BUF[USART_RX_LEN++]=USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
	}	
	else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)//接收到一帧数据（接收到空闲帧）
	{
		//Res = USART2->SR;	//读SR后读DR清空IDLE标志位；
		OS_ERR err;
		Res = USART1->DR;
		//发送消息
		OSTaskQPost((OS_TCB*	)&UART1_ANALYZE_TaskTCB,	//向任务ART1_ANALYZE发送消息
                    (void*		)USART_RX_BUF,
                    (OS_MSG_SIZE)USART_RX_LEN,
                    (OS_OPT		)OS_OPT_POST_FIFO,
					(OS_ERR*	)&err);
		if(err == OS_ERR_NONE)
			USART_RX_LEN=0;		//消息post完成，清空数组计数、重新开始接收
		//若消息post fail---	
		else USART_RX_LEN=0;			//暂时未想好怎么处理
	}
	else if(USART_GetITStatus(USART1, USART_IT_ORE_RX) != RESET)   //ORE_RX错误 
	{
		Res = USART1->DR;
		USART_RX_LEN=0;		//由于ORE_RX错误，清空数组计数、重新开始接收
		Res = Res;
	} 	
 #if SYSTEM_SUPPORT_OS  
	OSIntExit();    	//退出中断
#endif
} 
/*
************************************************************************************************************************
*                                                   USART4_IRQHandler
* Description: 接收串口以数据、并在触发空闲帧中断时将数据保存于Wind_Speed中.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
extern OS_TCB	SENSOR_SEND_TaskTCB;							//接收串口4数据的SENSOR_SEND_TaskTCB任务
unsigned char UART4_RX_BUF[UART4_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 UART4_RX_LEN=0;       												//记录接收长度

void UART4_IRQHandler(void)                	//串口4中断服务程序.
{
	unsigned char Res;
#if SYSTEM_SUPPORT_OS  //使用UCOS操作系统
	OSIntEnter();
#endif
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //接收到一个字节
	{
		if(UART4_RX_LEN > (UART4_REC_LEN-1))	UART4_RX_LEN = 0;	//防止数组溢出、一帧数据最大为254字节.
		UART4_RX_BUF[UART4_RX_LEN++]=USART_ReceiveData(UART4);//(UART4->DR);	//读取接收到的数据
	}	
	else if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)//接收到一帧数据（接收到空闲帧）
	{
		Res = UART4->DR;
		if((UART4_RX_LEN == 41) && (UART4_RX_BUF[40] == 'T') && (UART4_RX_BUF[39] == 'H') &&
			(UART4_RX_BUF[38] == 'Y') && (UART4_RX_BUF[37] == '\r') && (UART4_RX_BUF[36] == '\n'))//校验通过
		{
			OS_ERR err;
			//发送消息
			OSTaskQPost((OS_TCB*	)&SENSOR_SEND_TaskTCB,	//向任务SENSOR_SEND_TaskTCB发送消息
											(void*		)UART4_RX_BUF,
											(OS_MSG_SIZE)UART4_RX_LEN,
											(OS_OPT		)OS_OPT_POST_FIFO,
						(OS_ERR*	)&err);
			if(err == OS_ERR_NONE)
			UART4_RX_LEN=0;		//消息post完成，清空数组计数、重新开始接收
			//若消息post fail---	
			else USART_RX_LEN=0;			//暂时未想好怎么处理
		}
		UART4_RX_LEN = 0;
	}
	else if(USART_GetITStatus(UART4, USART_IT_ORE_RX) != RESET)   //ORE_RX错误 
	{
		Res = UART4->DR;
		UART4_RX_LEN=0;		//由于ORE_RX错误，清空数组计数、重新开始接收
		Res = Res;
	} 	
 #if SYSTEM_SUPPORT_OS  
	OSIntExit();    	//退出中断
#endif
} 

/*
************************************************************************************************************************
*                                                   USART3_IRQHandler
* Description: 接收串口以数据、并在触发空闲帧中断时将数据保存于Wind_Speed中.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
extern OS_TCB	SENSOR_SEND_TaskTCB;							//接收串口3数据的SENSOR_SEND_TaskTCB任务
unsigned char UART3_RX_BUF[UART3_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 UART3_RX_LEN=0;       												//记录接收长度

void UART3_IRQHandler(void)                	//串口4中断服务程序.
{
	unsigned char Res;
#if SYSTEM_SUPPORT_OS  //使用UCOS操作系统
	OSIntEnter();
#endif
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收到一个字节
	{
		if(UART3_RX_LEN > (UART3_REC_LEN-1))	UART3_RX_LEN = 0;	//防止数组溢出、一帧数据最大为254字节.
		UART3_RX_BUF[UART3_RX_LEN++]=USART_ReceiveData(USART3);//(UART4->DR);	//读取接收到的数据
	}	
	else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//接收到一帧数据（接收到空闲帧）
	{
		Res = USART3->DR;
		if((UART3_RX_LEN == 41) && (UART3_RX_BUF[40] == 'T') && (UART3_RX_BUF[39] == 'H') &&
			(UART3_RX_BUF[38] == 'Y') && (UART3_RX_BUF[37] == '\r') && (UART3_RX_BUF[36] == '\n'))//校验通过
		{
			OS_ERR err;
			//发送消息
			OSTaskQPost((OS_TCB*	)&SENSOR_SEND_TaskTCB,	//向任务SENSOR_SEND_TaskTCB发送消息
											(void*		)UART3_RX_BUF,
											(OS_MSG_SIZE)UART3_RX_LEN,
											(OS_OPT		)OS_OPT_POST_FIFO,
						(OS_ERR*	)&err);
			if(err == OS_ERR_NONE)
			UART3_RX_LEN=0;		//消息post完成，清空数组计数、重新开始接收
			//若消息post fail---	
			else UART3_RX_LEN=0;			//暂时未想好怎么处理
		}
		UART3_RX_LEN = 0;
	}
	else if(USART_GetITStatus(USART3, USART_IT_ORE_RX) != RESET)   //ORE_RX错误 
	{
		Res = USART3->DR;
		UART3_RX_LEN=0;		//由于ORE_RX错误，清空数组计数、重新开始接收
		Res = Res;
	} 	
 #if SYSTEM_SUPPORT_OS  
	OSIntExit();    	//退出中断
#endif
} 




