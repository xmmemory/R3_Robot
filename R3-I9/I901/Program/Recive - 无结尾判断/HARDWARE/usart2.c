#include "sys.h"
#include "usart2.h"
#include "usart.h"
#include "stm32f10x_rcc.h"

//串口3中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART3_RX_BUF[USART3_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART3_RX_STA=0;       //接收状态标记	


//初始化IO 串口1 
//bound:波特率
void uart3_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	//使能USART1，GPIOA时钟
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	USART_DeInit(USART3);  //复位串口1
	//USART3_TX   PB.10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure); 

	//USART3_RX	  PB.11
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);  //初始化PB.11
	

	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
//	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;

	USART_Init(USART3, &USART_InitStructure); //初始化串口
	USART_ClockInit(USART3, &USART_ClockInitStructure);
//	USART_ITConfig(USART3, USART_IT_TC, ENABLE);//开启中断
	USART_Cmd(USART3, ENABLE);                    //使能串口 
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//开启相关中断	

}

/*
************************************************************************************************************************
*                                                   uart2_init
* Description: Uart2_init.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
//初始化IO 串口1 
//bound:波特率
void uart2_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_ClockInitTypeDef USART_ClockInitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	USART_DeInit(USART2);  //复位串口1
//	//USART2_TX   PA.2
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
//	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	//USART2_RX	  PA.3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA.3

	//Usart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式
	
	USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;
	USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;
	USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;
	USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;

	USART_Init(USART2, &USART_InitStructure); 				//初始化串口
	USART_ClockInit(USART2, &USART_ClockInitStructure);
//	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	//开启中断
	USART_Cmd(USART2, ENABLE);                  			//使能串口 
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//开启相关中断
}
/*
************************************************************************************************************************
*                                                   USART2_IRQHandler
* Description: 接收串口以数据、并在触发空闲帧中断时将数据保存于Wind_Speed中.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
static short Wind_Speed = 0;
u16 UART2_RX_LEN=0;       											//记录接收长度
#define UART2_REC_LEN  			100  								//定义最大接收字节数 100
unsigned char UART2_RX_BUF[UART2_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.

void USART2_IRQHandler(void)                	//串口3中断服务程序
{		
	unsigned char Res;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收到一个字节
	{
		if(UART2_RX_LEN > (UART2_REC_LEN-1))	UART2_RX_LEN = 0;	//防止数组溢出、一帧数据最大为254字节.
		UART2_RX_BUF[UART2_RX_LEN++]=USART_ReceiveData(USART2);//(UART2->DR);	//读取接收到的数据
	}
	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)//接收到一帧数据（接收到空闲帧）
	{
		Res = USART2->DR;Res = Res;
		if((UART2_RX_LEN == 8) && ((u8)(UART2_RX_BUF[0]+UART2_RX_BUF[1]+UART2_RX_BUF[2]+
			UART2_RX_BUF[3]+UART2_RX_BUF[4]+UART2_RX_BUF[5]+UART2_RX_BUF[6]) == UART2_RX_BUF[7]))		//校验通过
		{
			Wind_Speed = ((UART2_RX_BUF[2]<<8)+ UART2_RX_BUF[3]);
		}
		UART2_RX_LEN = 0;
	}
	
	else if(USART_GetITStatus(USART2, USART_IT_ORE_RX) != RESET)   //ORE_RX错误 
	{
		Res = USART2->DR;
		UART2_RX_LEN=0;		//由于ORE_RX错误，清空数组计数、重新开始接收
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
* Description: 接收串口以数据、并在触发空闲帧中断时将数据保存于PM_Value中.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************
*/
static short Pm1_0 = 0;
static short Pm2_5 = 0;
static short Pm10 = 0;

u16 UART3_RX_LEN=0;       											//记录接收长度
#define UART3_REC_LEN  			100  								//定义最大接收字节数 100
unsigned char UART3_RX_BUF[UART3_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.

void USART3_IRQHandler(void)                	//串口3中断服务程序
{
	unsigned char Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收到一个字节
	{
		if(UART3_RX_LEN > (UART3_REC_LEN-1))	UART3_RX_LEN = 0;	//防止数组溢出、一帧数据最大为254字节.
		UART3_RX_BUF[UART3_RX_LEN++]=USART_ReceiveData(USART3);//(UART5->DR);	//读取接收到的数据
	}
	else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//接收到一帧数据（接收到空闲帧）
	{
		Res = USART3->DR;Res = Res;
		if((UART3_RX_LEN == 32) && (UART3_RX_BUF[0] == 0x42) && (UART3_RX_BUF[1] == 0x4d))		//校验通过
		{			
			Pm1_0 = ((UART3_RX_BUF[4]<<8)+ UART3_RX_BUF[5]);
			Pm2_5 = ((UART3_RX_BUF[6]<<8)+ UART3_RX_BUF[7]);
			Pm10 = ((UART3_RX_BUF[8]<<8)+ UART3_RX_BUF[9]);
		}
		UART3_RX_LEN = 0;
		
	}
	else if(USART_GetITStatus(USART3, USART_IT_ORE_RX) != RESET)   //ORE_RX错误 
	{
		Res = USART3->DR;
		UART3_RX_LEN=0;		//由于ORE_RX错误，清空数组计数、重新开始接收
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

