//********************************************************************************
//串口驱动 代码	 
//修改日期:2018-10-22
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 光线量子(北京)智能机器人科技有限公司 
//All rights reserved
//********************************************************************************

#include "sys.h"
#include "usart.h"
#include "beep.h"
#include <string.h>

u8 SendBuff1[SEND_BUF1_SIZE] = {0xA6,0x59,0x0C,0x01,0x01,0x01,0x01,0x01,0x0E,0x59,0x48,0x54};																//发送数据缓冲区
u8 SendBuff3[SEND_BUF3_SIZE] = {0x05,0x00,0x00,0x00,0x27,0x00,0x01,0x00,0x0B,0x20,0x20,0x10,0x23,0x16,0x03,
0x59,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0xFF,0x07};		//发送数据缓冲区

/////////////////////////////////////////////////////////////////
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "r3driver.h"					//ucos 使用
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

/************************************************************************************************************************
*                                                   A6_Format_Check
* Description: 接收n个字节,并进行数据校验;
* Arguments  : u8 *p_arg---数据指针,u8 length--数据长度
* Note(s)    : 1) 校验通过返回-0;
*				       2) 数据头错误返回-1;
*				       3) 数据尾错误返回-2;
*				       4) 长度错误返回-3;
*				       4) 累计和错误返回-4;
							 5) 设备id错误返回-4;
************************************************************************************************************************/
u8 A6_Format_Check(u8 *p_arg,u8 length)
{
	u16 check_sum= 0;
	for(int i=0;i<(length-5);i++)
	{
		check_sum += *(p_arg+i);
	}
	if(*(p_arg+3) != 0x01)
		return 5;		//指令不是发送给底盘控制器的返回-5;
	else if((*p_arg != 0x59) || (*(p_arg+1) != 0xA6))
		return 1;		//数据头错误返回-1;
	else if((*(p_arg+length-3) != 0x59) || (*(p_arg+length-2) != 0x48) || (*(p_arg+length-1) != 0x54))
		return 2;		//数据尾错误返回-2;
	else if(*(p_arg+2) != length)
		return 3;		//长度错误返回-3;
	else if((*(p_arg+length-4) != check_sum) && (*(p_arg+length-5) != (check_sum >> 8)))
		return 4;		//累加和错误返回-4;	
	else
		return 0;		//校验通过返回-0;
}
/************************************************************************************************************************
*                                                   _05_Format_Check
* Description: 接收n个字节,并进行数据校验;
* Arguments  : u8 *p_arg---数据指针,u8 length--数据长度
* Note(s)    : 1) 校验通过返回-0;
*				       2) 数据头错误返回-1;
*				       3) 数据尾错误返回-2;
*				       4) 长度错误返回-3;
*				       4) 累计和错误返回-4;
							 5) 设备id错误返回-4;
************************************************************************************************************************/
u8 _05_Format_Check(u8 *p_arg,u8 length)
{
	u16 check_sum= 0;
	for(int i=0;i<(length-5);i++)
	{
		check_sum += *(p_arg+i);
	}
	if(*p_arg != 0x05)
		return 1;		//数据头错误返回-1;
	else if(*(p_arg+length-1) != 0x07)
		return 2;		//数据尾错误返回-2;
//	else if(*(p_arg+2) != length)
//		return 3;		//长度错误返回-3;
//	else if((*(p_arg+length-4) != check_sum) && (*(p_arg+length-5) != (check_sum >> 8)))
//		return 4;		//累加和错误返回-4;	
//	else if(*(p_arg+3) != 0x01)
//		return 5;		//指令不是发送给底盘控制器的返回-5;
	else
		return 0;		//校验通过返回-0;
}

//初始化IO 串口
void uart_init(void)
{	
	uart1_init(115200);	//串口1初始化波特率为115200
//	uart2_init(115200);	//串口2初始化波特率为115200
	uart3_init(115200);	//串口3初始化波特率为115200
//	uart4_init(38400);	//串口4初始化波特率为19200
//	uart5_init(9600);	//串口4初始化波特率为9600
	MYDMA_Config(DMA2_Stream7,DMA_Channel_4,(u32)&USART1->DR,(u32)SendBuff1,SEND_BUF1_SIZE);//DMA2,STEAM7,CH4,外设为串口1,存储器为SendBuff,长度为:SEND_BUF_SIZE.
	MYDMA_Config(DMA1_Stream4,DMA_Channel_7,(u32)&USART3->DR,(u32)SendBuff3,SEND_BUF3_SIZE);//DMA2,STEAM7,CH4,外设为串口3,存储器为SendBuff,长度为:SEND_BUF_SIZE.
}

#if EN_USART1		  															//如果使能了串口1
#define USART_REC_LEN  			256  								//定义最大接收字节数 100
unsigned char UART1_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
unsigned char UART1_RX_BUFF[3][128];     			//接收缓存区
unsigned char UART1_RX_BUF_RECIVE=0;     			//接收缓冲计数
u8 USART1_RX_STA=0;       //接收状态标记
/*************************************************************************************************************************
*                                                   UART1_SEND
* Description: 串口数据格式整理、然后发送消息
* Arguments  : u8* arg,u8 length;
* Note(s)    : 1) Null；
************************************************************************************************************************/
u8 uart1_format_build_and_send(u8 *arg,u8 device_id,u8 commond_id,u8 register_address,u8 length)
{
	if(length > 128)	return 1;
//	u8 uart_data[128];
//	u8 check_count;
	u16 check_sum = 0;
	
	if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
	{
		//Header
		SendBuff1[0] = 0xA6;
		SendBuff1[1] = 0x59;
		//Length
		SendBuff1[2] = length+2+2+3+4;		//数据长度+帧头+累加和+帧尾+（Length+Device ID+Command ID+register_address）
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
		for(u8 check_count = 0;check_count < length+6;check_count++)		//“length+6”数据长度+起始6个字节
		check_sum += SendBuff1[check_count];		
		//Check_Sum
		SendBuff1[((length++)+6)] = (check_sum >> 8);
		SendBuff1[((length++)+6)] = check_sum;
		//Tail
		SendBuff1[((length++)+6)]='Y';//添加结束符
		SendBuff1[((length++)+6)]='H';//添加结束符
		SendBuff1[((length++)+6)]='T';//添加结束符
		//DMA_Send
		DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);		//清除DMA2_Steam7传输完成标志
		USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送
		MYDMA_Enable(DMA2_Stream7,(length+6));     		//开始一次DMA传输！
		//发送成功
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
//初始化IO 串口1——bound:波特率
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
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}
/************************************************************************************************************************
*                                                   USART1_IRQHandler
*
* Description: 接收串口数据、并在触发空闲帧中断时将数据post给SPEED_TRANSFORM_TaskTCB.
* Arguments  : 1.UART1_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
*              2.USART_RX_LEN=0;       						//记录接收长度	             
* Note(s)    : 1) Null.
************************************************************************************************************************/
//接收结束标志位
void USART1_IRQHandler(void)                										//串口1中断服务程序
{
	unsigned char Res;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  				//接收到一个字节
	{
		if(USART1_RX_STA > (USART_REC_LEN-1))	USART1_RX_STA = 0;		//防止数组溢出||判断接收是否完成、一帧数据最大为254字节.
		UART1_RX_BUF[USART1_RX_STA++]=USART_ReceiveData(USART1);		//(USART1->DR);	//读取接收到的数据
	}	
	else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)		//接收到一帧数据（接收到空闲帧）
	{
		Res = USART1->DR;
		if(!A6_Format_Check(UART1_RX_BUF,USART1_RX_STA))			//校验通过
		{
			if(UART1_RX_BUF_RECIVE < 3)				//3级接收缓存区还有空余
			memcpy(UART1_RX_BUFF[UART1_RX_BUF_RECIVE++],UART1_RX_BUF+4,USART1_RX_STA-4-5);			
		}
		USART1_RX_STA=0;					//清空数组计数、重新开始接收
//		else												//若消息fail---
//			USART1_RX_STA=0;					//清空数组计数、重新开始接收
	}
	else if(USART_GetITStatus(USART1, USART_IT_ORE_RX) != RESET)   //ORE_RX错误 
	{
		Res = USART1->DR;
		USART1_RX_STA=0;		//由于ORE_RX错误，清空数组计数、重新开始接收
		Res = Res;
	} 	 
}
#endif			//如果使能了串口1接收
#endif			//如果使能了串口1
/************************************************************************************************************************
*                                             	uart2_init
* Description: uart4_init_interrupt.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
#if EN_USART2		   																//如果使能了串口2
#define UART2_REC_LEN  			50  									//定义最大接收字节数 200
unsigned char UART2_RX_BUF[UART2_REC_LEN];     	//接收缓冲,最大USART_REC_LEN个字节.
u16 UART2_RX_LEN=0;       												//记录接收长度
//初始化IO 串口2——bound:波特率
void uart2_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
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
	
  USART_Cmd(USART2, ENABLE);											//使能串口2 
	
	USART_ClearFlag(USART2, USART_FLAG_TC);					//复位发送标志位
	USART_ClearFlag(USART2, USART_FLAG_IDLE);				//复位空闲标志位
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	//开启相关中断
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);	//开启相关中断
	
	//Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//串口2中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =2;		//子优先级4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器	
}

/***********************************************************************************************************************
*                                                   USART2_IRQHandler
* Description: 接收串口以数据、并在触发空闲帧中断时将数据保存于IR_M中.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
void USART2_IRQHandler(void)                	//串口2中断服务程序.
{
	unsigned char Res;
#if SYSTEM_SUPPORT_OS  //使用UCOS操作系统
	OSIntEnter();
#endif
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收到一个字节
	{
		if(UART2_RX_LEN > (UART2_REC_LEN-1))	UART2_RX_LEN = 0;	//防止数组溢出、一帧数据最大为254字节.
		UART2_RX_BUF[UART2_RX_LEN++]=USART_ReceiveData(USART2);//(UART4->DR);	//读取接收到的数据
	}	
	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)//接收到一帧数据（接收到空闲帧）
	{
		Res = USART2->DR;
		if(UART2_RX_LEN >= 13)
		{
			if((UART2_RX_BUF[0] == 0xA6) && (UART2_RX_BUF[1] == 0x59) &&(UART2_RX_BUF[2] == 13) && (UART2_RX_BUF[12] == 0x54))//校验通过
			{
//				IR_M = UART2_RX_BUF[6];
				UART2_RX_LEN=0;		//消息post完成，清空数组计数、重新开始接收
			}
			//若消息post fail---	
			else UART2_RX_LEN=0;			//暂时未想好怎么处理
		}
	}
	else if(USART_GetITStatus(USART2, USART_IT_ORE_RX) != RESET)   //ORE_RX错误 
	{
		Res = USART2->DR;
		UART2_RX_LEN=0;		//由于ORE_RX错误，清空数组计数、重新开始接收
		Res = Res;
	} 	
 #if SYSTEM_SUPPORT_OS  
	OSIntExit();    	//退出中断
#endif
}
#endif				//使能了串口2

/************************************************************************************************************************
*                                             usart3_init_Interrupt
* Description: usart3_init_Interrupt.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
#if EN_USART3 																		//如果使能了串口3
#define UART3_REC_LEN				200  									//定义最大接收字节数 200
unsigned char UART3_RX_BUF[UART3_REC_LEN];     	//接收缓冲,最大USART_REC_LEN个字节.
u16 USART3_RX_STA=0;       												//记录接收长度
//初始化IO 串口3——bound:波特率
void uart3_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//使能USART3时钟 
	//串口3对应引脚复用映射
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOB10复用为USART3
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOB11复用为USART3	
	//USART3端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOB11与GPIOB10
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
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART3, &USART_InitStructure); //初始化串口3	
  USART_Cmd(USART3, ENABLE);  										//使能串口3 	
	USART_ClearFlag(USART3, USART_FLAG_TC);					//复位发送标志位
	USART_ClearFlag(USART3, USART_FLAG_IDLE);				//复位空闲标志位
#if EN_USART3_RX	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);	//开启相关中断
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);	//开启相关中断	
	//Uart3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;//串口3中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级5
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器	
}
/***********************************************************************************************************************
*                                                   USART3_IRQHandler
* Description: 接收串口以数据、并在触发空闲帧中断时将数据保存发送给pc中.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
//串口3中断服务程序.
void USART3_IRQHandler(void)                	
{
	unsigned char Res;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收到一个字节
	{
		if(USART3_RX_STA > (UART3_REC_LEN-1))	USART3_RX_STA = 0;	//防止数组溢出||判断接收是否完成、一帧数据最大为254字节.
		UART3_RX_BUF[USART3_RX_STA++]=USART_ReceiveData(USART3);//(USART3->DR);	//读取接收到的数据
	}	
	else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)//接收到一帧数据（接收到空闲帧）
	{
		Res = USART3->DR;
		if(!_05_Format_Check(UART3_RX_BUF,USART3_RX_STA&0x7FFF))		//校验通过
			USART3_RX_STA|=0x8000;		//接收完成了-bit15，	接收完成标志
		else												//若消息fail---	
			USART3_RX_STA=0;					//清空数组计数、重新开始接收
	}
	else if(USART_GetITStatus(USART3, USART_IT_ORE_RX) != RESET)   //ORE_RX错误 
	{
		Res = USART3->DR;
		USART3_RX_STA=0;		//由于ORE_RX错误，清空数组计数、重新开始接收
		Res = Res;
	} 	 
}
#endif			//如果使能了串口3接收
#endif			//如果使能了串口3

/************************************************************************************************************************
*                                             uart4_init_interrupt
* Description: uart4_init_interrupt.
* Arguments  : Null
* Note(s)    : 1) Null.
************************************************************************************************************************/
#if EN_USART4
#define UART4_REC_LEN  			100  		//定义最大接收字节数 100
u8  UART4_RX_BUF[UART4_REC_LEN]; //接收缓冲,最大USART4_REC_LEN个字节.
//初始化IO 串口4——bound:波特率
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
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =6;		//子优先级4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}

#endif				//使能了串口4


/*************************************************************************************************************************
*                                                   uart5_init
* Description: Uart5_init.
* Arguments  : Null
* Note(s)    : 1) Null.
*************************************************************************************************************************/
#if EN_USART5
#define UART5_REC_LEN  			100  		//定义最大接收字节数 100
u8  UART5_RX_BUF[UART5_REC_LEN]; //接收缓冲,最大USART5_REC_LEN个字节.
//初始化IO 串口5——bound:波特率
void uart5_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);	//使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);	//使能UART4时钟
 
	//串口5对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOC10复用为UART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOC11复用为UART4
	
	//USART5端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOC11与GPIOC10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PC10，PC11

   //USART5 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式
  USART_Init(UART4, &USART_InitStructure); //初始化串口4
	
  USART_Cmd(UART4, ENABLE);  //使能串口5 
	
	//USART_ClearFlag(UART4, USART_FLAG_TC);
	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);//开启相关中断
	
	//Usart5 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口5中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =4;		//子优先级4
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}
#endif				//使能了串口5

/*************************************************************************************************************************
*                                                   UART3_SEND
* Description: 
* Arguments  : u8* arg,u8 length;
* Note(s)    : 1) Null；
************************************************************************************************************************/
u8 uart3_format_build_and_send(u8 *arg,u8 device_id,u8 data_id,u8 length)
{
	if(length > 198)	return 1;
	u16 check_sum = 0;
	
	if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//等待DMA1_Steam4传输完成
	{
		//Header
		SendBuff3[0] = 0x05;
		//Length
		SendBuff3[0+4] = length+1+1+2+4+2+2+7;		//数据长度+帧头+帧尾+校验+（Length+Device ID+Data ID+Time）
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
		for(u8 check_count = 0;check_count < length+6;check_count++)		//“length+6”数据长度+起始6个字节
		check_sum += SendBuff3[check_count];		
		//Check_Sum
		SendBuff3[((length++)+6)] = (check_sum >> 8);
		SendBuff3[((length++)+6)] = check_sum;
		//Tail
		SendBuff3[((length++)+6)]='Y';//添加结束符
		SendBuff3[((length++)+6)]='H';//添加结束符
		SendBuff3[((length++)+6)]='T';//添加结束符			
		//DMA_Send
		DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);		//清除DMA1_Stream4传输完成标志
		USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送
		MYDMA_Enable(DMA1_Stream4,length);  					//开始一次DMA传输！
		//发送成功
		return 0;	
	}
	else
		return 2;
}

/************************************************************************************************************************
*                                                   USART_END
************************************************************************************************************************/
