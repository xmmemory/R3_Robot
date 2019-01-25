/************************************************
 * 文件描述     : 串口2---电源控制板
 * 程序文件     : main.c               
 * 版    本     : 2.0
 * 作    者     : y
 * 日    期     : 2018.04.18
 * 芯    片     : stc15w204s
 * 时    钟		：内部11.0592MHz
 * 开发环境		：Keil uVision4 C51 V9.06
 ************************************************/
 //文件包含
 #include	"config.h"
 #include	"GPIO.h"
 #include	"USART1.h"
 #include	"timer.h"

 //宏定义
// #define   CHARTAST     1		//字符任务
// #define   NUMTAST    	2		//数字任务
// #define   MENUTAST     3		//菜单任务
 
 //定义位变量
 sbit	BEEP=P1^0;			//操作BEEP
 sbit LED = P1^1;			//操作LED
 sbit	LOCK=P3^2;			//操作LOCK
 sbit	Card=P1^2;			//操作OUT1
 sbit	Keyboard=P1^3;		//操作OUT2
 sbit	Identity=P1^4;		//操作OUT3
 sbit	Reserve=P1^5;		//操作OUT4
 sbit	OUT3=P1^6;			//操作OUT3
 sbit	OUT4=P1^7;			//操作OUT4
 
 //全局变量
 bit HaveTast=0;			    //任务标记
 unsigned char Uart2Cmd;		//串口命令

 unsigned int Lock_count;
 unsigned int LED_Count;

 //函数声明
 void Delay_ms(unsigned int ms);

 /******************** IO配置函数 **************************/
 void	GPIO_config(void)
 { 
	GPIO_InitTypeDef	GPIO_InitStructure;		//结构定义
	GPIO_InitStructure.Pin  = GPIO_Pin_All;		//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7, 或操作
	GPIO_InitStructure.Mode = GPIO_PullUp;		//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	//初始化P1

	GPIO_InitStructure.Pin  = GPIO_Pin_0;		//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7, 或操作
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	//初始化——BEEP
	
	GPIO_InitStructure.Pin  = GPIO_Pin_All;		//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7, 或操作
	GPIO_InitStructure.Mode = GPIO_PullUp;		//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	//初始化——LOCK

	GPIO_InitStructure.Pin  = GPIO_Pin_All;		//指定要初始化的IO, GPIO_Pin_0 ~ GPIO_Pin_7, 或操作
	GPIO_InitStructure.Mode = GPIO_PullUp;		//指定IO的输入或输出方式,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P5,&GPIO_InitStructure);	//初始化——LOCK
 }
 /*************  串口1初始化函数 *****************/
void	UART_config(void)
{
	COMx_InitDefine		COMx_InitStructure;					//结构定义
	COMx_InitStructure.UART_Mode      = UART_8bit_BRTx;		//模式,       UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
	COMx_InitStructure.UART_BRT_Use   = BRT_Timer2;			//使用波特率,   BRT_Timer1, BRT_Timer2 (注意: 串口2固定使用BRT_Timer2)
	COMx_InitStructure.UART_BaudRate  = 9600ul;			//波特率, 一般 110 ~ 115200
	COMx_InitStructure.UART_RxEnable  = ENABLE;				//接收允许,   ENABLE或DISABLE
	COMx_InitStructure.BaudRateDouble = DISABLE;			//波特率加倍, ENABLE或DISABLE
	COMx_InitStructure.UART_Interrupt = ENABLE;				//中断允许,   ENABLE或DISABLE
	COMx_InitStructure.UART_Polity    = PolityLow;			//中断优先级, PolityLow,PolityHigh
	COMx_InitStructure.UART_P_SW      = UART1_SW_P36_P37;	//切换端口,   UART1_SW_P30_P31,UART1_SW_P36_P37,UART1_SW_P16_P17(必须使用内部时钟)
	COMx_InitStructure.UART_RXD_TXD_Short = DISABLE;		//内部短路RXD与TXD, 做中继, ENABLE,DISABLE
	USART_Configuration(USART1, &COMx_InitStructure);		//初始化串口1 USART1,USART2

	PrintString1("RAY_IOBD_L63130_START!\r\n");	//SUART1发送一个字符串
}
/************************ 定时器配置 ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;					//结构定义
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_Polity    = PolityLow;			//指定中断优先级, PolityHigh,PolityLow
	TIM_InitStructure.TIM_Interrupt = ENABLE;				//中断是否允许,   ENABLE或DISABLE
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / (50*12));		//初值	  ---20ms
	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);				//初始化Timer0	  Timer0,Timer1,Timer2

//	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//指定工作模式,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
//	TIM_InitStructure.TIM_Polity    = PolityLow;			//指定中断优先级, PolityHigh,PolityLow
//	TIM_InitStructure.TIM_Interrupt = ENABLE;				//中断是否允许,   ENABLE或DISABLE
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;		//指定时钟源, TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
//	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / (50*12));		//初值
//	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
//	Timer_Inilize(Timer1,&TIM_InitStructure);				//初始化Timer1	  Timer0,Timer1,Timer2
//
//	TIM_InitStructure.TIM_Interrupt = ENABLE;				//中断是否允许,   ENABLE或DISABLE. (注意: Timer2固定为16位自动重装, 中断固定为低优先级)
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;		//指定时钟源,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//是否输出高速脉冲, ENABLE或DISABLE
//	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / (50*6*5));		//初值	 ---50ms
//	TIM_InitStructure.TIM_Run       = ENABLE;				//是否初始化后启动定时器, ENABLE或DISABLE
//	Timer_Inilize(Timer2,&TIM_InitStructure);				//初始化Timer2	  Timer0,Timer1,Timer2
}

 /******************** 主函数 **************************/
 void main(void)
 {
	 u8	i;
	 UART_config();
	 Timer_config();
	 EA = 1;

	 GPIO_config();
	 LED = 0;	 
	 Delay_ms(500);
	 BEEP = 0;
	 Delay_ms(500);
	 LED = 1;
	 
	 WDT_CONTR = 0x3e;			//WatchDogInit

	while (1)
	{	   		
		Delay_ms(1);

		if(LED_Count)	LED_Count--;
		else 
		{
			LED_Count = 1000;
			LED = ~LED;
		}  

		WDT_CONTR |= 0x10; 		//WatchDogClear

		if(COM1.RX_TimeOut > 0)		//超时计数
		{
			if(--COM1.RX_TimeOut == 0)
			{
				if(COM1.RX_Cnt > 0)
				{
//					if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == 'A' && RX1_Buffer[2] == 'F' && RX1_Buffer[3] == 'F')
//					{
//						for(i=0; i<COM1.RX_Cnt; i++)	TX1_write2buff(RX1_Buffer[i]);	//收到的数据原样返回
//					}
					if(RX1_Buffer[0] == 'A' || RX1_Buffer[3] == 'T')
					{
						if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == '1' && RX1_Buffer[2] == '1' && RX1_Buffer[3] == 'T')
						{								 							
							Lock_count = 400;
							LOCK = 0;
							TR0 = 1;	}
						else if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == '1' && RX1_Buffer[2] == '0' && RX1_Buffer[3] == 'T')
						{	
							LOCK = 1;	}
						else if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == '2' && RX1_Buffer[2] == '1' && RX1_Buffer[3] == 'T')
						{
							Card = 0;	}
						else if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == '2' && RX1_Buffer[2] == '0' && RX1_Buffer[3] == 'T')
						{
							Card = 1;	}
						else if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == '3' && RX1_Buffer[2] == '1' && RX1_Buffer[3] == 'T')
						{
							Keyboard = 0;	}
						else if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == '3' && RX1_Buffer[2] == '0' && RX1_Buffer[3] == 'T')
						{
							Keyboard = 1;	}
						else if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == '4' && RX1_Buffer[2] == '1' && RX1_Buffer[3] == 'T')
						{
							Identity = 0;	}
						else if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == '4' && RX1_Buffer[2] == '0' && RX1_Buffer[3] == 'T')
						{
							Identity = 1;	}
						else if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == '5' && RX1_Buffer[2] == '1' && RX1_Buffer[3] == 'T')
						{
							Reserve = 0;	}
						else if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == '5' && RX1_Buffer[2] == '0' && RX1_Buffer[3] == 'T')
						{
							Reserve = 1;	}													
					}
					
					for(i=0; i<COM1.RX_Cnt; i++)	TX1_write2buff(RX1_Buffer[i]);	//收到的数据原样返回
				}
				COM1.RX_Cnt = 0;
			}
		}
	}
 }
/***********************************************
函数名称：Delay_ms
功    能：STC15系列单片机1ms延时程序
入口参数：ms:延时的毫秒数
返 回 值：无	
备    注：示波器实测：0.997ms，内部时钟：11.0592MHz           
************************************************/
void Delay_ms(unsigned int ms)
{
  	unsigned int i;
  	while( (ms--) != 0)
   	{
    	for(i = 0; i < 580; i++); 
   	}             
}

/********************* Timer0中断函数************************/
void timer0_int (void) interrupt TIMER0_VECTOR
{
   if(Lock_count)
   {
   	Lock_count--;
   }
   else
   {
   	LOCK = 1;
		TR0 = 0;
   }      
}
/********************* Timer1中断函数************************/
void timer1_int (void) interrupt TIMER1_VECTOR
{

}

/********************* Timer2中断函数************************/
void timer2_int (void) interrupt TIMER2_VECTOR
{

}
