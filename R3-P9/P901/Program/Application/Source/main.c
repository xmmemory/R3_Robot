/************************************************
 * �ļ�����     : ����2---��Դ���ư�
 * �����ļ�     : main.c               
 * ��    ��     : 2.0
 * ��    ��     : y
 * ��    ��     : 2018.04.18
 * о    Ƭ     : stc15w204s
 * ʱ    ��		���ڲ�11.0592MHz
 * ��������		��Keil uVision4 C51 V9.06
 ************************************************/
 //�ļ�����
 #include	"config.h"
 #include	"GPIO.h"
 #include	"USART1.h"
 #include	"timer.h"

 //�궨��
// #define   CHARTAST     1		//�ַ�����
// #define   NUMTAST    	2		//��������
// #define   MENUTAST     3		//�˵�����
 
 //����λ����
 sbit	BEEP=P1^0;			//����BEEP
 sbit LED = P1^1;			//����LED
 sbit	LOCK=P3^2;			//����LOCK
 sbit	Card=P1^2;			//����OUT1
 sbit	Keyboard=P1^3;		//����OUT2
 sbit	Identity=P1^4;		//����OUT3
 sbit	Reserve=P1^5;		//����OUT4
 sbit	OUT3=P1^6;			//����OUT3
 sbit	OUT4=P1^7;			//����OUT4
 
 //ȫ�ֱ���
 bit HaveTast=0;			    //������
 unsigned char Uart2Cmd;		//��������

 unsigned int Lock_count;
 unsigned int LED_Count;

 //��������
 void Delay_ms(unsigned int ms);

 /******************** IO���ú��� **************************/
 void	GPIO_config(void)
 { 
	GPIO_InitTypeDef	GPIO_InitStructure;		//�ṹ����
	GPIO_InitStructure.Pin  = GPIO_Pin_All;		//ָ��Ҫ��ʼ����IO, GPIO_Pin_0 ~ GPIO_Pin_7, �����
	GPIO_InitStructure.Mode = GPIO_PullUp;		//ָ��IO������������ʽ,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	//��ʼ��P1

	GPIO_InitStructure.Pin  = GPIO_Pin_0;		//ָ��Ҫ��ʼ����IO, GPIO_Pin_0 ~ GPIO_Pin_7, �����
	GPIO_InitStructure.Mode = GPIO_OUT_PP;		//ָ��IO������������ʽ,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P1,&GPIO_InitStructure);	//��ʼ������BEEP
	
	GPIO_InitStructure.Pin  = GPIO_Pin_All;		//ָ��Ҫ��ʼ����IO, GPIO_Pin_0 ~ GPIO_Pin_7, �����
	GPIO_InitStructure.Mode = GPIO_PullUp;		//ָ��IO������������ʽ,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P3,&GPIO_InitStructure);	//��ʼ������LOCK

	GPIO_InitStructure.Pin  = GPIO_Pin_All;		//ָ��Ҫ��ʼ����IO, GPIO_Pin_0 ~ GPIO_Pin_7, �����
	GPIO_InitStructure.Mode = GPIO_PullUp;		//ָ��IO������������ʽ,GPIO_PullUp,GPIO_HighZ,GPIO_OUT_OD,GPIO_OUT_PP
	GPIO_Inilize(GPIO_P5,&GPIO_InitStructure);	//��ʼ������LOCK
 }
 /*************  ����1��ʼ������ *****************/
void	UART_config(void)
{
	COMx_InitDefine		COMx_InitStructure;					//�ṹ����
	COMx_InitStructure.UART_Mode      = UART_8bit_BRTx;		//ģʽ,       UART_ShiftRight,UART_8bit_BRTx,UART_9bit,UART_9bit_BRTx
	COMx_InitStructure.UART_BRT_Use   = BRT_Timer2;			//ʹ�ò�����,   BRT_Timer1, BRT_Timer2 (ע��: ����2�̶�ʹ��BRT_Timer2)
	COMx_InitStructure.UART_BaudRate  = 9600ul;			//������, һ�� 110 ~ 115200
	COMx_InitStructure.UART_RxEnable  = ENABLE;				//��������,   ENABLE��DISABLE
	COMx_InitStructure.BaudRateDouble = DISABLE;			//�����ʼӱ�, ENABLE��DISABLE
	COMx_InitStructure.UART_Interrupt = ENABLE;				//�ж�����,   ENABLE��DISABLE
	COMx_InitStructure.UART_Polity    = PolityLow;			//�ж����ȼ�, PolityLow,PolityHigh
	COMx_InitStructure.UART_P_SW      = UART1_SW_P36_P37;	//�л��˿�,   UART1_SW_P30_P31,UART1_SW_P36_P37,UART1_SW_P16_P17(����ʹ���ڲ�ʱ��)
	COMx_InitStructure.UART_RXD_TXD_Short = DISABLE;		//�ڲ���·RXD��TXD, ���м�, ENABLE,DISABLE
	USART_Configuration(USART1, &COMx_InitStructure);		//��ʼ������1 USART1,USART2

	PrintString1("RAY_IOBD_L63130_START!\r\n");	//SUART1����һ���ַ���
}
/************************ ��ʱ������ ****************************/
void	Timer_config(void)
{
	TIM_InitTypeDef		TIM_InitStructure;					//�ṹ����
	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
	TIM_InitStructure.TIM_Polity    = PolityLow;			//ָ���ж����ȼ�, PolityHigh,PolityLow
	TIM_InitStructure.TIM_Interrupt = ENABLE;				//�ж��Ƿ�����,   ENABLE��DISABLE
	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;		//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / (50*12));		//��ֵ	  ---20ms
	TIM_InitStructure.TIM_Run       = ENABLE;				//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
	Timer_Inilize(Timer0,&TIM_InitStructure);				//��ʼ��Timer0	  Timer0,Timer1,Timer2

//	TIM_InitStructure.TIM_Mode      = TIM_16BitAutoReload;	//ָ������ģʽ,   TIM_16BitAutoReload,TIM_16Bit,TIM_8BitAutoReload,TIM_16BitAutoReloadNoMask
//	TIM_InitStructure.TIM_Polity    = PolityLow;			//ָ���ж����ȼ�, PolityHigh,PolityLow
//	TIM_InitStructure.TIM_Interrupt = ENABLE;				//�ж��Ƿ�����,   ENABLE��DISABLE
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;		//ָ��ʱ��Դ, TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
//	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / (50*12));		//��ֵ
//	TIM_InitStructure.TIM_Run       = ENABLE;				//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
//	Timer_Inilize(Timer1,&TIM_InitStructure);				//��ʼ��Timer1	  Timer0,Timer1,Timer2
//
//	TIM_InitStructure.TIM_Interrupt = ENABLE;				//�ж��Ƿ�����,   ENABLE��DISABLE. (ע��: Timer2�̶�Ϊ16λ�Զ���װ, �жϹ̶�Ϊ�����ȼ�)
//	TIM_InitStructure.TIM_ClkSource = TIM_CLOCK_12T;		//ָ��ʱ��Դ,     TIM_CLOCK_1T,TIM_CLOCK_12T,TIM_CLOCK_Ext
//	TIM_InitStructure.TIM_ClkOut    = DISABLE;				//�Ƿ������������, ENABLE��DISABLE
//	TIM_InitStructure.TIM_Value     = 65536UL - (MAIN_Fosc / (50*6*5));		//��ֵ	 ---50ms
//	TIM_InitStructure.TIM_Run       = ENABLE;				//�Ƿ��ʼ����������ʱ��, ENABLE��DISABLE
//	Timer_Inilize(Timer2,&TIM_InitStructure);				//��ʼ��Timer2	  Timer0,Timer1,Timer2
}

 /******************** ������ **************************/
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

		if(COM1.RX_TimeOut > 0)		//��ʱ����
		{
			if(--COM1.RX_TimeOut == 0)
			{
				if(COM1.RX_Cnt > 0)
				{
//					if(RX1_Buffer[0] == 'A' && RX1_Buffer[1] == 'A' && RX1_Buffer[2] == 'F' && RX1_Buffer[3] == 'F')
//					{
//						for(i=0; i<COM1.RX_Cnt; i++)	TX1_write2buff(RX1_Buffer[i]);	//�յ�������ԭ������
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
					
					for(i=0; i<COM1.RX_Cnt; i++)	TX1_write2buff(RX1_Buffer[i]);	//�յ�������ԭ������
				}
				COM1.RX_Cnt = 0;
			}
		}
	}
 }
/***********************************************
�������ƣ�Delay_ms
��    �ܣ�STC15ϵ�е�Ƭ��1ms��ʱ����
��ڲ�����ms:��ʱ�ĺ�����
�� �� ֵ����	
��    ע��ʾ����ʵ�⣺0.997ms���ڲ�ʱ�ӣ�11.0592MHz           
************************************************/
void Delay_ms(unsigned int ms)
{
  	unsigned int i;
  	while( (ms--) != 0)
   	{
    	for(i = 0; i < 580; i++); 
   	}             
}

/********************* Timer0�жϺ���************************/
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
/********************* Timer1�жϺ���************************/
void timer1_int (void) interrupt TIMER1_VECTOR
{

}

/********************* Timer2�жϺ���************************/
void timer2_int (void) interrupt TIMER2_VECTOR
{

}
