
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	unsigned char Res;
#if SYSTEM_SUPPORT_OS  //使用UCOS操作系统
	OSIntEnter();    
	OS_ERR err;
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收到一个字节
	{
		if(USART_RX_LEN > (USART_REC_LEN-1))	USART_RX_LEN = 0;	//防止数组溢出、一帧数据最大为254字节.
		/***数据接收存储***/
		Res = USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		USART_RX_BUF[USART_RX_LEN++]=	Res;
//		/***帧头判断***/
//		if(Res == 0xA6 && USART_RX_LEN == 1 && End_Status == 0x00)
//			End_Status = 0x01;
//		else
//			Clear_Uart1_Recive_Status
//		if(End_Status == 0x01)
//		{
//			if(Res == 0x59 && USART_RX_LEN == 0)
//				End_Status = 0x02;
//			else
//				Clear_Uart1_Recive_Status
//		}
//		/***帧尾判断***/
//		if(Res == 0x54 && End_Status == 0x04)
//			End_Status = 0x05;
//		else
//			Clear_Uart1_Recive_Status
//		if(Res == 0x0A && End_Status == 0x03)
//			End_Status = 0x04;
//		else
//			Clear_Uart1_Recive_Status
//		
//		if(USART_RX_LEN > 10)
//		{
//			if(Res == 0x0D && End_Status == 0x02 && USART_RX_LEN > 10)
//				End_Status = 0x03;
//			else
//				Clear_Uart1_Recive_Status
//		}		
		switch(Res)
		{
			case 0x59:
				if(End_Status == 0)
					End_Status = 0x01;
				else
					End_Status = 0;
				break;				
			case 0x48:
				if(End_Status == 0x01)
					End_Status = 0x02;
				else
					End_Status = 0;
				break;
			case 0x54:
				if(End_Status == 0x02)
					End_Status = 0x03;
				else
					End_Status = 0;
				break;
			default:
				End_Status = 0;
				break;
		}
		
		if(End_Status == 0x03)
		{
			End_Status = 0;
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