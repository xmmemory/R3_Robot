
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	unsigned char Res;
#if SYSTEM_SUPPORT_OS  //ʹ��UCOS����ϵͳ
	OSIntEnter();    
	OS_ERR err;
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //���յ�һ���ֽ�
	{
		if(USART_RX_LEN > (USART_REC_LEN-1))	USART_RX_LEN = 0;	//��ֹ���������һ֡�������Ϊ254�ֽ�.
		/***���ݽ��մ洢***/
		Res = USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		USART_RX_BUF[USART_RX_LEN++]=	Res;
//		/***֡ͷ�ж�***/
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
//		/***֡β�ж�***/
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