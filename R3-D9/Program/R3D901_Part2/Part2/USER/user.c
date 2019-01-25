#include "user.h"

#include "r3driver.h"
#include "members.h"


//********************************************************************************
//�û����� ����	 
//�޸�����:2018-10-22
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ��������(����)���ܻ����˿Ƽ����޹�˾ 
//All rights reserved
//********************************************************************************
//u32 count_second1 = 0;

//��ʱ��6�жϷ�����
void TIM6_DAC_IRQHandler(void)												//1s
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET)				//����ж�
	{
		if(Charge_Exit_Seconds.d && Robot_Status==CHARGING_STATUS)
		{
			Charge_Exit_Seconds.d--;												//���ʱ��-1s
		}
//		count_second1++;
		temp_fr_control = 1;															//����tcp����Ƶ��
		LED1 = !LED1;																			//LED��˸
	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);					//����жϱ�־λ
}
//
//* Description: ��ʱ��7�жϷ�����
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null��
void TIM7_IRQHandler(void)														//
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) 				//����ж�
	{
		int PULSE_RIGHT=0,PULSE_LEFT=0;										//dtʱ���ڵ�����������,������̼Ƽ���
		PULSE_LEFT = ENCODER_LEFT_TIMER->CNT - 32768;			//�������̲ɼ���ʱ�� TIM3---TIM3������ģʽGPIOB6 GPIOB7 ����
		ENCODER_LEFT_TIMER->CNT = 32768;
		PULSE_RIGHT = ENCODER_RIGHT_TIMER->CNT - 32768;		//�ҵ�����̲ɼ���ʱ�� TIM4---TIM4������ģʽGPIOC6 GPIOC7 �ҵ��
		ENCODER_RIGHT_TIMER->CNT = 32768;
//	PULSE_LEFT_HIS += PULSE_LEFT;
//	PULSE_RIGHT_HIS += PULSE_RIGHT;		
		odometry_cal(PULSE_LEFT,PULSE_RIGHT);							//������̼�		
	}
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  				//����жϱ�־λ
}
//* Description: ���ڻ��洦��
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null��
void uart1_deal(void)																//���ڻ��洦��
{
	u8 i,*uart_p;
	switch(UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][0])				//�ж�ָ������
	{
		case 0x01:																					//ִ��ָ��---������0x01
		{
			switch(UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][1])
			{
				case REG_MOVING_SPEED:													//�Ĵ���-0x21-�˶��ٶ�
				{
					uart_p = &UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][2];
					for(u8 t=0;t<4;t++)														//�����ڽ��ܵ����ٶȴ洢�ڽṹ����;
					{
						rightdata.data[t]=*(uart_p+t);
						leftdata.data[t]=*(uart_p+t+4);
					}
					if(Robot_Status == NAVI_STATUS	
						&& rightdata.d < 1000 
					&& leftdata.d < 1000)													//���ڵ���״̬&&���ٶȼ�������
						car_control(rightdata.d,leftdata.d);	 			//�����յ����������ٶȸ���С��;
				}break;
				case REG_SERVER_SEND:
				if(temp_fr_control)
				{
					uart_p = &UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][2];
					memcpy(SendBuff3,uart_p,*(uart_p+4));
					if(*(uart_p+4) < 200 && DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//�ȴ�DMA1_Steam4�������
					{
						//DMA_Send
						DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);		//���DMA1_Stream4������ɱ�־
						USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA����
						MYDMA_Enable(DMA1_Stream4,*(uart_p+4));  			//��ʼһ��DMA���䣡
						temp_fr_control = 0;
					}
				}break;
			}
		}break;
		case 0x02:																					//��ȡָ��---������0x02
		{
			switch(UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][1])
			{
				case REG_STATUS:																//�Ĵ���-0x01-����״̬
				{
					uart_p = &Robot_Status;
					while(uart1_format_build_and_send(uart_p,DEVICE_ID_D9,READ_COMMOND,REG_STATUS,_1byte));		//����Robot_Status��1���ֽ�							
				}break;
				case REG_ODOMETRY:															//�Ĵ���-0x10-��̼�
				{
					odom_send_task();
//							uart1_format_build_and_send(odometry_data,DEVICE_ID_D9,READ_COMMOND,REG_ODOMETRY,31);	
				}break;	
				case REG_BATTERY_VOLTAGE:												//�Ĵ���-0x11-������
				{
					i = battery_check();
					uart_p = &i;
					while(uart1_format_build_and_send(uart_p,DEVICE_ID_D9,READ_COMMOND,REG_BATTERY_VOLTAGE,_2byte));
				}break;	
				case REG_CHARGING_TIME:													//�Ĵ���-0x20-���ʱ��
				{
					while(uart1_format_build_and_send(Charge_Exit_Seconds.data,DEVICE_ID_D9,READ_COMMOND,REG_CHARGING_TIME,_2byte));
				}break;
				case REG_ERROR:																	//�Ĵ���-0xF1-������
				{
					uart_p = &Robot_Error;
					while(uart1_format_build_and_send(uart_p,DEVICE_ID_D9,READ_COMMOND,REG_ERROR,_1byte));		//����Robot_Error��1���ֽ�	
				}break;
				default:																				//δָ֪��
					break;
			}
		}break;
		case 0x03:																					//д��ָ��---������0x03
		{
			switch(UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][1])
			{
				case REG_CHARGING_TIME:													//�Ĵ���-0x20-���ʱ��
				{
					uart_p = &UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][2];
					Charge_Exit_Seconds.data[1] = *(uart_p+1);
					Charge_Exit_Seconds.data[0] = *(uart_p);
					if((Charge_Exit_Seconds.d > 5) && (Charge_Exit_Seconds.d < 60000))
					Robot_Status = FINDING_DOCK_STATUS;						//����״̬-�л�Ϊ-�ҵ�״̬							
					while(uart1_format_build_and_send(Charge_Exit_Seconds.data,DEVICE_ID_D9,WRITE_COMMOND,REG_CHARGING_TIME,_2byte));//����
				}break;
				case REG_SERVER_SEND:
				if(temp_fr_control)
				{
					uart_p = &UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][2];
					memcpy(SendBuff3,uart_p,*(uart_p+4));
					if(*(uart_p+4) < 200 && DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//�ȴ�DMA1_Steam4�������
					{
						//DMA_Send
						DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);		//���DMA1_Stream4������ɱ�־
						USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA����
						MYDMA_Enable(DMA1_Stream4,*(uart_p+4));  			//��ʼһ��DMA���䣡
						temp_fr_control = 0;
					}
				}break;
			}
		}break;				
	}UART1_RX_BUF_RECIVE--;															//������ȡ��ɡ��������
}

//* Description: �ڲ���ʼ��
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null��
void system_init_step1(void)														//�ڲ���ʼ��
{
	delay_init(168);       																//��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//����ϵͳ�ж����ȼ�����2
	BEEP_Init();																					//��������ʼ��
	uart_init();   																				//���ڳ�ʼ��
//	usmart_dev.init(84); 																	//��ʼ��USMART
	LED_Init();  																					//LED��ʼ��
	KEY_Init();  																					//������ʼ��
	LCD_Init(); 																					//LCD��ʼ��
	Remote_Init();																				//������ճ�ʼ��	
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN��ʼ����ͨģʽ,������500Kbps	
	ENC_Init_Left();																			//���õ��A	TIM4������ģʽPB6 PB7 ���� 
	ENC_Init_Right();																			//���õ��B	TIM3������ģʽPC6 PC7 �ҵ��
	POINT_COLOR = BLACK; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_1,300,16,16,"System Init Step1......");
//	FSMC_SRAM_Init();																		//��ʼ���ⲿSRAM
//	My_RTC_Init();  																		//RTC��ʼ��
	Adc_Init(GPIOA,GPIO_Pin_6);         									//��ʼ��ADC-GPIOA-6
	Get_Adc_Average(ADC_Channel_6,20);										//��ȡͨ��5��ת��ֵ��20��ȡƽ��
//	Adc_Temperate_Init(); 																//�ڲ��¶ȴ�������ʼ��
//	TIM3_Int_Init(999,839); 															//��ʱ��ʱ��84M����Ƶϵ��840������84M/840=100Khz�ļ���Ƶ�ʣ�����100��Ϊ1ms 
	TIM6_Int_Init(10000-1,8400-1); 												//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����10000��Ϊ1s
	TIM7_Int_Init(100-1,8400-1); 													//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����100��Ϊ10ms
//	mymem_init(SRAMIN);																		//��ʼ���ڲ��ڴ��
//	mymem_init(SRAMEX);																		//��ʼ���ⲿ�ڴ��
//	mymem_init(SRAMCCM);																	//��ʼ��CCM�ڴ��

	Charge_Exit_Seconds.d = 20;														//20s�ĳ��ʱ��
	Moving_Seconds = 250;																	//�������ʱ��---n*10ms

	delay_ms(1000);																				//�ϵ�ȴ�����������

	Robot_Status = NAVI_STATUS;														//�����˿���Ĭ��Ϊ����״̬
	/*********ʹ�ܴ���1��DMA����********************/
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  				//ʹ�ܴ���1��DMA����
	MYDMA_Enable(DMA2_Stream7,11+1);     									//��ʼһ��DMA����	
	/*********ʹ�ܴ���3��DMA����********************/	
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  				//ʹ�ܴ���3��DMA����
	MYDMA_Enable(DMA1_Stream4,39);     										//��ʼһ��DMA����	
	motor_init_task();																		//��������ʼ��	
	POINT_COLOR = BLACK; 		/*��ɫ����*/	LCD_ShowString(30,LED_ROW_2,300,16,16,"System Init Step1 Succeed...");
	BEEP = 1;delay_ms(200);BEEP = 0;delay_ms(2000);
}
//* Description: �˶����ֳ�ʼ��
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null��
void system_init_step2(void)														//�˶����ֳ�ʼ��
{	
//		LCD_ShowxNum(30,150,count_second1,10,16,0X80);//��ʾ��IP
//	while(1)
//	{
//		delay_ms(100);
//	}
	POINT_COLOR = BLACK; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_3,300,16,16,"System Init Step2......");	
	//motor check step1 start
	POINT_COLOR = BLUE; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_4,300,16,16,"Motor Check Step1......");
	position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;		/*�����̼�*/	
	Robot_Moving(100.0,-100.0,200);	Robot_Moving(0,0,50);		//��ת��ֹͣ		
	Robot_Moving(-100.0,100.0,200);	Robot_Moving(0,0,50);		//��ת��ֹͣ	
	if(position_x>5.1f || position_y>5.1f || oriention>0.1f){
		POINT_COLOR = RED; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_5,300,16,16,"Motor Check Step1 Fail......");	
		while(1){BEEP = !BEEP;delay_ms(500);}
	}else{
		POINT_COLOR = GREEN; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_5,300,16,16,"Motor Check Step1 Succeed......");
	}
	//motor check step2 start//ǰ������ж�
	POINT_COLOR = BLUE; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_6,300,16,16,"Motor Check Step2......");
	position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;		/*�����̼�*/		delay_ms(1000);	
	Robot_Moving(100.0,100.0,100);	Robot_Moving(0,0,50);			//ǰ����ֹͣ
 	if(!position_x || position_x <50.0f){
		POINT_COLOR = RED; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_7,300,16,16,"Motor Check Step2 Fail......");	
		while(1){BEEP = !BEEP;delay_ms(500);}
	}else{
		POINT_COLOR = GREEN; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_7,300,16,16,"Motor Check Step2 Succeed......");	
		Robot_Moving(-100.0,-100.0,100);	Robot_Moving(0,0,50);	//���ˣ�ֹͣ
	}
	//motor check step3 start//��ת����ж�
	POINT_COLOR = BLUE; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_8,300,16,16,"Motor Check Step3......");
	position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;		/*�����̼�*/		delay_ms(1000);	
	Robot_Moving(-100.0,100.0,200);	Robot_Moving(0,0,50);	//��ת��ֹͣ
	if(!oriention || oriention<0.5f){				//��ת�Ƕȴ���0С��0.5
		POINT_COLOR = RED; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_9,300,16,16,"Motor Check Step3 Fail......");	
		while(1){BEEP = !BEEP;delay_ms(500);}
	}
	else{
		POINT_COLOR = GREEN; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_9,300,16,16,"Motor Check Step3 Succeed......");
		Robot_Moving(100.0,-100.0,200);	Robot_Moving(0,0,50);//��ת��ֹͣ
	}
	position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;		/*�����̼�*/		delay_ms(1000);	
	POINT_COLOR = BLACK; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_10,300,16,16,"System Init Step2 Succeed...");	
	BEEP = 1;delay_ms(200);BEEP = 0;delay_ms(2000);
}
//* Description: ������ʼ��
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null��
void system_init_step3(void)														//������ʼ��
{
	POINT_COLOR = BLACK; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_11,300,16,16,"System Init Step3......");	
	POINT_COLOR = BLACK; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_12,300,16,16,"System Init Step3 Succeed...");	
	BEEP = 1;delay_ms(200);BEEP = 0;delay_ms(2000);
}
//* Description: ����ʼ��
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null��
void system_init_step4(void)														//����ʼ��
{
	POINT_COLOR = BLACK; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_13,300,16,16,"System Init Step4......");
	Robot_Status = FINDING_DOCK_STATUS;										//�����˿���Ĭ��ΪѰ��ģʽ	
	while(1)										
	{
		auto_charge_task();
		exit_charge_task();
		if(Robot_Status == NAVI_STATUS)	
			break;																						//�ȴ����������л�Ϊ����״̬
	}
	POINT_COLOR = BLACK; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_14,300,16,16,"System Init Step4 Succeed...");	
}

//* Description: ͨѶ��ʼ��
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null��
void system_init_step5(void)														//ͨѶ��ʼ��
{
	POINT_COLOR = BLACK; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_15,300,16,16,"System Init Step5......");	
	
	while(1)												//δ���յ�ͨѶģ��ͨѶ��Ӧ
	{
		POINT_COLOR = RED; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_13,200,16,16,"Connect Server Fail......");
		if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//�ȴ�DMA1_Stream4�������	
		{
			DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);		//���DMA1_Stream4������ɱ�־
			USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA����
			MYDMA_Enable(DMA1_Stream4,39);  			//��ʼһ��DMA���䣡			
		}
//		BEEP = !BEEP;		
		delay_ms(3000);
	}
	POINT_COLOR = GREEN; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_17,300,16,16,"Connect Server Succeed......");
	
	while(!(UART1_RX_BUF_RECIVE))													//δ���յ�����ģ��ͨѶ��Ӧ
	{
		POINT_COLOR = RED; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_16,300,16,16,"Connect Navigation Fail......");
		if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������	
		{
			DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);				//���DMA2_Steam7������ɱ�־
			USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  		//ʹ�ܴ���1��DMA����
			MYDMA_Enable(DMA2_Stream7,11+1);     							//��ʼһ��DMA����
		}
		BEEP = !BEEP;		delay_ms(300);
	}UART1_RX_BUF_RECIVE--;																//��UART1_RX_BUF_RECIVE������ͷ-1
	POINT_COLOR = GREEN; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_16,300,16,16,"Connect Navigation Succeed......");
		
	POINT_COLOR = BLACK; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_18,300,16,16,"System Init Step5 Succeed...");
}

//* Description: Remote Get.
//* Arguments  : NULL
//* Note(s)    : Null.
u8 remote_get(void)
{
	u8 i;
	i=Remote_Scan();
	if(i == 104)
		return IR_LEFT;
	else if(i == 152)
		return IR_MIDDLE;
	else if(i == 176)
		return IR_RIGHT;
	else
		return 0;
}
//* Description: �Զ��������;
//* Arguments  : NULL
//* Note(s)    :  Null.
void auto_charge_task(void)
{
	u8 k = 1,ir_pos;
	
	if(_If_Charging == TRUE)											//���ϵ���
	{
		Robot_Moving(0,0,1);												//ֹͣ
		delay_ms(500);															//��ʱ500ms���Ȼ������ȶ�
		if(_If_Charging == TRUE)										//���ϵ���
		{
			Robot_Status = CHARGING_STATUS;						//����״̬-�л�Ϊ-���״̬
		}
	}
	else if(_If_Charging==FALSE)									//��δ�Ӵ����׮
	{
		ir_pos=remote_get();												//��ȡ����λ���ź�
		if(ir_pos==0)																//����λ���źš���Ϊ��
		{
			IR_No_Find_Count++;												//�������źż���+1
			if(IR_No_Find_Count >= 150)								//�����޺����źż��� > 150
			{
				IR_No_Find_Count = 0;										//��պ������źż���
				Circle_Moving_Count++;									//ԭ����ת��������+1
				Robot_Moving(-20.00*k,20.00*k,1);				//��������ʱ����ת
			}
		}
		else																	 			//�к���λ���ź�
		{
			Circle_Moving_Count = NULL;								//���ԭ����ת��������
			IR_No_Find_Count = NULL;									//���ԭ����ת��������
			switch(ir_pos)														//���ֺ����źš�������Ӧ����
			{
				case IR_LEFT:
					Robot_Moving(-40.00*k,-20.00*k,1);		//�����������һ�ξ���-���ֿ�-��ʱ��
					break;
				case IR_MIDDLE:
					Robot_Moving(-20.00*k,-20.00*k,1);		//�����������һ�ξ���
					break;
				case IR_RIGHT:
					Robot_Moving(-20.00*k,-40.00*k,1);		//�����������һ�ξ���-���ֿ�-˳ʱ��
					break;
				default:															
					break;
			}
		}
		/**********��תѰ���źų����趨ֵ*********/
		if(Circle_Moving_Count >= MAX_Circle_Moving_Count)	//ԭ����ת���� > ��תѰ���ź�����������Դ���
		{
			Circle_Moving_Count = 0;												//���ԭ����ת��������
//				Charge_Exit_Seconds.d = NULL;											//��ճ��ʱ��	
//				Robot_Status = ERROT_STATUS;											//����״̬-�л�Ϊ-�쳣״̬
			Robot_Status = NAVI_STATUS;											//����״̬-�л�Ϊ-����״̬
			Robot_Error = CHARGE_FALSE;											//�쳣����-��������-CHARGE_FALSE
			return;																					//�Զ����ʧ�ܡ����������γ��	
		}
		/**********��ʱ*********/
		delay_ms(50); 		//��ʱ10ms
	}
}
//* Description: �Զ��������;
//* Arguments  : NULL
//* Note(s)    : Null. 
void exit_charge_task(void)
{
	/**********����ʱ�������������׮�����е������*********/
	if(Robot_Status==CHARGING_STATUS && !Charge_Exit_Seconds.d)			//���״̬&&���ʱ��ľ�
	{
		Robot_Moving(150.00,150.00,Moving_Seconds);										//��������ǰ����һ�ξ���
		Robot_Moving(0,0,10);																					//ֹͣ
		Robot_Status = NAVI_STATUS;																		//����״̬-�л�Ϊ-����״̬
		return;
		
	}		
}
//* Description: ��̼������������odometry_data
//* Arguments  : Null
//* Note(s)    : 1) Null��
void odom_send_task(void)
{
	//���͸����ڵ���̼���������---Ԥ����2���ĳ��ȷ�ֹ�쳣�����Ӧ���ò��ϣ�		//Odom_Get
	x_data.odoemtry_float=position_x;//��λmm		
	y_data.odoemtry_float=position_y;//��λmm
	theta_data.odoemtry_float=oriention;//��λrad
	vel_linear.odoemtry_float=velocity_linear;//��λmm/s
	vel_angular.odoemtry_float=velocity_angular;//��λrad/s
	//��������̼����ݴ浽Ҫ���͵�����
	for(u8 j=0;j<4;j++)
	{
		odometry_data[j]=x_data.odometry_char[j];
		odometry_data[j+4]=y_data.odometry_char[j];
		odometry_data[j+8]=theta_data.odometry_char[j];
		odometry_data[j+12]=vel_linear.odometry_char[j];
		odometry_data[j+16]=vel_angular.odometry_char[j];			
	}
	while(uart1_format_build_and_send(odometry_data,DEVICE_ID_D9,COM_COMMOND,REG_ODOMETRY,31));	
}

//* Description: ���ʣ���������
//* Arguments  : Null.
//* Note(s)    : 1) Null
u8 battery_check(void)
{	
	u16 battery_adc = 0;																//������ѹ��Χ��0-4096��
	float battery_vol = 0;															//������ѹֵ
	u8 battery_p = 0;																		//��ѹ�ٷֱ�
	battery_adc=Get_Adc_Average(ADC_Channel_6,20);			//��ȡͨ��6��ת��ֵ��20��ȡƽ��
	battery_vol = (float)((battery_adc*3.3f)/4096);			//��ȡ�����Ĵ�С����ʵ�ʵ�ѹֵ������3.1111
	if(battery_vol > 2.64f)															//��������ѹ
		battery_vol = 2.64f;					
	else	if(battery_vol < 2.26f)												//������С��ѹ
		battery_vol = 2.26f;
	
	battery_p = (u8)(258.75f * battery_vol - 584.66f);
	
	return battery_p;	
}
//
//
//

//********************************************************************************
//�������߼� ����	 
//�޸�����:2018-10-22
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ��������(����)���ܻ����˿Ƽ����޹�˾ 
//All rights reserved
//********************************************************************************
extern u8 UART1_RX_BUF[];
int main(void)
{ 
//	u8 i;
//	u8 main_sta;
//	u8 key;	
	system_init_step1();
	if(Program_Check)
	{
//		system_init_step2();
//		system_init_step3();
//		system_init_step4();
		system_init_step5();
	}

	while(1)
	{
		/*********���ջ�����������***************/
		if(UART1_RX_BUF_RECIVE)
		{
			uart1_deal();
		}
		/*********ȷ������״̬==�ҵ�״̬*********/
		if(Robot_Status == FINDING_DOCK_STATUS)
		{
			auto_charge_task();
		}
		
		/*********ȷ������״̬==���״̬*********/
		if(Robot_Status == CHARGING_STATUS)
		{
			exit_charge_task();
		}
		
		/*********ȷ������״̬==����״̬*********/
		if(Robot_Status == NAVI_STATUS)
		{
			;
		}
		
		odom_send_task();				//��̼Ʒ���
		delay_ms(10);
//		battery_check();
		
	}	
}




