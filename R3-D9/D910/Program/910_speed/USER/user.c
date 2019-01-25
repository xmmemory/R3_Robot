#include "user.h"

#include "r3driver.h"
#include "members.h"

static int_2byte Charge_Exit_Seconds,quantity_of_electric;							//���ʱ��,���ʣ���������
static u8 battert_check_flag,robot_status_flag,odom_upload_flag;				//�������ͱ�־λ,������״̬���ͱ�־λ
static u8 Reboot_Enable;																								//����ʹ��
static SensorToServer_TypeDef SensorToServer_Struct;										//�ϴ��������ṹ��
//********************************************************************************
//�û����� ����	 
//�޸�����:2018-10-22
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ��������(����)���ܻ����˿Ƽ����޹�˾ 
//All rights reserved
//********************************************************************************
//u32 count_second1 = 0;

void TIM6_DAC_IRQHandler(void)		//* Description:  ��ʱ��6�жϷ�����
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET)						//����ж�
	{
		if(_One_Second_Count)
			_One_Second_Count--;
		else															 //1s
		{
			if(Charge_Exit_Seconds.d && Robot_Status==CHARGING_STATUS)
				Charge_Exit_Seconds.d--;			//���ʱ��-1s
			if(battert_check_flag)					//����������
				battert_check_flag--;
			if(robot_status_flag)						//״̬���ͼ���
				robot_status_flag--;
			_One_Second_Count = 1;					//1s������		
		}
		temp_fr_control = 1;							//����tcp����Ƶ��	
		LED1 = !LED1;											//LED��˸
		if(Speed_Clear_Count)
			Speed_Clear_Count--;						//�ٶ�n*500ms���㡢����ͨѶ�жϵ����¹�
	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);							//����жϱ�־λ
}
//
void TIM7_IRQHandler(void)				//* Description: ��ʱ��7�жϷ�����
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) 				//����ж�
	{
		int PULSE_RIGHT=0,PULSE_LEFT=0;										//dtʱ���ڵ�����������,������̼Ƽ���
		
		PULSE_LEFT = ENCODER_RIGHT_TIMER->CNT - 32768;		//�ҵ�����̲ɼ���ʱ�� TIM4---TIM4������ģʽGPIOC6 GPIOC7 �ҵ��
		ENCODER_RIGHT_TIMER->CNT = 32768;	
		
		PULSE_RIGHT = ENCODER_LEFT_TIMER->CNT - 32768;			//�������̲ɼ���ʱ�� TIM3---TIM3������ģʽGPIOB6 GPIOB7 ����
		ENCODER_LEFT_TIMER->CNT = 32768;
		
//		position_x1 = PULSE_LEFT * 1000;
//		position_y1 = PULSE_RIGHT * 1000;
		
		//������̼�
		odometry_cal(PULSE_LEFT,PULSE_RIGHT);
	}
	
	odom_upload_flag = 1;							//������̼��ϴ�Ƶ��	
	
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  				//����жϱ�־λ
}
//
void uart1_deal(void)						//* Description: ���ڻ��洦��
{
	u8 *uart_p;
	uart_p = NULL;							//��ָ��ָ��հ״�
	if(get_uart1_data > 0)
	switch(UART1_RX_BUFF[0])				//�ж�ָ������
	{
		case 0x01:																//ִ��ָ��---������0x01
		{
			switch(UART1_RX_BUFF[1])
			{
				case REG_MOVING_SPEED:											//�Ĵ���-0x21-�˶��ٶ�
				{
					uart_p = &UART1_RX_BUFF[2];
					for(u8 t=0;t<4;t++)											//�����ڽ��ܵ����ٶȴ洢�ڽṹ����;
					{
						linear_vel.data[t]=*(uart_p+t);
						angular_vel.data[t]=*(uart_p+t+4);
					}
					Speed_Clear_Count = 2;
//					if(Robot_Status == NAVI_STATUS	&& linear_vel.d < 500.0f)										//���ڵ���״̬&&���ٶȼ�������
//						speed_convert(linear_vel.d,angular_vel.d);	 															//�����յ����������ٶȸ���С��;
				}break;
				case REG_REAL_ODOM:												//�Ĵ���-0x33-ʵ��ODOM
				{
					uart_p = &UART1_RX_BUFF[2];
					for(u8 t=0;t<8;t++)											//�����ڽ��ܵ����ٶȴ洢�ڽṹ����;
					{
						SensorToServer_Struct.data[t+4+2+1]=*(uart_p+t);
						SensorToServer_Struct.data[t+4+2+1+8]=*(uart_p+t+8);
						SensorToServer_Struct.data[t+4+2+1+16]=*(uart_p+t+16);
					}
				}break;				
			}
		}break;
		case 0x02:																//��ȡָ��---������0x02
		{
			switch(UART1_RX_BUFF[1])
			{
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
			switch(UART1_RX_BUFF[1])
			{
				case REG_CHARGING_TIME:													//�Ĵ���-0x20-���ʱ��
				{
					uart_p = &UART1_RX_BUFF[2];
					Charge_Exit_Seconds.data[1] = *(uart_p+1);
					Charge_Exit_Seconds.data[0] = *(uart_p);
					if((Charge_Exit_Seconds.d > 5) && (Charge_Exit_Seconds.d < 60000))
					{
						Reboot_Enable = TRUE;
						Robot_Status = FINDING_DOCK_STATUS;						//����״̬-�л�Ϊ-�ҵ�״̬			
					}
					while(uart1_format_build_and_send(Charge_Exit_Seconds.data,DEVICE_ID_D9,WRITE_COMMOND,REG_CHARGING_TIME,_2byte));//����
				}break;
			}
		}break;
	}
}
//
void system_init(void)						//* Description: �ڲ���ʼ��
{
	delay_init(168);       										//��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//����ϵͳ�ж����ȼ�����2
	BEEP_Init();												//��������ʼ��
	uart_init();   												//���ڳ�ʼ��
//	usmart_dev.init(84); 										//��ʼ��USMART
	LED_Init();  												//LED��ʼ��
	KEY_Init();  												//������ʼ��
	LCD_Init(); 												//LCD��ʼ��
	Remote_Init();												//������ճ�ʼ��	
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,
	CAN_BS1_7tq,6,CAN_Mode_Normal);								//CAN��ʼ����ͨģʽ,������500Kbps	
	ENC_Init_Left();											//���õ��A	TIM4������ģʽPB6 PB7 ���� 
	ENC_Init_Right();											//���õ��B	TIM3������ģʽPC6 PC7 �ҵ��
//	FSMC_SRAM_Init();												//��ʼ���ⲿSRAM
//	My_RTC_Init();  												//RTC��ʼ��
	Adc_Init(GPIOA,GPIO_Pin_6);         						//��ʼ��ADC-GPIOA-6
	Get_Adc_Average(ADC_Channel_6,20);							//��ȡͨ��5��ת��ֵ��20��ȡƽ��
//	Adc_Temperate_Init(); 											//�ڲ��¶ȴ�������ʼ��
//	TIM3_Int_Init(999,839); 										//��ʱ��ʱ��84M����Ƶϵ��840������84M/840=100Khz�ļ���Ƶ�ʣ�����100��Ϊ1ms 
	TIM6_Int_Init(5000-1,8400-1); 							//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����5000��Ϊ500ms
	TIM7_Int_Init(100-1,8400-1); 								//��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����100��Ϊ10ms
//	mymem_init(SRAMIN);												//��ʼ���ڲ��ڴ��
//	mymem_init(SRAMEX);												//��ʼ���ⲿ�ڴ��
//	mymem_init(SRAMCCM);											//��ʼ��CCM�ڴ��
	Robot_Status = NAVI_STATUS;									//�����˿���Ĭ��Ϊ����״̬
	/*********ʹ�ܴ���1��DMA����********************/
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  				//ʹ�ܴ���1��DMA����
	MYDMA_Enable(DMA2_Stream7,11+1);     						//��ʼһ��DMA����	
	/*********ʹ�ܴ���3��DMA����********************/	
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  				//ʹ�ܴ���3��DMA����
	MYDMA_Enable(DMA1_Stream4,39);     							//��ʼһ��DMA����
	
	delay_ms(1000);												//�ϵ�ȴ�����������
	motor_init_task();										//��������ʼ��	
	
	BEEP = 1;delay_ms(50);BEEP = 0;delay_ms(1000);
}
//
void system_init1(void)					//* Description: �˶����ֳ�ʼ��
{	
	//motor check step1 start
	odometry_clear();				/*�����̼�*/
	Robot_Moving(100.0,-100.0,200);	Robot_Moving(0,0,50);		//��ת��ֹͣ		
	Robot_Moving(-100.0,100.0,200);	Robot_Moving(0,0,50);		//��ת��ֹͣ	
	if(position_x>5.1f || position_y>5.1f || oriention>0.1f){
		while(1){BEEP = !BEEP;delay_ms(500);}
	}
	//motor check step2 start//ǰ������ж�
	odometry_clear();				/*�����̼�*/		delay_ms(1000);	
	Robot_Moving(100.0,100.0,100);	Robot_Moving(0,0,50);			//ǰ����ֹͣ
 	if(!position_x || position_x <50.0f){
		while(1){BEEP = !BEEP;delay_ms(500);}
	}else{
		Robot_Moving(-100.0,-100.0,100);	Robot_Moving(0,0,50);	//���ˣ�ֹͣ
	}
	//motor check step3 start//��ת����ж�
	odometry_clear();				/*�����̼�*/		delay_ms(1000);	
	Robot_Moving(-100.0,100.0,200);	Robot_Moving(0,0,50);	//��ת��ֹͣ
	if(!oriention || oriention<0.5f){				//��ת�Ƕȴ���0С��0.5
		while(1){BEEP = !BEEP;delay_ms(500);}
	}
	else{
		Robot_Moving(100.0,-100.0,200);	Robot_Moving(0,0,50);//��ת��ֹͣ
	}
	odometry_clear();				/*�����̼�*/		delay_ms(1000);	
	BEEP = 1;delay_ms(200);BEEP = 0;delay_ms(2000);
}

//
void system_init2(void)					//* Description: ����ʼ��
{
	POINT_COLOR = BLACK; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_13,300,16,16,"System Init Step4......");
	Charge_Exit_Seconds.d = 10;												//10s�ĳ��ʱ��
	Robot_Status = FINDING_DOCK_STATUS;										//����������ΪѰ��ģʽ	
	while(1)										
	{
		auto_charge_task();
		exit_charge_task();
		if(Robot_Status == NAVI_STATUS)	
			break;																						//�ȴ����������л�Ϊ����״̬
	}
	POINT_COLOR = BLACK; 		/*��ɫ����*/LCD_ShowString(30,LED_ROW_14,300,16,16,"System Init Step4 Succeed...");	
}
//
void system_init3(void)					//* Description: ͨѶ��ʼ��
{
	//δ���յ�ͨѶģ��ͨѶ��Ӧ
	while(!(USART3_RX_STA&0x8000))												
	{
		if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//�ȴ�DMA1_Stream4�������	
		{
			DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);		//���DMA1_Stream4������ɱ�־
			USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���3��DMA����
			MYDMA_Enable(DMA1_Stream4,39);  			//��ʼһ��DMA���䣡			
		}
		else{
			BEEP = !BEEP;		delay_ms(3000);
		}
	}
	//δ���յ�����ģ��ͨѶ��Ӧ
	while(!(get_uart1_data))													
	{
		if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA2_Steam7�������	
		{
			DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);				//���DMA2_Steam7������ɱ�־
			USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  		//ʹ�ܴ���1��DMA����
			MYDMA_Enable(DMA2_Stream7,11+1);     							//��ʼһ��DMA����
		}
		else{
			BEEP = !BEEP;		delay_ms(300);
		}
	}get_uart1_data = 0;																//
}
//
u8 remote_get(void)								//* Description: Remote Get.
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
//
void auto_charge_task(void)			//* Description: �Զ��������;
{
	u8 k = 1,ir_pos;
	
	if(_If_Charging == TRUE)										//���ϵ���
	{
		Robot_Moving(0,0,1);										//ֹͣ
		delay_ms(500);												//��ʱ500ms���Ȼ������ȶ�
		if(_If_Charging == TRUE)									//���ϵ���
		{
			Robot_Status = CHARGING_STATUS;							//����״̬-�л�Ϊ-���״̬
		}
	}
	else if(_If_Charging==FALSE)									//��δ�Ӵ����׮
	{
		ir_pos=remote_get();										//��ȡ����λ���ź�
		if(ir_pos==0)												//����λ���źš���Ϊ��
		{
			IR_No_Find_Count++;										//�������źż���+1
			if(IR_No_Find_Count >= 50)								//�����޺����źż��� > 50
			{
				IR_No_Find_Count = 0;									//��պ������źż���
				Circle_Moving_Count++;								//ԭ����ת��������+1
				Robot_Moving(-10.00*k,10.00*k,1);			//��������ʱ����ת
			}
		}
		else														//�к���λ���ź�
		{
			Circle_Moving_Count = NULL;								//���ԭ����ת��������
			IR_No_Find_Count = NULL;								//���ԭ����ת��������
			switch(ir_pos)											//���ֺ����źš�������Ӧ����
			{
				case IR_LEFT:
					Robot_Moving(-30.00*k,-10.00*k,1);		//�����������һ�ξ���-���ֿ�-��ʱ��
					break;
				case IR_MIDDLE:
					Robot_Moving(-20.00*k,-20.00*k,1);		//�����������һ�ξ���
					break;
				case IR_RIGHT:
					Robot_Moving(-10.00*k,-30.00*k,1);		//�����������һ�ξ���-���ֿ�-˳ʱ��
					break;
				default:															
					break;
			}
		}
		/**********��תѰ���źų����趨ֵ*********/
		if(Circle_Moving_Count >= MAX_Circle_Moving_Count)			//ԭ����ת���� > ��תѰ���ź�����������Դ���
		{
			Circle_Moving_Count = 0;								//���ԭ����ת��������
//				Charge_Exit_Seconds.d = NULL;						//��ճ��ʱ��	
//				Robot_Status = ERROT_STATUS;						//����״̬-�л�Ϊ-�쳣״̬
			Robot_Status = NAVI_STATUS;								//����״̬-�л�Ϊ-����״̬
			Robot_Error = CHARGE_FALSE;								//�쳣����-��������-CHARGE_FALSE
			return;													//�Զ����ʧ�ܡ����������γ��	
		}
		/**********��ʱ*********/
		delay_ms(20); 		//��ʱ20ms
	}
}
//
void exit_charge_task(void)			//* Description: �Զ��������;
{
	if(Robot_Status==CHARGING_STATUS && Charge_Exit_Seconds.d < 30 && Reboot_Enable)	//���״̬&&���ʱ��<30s&&����ʹ������
	{
		while(uart1_format_build_and_send((u8 *)RebootBuff1,DEVICE_ID_D9,REBOOT_COMMOND,0x0A,_3byte));
		delay_ms(500);					//��ʱ100ms
	}
	/**********����ʱ�������� �����׮�����е������*********/
	if(Robot_Status==CHARGING_STATUS && !Charge_Exit_Seconds.d)			//���״̬&&���ʱ��ľ�
	{
		Robot_Moving(150.00,150.00,Moving_Seconds);										//��������ǰ����һ�ξ���
		Robot_Moving(0,0,10);																					//ֹͣ
		Robot_Status = NAVI_STATUS;																		//����״̬-�л�Ϊ-����״̬
		return;
	}		
}
//
void upload_odom_task(void)			//* Description: ��̼������������odometry_data
{
	while(uart1_format_build_and_send((u8 *)&position_x,DEVICE_ID_D9,COM_COMMOND,REG_ODOMETRY,20));	
}

//
u8 battery_check(void)						//* Description: ���ʣ���������
{	
	u16 battery_adc = 0;																//������ѹ��Χ��0-4096��
	float battery_vol = 0;															//������ѹֵ
	u8 battery_p = 0;																		//��ѹ�ٷֱ�
	battery_adc=Get_Adc_Average(ADC_Channel_6,25);			//��ȡͨ��6��ת��ֵ��20��ȡƽ��
	battery_vol = (float)((battery_adc*3.3f)/4096);			//��ȡ�����Ĵ�С����ʵ�ʵ�ѹֵ������3.1111
	if(battery_vol > 2.64f)															//��������ѹ
		battery_vol = 2.64f;					
	else	if(battery_vol < 2.26f)												//������С��ѹ
		battery_vol = 2.26f;
	
	battery_p = (u8)(258.75f * battery_vol - 584.66f);
	
	return battery_p;	
}
//д����˵�Ϲ��ô�ӣ�����Ը��ģ�û�취���İɡ�
u8 upload_sensor_task(u8 *arg)
{
	static u8 i,current_p,target_p,sensor_upload_pointer[100];
		
	SensorToServer_Struct.d.ser_num = 1;
	SensorToServer_Struct.d.channel = 2;
	SensorToServer_Struct.d.battery = quantity_of_electric.d;
//	SensorToServer_Struct.d.pose_x = position_x;
//	SensorToServer_Struct.d.pose_y = position_y;
//	SensorToServer_Struct.d.pose_theta = 0;
	SensorToServer_Struct.d.infred = 0;
	SensorToServer_Struct.d.charing_time_surplus = 8;
	SensorToServer_Struct.d.pm1_0 = (uint32_t)pm1_0.d;
	SensorToServer_Struct.d.pm2_5 = (uint32_t)pm2_5.d;
	SensorToServer_Struct.d.pm10 = (uint32_t)pm10.d;
	SensorToServer_Struct.d.air_quality = air_quality.d;
	SensorToServer_Struct.d.voice1 = voice.d;
	SensorToServer_Struct.d.voice2 = voice.d;
	SensorToServer_Struct.d.hydr = h2_value.d;
	SensorToServer_Struct.d.wind_speed = air_velocity.d;
	SensorToServer_Struct.d.temp = temperature.d-2;
	SensorToServer_Struct.d.humi = humidity.d;
	//���״̬�ж�
	if(Robot_Status==CHARGING_STATUS)
		SensorToServer_Struct.d.is_Charge = 1;
	else
		SensorToServer_Struct.d.is_Charge = 0;
			
//	i = sizeof(SensorToServer_Struct.d);
	
	if(Server_Developer_Is_Fool == TRUE)
	{
		//ser_num
		target_p = target_p + sizeof(SensorToServer_Struct.d.ser_num);
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//channel
		target_p = target_p + sizeof(SensorToServer_Struct.d.channel);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//battery
		target_p = target_p + sizeof(SensorToServer_Struct.d.battery);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//pose_x
		target_p = target_p + sizeof(SensorToServer_Struct.d.pose_x);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//pose_y
		target_p = target_p + sizeof(SensorToServer_Struct.d.pose_y);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//pose_theta
		target_p = target_p + sizeof(SensorToServer_Struct.d.pose_theta);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//infred
		target_p = target_p + sizeof(SensorToServer_Struct.d.infred);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//charing_time_surplus
		target_p = target_p + sizeof(SensorToServer_Struct.d.charing_time_surplus);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//pm1_0
		target_p = target_p + sizeof(SensorToServer_Struct.d.pm1_0);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//pm2_5
		target_p = target_p + sizeof(SensorToServer_Struct.d.pm2_5);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//pm10
		target_p = target_p + sizeof(SensorToServer_Struct.d.pm10);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//air_quality
		target_p = target_p + sizeof(SensorToServer_Struct.d.air_quality);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//voice1
		target_p = target_p + sizeof(SensorToServer_Struct.d.voice1);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//voice2
		target_p = target_p + sizeof(SensorToServer_Struct.d.voice2);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//hydr
		target_p = target_p + sizeof(SensorToServer_Struct.d.hydr);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//wind_speed
		target_p = target_p + sizeof(SensorToServer_Struct.d.wind_speed);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//temp
		target_p = target_p + sizeof(SensorToServer_Struct.d.temp);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//humi
		target_p = target_p + sizeof(SensorToServer_Struct.d.humi);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//co
		target_p = target_p + sizeof(SensorToServer_Struct.d.co);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		//is_Charge
		target_p = target_p + sizeof(SensorToServer_Struct.d.is_Charge);	
		for(i = 0;i < target_p-current_p;i++)
		{
			sensor_upload_pointer[current_p+i] = SensorToServer_Struct.data[target_p-i-1];
		}current_p = target_p;
		
		current_p = target_p=0;
	
		uart3_format_build_and_send(sensor_upload_pointer,1,10,sizeof(SensorToServer_Struct.d));
	}
	else
	{
		uart3_format_build_and_send(SensorToServer_Struct.data,1,10,sizeof(SensorToServer_Struct.d));
	}
	
	return 0;
}
/**************************************************USER_TASK_END************************************************************/
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
	u8 i,j[10],*p;
	p = j;
//	u8 main_sta;
//	u8 key;	
	system_init();
	if(Program_Check && EN_Check)
	{
		if(EN_Check_1)
			system_init1();
		for(i = 0;i < 2;i++){
			BEEP = !BEEP;delay_ms(100);
		}delay_ms(1000);
		if(EN_Check_2)
			system_init2();
		for(i = 0;i < 2;i++){
			BEEP = !BEEP;delay_ms(150);
		}delay_ms(1000);
		if(EN_Check_3)
			system_init3();
		for(i = 0;i < 2;i++){
			BEEP = !BEEP;delay_ms(200);
		}delay_ms(1000);
	}BEEP = 0;
	
	while(1)
	{
		/*********�ٶ��·�����***************/
		if(Speed_Clear_Count)																								//�ٶ�δ����
		{
			if(Robot_Status == NAVI_STATUS && linear_vel.d < 1000.0f)					//���ڵ���״̬&&���ٶȼ�������
			{
				speed_convert(linear_vel.d,angular_vel.d);	 										//�����յ����������ٶȸ���С��
			}
		}
		else																																//�ٶ�����
		{
			speed_convert(0,0);
		}
		/*********���ջ�����������***************/
		if(get_uart1_data)
		{
			uart1_deal();
			get_uart1_data = 0;														//������ȡ��ɡ��������
		}
		/*********ȷ������״̬==����״̬*********/
		if(Robot_Status == NAVI_STATUS)
		{
			upload_odom_task();														//��̼Ʒ��͡�������Ӧ����10Hz�ķ���Ƶ��
			delay_ms(10);
		}
		else if(odom_upload_flag)
		{
			upload_odom_task();														//��̼Ʒ���
			odom_upload_flag = 0;
		}
		/*********ȷ������״̬==�ҵ�״̬*********/
		if(Robot_Status == FINDING_DOCK_STATUS)
		{
			Speed_Clear_Count = 2;
			auto_charge_task();
		}
		/*********ȷ������״̬==���״̬*********/
		if(Robot_Status == CHARGING_STATUS)
		{
			exit_charge_task();
		}
		/*********���ӳ���*********/
		if(temp_fr_control == 1)
		{
			upload_sensor_task(0);
			temp_fr_control = 0;
		}
		if(robot_status_flag == 0)
		{		
			p = &Robot_Status;
			while(uart1_format_build_and_send(p,DEVICE_ID_D9,READ_COMMOND,REG_STATUS,_1byte));		//����Robot_Status��1���ֽ�
			robot_status_flag = 15;											//15s����һ��״̬
			delay_ms(10);
		}
		/*********���ӳ���*********/
		else if(battert_check_flag == 0)
		{
			quantity_of_electric.d = battery_check();			//50ms			
			battert_check_flag = 30;											//30s��ѯһ��ʣ�����
			while(uart1_format_build_and_send(quantity_of_electric.data,DEVICE_ID_D9,READ_COMMOND,REG_BATTERY_VOLTAGE,_2byte));
			delay_ms(10);
		}
		else
		{
			delay_ms(30);
		}
	}
}

//
//
//


