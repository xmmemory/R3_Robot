#include "user.h"

#include "r3driver.h"
#include "members.h"


//********************************************************************************
//用户程序 代码	 
//修改日期:2018-10-22
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 光线量子(北京)智能机器人科技有限公司 
//All rights reserved
//********************************************************************************
//u32 count_second1 = 0;

//定时器6中断服务函数
void TIM6_DAC_IRQHandler(void)												//1s
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET)				//溢出中断
	{
		if(Charge_Exit_Seconds.d && Robot_Status==CHARGING_STATUS)
		{
			Charge_Exit_Seconds.d--;												//充电时间-1s
		}
//		count_second1++;
		temp_fr_control = 1;															//控制tcp发送频率
		LED1 = !LED1;																			//LED闪烁
	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);					//清除中断标志位
}
//
//* Description: 定时器7中断服务函数
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null；
void TIM7_IRQHandler(void)														//
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) 				//溢出中断
	{
		int PULSE_RIGHT=0,PULSE_LEFT=0;										//dt时间内的左右轮脉冲,用于里程计计算
		PULSE_LEFT = ENCODER_LEFT_TIMER->CNT - 32768;			//左电机码盘采集定时器 TIM3---TIM3编码器模式GPIOB6 GPIOB7 左电机
		ENCODER_LEFT_TIMER->CNT = 32768;
		PULSE_RIGHT = ENCODER_RIGHT_TIMER->CNT - 32768;		//右电机码盘采集定时器 TIM4---TIM4编码器模式GPIOC6 GPIOC7 右电机
		ENCODER_RIGHT_TIMER->CNT = 32768;
//	PULSE_LEFT_HIS += PULSE_LEFT;
//	PULSE_RIGHT_HIS += PULSE_RIGHT;		
		odometry_cal(PULSE_LEFT,PULSE_RIGHT);							//计算里程计		
	}
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  				//清除中断标志位
}
//* Description: 串口缓存处理
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null；
void uart1_deal(void)																//串口缓存处理
{
	u8 i,*uart_p;
	switch(UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][0])				//判断指令类型
	{
		case 0x01:																					//执行指令---命令字0x01
		{
			switch(UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][1])
			{
				case REG_MOVING_SPEED:													//寄存器-0x21-运动速度
				{
					uart_p = &UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][2];
					for(u8 t=0;t<4;t++)														//将串口接受到的速度存储在结构体中;
					{
						rightdata.data[t]=*(uart_p+t);
						leftdata.data[t]=*(uart_p+t+4);
					}
					if(Robot_Status == NAVI_STATUS	
						&& rightdata.d < 1000 
					&& leftdata.d < 1000)													//处于导航状态&&对速度加以限制
						car_control(rightdata.d,leftdata.d);	 			//将接收到的左右轮速度赋给小车;
				}break;
				case REG_SERVER_SEND:
				if(temp_fr_control)
				{
					uart_p = &UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][2];
					memcpy(SendBuff3,uart_p,*(uart_p+4));
					if(*(uart_p+4) < 200 && DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//等待DMA1_Steam4传输完成
					{
						//DMA_Send
						DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);		//清除DMA1_Stream4传输完成标志
						USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送
						MYDMA_Enable(DMA1_Stream4,*(uart_p+4));  			//开始一次DMA传输！
						temp_fr_control = 0;
					}
				}break;
			}
		}break;
		case 0x02:																					//读取指令---命令字0x02
		{
			switch(UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][1])
			{
				case REG_STATUS:																//寄存器-0x01-运行状态
				{
					uart_p = &Robot_Status;
					while(uart1_format_build_and_send(uart_p,DEVICE_ID_D9,READ_COMMOND,REG_STATUS,_1byte));		//发送Robot_Status，1个字节							
				}break;
				case REG_ODOMETRY:															//寄存器-0x10-里程计
				{
					odom_send_task();
//							uart1_format_build_and_send(odometry_data,DEVICE_ID_D9,READ_COMMOND,REG_ODOMETRY,31);	
				}break;	
				case REG_BATTERY_VOLTAGE:												//寄存器-0x11-电量计
				{
					i = battery_check();
					uart_p = &i;
					while(uart1_format_build_and_send(uart_p,DEVICE_ID_D9,READ_COMMOND,REG_BATTERY_VOLTAGE,_2byte));
				}break;	
				case REG_CHARGING_TIME:													//寄存器-0x20-充电时长
				{
					while(uart1_format_build_and_send(Charge_Exit_Seconds.data,DEVICE_ID_D9,READ_COMMOND,REG_CHARGING_TIME,_2byte));
				}break;
				case REG_ERROR:																	//寄存器-0xF1-错误码
				{
					uart_p = &Robot_Error;
					while(uart1_format_build_and_send(uart_p,DEVICE_ID_D9,READ_COMMOND,REG_ERROR,_1byte));		//发送Robot_Error，1个字节	
				}break;
				default:																				//未知指令
					break;
			}
		}break;
		case 0x03:																					//写入指令---命令字0x03
		{
			switch(UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][1])
			{
				case REG_CHARGING_TIME:													//寄存器-0x20-充电时长
				{
					uart_p = &UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][2];
					Charge_Exit_Seconds.data[1] = *(uart_p+1);
					Charge_Exit_Seconds.data[0] = *(uart_p);
					if((Charge_Exit_Seconds.d > 5) && (Charge_Exit_Seconds.d < 60000))
					Robot_Status = FINDING_DOCK_STATUS;						//运行状态-切换为-找电状态							
					while(uart1_format_build_and_send(Charge_Exit_Seconds.data,DEVICE_ID_D9,WRITE_COMMOND,REG_CHARGING_TIME,_2byte));//回显
				}break;
				case REG_SERVER_SEND:
				if(temp_fr_control)
				{
					uart_p = &UART1_RX_BUFF[UART1_RX_BUF_RECIVE-1][2];
					memcpy(SendBuff3,uart_p,*(uart_p+4));
					if(*(uart_p+4) < 200 && DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//等待DMA1_Steam4传输完成
					{
						//DMA_Send
						DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);		//清除DMA1_Stream4传输完成标志
						USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送
						MYDMA_Enable(DMA1_Stream4,*(uart_p+4));  			//开始一次DMA传输！
						temp_fr_control = 0;
					}
				}break;
			}
		}break;				
	}UART1_RX_BUF_RECIVE--;															//数据提取完成、清空数组
}

//* Description: 内部初始化
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null；
void system_init_step1(void)														//内部初始化
{
	delay_init(168);       																//延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//设置系统中断优先级分组2
	BEEP_Init();																					//蜂鸣器初始化
	uart_init();   																				//串口初始化
//	usmart_dev.init(84); 																	//初始化USMART
	LED_Init();  																					//LED初始化
	KEY_Init();  																					//按键初始化
	LCD_Init(); 																					//LCD初始化
	Remote_Init();																				//红外接收初始化	
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_Normal);//CAN初始化普通模式,波特率500Kbps	
	ENC_Init_Left();																			//设置电机A	TIM4编码器模式PB6 PB7 左电机 
	ENC_Init_Right();																			//设置电机B	TIM3编码器模式PC6 PC7 右电机
	POINT_COLOR = BLACK; 		/*黑色字体*/LCD_ShowString(30,LED_ROW_1,300,16,16,"System Init Step1......");
//	FSMC_SRAM_Init();																		//初始化外部SRAM
//	My_RTC_Init();  																		//RTC初始化
	Adc_Init(GPIOA,GPIO_Pin_6);         									//初始化ADC-GPIOA-6
	Get_Adc_Average(ADC_Channel_6,20);										//获取通道5的转换值，20次取平均
//	Adc_Temperate_Init(); 																//内部温度传感器初始化
//	TIM3_Int_Init(999,839); 															//定时器时钟84M，分频系数840，所以84M/840=100Khz的计数频率，计数100次为1ms 
	TIM6_Int_Init(10000-1,8400-1); 												//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数10000次为1s
	TIM7_Int_Init(100-1,8400-1); 													//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数100次为10ms
//	mymem_init(SRAMIN);																		//初始化内部内存池
//	mymem_init(SRAMEX);																		//初始化外部内存池
//	mymem_init(SRAMCCM);																	//初始化CCM内存池

	Charge_Exit_Seconds.d = 20;														//20s的充电时长
	Moving_Seconds = 250;																	//充电脱离时长---n*10ms

	delay_ms(1000);																				//上电等待驱动器启动

	Robot_Status = NAVI_STATUS;														//机器人开机默认为导航状态
	/*********使能串口1的DMA发送********************/
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  				//使能串口1的DMA发送
	MYDMA_Enable(DMA2_Stream7,11+1);     									//开始一次DMA传输	
	/*********使能串口3的DMA发送********************/	
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  				//使能串口3的DMA发送
	MYDMA_Enable(DMA1_Stream4,39);     										//开始一次DMA传输	
	motor_init_task();																		//驱动器初始化	
	POINT_COLOR = BLACK; 		/*黑色字体*/	LCD_ShowString(30,LED_ROW_2,300,16,16,"System Init Step1 Succeed...");
	BEEP = 1;delay_ms(200);BEEP = 0;delay_ms(2000);
}
//* Description: 运动部分初始化
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null；
void system_init_step2(void)														//运动部分初始化
{	
//		LCD_ShowxNum(30,150,count_second1,10,16,0X80);//显示新IP
//	while(1)
//	{
//		delay_ms(100);
//	}
	POINT_COLOR = BLACK; 		/*黑色字体*/LCD_ShowString(30,LED_ROW_3,300,16,16,"System Init Step2......");	
	//motor check step1 start
	POINT_COLOR = BLUE; 		/*蓝色字体*/LCD_ShowString(30,LED_ROW_4,300,16,16,"Motor Check Step1......");
	position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;		/*清空里程计*/	
	Robot_Moving(100.0,-100.0,200);	Robot_Moving(0,0,50);		//右转，停止		
	Robot_Moving(-100.0,100.0,200);	Robot_Moving(0,0,50);		//左转，停止	
	if(position_x>5.1f || position_y>5.1f || oriention>0.1f){
		POINT_COLOR = RED; 		/*红色字体*/LCD_ShowString(30,LED_ROW_5,300,16,16,"Motor Check Step1 Fail......");	
		while(1){BEEP = !BEEP;delay_ms(500);}
	}else{
		POINT_COLOR = GREEN; 		/*绿色字体*/LCD_ShowString(30,LED_ROW_5,300,16,16,"Motor Check Step1 Succeed......");
	}
	//motor check step2 start//前进里程判断
	POINT_COLOR = BLUE; 		/*蓝色字体*/LCD_ShowString(30,LED_ROW_6,300,16,16,"Motor Check Step2......");
	position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;		/*清空里程计*/		delay_ms(1000);	
	Robot_Moving(100.0,100.0,100);	Robot_Moving(0,0,50);			//前进，停止
 	if(!position_x || position_x <50.0f){
		POINT_COLOR = RED; 		/*红色字体*/LCD_ShowString(30,LED_ROW_7,300,16,16,"Motor Check Step2 Fail......");	
		while(1){BEEP = !BEEP;delay_ms(500);}
	}else{
		POINT_COLOR = GREEN; 		/*绿色字体*/LCD_ShowString(30,LED_ROW_7,300,16,16,"Motor Check Step2 Succeed......");	
		Robot_Moving(-100.0,-100.0,100);	Robot_Moving(0,0,50);	//后退，停止
	}
	//motor check step3 start//左转里程判断
	POINT_COLOR = BLUE; 		/*蓝色字体*/LCD_ShowString(30,LED_ROW_8,300,16,16,"Motor Check Step3......");
	position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;		/*清空里程计*/		delay_ms(1000);	
	Robot_Moving(-100.0,100.0,200);	Robot_Moving(0,0,50);	//左转，停止
	if(!oriention || oriention<0.5f){				//旋转角度大于0小于0.5
		POINT_COLOR = RED; 		/*红色字体*/LCD_ShowString(30,LED_ROW_9,300,16,16,"Motor Check Step3 Fail......");	
		while(1){BEEP = !BEEP;delay_ms(500);}
	}
	else{
		POINT_COLOR = GREEN; 		/*绿色字体*/LCD_ShowString(30,LED_ROW_9,300,16,16,"Motor Check Step3 Succeed......");
		Robot_Moving(100.0,-100.0,200);	Robot_Moving(0,0,50);//右转，停止
	}
	position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;		/*清空里程计*/		delay_ms(1000);	
	POINT_COLOR = BLACK; 		/*黑色字体*/LCD_ShowString(30,LED_ROW_10,300,16,16,"System Init Step2 Succeed...");	
	BEEP = 1;delay_ms(200);BEEP = 0;delay_ms(2000);
}
//* Description: 参数初始化
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null；
void system_init_step3(void)														//参数初始化
{
	POINT_COLOR = BLACK; 		/*黑色字体*/LCD_ShowString(30,LED_ROW_11,300,16,16,"System Init Step3......");	
	POINT_COLOR = BLACK; 		/*黑色字体*/LCD_ShowString(30,LED_ROW_12,300,16,16,"System Init Step3 Succeed...");	
	BEEP = 1;delay_ms(200);BEEP = 0;delay_ms(2000);
}
//* Description: 充电初始化
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null；
void system_init_step4(void)														//充电初始化
{
	POINT_COLOR = BLACK; 		/*黑色字体*/LCD_ShowString(30,LED_ROW_13,300,16,16,"System Init Step4......");
	Robot_Status = FINDING_DOCK_STATUS;										//机器人开机默认为寻电模式	
	while(1)										
	{
		auto_charge_task();
		exit_charge_task();
		if(Robot_Status == NAVI_STATUS)	
			break;																						//等待充电结束、切换为导航状态
	}
	POINT_COLOR = BLACK; 		/*黑色字体*/LCD_ShowString(30,LED_ROW_14,300,16,16,"System Init Step4 Succeed...");	
}

//* Description: 通讯初始化
//* Arguments  : 1) Null;
//* Note(s)    : 1) Null；
void system_init_step5(void)														//通讯初始化
{
	POINT_COLOR = BLACK; 		/*黑色字体*/LCD_ShowString(30,LED_ROW_15,300,16,16,"System Init Step5......");	
	
	while(1)												//未接收到通讯模块通讯回应
	{
		POINT_COLOR = RED; 		/*红色字体*/LCD_ShowString(30,LED_ROW_13,200,16,16,"Connect Server Fail......");
		if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//等待DMA1_Stream4传输完成	
		{
			DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);		//清除DMA1_Stream4传输完成标志
			USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送
			MYDMA_Enable(DMA1_Stream4,39);  			//开始一次DMA传输！			
		}
//		BEEP = !BEEP;		
		delay_ms(3000);
	}
	POINT_COLOR = GREEN; 		/*绿色字体*/LCD_ShowString(30,LED_ROW_17,300,16,16,"Connect Server Succeed......");
	
	while(!(UART1_RX_BUF_RECIVE))													//未接收到导航模块通讯回应
	{
		POINT_COLOR = RED; 		/*红色字体*/LCD_ShowString(30,LED_ROW_16,300,16,16,"Connect Navigation Fail......");
		if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成	
		{
			DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);				//清除DMA2_Steam7传输完成标志
			USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  		//使能串口1的DMA发送
			MYDMA_Enable(DMA2_Stream7,11+1);     							//开始一次DMA传输
		}
		BEEP = !BEEP;		delay_ms(300);
	}UART1_RX_BUF_RECIVE--;																//将UART1_RX_BUF_RECIVE的索引头-1
	POINT_COLOR = GREEN; 		/*绿色字体*/LCD_ShowString(30,LED_ROW_16,300,16,16,"Connect Navigation Succeed......");
		
	POINT_COLOR = BLACK; 		/*黑色字体*/LCD_ShowString(30,LED_ROW_18,300,16,16,"System Init Step5 Succeed...");
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
//* Description: 自动充电任务;
//* Arguments  : NULL
//* Note(s)    :  Null.
void auto_charge_task(void)
{
	u8 k = 1,ir_pos;
	
	if(_If_Charging == TRUE)											//充上电了
	{
		Robot_Moving(0,0,1);												//停止
		delay_ms(500);															//延时500ms，等机器人稳定
		if(_If_Charging == TRUE)										//充上电了
		{
			Robot_Status = CHARGING_STATUS;						//运行状态-切换为-充电状态
		}
	}
	else if(_If_Charging==FALSE)									//还未接触充电桩
	{
		ir_pos=remote_get();												//获取红外位置信号
		if(ir_pos==0)																//红外位置信号——为空
		{
			IR_No_Find_Count++;												//红外无信号计数+1
			if(IR_No_Find_Count >= 150)								//连续无红外信号计数 > 150
			{
				IR_No_Find_Count = 0;										//清空红外无信号计数
				Circle_Moving_Count++;									//原地旋转计数计数+1
				Robot_Moving(-20.00*k,20.00*k,1);				//机器人逆时针旋转
			}
		}
		else																	 			//有红外位置信号
		{
			Circle_Moving_Count = NULL;								//清空原地旋转次数计数
			IR_No_Find_Count = NULL;									//清空原地旋转次数计数
			switch(ir_pos)														//区分红外信号——做相应处理
			{
				case IR_LEFT:
					Robot_Moving(-40.00*k,-20.00*k,1);		//机器人向后退一段距离-左轮快-逆时针
					break;
				case IR_MIDDLE:
					Robot_Moving(-20.00*k,-20.00*k,1);		//机器人向后退一段距离
					break;
				case IR_RIGHT:
					Robot_Moving(-20.00*k,-40.00*k,1);		//机器人向后退一段距离-右轮快-顺时针
					break;
				default:															
					break;
			}
		}
		/**********旋转寻找信号超过设定值*********/
		if(Circle_Moving_Count >= MAX_Circle_Moving_Count)	//原地旋转计数 > 旋转寻找信号最大连续尝试次数
		{
			Circle_Moving_Count = 0;												//清空原地旋转次数计数
//				Charge_Exit_Seconds.d = NULL;											//清空充电时间	
//				Robot_Status = ERROT_STATUS;											//运行状态-切换为-异常状态
			Robot_Status = NAVI_STATUS;											//运行状态-切换为-导航状态
			Robot_Error = CHARGE_FALSE;											//异常报警-错误类型-CHARGE_FALSE
			return;																					//自动充电失败——放弃本次充电	
		}
		/**********延时*********/
		delay_ms(50); 		//延时10ms
	}
}
//* Description: 自动充电任务;
//* Arguments  : NULL
//* Note(s)    : Null. 
void exit_charge_task(void)
{
	/**********充电计时结束，贴近充电桩、运行弹射程序*********/
	if(Robot_Status==CHARGING_STATUS && !Charge_Exit_Seconds.d)			//充电状态&&充电时间耗尽
	{
		Robot_Moving(150.00,150.00,Moving_Seconds);										//机器人向前弹出一段距离
		Robot_Moving(0,0,10);																					//停止
		Robot_Status = NAVI_STATUS;																		//运行状态-切换为-导航状态
		return;
		
	}		
}
//* Description: 里程计数据整理存入odometry_data
//* Arguments  : Null
//* Note(s)    : 1) Null；
void odom_send_task(void)
{
	//发送给串口的里程计数据数组---预留出2倍的长度防止异常溢出（应该用不上）		//Odom_Get
	x_data.odoemtry_float=position_x;//单位mm		
	y_data.odoemtry_float=position_y;//单位mm
	theta_data.odoemtry_float=oriention;//单位rad
	vel_linear.odoemtry_float=velocity_linear;//单位mm/s
	vel_angular.odoemtry_float=velocity_angular;//单位rad/s
	//将所有里程计数据存到要发送的数组
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

//* Description: 电池剩余电量计算
//* Arguments  : Null.
//* Note(s)    : 1) Null
u8 battery_check(void)
{	
	u16 battery_adc = 0;																//电量电压范围（0-4096）
	float battery_vol = 0;															//电量电压值
	u8 battery_p = 0;																		//电压百分比
	battery_adc=Get_Adc_Average(ADC_Channel_6,20);			//获取通道6的转换值，20次取平均
	battery_vol = (float)((battery_adc*3.3f)/4096);			//获取计算后的带小数的实际电压值，比如3.1111
	if(battery_vol > 2.64f)															//限制最大电压
		battery_vol = 2.64f;					
	else	if(battery_vol < 2.26f)												//限制最小电压
		battery_vol = 2.26f;
	
	battery_p = (u8)(258.75f * battery_vol - 584.66f);
	
	return battery_p;	
}
//
//
//

//********************************************************************************
//主程序逻辑 代码	 
//修改日期:2018-10-22
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 光线量子(北京)智能机器人科技有限公司 
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
		/*********接收缓存区有数据***************/
		if(UART1_RX_BUF_RECIVE)
		{
			uart1_deal();
		}
		/*********确认运行状态==找电状态*********/
		if(Robot_Status == FINDING_DOCK_STATUS)
		{
			auto_charge_task();
		}
		
		/*********确认运行状态==充电状态*********/
		if(Robot_Status == CHARGING_STATUS)
		{
			exit_charge_task();
		}
		
		/*********确认运行状态==导航状态*********/
		if(Robot_Status == NAVI_STATUS)
		{
			;
		}
		
		odom_send_task();				//里程计发送
		delay_ms(10);
//		battery_check();
		
	}	
}




