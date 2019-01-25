#include "user.h"

#include "r3driver.h"
#include "members.h"

static int_2byte Charge_Exit_Seconds,quantity_of_electric;							//充电时长,电池剩余电量比例
static u8 battert_check_flag,robot_status_flag,odom_upload_flag;				//电量发送标志位,机器人状态发送标志位
static u8 Reboot_Enable;																								//重启使能
static SensorToServer_TypeDef SensorToServer_Struct;										//上传服务器结构体
//********************************************************************************
//用户程序 代码	 
//修改日期:2018-10-22
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 光线量子(北京)智能机器人科技有限公司 
//All rights reserved
//********************************************************************************
//u32 count_second1 = 0;

void TIM6_DAC_IRQHandler(void)		//* Description:  定时器6中断服务函数
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET)						//溢出中断
	{
		if(_One_Second_Count)
			_One_Second_Count--;
		else															 //1s
		{
			if(Charge_Exit_Seconds.d && Robot_Status==CHARGING_STATUS)
				Charge_Exit_Seconds.d--;			//充电时间-1s
			if(battert_check_flag)					//电量检测计数
				battert_check_flag--;
			if(robot_status_flag)						//状态发送计数
				robot_status_flag--;
			_One_Second_Count = 1;					//1s计数器		
		}
		temp_fr_control = 1;							//控制tcp发送频率	
		LED1 = !LED1;											//LED闪烁
		if(Speed_Clear_Count)
			Speed_Clear_Count--;						//速度n*500ms清零、反正通讯中断导致事故
	}
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);							//清除中断标志位
}
//
void TIM7_IRQHandler(void)				//* Description: 定时器7中断服务函数
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) 				//溢出中断
	{
		int PULSE_RIGHT=0,PULSE_LEFT=0;										//dt时间内的左右轮脉冲,用于里程计计算
		
		PULSE_LEFT = ENCODER_RIGHT_TIMER->CNT - 32768;		//右电机码盘采集定时器 TIM4---TIM4编码器模式GPIOC6 GPIOC7 右电机
		ENCODER_RIGHT_TIMER->CNT = 32768;	
		
		PULSE_RIGHT = ENCODER_LEFT_TIMER->CNT - 32768;			//左电机码盘采集定时器 TIM3---TIM3编码器模式GPIOB6 GPIOB7 左电机
		ENCODER_LEFT_TIMER->CNT = 32768;
		
//		position_x1 = PULSE_LEFT * 1000;
//		position_y1 = PULSE_RIGHT * 1000;
		
		//计算里程计
		odometry_cal(PULSE_LEFT,PULSE_RIGHT);
	}
	
	odom_upload_flag = 1;							//控制里程计上传频率	
	
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  				//清除中断标志位
}
//
void uart1_deal(void)						//* Description: 串口缓存处理
{
	u8 *uart_p;
	uart_p = NULL;							//将指针指向空白处
	if(get_uart1_data > 0)
	switch(UART1_RX_BUFF[0])				//判断指令类型
	{
		case 0x01:																//执行指令---命令字0x01
		{
			switch(UART1_RX_BUFF[1])
			{
				case REG_MOVING_SPEED:											//寄存器-0x21-运动速度
				{
					uart_p = &UART1_RX_BUFF[2];
					for(u8 t=0;t<4;t++)											//将串口接受到的速度存储在结构体中;
					{
						linear_vel.data[t]=*(uart_p+t);
						angular_vel.data[t]=*(uart_p+t+4);
					}
					Speed_Clear_Count = 2;
//					if(Robot_Status == NAVI_STATUS	&& linear_vel.d < 500.0f)										//处于导航状态&&对速度加以限制
//						speed_convert(linear_vel.d,angular_vel.d);	 															//将接收到的左右轮速度赋给小车;
				}break;
				case REG_REAL_ODOM:												//寄存器-0x33-实际ODOM
				{
					uart_p = &UART1_RX_BUFF[2];
					for(u8 t=0;t<8;t++)											//将串口接受到的速度存储在结构体中;
					{
						SensorToServer_Struct.data[t+4+2+1]=*(uart_p+t);
						SensorToServer_Struct.data[t+4+2+1+8]=*(uart_p+t+8);
						SensorToServer_Struct.data[t+4+2+1+16]=*(uart_p+t+16);
					}
				}break;				
			}
		}break;
		case 0x02:																//读取指令---命令字0x02
		{
			switch(UART1_RX_BUFF[1])
			{
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
			switch(UART1_RX_BUFF[1])
			{
				case REG_CHARGING_TIME:													//寄存器-0x20-充电时长
				{
					uart_p = &UART1_RX_BUFF[2];
					Charge_Exit_Seconds.data[1] = *(uart_p+1);
					Charge_Exit_Seconds.data[0] = *(uart_p);
					if((Charge_Exit_Seconds.d > 5) && (Charge_Exit_Seconds.d < 60000))
					{
						Reboot_Enable = TRUE;
						Robot_Status = FINDING_DOCK_STATUS;						//运行状态-切换为-找电状态			
					}
					while(uart1_format_build_and_send(Charge_Exit_Seconds.data,DEVICE_ID_D9,WRITE_COMMOND,REG_CHARGING_TIME,_2byte));//回显
				}break;
			}
		}break;
	}
}
//
void system_init(void)						//* Description: 内部初始化
{
	delay_init(168);       										//延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//设置系统中断优先级分组2
	BEEP_Init();												//蜂鸣器初始化
	uart_init();   												//串口初始化
//	usmart_dev.init(84); 										//初始化USMART
	LED_Init();  												//LED初始化
	KEY_Init();  												//按键初始化
	LCD_Init(); 												//LCD初始化
	Remote_Init();												//红外接收初始化	
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,
	CAN_BS1_7tq,6,CAN_Mode_Normal);								//CAN初始化普通模式,波特率500Kbps	
	ENC_Init_Left();											//设置电机A	TIM4编码器模式PB6 PB7 左电机 
	ENC_Init_Right();											//设置电机B	TIM3编码器模式PC6 PC7 右电机
//	FSMC_SRAM_Init();												//初始化外部SRAM
//	My_RTC_Init();  												//RTC初始化
	Adc_Init(GPIOA,GPIO_Pin_6);         						//初始化ADC-GPIOA-6
	Get_Adc_Average(ADC_Channel_6,20);							//获取通道5的转换值，20次取平均
//	Adc_Temperate_Init(); 											//内部温度传感器初始化
//	TIM3_Int_Init(999,839); 										//定时器时钟84M，分频系数840，所以84M/840=100Khz的计数频率，计数100次为1ms 
	TIM6_Int_Init(5000-1,8400-1); 							//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数5000次为500ms
	TIM7_Int_Init(100-1,8400-1); 								//定时器时钟84M，分频系数8400，所以84M/8400=10Khz的计数频率，计数100次为10ms
//	mymem_init(SRAMIN);												//初始化内部内存池
//	mymem_init(SRAMEX);												//初始化外部内存池
//	mymem_init(SRAMCCM);											//初始化CCM内存池
	Robot_Status = NAVI_STATUS;									//机器人开机默认为导航状态
	/*********使能串口1的DMA发送********************/
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  				//使能串口1的DMA发送
	MYDMA_Enable(DMA2_Stream7,11+1);     						//开始一次DMA传输	
	/*********使能串口3的DMA发送********************/	
	USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  				//使能串口3的DMA发送
	MYDMA_Enable(DMA1_Stream4,39);     							//开始一次DMA传输
	
	delay_ms(1000);												//上电等待驱动器启动
	motor_init_task();										//驱动器初始化	
	
	BEEP = 1;delay_ms(50);BEEP = 0;delay_ms(1000);
}
//
void system_init1(void)					//* Description: 运动部分初始化
{	
	//motor check step1 start
	odometry_clear();				/*清空里程计*/
	Robot_Moving(100.0,-100.0,200);	Robot_Moving(0,0,50);		//右转，停止		
	Robot_Moving(-100.0,100.0,200);	Robot_Moving(0,0,50);		//左转，停止	
	if(position_x>5.1f || position_y>5.1f || oriention>0.1f){
		while(1){BEEP = !BEEP;delay_ms(500);}
	}
	//motor check step2 start//前进里程判断
	odometry_clear();				/*清空里程计*/		delay_ms(1000);	
	Robot_Moving(100.0,100.0,100);	Robot_Moving(0,0,50);			//前进，停止
 	if(!position_x || position_x <50.0f){
		while(1){BEEP = !BEEP;delay_ms(500);}
	}else{
		Robot_Moving(-100.0,-100.0,100);	Robot_Moving(0,0,50);	//后退，停止
	}
	//motor check step3 start//左转里程判断
	odometry_clear();				/*清空里程计*/		delay_ms(1000);	
	Robot_Moving(-100.0,100.0,200);	Robot_Moving(0,0,50);	//左转，停止
	if(!oriention || oriention<0.5f){				//旋转角度大于0小于0.5
		while(1){BEEP = !BEEP;delay_ms(500);}
	}
	else{
		Robot_Moving(100.0,-100.0,200);	Robot_Moving(0,0,50);//右转，停止
	}
	odometry_clear();				/*清空里程计*/		delay_ms(1000);	
	BEEP = 1;delay_ms(200);BEEP = 0;delay_ms(2000);
}

//
void system_init2(void)					//* Description: 充电初始化
{
	POINT_COLOR = BLACK; 		/*黑色字体*/LCD_ShowString(30,LED_ROW_13,300,16,16,"System Init Step4......");
	Charge_Exit_Seconds.d = 10;												//10s的充电时长
	Robot_Status = FINDING_DOCK_STATUS;										//机器人设置为寻电模式	
	while(1)										
	{
		auto_charge_task();
		exit_charge_task();
		if(Robot_Status == NAVI_STATUS)	
			break;																						//等待充电结束、切换为导航状态
	}
	POINT_COLOR = BLACK; 		/*黑色字体*/LCD_ShowString(30,LED_ROW_14,300,16,16,"System Init Step4 Succeed...");	
}
//
void system_init3(void)					//* Description: 通讯初始化
{
	//未接收到通讯模块通讯回应
	while(!(USART3_RX_STA&0x8000))												
	{
		if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//等待DMA1_Stream4传输完成	
		{
			DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);		//清除DMA1_Stream4传输完成标志
			USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);  //使能串口3的DMA发送
			MYDMA_Enable(DMA1_Stream4,39);  			//开始一次DMA传输！			
		}
		else{
			BEEP = !BEEP;		delay_ms(3000);
		}
	}
	//未接收到导航模块通讯回应
	while(!(get_uart1_data))													
	{
		if(DMA_GetFlagStatus(DMA2_Stream7,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成	
		{
			DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);				//清除DMA2_Steam7传输完成标志
			USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  		//使能串口1的DMA发送
			MYDMA_Enable(DMA2_Stream7,11+1);     							//开始一次DMA传输
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
void auto_charge_task(void)			//* Description: 自动充电任务;
{
	u8 k = 1,ir_pos;
	
	if(_If_Charging == TRUE)										//充上电了
	{
		Robot_Moving(0,0,1);										//停止
		delay_ms(500);												//延时500ms，等机器人稳定
		if(_If_Charging == TRUE)									//充上电了
		{
			Robot_Status = CHARGING_STATUS;							//运行状态-切换为-充电状态
		}
	}
	else if(_If_Charging==FALSE)									//还未接触充电桩
	{
		ir_pos=remote_get();										//获取红外位置信号
		if(ir_pos==0)												//红外位置信号——为空
		{
			IR_No_Find_Count++;										//红外无信号计数+1
			if(IR_No_Find_Count >= 50)								//连续无红外信号计数 > 50
			{
				IR_No_Find_Count = 0;									//清空红外无信号计数
				Circle_Moving_Count++;								//原地旋转计数计数+1
				Robot_Moving(-10.00*k,10.00*k,1);			//机器人逆时针旋转
			}
		}
		else														//有红外位置信号
		{
			Circle_Moving_Count = NULL;								//清空原地旋转次数计数
			IR_No_Find_Count = NULL;								//清空原地旋转次数计数
			switch(ir_pos)											//区分红外信号——做相应处理
			{
				case IR_LEFT:
					Robot_Moving(-30.00*k,-10.00*k,1);		//机器人向后退一段距离-左轮快-逆时针
					break;
				case IR_MIDDLE:
					Robot_Moving(-20.00*k,-20.00*k,1);		//机器人向后退一段距离
					break;
				case IR_RIGHT:
					Robot_Moving(-10.00*k,-30.00*k,1);		//机器人向后退一段距离-右轮快-顺时针
					break;
				default:															
					break;
			}
		}
		/**********旋转寻找信号超过设定值*********/
		if(Circle_Moving_Count >= MAX_Circle_Moving_Count)			//原地旋转计数 > 旋转寻找信号最大连续尝试次数
		{
			Circle_Moving_Count = 0;								//清空原地旋转次数计数
//				Charge_Exit_Seconds.d = NULL;						//清空充电时间	
//				Robot_Status = ERROT_STATUS;						//运行状态-切换为-异常状态
			Robot_Status = NAVI_STATUS;								//运行状态-切换为-导航状态
			Robot_Error = CHARGE_FALSE;								//异常报警-错误类型-CHARGE_FALSE
			return;													//自动充电失败——放弃本次充电	
		}
		/**********延时*********/
		delay_ms(20); 		//延时20ms
	}
}
//
void exit_charge_task(void)			//* Description: 自动充电任务;
{
	if(Robot_Status==CHARGING_STATUS && Charge_Exit_Seconds.d < 30 && Reboot_Enable)	//充电状态&&充电时间<30s&&重启使能启动
	{
		while(uart1_format_build_and_send((u8 *)RebootBuff1,DEVICE_ID_D9,REBOOT_COMMOND,0x0A,_3byte));
		delay_ms(500);					//延时100ms
	}
	/**********充电计时结束，贴 近充电桩、运行弹射程序*********/
	if(Robot_Status==CHARGING_STATUS && !Charge_Exit_Seconds.d)			//充电状态&&充电时间耗尽
	{
		Robot_Moving(150.00,150.00,Moving_Seconds);										//机器人向前弹出一段距离
		Robot_Moving(0,0,10);																					//停止
		Robot_Status = NAVI_STATUS;																		//运行状态-切换为-导航状态
		return;
	}		
}
//
void upload_odom_task(void)			//* Description: 里程计数据整理存入odometry_data
{
	while(uart1_format_build_and_send((u8 *)&position_x,DEVICE_ID_D9,COM_COMMOND,REG_ODOMETRY,20));	
}

//
u8 battery_check(void)						//* Description: 电池剩余电量计算
{	
	u16 battery_adc = 0;																//电量电压范围（0-4096）
	float battery_vol = 0;															//电量电压值
	u8 battery_p = 0;																		//电压百分比
	battery_adc=Get_Adc_Average(ADC_Channel_6,25);			//获取通道6的转换值，20次取平均
	battery_vol = (float)((battery_adc*3.3f)/4096);			//获取计算后的带小数的实际电压值，比如3.1111
	if(battery_vol > 2.64f)															//限制最大电压
		battery_vol = 2.64f;					
	else	if(battery_vol < 2.26f)												//限制最小电压
		battery_vol = 2.26f;
	
	battery_p = (u8)(258.75f * battery_vol - 584.66f);
	
	return battery_p;	
}
//写服务端的瞎特么接，还不愿意改，没办法，改吧。
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
	//充电状态判断
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
		/*********速度下发部分***************/
		if(Speed_Clear_Count)																								//速度未清零
		{
			if(Robot_Status == NAVI_STATUS && linear_vel.d < 1000.0f)					//处于导航状态&&对速度加以限制
			{
				speed_convert(linear_vel.d,angular_vel.d);	 										//将接收到的左右轮速度赋给小车
			}
		}
		else																																//速度清零
		{
			speed_convert(0,0);
		}
		/*********接收缓存区有数据***************/
		if(get_uart1_data)
		{
			uart1_deal();
			get_uart1_data = 0;														//数据提取完成、清空数组
		}
		/*********确认运行状态==导航状态*********/
		if(Robot_Status == NAVI_STATUS)
		{
			upload_odom_task();														//里程计发送——至少应保持10Hz的发送频率
			delay_ms(10);
		}
		else if(odom_upload_flag)
		{
			upload_odom_task();														//里程计发送
			odom_upload_flag = 0;
		}
		/*********确认运行状态==找电状态*********/
		if(Robot_Status == FINDING_DOCK_STATUS)
		{
			Speed_Clear_Count = 2;
			auto_charge_task();
		}
		/*********确认运行状态==充电状态*********/
		if(Robot_Status == CHARGING_STATUS)
		{
			exit_charge_task();
		}
		/*********钩子程序*********/
		if(temp_fr_control == 1)
		{
			upload_sensor_task(0);
			temp_fr_control = 0;
		}
		if(robot_status_flag == 0)
		{		
			p = &Robot_Status;
			while(uart1_format_build_and_send(p,DEVICE_ID_D9,READ_COMMOND,REG_STATUS,_1byte));		//发送Robot_Status，1个字节
			robot_status_flag = 15;											//15s发送一次状态
			delay_ms(10);
		}
		/*********钩子程序*********/
		else if(battert_check_flag == 0)
		{
			quantity_of_electric.d = battery_check();			//50ms			
			battert_check_flag = 30;											//30s查询一次剩余电量
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


