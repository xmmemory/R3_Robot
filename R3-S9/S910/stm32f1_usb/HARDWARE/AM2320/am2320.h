#ifndef __AM2320_H
#define __AM2320_H 
#include "sys.h"   
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK战舰STM32开发板
//AM2320数字温湿度传感器驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/9/12
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 
//IO方向设置
#define AM2320_IO_IN()  {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=4<<24;}
#define AM2320_IO_OUT() {GPIOB->CRH&=0XF0FFFFFF;GPIOB->CRH|=3<<24;}
////IO操作函数											   
#define	AM2320_DQ_OUT PBout(14) //数据端口	PA0 
#define	AM2320_DQ_IN  PBin(14)  //数据端口	PA0 


u8 AM2320_Init(void);//初始化AM2320
u8 AM2320_Read_Data(void);//读取温湿度
u8 AM2320_Read_Byte(void);//读出一个字节
u8 AM2320_Read_Bit(void);//读出一个位
u8 AM2320_Check(void);//检测是否存在AM2320
void AM2320_Rst(void);//复位AM2320

extern u16 temperature_u16,humidity_u16;

#endif















