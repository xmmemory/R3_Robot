#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f10x.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//LX-1A开发板		   
//青岛智芯科技
//电话：13012418100
//修改日期:2011/10/28 
//版权所有，盗版必究。
//********************************************************************************
////////////////////////////////////////////////////////////////////////////////// 
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void Systick_Init(void);

#endif





























