#ifndef _AM2305_DEV_H
#define _AM2305_DEV_H
#include "sys.h"  

//IO方向设置
#define AM2305_GPIO_IN()  {GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=8<<20;}
#define AM2305_GPIO_OUT() {GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=3<<20;}
////IO操作函数											   
#define	AM2305_DATA_OUT PBout(13) //数据端口	PA0 
#define	AM2305_DATA_IN  PBin(13)  //数据端口	PA0 

extern u8 AM2305_Init(void);
extern void AM2305_UpdateData(void);

extern short  Temperature;
extern short  Humidity;

#endif
