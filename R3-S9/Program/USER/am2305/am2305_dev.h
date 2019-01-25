#ifndef _AM2305_DEV_H
#define _AM2305_DEV_H
#include "sys.h"  

//IO��������
#define AM2305_GPIO_IN()  {GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=8<<20;}
#define AM2305_GPIO_OUT() {GPIOB->CRH&=0XFF0FFFFF;GPIOB->CRH|=3<<20;}
////IO��������											   
#define	AM2305_DATA_OUT PBout(13) //���ݶ˿�	PA0 
#define	AM2305_DATA_IN  PBin(13)  //���ݶ˿�	PA0 

extern u8 AM2305_Init(void);
extern void AM2305_UpdateData(void);

extern short  Temperature;
extern short  Humidity;

#endif
