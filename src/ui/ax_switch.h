/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_SWITCH_H
#define __AX_SWITCH_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

//X-SOFT 接口函数
void AX_SW_Init(void);       //拨码开关初始化
uint8_t AX_SW_GetS1(void);   //获取开关1状态
uint8_t AX_SW_GetS2(void);   //获取开关2状态
uint8_t AX_SW_GetS12(void);  //获取开关总状态

#endif 


