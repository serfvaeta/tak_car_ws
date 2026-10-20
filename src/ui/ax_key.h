/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_KEY_H
#define __AX_KEY_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

// 接口函数
void AX_KEY_Init(void);  //按键初始化
uint8_t AX_KEY_Scan(void);  //按键扫描

#endif 


