/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_VIN_H
#define __AX_VIN_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

//X-SOFT 接口函数
void AX_VIN_Init(void);  //VIN 输入电压检测初始化
uint16_t AX_VIN_GetVol_X100(void);  //VIN 获得输入电压

#endif


