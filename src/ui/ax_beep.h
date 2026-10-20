/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_BEEP_H
#define __AX_BEEP_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

// 接口函数
void AX_BEEP_Init(void);

// 蜂鸣器操作函数宏定义
#define AX_BEEP_Off()       GPIO_ResetBits(GPIOE, GPIO_Pin_1)       //蜂鸣器不鸣
#define AX_BEEP_On()		GPIO_SetBits(GPIOE, GPIO_Pin_1)     //蜂鸣器鸣叫
#define AX_BEEP_Toggle()    GPIO_WriteBit(GPIOE, GPIO_Pin_1, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1)))    //蜂鸣器状态翻转

#endif 


