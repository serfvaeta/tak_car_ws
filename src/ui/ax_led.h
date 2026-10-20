/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_LED_H
#define __AX_LED_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

//接口函数
void AX_LED_Init(void);

#define AX_LED_Red_Off()        GPIO_SetBits(GPIOD, GPIO_Pin_3)      //LEDR红色熄灭
#define AX_LED_Red_On()  	    GPIO_ResetBits(GPIOD, GPIO_Pin_3)    //LEDR红色点亮
#define AX_LED_Red_Toggle()     GPIO_WriteBit(GPIOD, GPIO_Pin_3, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_3)))	//LEDR红色状态翻转

#define AX_LED_Green_Off()  	GPIO_SetBits(GPIOD, GPIO_Pin_4)      //LEDG绿色熄灭
#define AX_LED_Green_On()		GPIO_ResetBits(GPIOD, GPIO_Pin_4)    //LEDG绿色点亮
#define AX_LED_Green_Toggle()   GPIO_WriteBit(GPIOD, GPIO_Pin_4, (BitAction) (1 - GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_4)))	//LEDG绿色状态翻转


#endif 


