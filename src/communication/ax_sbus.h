/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_SBUS_H
#define __AX_SBUS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//OpenCTR驱动接口函数
void AX_SBUS_Init(void);  //UART SBUS串口初始化
uint8_t AX_SBUS_GetRxData(uint16_t *pbuf);  //UART 获取遥控器接收的数据


#endif 


