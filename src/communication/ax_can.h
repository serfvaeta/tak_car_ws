/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_CAN_H
#define __AX_CAN_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

//接口函数
void AX_CAN_Init(void);  //CAN初始化
uint8_t AX_CAN_SendPacket(uint32_t id, uint32_t len, uint8_t *pbuf); //CAN发送数据

#endif


