/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_UART1_H
#define __AX_UART1_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//OpenCTR驱动接口函数
void AX_UART1_Init(uint32_t baud);  //UART 调试串口初始化

//uint8_t AX_UART_DB_GetData(uint8_t *pbuf);  //UART 获取接收的数据
//void AX_UART_DB_SendPacket(uint8_t *pbuf, uint8_t len, uint8_t num);  //UART 发送数据（X-Protocol协议）

#endif 


