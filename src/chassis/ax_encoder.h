/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_ENCODER_H
#define __AX_ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//X-SOFT 接口函数
void AX_ENCODER_A_Init(void);          //编码器初始化
uint16_t AX_ENCODER_A_GetCounter(void);          //编码器获取计数器数值
void AX_ENCODER_A_SetCounter(uint16_t count);    //编码器设置计数器数值

void AX_ENCODER_B_Init(void);          //编码器初始化
uint16_t AX_ENCODER_B_GetCounter(void);          //编码器获取计数器数值
void AX_ENCODER_B_SetCounter(uint16_t count);    //编码器设置计数器数值

void AX_ENCODER_C_Init(void);          //编码器初始化
uint16_t AX_ENCODER_C_GetCounter(void);          //编码器获取计数器数值
void AX_ENCODER_C_SetCounter(uint16_t count);    //编码器设置计数器数值

void AX_ENCODER_D_Init(void);          //编码器初始化
uint16_t AX_ENCODER_D_GetCounter(void);          //编码器获取计数器数值
void AX_ENCODER_D_SetCounter(uint16_t count);    //编码器设置计数器数值

#endif


