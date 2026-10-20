/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_SERVO_H
#define __AX_SERVO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//X-SOFT 接口函数
void AX_SERVO_S1_Init(void);                 //舵机接口初始化
void AX_SERVO_S1_SetAngle(int16_t angle);    //舵机控制 
void AX_SERVO_S1_Release(void);               //停止 S1 PWM 输出

void AX_SERVO_S2_Init(void);                 //舵机接口初始化
void AX_SERVO_S2_SetAngle(int16_t angle);    //舵机控制  

void AX_SERVO_S3456_Init(void);              //舵机接口初始化
void AX_SERVO_S3_SetAngle(int16_t angle);    //舵机控制   
void AX_SERVO_S4_SetAngle(int16_t angle);    //舵机控制
void AX_SERVO_S5_SetAngle(int16_t angle);    //舵机控制   
void AX_SERVO_S6_SetAngle(int16_t angle);    //舵机控制

#endif



