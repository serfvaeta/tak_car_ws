/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_MOTOR_H
#define __AX_MOTOR_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

//X-SOFT 接口函数
void AX_MOTOR_AB_Init(void); //电机PWM控制初始化
void AX_MOTOR_A_SetSpeed(int16_t speed);   //电机A控制
void AX_MOTOR_B_SetSpeed(int16_t speed);   //电机B控制

void AX_MOTOR_CD_Init(void); //电机PWM控制初始化
void AX_MOTOR_C_SetSpeed(int16_t speed);   //电机C控制
void AX_MOTOR_D_SetSpeed(int16_t speed);   //电机D控制

#endif


