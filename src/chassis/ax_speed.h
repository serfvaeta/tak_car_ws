/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_SPEED_H
#define __AX_SPEED_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"

//电机PID闭环速度控制函数
int16_t AX_SPEED_PidCtlA(float spd_target, float spd_current);   //PID控制函数，电机A
int16_t AX_SPEED_PidCtlB(float spd_target, float spd_current);    //PID控制函数，电机B
int16_t AX_SPEED_PidCtlC(float spd_target, float spd_current);    //PID控制函数，电机C
int16_t AX_SPEED_PidCtlD(float spd_target, float spd_current);    //PID控制函数，电机D


#endif


