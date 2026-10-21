/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_CONTROL_H
#define __AX_CONTROL_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f4xx.h"
#include "ax_ps2.h"
#include "ax_ps2_mode.h"

//机器人各种控制方式处理文件
AX_PS2_InputAction AX_CTL_Ps2(JOYSTICK_TypeDef joystick); //PS2手柄控制
void AX_CTL_App(void);     //APP控制
void AX_CTL_RemoteSbus(void);  //SBUS航模遥控器控制

#endif

