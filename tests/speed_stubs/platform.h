#ifndef TEST_SPEED_PLATFORM_H
#define TEST_SPEED_PLATFORM_H

#include <stdint.h>

#define __AX_SPEED_H
#define __AX_ROBOT_H

extern int16_t ax_motor_kp;
extern int16_t ax_motor_kd;

int16_t AX_SPEED_PidCtlA(float target, float current);
int16_t AX_SPEED_PidCtlB(float target, float current);
int16_t AX_SPEED_PidCtlC(float target, float current);
int16_t AX_SPEED_PidCtlD(float target, float current);

#endif
