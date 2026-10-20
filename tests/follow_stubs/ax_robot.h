#ifndef TEST_AX_ROBOT_H
#define TEST_AX_ROBOT_H

#include <stdint.h>

#define CTL_PS2 0x01u
#define R_VX_LIMIT 2000
#define R_VW_LIMIT 6280

void AX_ROBOT_SetTargetIfActive(uint8_t source, int16_t x, int16_t y, int16_t w);

#endif
