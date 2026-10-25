#ifndef TEST_AX_ROBOT_H
#define TEST_AX_ROBOT_H

#include <stdint.h>

#define ROBOT_FWD 2
#define CTL_PS2 1
#define R_VX_LIMIT 2000
#define R_VW_LIMIT 4000
#define ID_UTX_AVOID2_DIAG 0x17u

typedef struct { int16_t GYRO_Z; } TestImu;
typedef struct { int16_t RT_IX; } TestVelocity;

extern int ax_robot_type;
extern TestImu R_Imu;
extern TestVelocity R_Vel;

uint32_t xTaskGetTickCount(void);
void AX_ROBOT_SetTargetIfActive(uint8_t source, int16_t forward_mm_s,
                                int16_t lateral_mm_s, int16_t yaw_mrad_s);

#endif
