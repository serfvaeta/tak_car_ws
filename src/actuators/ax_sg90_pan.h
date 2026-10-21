#ifndef AX_SG90_PAN_H
#define AX_SG90_PAN_H

#include <stdint.h>

/* SG90 signal: S1 / PB8 / TIM10_CH1. Power: 5 V and common GND.
 * No PWM is emitted until AX_SG90_PanInit() is called. */
void AX_SG90_PanPinsSafe(void); /* PB8 output low while module is inactive */
void AX_SG90_PanInit(void);
void AX_SG90_PanSetAngle(int16_t degrees); /* -90 to +90, 0 is center */
void AX_SG90_PanRelease(void);

#endif
