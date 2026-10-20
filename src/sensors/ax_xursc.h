#ifndef AX_XURSC_H
#define AX_XURSC_H

#include <stdint.h>

/* XTARK XURSC on the H60 factory wiring:
 * ECHO = PC6 / TIM8_CH1, TRIG = PC7, VCC = 5 V, common GND.
 * Call the API from a task, never from an interrupt. The driver does not
 * create tasks or change the chassis motion. */
typedef enum {
    AX_XURSC_UNINITIALIZED = 0,
    AX_XURSC_IDLE,
    AX_XURSC_WAITING,
    AX_XURSC_READY,
    AX_XURSC_TIMEOUT,
    AX_XURSC_OUT_OF_RANGE
} AX_XURSC_Status;

/* Safe state for a wired but inactive module: PC6 input, PC7 output low. */
void AX_XURSC_PinsSafe(void);
void AX_XURSC_Init(void);
/* Call after AX_DELAY_Init(). Each trigger requires a later GetStatus() call.
 * 1: started; 0: not initialized, busy or less than 60 ms since last pulse. */
uint8_t AX_XURSC_Trigger(void);
/* distance_mm is written only when the return value is AX_XURSC_READY. */
AX_XURSC_Status AX_XURSC_GetStatus(uint16_t *distance_mm);

#endif
