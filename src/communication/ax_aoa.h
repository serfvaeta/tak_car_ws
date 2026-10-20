#ifndef AX_AOA_H
#define AX_AOA_H

#include <stdint.h>
#include "ax_aoa_protocol.h"

#define ID_UTX_AOA_RAW    0x13u /* USB-COM: raw base-station bytes */
#define ID_UTX_AOA_STATUS 0x14u /* USB-COM: received and dropped counters */

typedef struct {
    AX_AOA_Measurement measurement;
    uint32_t received_tick; /* FreeRTOS tick; currently 1 tick = 1 ms. */
} AX_AOA_Sample;

/* USART2 IRQ input. This module only observes base-station bytes. */
void AX_AOA_OnRxByte(uint8_t byte);

/* Called from Robot_Task every 20 ms; forwards diagnostic frames to USB-COM. */
void AX_AOA_TaskStep(void);

/* Task-context API. Returns 1 only when the latest location frame is no older
 * than max_age_ms. out includes the measured fields and receipt tick. */
uint8_t AX_AOA_GetLatest(AX_AOA_Sample *out, uint32_t max_age_ms);

#endif
