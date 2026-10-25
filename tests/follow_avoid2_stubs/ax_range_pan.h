#ifndef TEST_AX_RANGE_PAN_H
#define TEST_AX_RANGE_PAN_H

#include <stdint.h>
#include "ax_xursc.h"

typedef struct {
    int16_t angle_deg;
    AX_XURSC_Status status;
    uint16_t distance_mm;
    uint32_t triggered_tick_ms;
    uint32_t received_tick_ms;
    uint32_t sequence;
} AX_RANGE_PAN_Sample;

uint8_t AX_RANGE_PAN_GetAvoidSample(uint32_t max_age_ms,
                                   AX_RANGE_PAN_Sample *out);
void AX_RANGE_PAN_AvoidRequestAngle(int16_t degrees);

#endif
