#ifndef AX_RANGE_PAN_H
#define AX_RANGE_PAN_H

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

/* Called from the USB-COM receive interrupt. The newest request wins. */
void AX_RANGE_PAN_RequestAngle(int16_t degrees);

/* Manual USB pan/range is permitted only outside PS2 control. */
void AX_RANGE_PAN_SetManualAllowed(uint8_t allowed);

/* Robot_Task selects the PS2 scan mode. Entry centers the SG90 at 0 degrees. */
void AX_RANGE_PAN_SetScanEnabled(uint8_t enabled);

/* AVOID takes ownership of the pan/range scheduler and starts centered. */
void AX_RANGE_PAN_SetAvoidEnabled(uint8_t enabled);
/* Valid only while AVOID owns the scheduler. */
void AX_RANGE_PAN_AvoidRequestAngle(int16_t degrees);
/* TRACK's 0 / +/-5 degree probes use the shorter settle interval. */
void AX_RANGE_PAN_AvoidRequestProbeAngle(int16_t degrees);
/* Returns the latest sample, including timeout/out-of-range status, if fresh. */
uint8_t AX_RANGE_PAN_GetAvoidSample(uint32_t max_age_ms,
                                   AX_RANGE_PAN_Sample *out);

/* One new X press decreases 15 degrees; one new B press increases 15 degrees. */
void AX_RANGE_PAN_ScanButtons(uint8_t x_down, uint8_t b_down);

/* Called once per 20 ms from Robot_Task. */
void AX_RANGE_PAN_TaskStep(void);

#endif
