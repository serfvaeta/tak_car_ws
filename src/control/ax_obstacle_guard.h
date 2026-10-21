#ifndef AX_OBSTACLE_GUARD_H
#define AX_OBSTACLE_GUARD_H

#include <stdint.h>
#include "ax_follow_policy.h"

typedef struct {
    uint16_t stop_mm;
    uint16_t resume_mm;
    int16_t max_forward_mm_s;
} AX_OBSTACLE_Config;

typedef struct {
    uint8_t blocked;
} AX_OBSTACLE_State;

/* Unwired distance guard for a future obstacle-control mode. Reset starts
 * blocked until a clear, valid front reading arrives. */
void AX_OBSTACLE_Reset(AX_OBSTACLE_State *state);
AX_FOLLOW_Target AX_OBSTACLE_Apply(AX_OBSTACLE_State *state,
                                   const AX_OBSTACLE_Config *config,
                                   uint8_t range_valid,
                                   uint16_t distance_mm,
                                   AX_FOLLOW_Target desired);

#endif
