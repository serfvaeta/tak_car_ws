#include "ax_obstacle_guard.h"

void AX_OBSTACLE_Reset(AX_OBSTACLE_State *state)
{
    state->blocked = 1;
}

AX_FOLLOW_Target AX_OBSTACLE_Apply(AX_OBSTACLE_State *state,
                                   const AX_OBSTACLE_Config *config,
                                   uint8_t range_valid,
                                   uint16_t distance_mm,
                                   AX_FOLLOW_Target desired)
{
    AX_FOLLOW_Target stopped = {0, 0};

    if (config == 0 || config->stop_mm == 0 ||
        config->resume_mm <= config->stop_mm ||
        config->max_forward_mm_s <= 0 || !range_valid) {
        state->blocked = 1;
        return stopped;
    }

    if (distance_mm < config->stop_mm) state->blocked = 1;
    else if (distance_mm >= config->resume_mm) state->blocked = 0;

    if (state->blocked) return stopped;

    if (desired.forward_mm_s < 0) desired.forward_mm_s = 0;
    if (desired.forward_mm_s > config->max_forward_mm_s)
        desired.forward_mm_s = config->max_forward_mm_s;
    return desired;
}
