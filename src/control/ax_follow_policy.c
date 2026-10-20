#include "ax_follow_policy.h"

static int16_t ProportionalSpeed(uint32_t error, uint16_t gain,
                                 int16_t minimum, int16_t maximum)
{
    uint32_t speed = error * gain;
    if (speed < (uint32_t)minimum) speed = (uint32_t)minimum;
    if (speed > (uint32_t)maximum) speed = (uint32_t)maximum;
    return (int16_t)speed;
}

void AX_FOLLOW_Reset(AX_FOLLOW_State *state)
{
    state->advancing = 0;
    state->turning = 0;
    state->reversing = 0;
}

AX_FOLLOW_Target AX_FOLLOW_Compute(AX_FOLLOW_State *state,
                                   const AX_FOLLOW_Config *config,
                                   const AX_FOLLOW_Observation *observation)
{
    AX_FOLLOW_Target target = {0, 0};
    int32_t azimuth = observation->azimuth_deg;
    uint32_t distance_cm = observation->distance_cm;

    if (azimuth < 0) azimuth = -azimuth;

    if ((config->azimuth_to_yaw_sign != 1 &&
         config->azimuth_to_yaw_sign != -1) ||
        config->forward_mm_s <= 0 || config->turn_mrad_s <= 0 ||
        config->min_forward_mm_s <= 0 ||
        config->min_forward_mm_s > config->forward_mm_s ||
        config->min_turn_mrad_s <= 0 ||
        config->min_turn_mrad_s > config->turn_mrad_s ||
        config->forward_kp_mm_s_per_cm == 0u ||
        config->turn_kp_mrad_s_per_deg == 0u ||
        config->min_reverse_mm_s <= 0 ||
        config->min_reverse_mm_s > config->reverse_mm_s ||
        config->reverse_kp_mm_s_per_cm == 0u ||
        config->stop_forward_cm >= config->start_forward_cm ||
        config->min_valid_distance_cm >= config->start_reverse_cm ||
        config->start_reverse_cm >= config->stop_reverse_cm ||
        config->stop_reverse_cm >= config->stop_forward_cm ||
        config->start_forward_cm >= config->max_distance_cm ||
        config->stop_turn_deg >= config->start_turn_deg ||
        distance_cm <= config->min_valid_distance_cm ||
        distance_cm > config->max_distance_cm) {
        AX_FOLLOW_Reset(state);
        return target;
    }

    if (state->turning) {
        if (azimuth <= config->stop_turn_deg) state->turning = 0;
    } else if (azimuth >= config->start_turn_deg) {
        state->turning = 1;
    }

    if (state->turning) {
        int16_t turn_speed;
        turn_speed = ProportionalSpeed((uint32_t)azimuth,
                                      config->turn_kp_mrad_s_per_deg,
                                      config->min_turn_mrad_s,
                                      config->turn_mrad_s);
        target.yaw_mrad_s = observation->azimuth_deg > 0
                          ? turn_speed * config->azimuth_to_yaw_sign
                          : -turn_speed * config->azimuth_to_yaw_sign;
        return target;
    }

    if (state->reversing) {
        if (distance_cm >= config->stop_reverse_cm) state->reversing = 0;
    } else if (distance_cm < config->start_reverse_cm) {
        state->reversing = 1;
    }

    if (state->reversing) {
        state->advancing = 0;
        target.forward_mm_s = -ProportionalSpeed(
            config->stop_reverse_cm - distance_cm,
            config->reverse_kp_mm_s_per_cm,
            config->min_reverse_mm_s,
            config->reverse_mm_s);
        return target;
    }

    if (state->advancing) {
        if (distance_cm <= config->stop_forward_cm)
            state->advancing = 0;
    } else if (distance_cm >= config->start_forward_cm) {
        state->advancing = 1;
    }

    if (state->advancing)
        target.forward_mm_s = ProportionalSpeed(
            distance_cm - config->stop_forward_cm,
            config->forward_kp_mm_s_per_cm,
            config->min_forward_mm_s,
            config->forward_mm_s);
    return target;
}
