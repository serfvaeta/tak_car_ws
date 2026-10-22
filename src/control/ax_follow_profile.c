#include "ax_follow_profile.h"

#include "openctr_follow_config.h"

AX_FOLLOW_Config AX_FOLLOW_Profile(int16_t forward_max_mm_s)
{
    AX_FOLLOW_Config config = {
        .start_reverse_cm = OPENCTR_FOLLOW_START_REVERSE_CM,
        .start_forward_cm = OPENCTR_FOLLOW_START_FORWARD_CM,
        .stop_forward_cm = OPENCTR_FOLLOW_STOP_FORWARD_CM,
        .max_distance_cm = OPENCTR_FOLLOW_MAX_DISTANCE_CM,
        .start_turn_deg = OPENCTR_FOLLOW_START_TURN_DEG,
        .stop_turn_deg = OPENCTR_FOLLOW_STOP_TURN_DEG,
        .forward_mm_s = forward_max_mm_s,
        .turn_mrad_s = OPENCTR_FOLLOW_TURN_MAX_MRAD_S,
        .azimuth_to_yaw_sign = OPENCTR_FOLLOW_AZIMUTH_TO_YAW_SIGN,
        .min_forward_mm_s = OPENCTR_FOLLOW_FORWARD_MIN_MM_S,
        .min_turn_mrad_s = OPENCTR_FOLLOW_TURN_MIN_MRAD_S,
        .forward_kp_mm_s_per_cm = OPENCTR_FOLLOW_FORWARD_KP_MM_S_PER_CM,
        .turn_kp_mrad_s_per_deg = OPENCTR_FOLLOW_TURN_KP_MRAD_S_PER_DEG,
        .min_valid_distance_cm = OPENCTR_FOLLOW_MIN_VALID_DISTANCE_CM,
        .stop_reverse_cm = OPENCTR_FOLLOW_STOP_REVERSE_CM,
        .min_reverse_mm_s = OPENCTR_FOLLOW_REVERSE_MIN_MM_S,
        .reverse_mm_s = OPENCTR_FOLLOW_REVERSE_MAX_MM_S,
        .reverse_kp_mm_s_per_cm = OPENCTR_FOLLOW_REVERSE_KP_MM_S_PER_CM
    };
    return config;
}
