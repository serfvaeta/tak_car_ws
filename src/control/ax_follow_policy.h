#ifndef AX_FOLLOW_POLICY_H
#define AX_FOLLOW_POLICY_H

#include <stdint.h>

typedef struct {
    uint32_t distance_cm;
    int16_t azimuth_deg;
} AX_FOLLOW_Observation;

typedef struct {
    uint16_t start_reverse_cm;
    uint16_t start_forward_cm;
    uint16_t stop_forward_cm;
    uint16_t max_distance_cm;
    uint8_t start_turn_deg;
    uint8_t stop_turn_deg;
    int16_t forward_mm_s; /* maximum forward speed */
    int16_t turn_mrad_s;  /* maximum turn rate */
    int8_t azimuth_to_yaw_sign; /* +1 or -1 after mounting calibration; 0 stops. */
    int16_t min_forward_mm_s;
    int16_t min_turn_mrad_s;
    uint16_t forward_kp_mm_s_per_cm;
    uint16_t turn_kp_mrad_s_per_deg;
    uint16_t min_valid_distance_cm;
    uint16_t stop_reverse_cm;
    int16_t min_reverse_mm_s;
    int16_t reverse_mm_s; /* maximum reverse speed */
    uint16_t reverse_kp_mm_s_per_cm;
} AX_FOLLOW_Config;

typedef struct {
    uint8_t advancing;
    uint8_t turning;
    uint8_t reversing;
} AX_FOLLOW_State;

typedef struct {
    int16_t forward_mm_s;
    int16_t yaw_mrad_s;
} AX_FOLLOW_Target;

void AX_FOLLOW_Reset(AX_FOLLOW_State *state);
AX_FOLLOW_Target AX_FOLLOW_Compute(AX_FOLLOW_State *state,
                                   const AX_FOLLOW_Config *config,
                                   const AX_FOLLOW_Observation *observation);

#endif
