#ifndef AX_FOLLOW_AVOID2_POLICY_H
#define AX_FOLLOW_AVOID2_POLICY_H

#include <stdint.h>
#include "ax_xursc.h"

#define AX_AVOID2_COARSE_POINT_COUNT 31u
#define AX_AVOID2_FINE_POINT_COUNT 5u
#define AX_AVOID2_CANDIDATE_CAPACITY 5u
#define AX_AVOID2_VISITED_CAPACITY 11u

typedef enum {
    AX_AVOID2_WAIT_TARGET = 0,
    AX_AVOID2_COARSE_SCAN,
    AX_AVOID2_FINE_SCAN,
    AX_AVOID2_TURN,
    AX_AVOID2_VERIFY_FRONT,
    AX_AVOID2_MOVE,
    AX_AVOID2_WAIT_RESCAN
} AX_AVOID2_Phase;

typedef enum {
    AX_AVOID2_REASON_NONE = 0,
    AX_AVOID2_REASON_AOA_INVALID,
    AX_AVOID2_REASON_TARGET_NEAR,
    AX_AVOID2_REASON_COARSE_TIMEOUT,
    AX_AVOID2_REASON_NO_COARSE_CANDIDATE,
    AX_AVOID2_REASON_FINE_TIMEOUT,
    AX_AVOID2_REASON_NO_SAFE_FINE,
    AX_AVOID2_REASON_TURN_TIMEOUT,
    AX_AVOID2_REASON_FRONT_EMERGENCY,
    AX_AVOID2_REASON_VERIFY_TIMEOUT,
    AX_AVOID2_REASON_VERIFY_BLOCKED,
    AX_AVOID2_REASON_FRONT_INVALID,
    AX_AVOID2_REASON_MOVE_STALL,
    AX_AVOID2_REASON_BAD_CONFIG,
    AX_AVOID2_REASON_CHASSIS,
    AX_AVOID2_REASON_INVALID_STATE
} AX_AVOID2_Reason;

typedef struct {
    uint16_t safe_distance_mm;
    uint16_t emergency_distance_mm;
    int16_t scan_max_angle_deg;
    int16_t coarse_step_deg;
    int16_t fine_range_deg;
    int16_t fine_step_deg;
    uint8_t max_candidates;
    uint8_t max_adjust_count;
    uint16_t range_max_age_ms;
    uint16_t approach_start_cm;
    uint16_t approach_stop_cm;
    int16_t advance_max_mm_s;
    int16_t yaw_max_mrad_s;
    uint16_t turn_timeout_ms;
    uint16_t turn_settle_max_yaw_mrad_s;
    uint16_t advance_range_cap_mm;
    uint16_t advance_clearance_mm;
    uint16_t stall_timeout_ms;
    uint16_t stall_min_mm;
    uint16_t rescan_wait_ms;
    uint16_t coarse_scan_timeout_ms;
    uint32_t fine_scan_timeout_ms;
    uint16_t verify_timeout_ms;
    int8_t aoa_to_yaw_sign; /* Raw positive AOA to chassis yaw sign. */
} AX_AVOID2_Config;

typedef struct {
    uint32_t now_ms;
    uint8_t aoa_valid;
    uint16_t aoa_distance_cm;
    int16_t aoa_azimuth_deg; /* Normalized: positive right, negative left. */
    uint8_t range_valid;
    int16_t range_angle_deg;
    AX_XURSC_Status range_status;
    uint16_t range_distance_mm;
    uint32_t range_sequence;
    uint32_t range_triggered_ms;
    int16_t yaw_rate_mrad_s;
    int16_t forward_mm_s;
} AX_AVOID2_Input;

typedef struct {
    int16_t pan_angle_deg;
    int16_t forward_mm_s;
    int16_t yaw_mrad_s;
} AX_AVOID2_Output;

typedef struct {
    AX_AVOID2_Phase phase;
    AX_AVOID2_Reason reason;
    uint8_t initialized;
    uint8_t have_tick;
    uint8_t approach_needed;
    uint8_t scan_index;
    uint8_t candidate_count;
    uint8_t candidate_index;
    uint8_t fine_index;
    uint8_t fine_known_mask;
    uint8_t fine_has_unknown;
    uint8_t adjust_count;
    uint8_t visited_count;
    int8_t last_safe_side;
    int8_t turn_direction;
    int16_t pan_angle_deg;
    int16_t fine_center_angle_deg;
    int16_t candidate_angles[AX_AVOID2_CANDIDATE_CAPACITY];
    uint16_t candidate_distances_mm[AX_AVOID2_CANDIDATE_CAPACITY];
    int16_t visited_centers_deg[AX_AVOID2_VISITED_CAPACITY];
    uint16_t coarse_distances_mm[AX_AVOID2_COARSE_POINT_COUNT];
    uint32_t coarse_valid_mask;
    uint16_t fine_distances_mm[AX_AVOID2_FINE_POINT_COUNT];
    uint32_t range_wait_sequence;
    uint32_t last_tick_ms;
    uint32_t phase_start_ms;
    uint32_t range_after_ms;
    uint32_t segment_distance_mm;
    uint32_t segment_target_mm;
    uint16_t segment_distance_fraction;
    uint32_t stall_check_ms;
    uint32_t stall_check_distance_mm;
    uint32_t turn_settle_sequence;
    int32_t turn_progress_mrad;
    int32_t turn_target_mrad;
} AX_AVOID2_State;

void AX_AVOID2_Reset(AX_AVOID2_State *state);
AX_AVOID2_Output AX_AVOID2_Step(AX_AVOID2_State *state,
                                const AX_AVOID2_Config *config,
                                const AX_AVOID2_Input *input);

#endif
