#ifndef AX_FOLLOW_AVOID_POLICY_H
#define AX_FOLLOW_AVOID_POLICY_H

#include <stdint.h>
#include "ax_follow_policy.h"
#include "ax_xursc.h"

#define AX_AVOID_SCAN_MAX_POINTS 13

typedef enum {
    AX_AVOID_WAIT_FRONT = 0,
    AX_AVOID_TRACK,
    AX_AVOID_SCAN_SIDE,
    AX_AVOID_TURN_OUT,
    AX_AVOID_ADVANCE,
    AX_AVOID_TRY_REJOIN,
    AX_AVOID_HOLD,
    AX_AVOID_RESCAN_WAIT
} AX_AVOID_Phase;

typedef enum {
    AX_AVOID_DIAG_NONE = 0,
    AX_AVOID_DIAG_AOA_STALE = 1,
    AX_AVOID_DIAG_FRONT_STALE = 2,
    AX_AVOID_DIAG_WAIT_LEFT = 3,
    AX_AVOID_DIAG_WAIT_RIGHT = 4,
    AX_AVOID_DIAG_NO_SIDE = 5,
    AX_AVOID_DIAG_TURN_CENTER = 6,
    AX_AVOID_DIAG_TURN_TIMEOUT = 7,
    AX_AVOID_DIAG_TURN_BLOCKED = 8,
    AX_AVOID_DIAG_ADVANCE_STALL = 9,
    AX_AVOID_DIAG_DISTANCE_LIMIT = 10,
    AX_AVOID_DIAG_TIME_LIMIT = 11,
    AX_AVOID_DIAG_REJOIN_CLEAR = 12,
    AX_AVOID_DIAG_REJOIN_TIMEOUT = 13,
    AX_AVOID_DIAG_REJOIN_TURN_LIMIT = 14,
    AX_AVOID_DIAG_LOOP_LATE = 15,
    AX_AVOID_DIAG_CHASSIS = 16,
    AX_AVOID_DIAG_BAD_CONFIG = 17,
    AX_AVOID_DIAG_FOLLOW_STOP = 18,
    AX_AVOID_DIAG_FRONT_CLOSE = 19,
    AX_AVOID_DIAG_HOLD_UNKNOWN = 20,
    AX_AVOID_DIAG_WAIT_RESCAN = 21
} AX_AVOID_DiagReason;

typedef struct {
    uint16_t front_stop_mm;
    uint16_t front_release_mm;
    uint16_t side_min_mm;
    int16_t scan_max_angle_deg;
    int16_t scan_step_deg;
    int16_t track_probe_angle_deg;
    int16_t track_probe_step_deg;
    uint16_t range_max_age_ms;
    uint16_t aoa_max_age_ms;
    uint16_t approach_start_cm;
    uint16_t approach_stop_cm;
    int16_t follow_max_mm_s;
    int16_t advance_max_mm_s;
    int16_t yaw_max_mrad_s;
    uint16_t turn_timeout_ms;
    uint16_t turn_settle_max_yaw_mrad_s;
    uint16_t turn_overshoot_mrad;
    uint16_t advance_segment_mm;
    /* Zero disables the cumulative limit; per-action limits still apply. */
    uint16_t max_distance_mm;
    uint32_t max_time_ms;
    uint16_t stall_timeout_ms;
    uint16_t stall_min_mm;
    uint8_t rejoin_clear_samples;
    uint8_t rejoin_stop_turn_deg;
    int8_t azimuth_to_yaw_sign;
    uint16_t rescan_wait_ms;
    uint16_t side_scan_timeout_ms;
} AX_AVOID_Config;

typedef struct {
    uint32_t now_ms;
    uint8_t aoa_valid;
    uint16_t aoa_distance_cm;
    int16_t aoa_azimuth_deg;
    AX_FOLLOW_Target follow_target;
    uint8_t range_valid;
    int16_t range_angle_deg;
    AX_XURSC_Status range_status;
    uint16_t range_distance_mm;
    uint32_t range_sequence;
    uint32_t range_triggered_ms;
    int16_t yaw_rate_mrad_s;
    int16_t forward_mm_s;
} AX_AVOID_Input;

typedef struct {
    int16_t pan_angle_deg;
    int16_t forward_mm_s;
    int16_t yaw_mrad_s;
} AX_AVOID_Output;

typedef struct {
    AX_AVOID_Phase phase;
    AX_AVOID_DiagReason hold_reason;
    int16_t pan_angle_deg;
    int16_t detour_angle_deg;
    int8_t detour_side;
    uint8_t scan_index;
    uint8_t detour_active;
    uint8_t approach_needed;
    uint8_t turn_front_ready_seen;
    uint8_t turn_settle_pending;
    uint8_t initialized;
    uint8_t have_tick;
    uint8_t rejoin_clear_count;
    uint8_t rejoin_aligned;
    uint8_t recovery_rescan;
    uint16_t track_probe_index;
    uint8_t track_wait_fresh_front;
    uint32_t scan_wait_sequence;
    uint32_t track_sample_sequence;
    uint32_t track_last_sample_ms;
    uint32_t last_tick_ms;
    uint32_t phase_start_ms;
    uint32_t detour_start_ms;
    uint32_t segment_distance_mm;
    uint32_t total_distance_mm;
    int32_t turn_progress_mrad;
    int32_t turn_target_mrad;
    uint32_t turn_settle_ms;
    uint32_t turn_settle_sequence;
    uint32_t rejoin_progress_mrad;
    uint32_t last_rejoin_clear_sequence;
    uint32_t rejoin_aligned_ms;
    uint32_t recovery_sequence;
    uint32_t recovery_start_ms;
    uint16_t scan_valid_mask;
    uint16_t scan_distance_mm[AX_AVOID_SCAN_MAX_POINTS];
} AX_AVOID_State;

void AX_AVOID_Reset(AX_AVOID_State *state);
AX_AVOID_Output AX_AVOID_Step(AX_AVOID_State *state,
                              const AX_AVOID_Config *config,
                              const AX_AVOID_Input *input);

#endif
