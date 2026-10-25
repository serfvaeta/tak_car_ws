#include <assert.h>

#include "ax_follow_avoid_diag.h"

int main(void)
{
    const AX_AVOID_Config config = {
        .front_stop_mm = 350, .front_release_mm = 400, .side_min_mm = 400,
        .scan_max_angle_deg = 90, .scan_step_deg = 15,
        .coarse_scan_radius_deg = 20,
        .fine_scan_radius_deg = 12, .fine_scan_step_deg = 3,
        .track_probe_angle_deg = 25, .track_probe_step_deg = 5,
        .range_max_age_ms = 250, .approach_start_cm = 110,
        .approach_stop_cm = 100, .follow_max_mm_s = 300,
        .advance_max_mm_s = 100, .yaw_max_mrad_s = 500,
        .turn_timeout_ms = 5000, .turn_settle_max_yaw_mrad_s = 100,
        .turn_overshoot_mrad = 100, .advance_range_cap_mm = 2000,
        .advance_clearance_mm = 400,
        .stall_timeout_ms = 1000, .stall_min_mm = 20,
        .align_stop_turn_deg = 12,
        .azimuth_to_yaw_sign = 1, .rescan_wait_ms = 2000,
        .coarse_scan_timeout_ms = 7000, .fine_scan_timeout_ms = 5000
    };
    AX_AVOID_State state;
    AX_AVOID_Input input = {0};
    AX_AVOID_Output output;
    uint8_t packet[AX_AVOID_DIAG_LENGTH];

    AX_AVOID_Reset(&state);
    output = AX_AVOID_Step(&state, &config, &input);
    assert(AX_AVOID_DiagEncode(&state, &input, &output,
                               1, 0, packet) == AX_AVOID_DIAG_AOA_STALE);
    assert(packet[0] == AX_AVOID_ALIGN_TARGET);

    input.aoa_valid = 1;
    input.aoa_distance_cm = 130;
    input.aoa_azimuth_deg = 30;
    input.follow_target.yaw_mrad_s = 500;
    output = AX_AVOID_Step(&state, &config, &input);
    assert(output.yaw_mrad_s == 500);
    assert(AX_AVOID_DiagEncode(&state, &input, &output,
                               1, 130, packet) == AX_AVOID_DIAG_NONE);
    assert(packet[0] == AX_AVOID_ALIGN_TARGET);
    assert(packet[16] == 0x01 && packet[17] == 0xf4);

    input.aoa_azimuth_deg = 0;
    input.follow_target.yaw_mrad_s = 0;
    AX_AVOID_Step(&state, &config, &input);
    assert(state.phase == AX_AVOID_FORWARD);
    output = AX_AVOID_Step(&state, &config, &input);
    assert(output.forward_mm_s == 0);
    assert(AX_AVOID_DiagEncode(&state, &input, &output,
                               1, 130, packet) == AX_AVOID_DIAG_FRONT_STALE);
    return 0;
}
