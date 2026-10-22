#include <assert.h>
#include <stdint.h>

#include "ax_follow_avoid_diag.h"

int main(void)
{
    AX_AVOID_State state;
    const AX_AVOID_Config config = {
        .front_stop_mm = 350,
        .front_release_mm = 650,
        .side_min_mm = 600,
        .scan_max_angle_deg = 90,
        .scan_step_deg = 15,
        .track_probe_angle_deg = 25,
        .track_probe_step_deg = 5,
        .range_max_age_ms = 250,
        .approach_start_cm = 110,
        .approach_stop_cm = 100,
        .follow_max_mm_s = 300,
        .advance_max_mm_s = 100,
        .yaw_max_mrad_s = 500,
        .turn_timeout_ms = 5000,
        .turn_settle_max_yaw_mrad_s = 100,
        .turn_overshoot_mrad = 100,
        .advance_segment_mm = 150,
        .max_distance_mm = 1200,
        .max_time_ms = 25000,
        .stall_timeout_ms = 1000,
        .stall_min_mm = 20,
        .rejoin_clear_samples = 2,
        .rejoin_stop_turn_deg = 5,
        .azimuth_to_yaw_sign = 1,
        .rescan_wait_ms = 2000,
        .side_scan_timeout_ms = 8000
    };
    AX_AVOID_Input input = {0};
    AX_AVOID_Output output = {0};
    uint8_t packet[AX_AVOID_DIAG_LENGTH];

    AX_AVOID_Reset(&state);
    input.now_ms = 1200;
    input.aoa_valid = 0;
    input.range_valid = 0;
    AX_AVOID_Step(&state, &config, &input);
    assert(AX_AVOID_DiagEncode(&state, &config, &input, &output,
                               1, 0, packet) ==
           AX_AVOID_DIAG_AOA_STALE);
    assert(packet[0] == AX_AVOID_WAIT_FRONT);
    assert(packet[1] == AX_AVOID_DIAG_AOA_STALE);
    assert(packet[2] == 0x04); /* FWD; neither input is fresh. */

    input.aoa_valid = 1;
    input.aoa_distance_cm = 130;
    input.range_valid = 1;
    input.aoa_azimuth_deg = -18;
    input.range_angle_deg = 0;
    input.range_status = AX_XURSC_READY;
    input.range_distance_mm = 1000;
    input.follow_target.forward_mm_s = 160;
    input.yaw_rate_mrad_s = -250;
    input.forward_mm_s = 42;
    input.now_ms = 1220;
    AX_AVOID_Step(&state, &config, &input);
    output.forward_mm_s = 160;
    assert(AX_AVOID_DiagEncode(&state, &config, &input, &output,
                               1, 130, packet) ==
           AX_AVOID_DIAG_NONE);
    assert(packet[0] == AX_AVOID_TRACK);
    assert(packet[2] == 0x1f); /* AOA, range, FWD, ready, moving. */
    assert(packet[4] == 0xff && packet[5] == 0xee); /* -18 deg */
    assert(packet[6] == 0x00 && packet[7] == 130);
    assert(packet[10] == 0x03 && packet[11] == 0xe8); /* 1000 mm */
    assert(packet[12] == 0x00 && packet[13] == 160);
    assert(packet[14] == 0x00 && packet[15] == 42);
    assert(packet[18] == 0xff && packet[19] == 0x06); /* -250 mrad/s */

    assert(AX_AVOID_DiagEncode(&state, &config, &input, &output,
                               0, 130, packet) ==
           AX_AVOID_DIAG_CHASSIS);
    assert(packet[2] == 0x1b);

    /* Rejoining may rotate without a front echo; diagnostics should report
     * the commanded turn rather than a front-range fault. */
    state.phase = AX_AVOID_TRY_REJOIN;
    input.now_ms = 1240;
    input.aoa_azimuth_deg = 85;
    input.range_valid = 0;
    output = AX_AVOID_Step(&state, &config, &input);
    assert(output.yaw_mrad_s == 500);
    assert(AX_AVOID_DiagEncode(&state, &config, &input, &output,
                               1, 130, packet) == AX_AVOID_DIAG_NONE);

    /* A recoverable timeout keeps its reason visible while the car is
     * stopped at center waiting for a new forward echo. */
    state.phase = AX_AVOID_WAIT_FRONT;
    state.recovery_rescan = 1;
    state.hold_reason = AX_AVOID_DIAG_TURN_TIMEOUT;
    input.aoa_valid = 1;
    output.yaw_mrad_s = 0;
    assert(AX_AVOID_DiagEncode(&state, &config, &input, &output,
                               1, 130, packet) == AX_AVOID_DIAG_TURN_TIMEOUT);
    assert(packet[0] == AX_AVOID_WAIT_FRONT);
    assert(packet[1] == AX_AVOID_DIAG_TURN_TIMEOUT);

    /* After a probe timeout, an old centered echo cannot release WAIT_FRONT. */
    state.recovery_rescan = 0;
    state.track_wait_fresh_front = 1;
    input.range_valid = 1;
    input.range_angle_deg = 0;
    input.range_status = AX_XURSC_READY;
    input.range_distance_mm = 1000;
    assert(AX_AVOID_DiagEncode(&state, &config, &input, &output,
                               1, 130, packet) == AX_AVOID_DIAG_FRONT_STALE);
    return 0;
}
