#include <assert.h>

#include "openctr_avoid_config.h"
#include "ax_follow_avoid_policy.h"

static const AX_AVOID_Config config = {
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
    .advance_segment_cap_mm = OPENCTR_AVOID_ADVANCE_SEGMENT_CAP_MM,
    .advance_clearance_mm = 200,
    .stall_timeout_ms = 1000, .stall_min_mm = 20,
    .align_stop_turn_deg = 12,
    .azimuth_to_yaw_sign = 1, .rescan_wait_ms = 2000,
    .coarse_scan_timeout_ms = 7000, .fine_scan_timeout_ms = 5000
};

static AX_AVOID_Input input(void)
{
    AX_AVOID_Input in = {0};
    in.aoa_valid = 1;
    in.aoa_distance_cm = 130;
    in.aoa_azimuth_deg = 30;
    in.follow_target.yaw_mrad_s = 500;
    in.range_valid = 1;
    in.range_status = AX_XURSC_READY;
    in.range_distance_mm = 900;
    in.range_sequence = 1;
    return in;
}

static AX_AVOID_Output echo(AX_AVOID_State *state, AX_AVOID_Input *in,
                             int16_t angle, uint16_t mm, uint32_t dt)
{
    in->now_ms += dt;
    in->range_valid = 1;
    in->range_status = AX_XURSC_READY;
    in->range_angle_deg = angle;
    in->range_distance_mm = mm;
    in->range_triggered_ms = in->now_ms - 20;
    ++in->range_sequence;
    return AX_AVOID_Step(state, &config, in);
}

int main(void)
{
    AX_AVOID_State state;
    AX_AVOID_Input in = input();
    AX_AVOID_Output out;

    AX_AVOID_Reset(&state);
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_ALIGN_TARGET);
    assert(out.yaw_mrad_s == 500 && out.forward_mm_s == 0);

    in.aoa_azimuth_deg = 0;
    in.follow_target.yaw_mrad_s = 0;
    in.follow_target.forward_mm_s = 200;
    in.now_ms = 20;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_FORWARD);
    out = echo(&state, &in, 0, 400, 100);
    assert(state.phase == AX_AVOID_FORWARD && out.forward_mm_s == 200);
    out = echo(&state, &in, 0, 400, 100);
    assert(state.phase == AX_AVOID_FORWARD && out.forward_mm_s == 200);
    out = echo(&state, &in, 5, 330, 100);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    assert(out.forward_mm_s == 0 && out.pan_angle_deg == 90);

    for (int16_t angle = 90; angle >= -90; angle -= 15)
        out = echo(&state, &in, angle,
                   angle == 15 || angle == 30 || angle == 45 ||
                   angle == 60 || angle == 75 ? 3000 : 350,
                   350);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    assert(state.scan_stage == AX_AVOID_SCAN_FINE);
    for (int16_t angle = 57; angle >= 33; angle -= 3)
        out = echo(&state, &in, angle,
                   3000, 350);
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(state.detour_angle_deg == 45);
    assert(out.forward_mm_s == 0);

    in.range_angle_deg = 0;
    in.yaw_rate_mrad_s = 500;
    for (unsigned i = 0; i < 16; ++i)
        out = echo(&state, &in, 0, 330, 100);
    in.yaw_rate_mrad_s = 0;
    out = echo(&state, &in, 0, 400, 100);
    assert(state.phase == AX_AVOID_TURN_OUT && out.forward_mm_s == 0);
    out = echo(&state, &in, 0, 400, 100);
    assert(state.phase == AX_AVOID_ADVANCE && out.forward_mm_s == 100);
    assert(state.advance_target_mm == 600);

    in.forward_mm_s = 100;
    for (unsigned i = 0; i < 60; ++i)
        out = echo(&state, &in, 0, 400, 100);
    assert(state.phase == AX_AVOID_ALIGN_TARGET);
    assert(out.forward_mm_s == 0);

    in.aoa_azimuth_deg = -25;
    in.follow_target.yaw_mrad_s = -500;
    out = echo(&state, &in, 0, 400, 20);
    assert(state.phase == AX_AVOID_ALIGN_TARGET);
    assert(out.yaw_mrad_s == -500 && out.forward_mm_s == 0);
    in.aoa_azimuth_deg = -8;
    in.follow_target.yaw_mrad_s = -250;
    out = echo(&state, &in, 0, 400, 20);
    assert(state.phase == AX_AVOID_ALIGN_TARGET && out.yaw_mrad_s == -250);

    /* A blocked post-turn front must rejoin the AOA heading before rescanning. */
    AX_AVOID_Reset(&state);
    in = input();
    state.phase = AX_AVOID_TURN_OUT;
    state.approach_needed = 1;
    state.detour_active = 1;
    state.turn_target_mrad = 500;
    state.turn_progress_mrad = 500;
    state.detour_angle_deg = 30;
    state.detour_side = 1;
    state.pan_angle_deg = 0;
    in.aoa_azimuth_deg = -25;
    in.follow_target.yaw_mrad_s = -500;
    in.yaw_rate_mrad_s = 0;
    echo(&state, &in, 0, 350, 100);
    assert(state.phase == AX_AVOID_TURN_OUT);
    out = echo(&state, &in, 0, 350, 100);
    assert(state.phase == AX_AVOID_ALIGN_TARGET);
    assert(state.hold_reason == AX_AVOID_DIAG_TURN_BLOCKED);
    assert(!state.detour_active && out.forward_mm_s == 0);
    out = echo(&state, &in, 0, 350, 20);
    assert(state.phase == AX_AVOID_ALIGN_TARGET && out.yaw_mrad_s == -500);
    in.aoa_azimuth_deg = 0;
    in.follow_target.yaw_mrad_s = 0;
    echo(&state, &in, 0, 350, 20);
    assert(state.phase == AX_AVOID_FORWARD);
    out = echo(&state, &in, 0, 350, 100);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    assert(out.pan_angle_deg == 90 && out.forward_mm_s == 0);
    return 0;
}
