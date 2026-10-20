#include <assert.h>
#include <stdint.h>

#include "ax_follow_avoid_policy.h"

static const AX_AVOID_Config config = {
    .front_stop_mm = 500,
    .front_release_mm = 650,
    .side_min_mm = 800,
    .scan_max_angle_deg = 90,
    .scan_step_deg = 15,
    .range_max_age_ms = 250,
    .aoa_max_age_ms = 300,
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

static AX_AVOID_Input input(void)
{
    AX_AVOID_Input in = {0};
    in.aoa_valid = 1;
    in.follow_target.forward_mm_s = 400;
    in.follow_target.yaw_mrad_s = 900;
    in.range_valid = 1;
    in.range_angle_deg = 0;
    in.range_status = AX_XURSC_READY;
    in.range_distance_mm = 1000;
    in.range_sequence = 1;
    return in;
}

static void start_scan(AX_AVOID_State *state, AX_AVOID_Input *in)
{
    AX_AVOID_Reset(state);
    AX_AVOID_Step(state, &config, in);
    assert(state->phase == AX_AVOID_TRACK);
    in->now_ms = 20;
    in->range_distance_mm = 330;
    in->range_triggered_ms = in->now_ms;
    ++in->range_sequence;
    AX_AVOID_Output out = AX_AVOID_Step(state, &config, in);
    assert(state->phase == AX_AVOID_SCAN_SIDE);
    assert(out.pan_angle_deg == 90);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
}

static AX_AVOID_Output finish_scan(AX_AVOID_State *state, AX_AVOID_Input *in,
                                    int16_t clear_angle)
{
    AX_AVOID_Output out = {0};
    for (int16_t angle = 90; angle >= -90; angle -= 15) {
        in->now_ms += 350;
        in->range_angle_deg = angle;
        in->range_distance_mm = angle == clear_angle ? 1000 : 350;
        in->range_triggered_ms = in->now_ms - 20;
        ++in->range_sequence;
        out = AX_AVOID_Step(state, &config, in);
        assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    }
    return out;
}

static void enter_advance(AX_AVOID_State *state, AX_AVOID_Input *in)
{
    start_scan(state, in);
    finish_scan(state, in, 45);
    assert(state->phase == AX_AVOID_TURN_OUT);
    assert(state->turn_target_mrad >= 785 &&
           state->turn_target_mrad <= 786);

    in->now_ms += 20;
    in->range_valid = 0;
    AX_AVOID_Output out = AX_AVOID_Step(state, &config, in);
    assert(state->phase == AX_AVOID_TURN_OUT);
    assert(out.yaw_mrad_s == 0);
    in->range_valid = 1;
    in->range_angle_deg = 0;
    in->range_distance_mm = 330;
    in->yaw_rate_mrad_s = 500;
    for (unsigned n = 0; n < 16; ++n) {
        in->now_ms += 100;
        in->range_distance_mm = n == 15 ? 900 : 330;
        in->range_triggered_ms = in->now_ms - 20;
        ++in->range_sequence;
        out = AX_AVOID_Step(state, &config, in);
    }
    assert(state->phase == AX_AVOID_TURN_OUT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in->now_ms += 100;
    in->range_triggered_ms = in->now_ms - 20;
    in->yaw_rate_mrad_s = 0;
    ++in->range_sequence;
    out = AX_AVOID_Step(state, &config, in);
    assert(state->phase == AX_AVOID_TURN_OUT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in->now_ms += 100;
    in->range_triggered_ms = in->now_ms - 20;
    ++in->range_sequence;
    out = AX_AVOID_Step(state, &config, in);
    assert(state->phase == AX_AVOID_ADVANCE);
    assert(out.forward_mm_s == 100 && out.yaw_mrad_s == 0);
}

int main(void)
{
    AX_AVOID_State state;
    AX_AVOID_Input in = input();
    AX_AVOID_Output out;

    /* Ordinary following stays limited by the obstacle controller. */
    AX_AVOID_Reset(&state);
    AX_AVOID_Step(&state, &config, &in);
    in.now_ms = 20;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRACK);
    assert(out.forward_mm_s == 300 && out.yaw_mrad_s == 500);
    in.aoa_valid = 0;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_WAIT_FRONT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    /* Without any eligible ray, the car waits and rescans. */
    in = input();
    start_scan(&state, &in);
    out = finish_scan(&state, &in, 100); /* No measured angle equals 100. */
    assert(state.phase == AX_AVOID_RESCAN_WAIT);
    assert(out.pan_angle_deg == 0);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.now_ms += 2000;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_WAIT_FRONT);
    in.now_ms += 20;
    in.range_angle_deg = 0;
    in.range_distance_mm = 900;
    ++in.range_sequence;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRACK);

    /* A 45-degree route turns, advances one short segment, then rejoins. */
    in = input();
    enter_advance(&state, &in);
    in.yaw_rate_mrad_s = 0;
    in.forward_mm_s = 100;
    for (unsigned n = 0; n < 15; ++n) {
        in.now_ms += 100;
        ++in.range_sequence;
        out = AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.aoa_azimuth_deg = 0;
    in.now_ms += 20;
    ++in.range_sequence;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    in.now_ms += 20;
    AX_AVOID_Step(&state, &config, &in); /* Duplicate echo cannot rejoin. */
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    in.now_ms += 20;
    ++in.range_sequence;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRACK);
    assert(state.detour_active == 0 && state.total_distance_mm == 0);

    /* A partial obstruction after one segment starts another scan. */
    in = input();
    enter_advance(&state, &in);
    in.yaw_rate_mrad_s = 0;
    in.forward_mm_s = 100;
    for (unsigned n = 0; n < 15; ++n) {
        in.now_ms += 100;
        ++in.range_sequence;
        AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    in.now_ms += 100;
    in.range_distance_mm = 600;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    assert(out.pan_angle_deg == 90);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    /* A commanded advance without encoder motion enters HOLD. */
    in = input();
    enter_advance(&state, &in);
    in.forward_mm_s = 0;
    for (unsigned n = 0; n < 10; ++n) {
        in.now_ms += 100;
        ++in.range_sequence;
        out = AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.phase == AX_AVOID_HOLD);
    assert(state.hold_reason == AX_AVOID_DIAG_ADVANCE_STALL);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    /* The distance cap still applies to the new scan policy. */
    in = input();
    enter_advance(&state, &in);
    state.total_distance_mm = config.max_distance_mm - 10u;
    state.segment_distance_mm = 0;
    in.now_ms += 100;
    in.forward_mm_s = 100;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_HOLD);
    assert(state.hold_reason == AX_AVOID_DIAG_DISTANCE_LIMIT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    /* Turning without measured rotation cannot run indefinitely. */
    in = input();
    start_scan(&state, &in);
    finish_scan(&state, &in, 45);
    in.range_angle_deg = 0;
    in.range_distance_mm = 330;
    in.yaw_rate_mrad_s = 0;
    for (unsigned n = 0; n < 50; ++n) {
        in.now_ms += 100;
        ++in.range_sequence;
        out = AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.phase == AX_AVOID_HOLD);
    assert(state.hold_reason == AX_AVOID_DIAG_TURN_TIMEOUT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    /* The moving detour still has a total time limit. */
    in = input();
    enter_advance(&state, &in);
    in.now_ms += config.max_time_ms;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_HOLD);
    assert(state.hold_reason == AX_AVOID_DIAG_TIME_LIMIT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    return 0;
}
