#include <assert.h>
#include <stdint.h>

#include "ax_follow_avoid_policy.h"

static AX_AVOID_Config config = {
    .front_stop_mm = 350,
    .front_release_mm = 650,
    .side_min_mm = 650,
    .scan_max_angle_deg = 90,
    .scan_step_deg = 15,
    .fine_scan_radius_deg = 20,
    .fine_scan_step_deg = 5,
    .fine_distance_tie_mm = 50,
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
    .turn_outward_offset_deg = 0,
    .advance_range_cap_mm = 2000,
    .advance_clearance_mm = 400,
    .max_distance_mm = 1200,
    .max_time_ms = 25000,
    .stall_timeout_ms = 1000,
    .stall_min_mm = 20,
    .align_stop_turn_deg = 5,
    .azimuth_to_yaw_sign = 1,
    .rescan_wait_ms = 2000,
    .coarse_scan_timeout_ms = 7000,
    .fine_scan_timeout_ms = 5000
};

static AX_AVOID_Input input(void)
{
    AX_AVOID_Input in = {0};
    in.aoa_valid = 1;
    in.aoa_distance_cm = 130;
    in.range_valid = 1;
    in.range_status = AX_XURSC_READY;
    in.range_angle_deg = 0;
    in.range_distance_mm = 1000;
    in.range_sequence = 1;
    return in;
}

static void start_scan(AX_AVOID_State *state, AX_AVOID_Input *in)
{
    AX_AVOID_Reset(state);
    AX_AVOID_Step(state, &config, in);
    assert(state->phase == AX_AVOID_FORWARD);
    in->now_ms = 100;
    in->range_triggered_ms = 80;
    ++in->range_sequence;
    AX_AVOID_Step(state, &config, in);
    assert(state->phase == AX_AVOID_FORWARD);
    in->now_ms = 120;
    in->range_angle_deg = 5;
    ++in->range_sequence;
    in->range_distance_mm = 330;
    ++in->range_sequence;
    AX_AVOID_Output out = AX_AVOID_Step(state, &config, in);
    assert(state->phase == AX_AVOID_SCAN_SIDE);
    assert(out.pan_angle_deg == 90);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
}

static AX_AVOID_Output sample(AX_AVOID_State *state, AX_AVOID_Input *in,
                              int16_t angle, uint16_t distance,
                              AX_XURSC_Status status)
{
    in->now_ms += 350;
    in->range_angle_deg = angle;
    in->range_distance_mm = distance;
    in->range_status = status;
    in->range_triggered_ms = in->now_ms - 20;
    ++in->range_sequence;
    return AX_AVOID_Step(state, &config, in);
}

static AX_AVOID_Output finish_fine_scan(AX_AVOID_State *state,
                                        AX_AVOID_Input *in,
                                        int16_t open_angle_1,
                                        uint16_t distance_1,
                                        int16_t open_angle_2,
                                        uint16_t distance_2)
{
    int16_t low = (int16_t)(state->scan_center_angle_deg - 20);
    int16_t high = (int16_t)(state->scan_center_angle_deg + 20);
    if (low < -90) low = -90;
    if (high > 90) high = 90;
    assert(state->scan_stage == AX_AVOID_SCAN_FINE);
    for (int16_t angle = high; angle >= low; angle -= 5) {
        uint16_t distance = 350;
        if (angle == open_angle_1) distance = distance_1;
        if (angle == open_angle_2) distance = distance_2;
        sample(state, in, angle, distance, AX_XURSC_READY);
    }
    return AX_AVOID_Step(state, &config, in);
}

int main(void)
{
    AX_AVOID_State state;
    AX_AVOID_Input in = input();
    AX_AVOID_Output out;
    int16_t angle;

    start_scan(&state, &in);
    /* An old sample or one from the previous angle cannot advance scan. */
    in.now_ms += 20;
    in.range_angle_deg = 0;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.scan_index == 0 && out.pan_angle_deg == 90);

    /* All 13 points are visited; a reading below 650 mm is excluded. */
    for (angle = 90; angle >= -90; angle -= 15) {
        uint16_t distance = 350;
        if (angle == 45) distance = 550;
        if (angle == -60 || angle == -75) distance = 850;
        out = sample(&state, &in, angle, distance, AX_XURSC_READY);
        if (angle > -90) {
            assert(state.phase == AX_AVOID_SCAN_SIDE);
            assert(out.pan_angle_deg == angle - 15);
            assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
        }
    }
    assert(state.scan_stage == AX_AVOID_SCAN_FINE);
    assert(state.coarse_candidate_angle_deg == -60);
    out = finish_fine_scan(&state, &in, -60, 850, -75, 850);
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(state.fine_selected_angle_deg == -60);
    assert(state.detour_angle_deg == -60);
    assert(state.detour_side == -1);
    assert(state.turn_target_mrad >= 1046 && state.turn_target_mrad <= 1048);
    assert(out.pan_angle_deg == 0);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    /* No fresh centered echo means no turning yet. */
    in.now_ms += 20;
    in.range_valid = 0;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(out.yaw_mrad_s == 0 && state.phase == AX_AVOID_TURN_OUT);
    in.range_valid = 1;
    in.range_angle_deg = 0;
    in.range_status = AX_XURSC_READY;
    in.range_distance_mm = 330;
    in.yaw_rate_mrad_s = -500;
    for (unsigned n = 0; n < 10; ++n) {
        in.now_ms += 100;
        in.range_triggered_ms = in.now_ms - 20;
        ++in.range_sequence;
        out = AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.turn_progress_mrad == 500);
    in.yaw_rate_mrad_s = 500;
    for (unsigned n = 0; n < 5; ++n) {
        in.now_ms += 100;
        in.range_triggered_ms = in.now_ms - 20;
        ++in.range_sequence;
        out = AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.turn_progress_mrad == 250);
    in.yaw_rate_mrad_s = -500;
    for (unsigned n = 0; n < 16; ++n) {
        in.now_ms += 100;
        in.range_triggered_ms = in.now_ms - 20;
        in.range_distance_mm = n == 15 ? 900 : 330;
        ++in.range_sequence;
        out = AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    /* The clear echo seen while rotating cannot authorize forward motion. */
    in.yaw_rate_mrad_s = -150;
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.yaw_rate_mrad_s = 150;
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.yaw_rate_mrad_s = 0;
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(state.turn_settle_pending);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.now_ms += 20;
    out = AX_AVOID_Step(&state, &config, &in); /* Same echo cannot pass. */
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.now_ms += 100;
    in.range_triggered_ms = state.turn_settle_ms;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_ADVANCE);
    assert(out.forward_mm_s == 100 && out.yaw_mrad_s == 0);

    /* Prefer the route nearest the tracked beacon, then the farther route. */
    in = input();
    start_scan(&state, &in);
    in.aoa_azimuth_deg = 40;
    for (angle = 90; angle >= -90; angle -= 15) {
        uint16_t distance = 350;
        if (angle == 45) distance = 1000;
        if (angle == 15) distance = 700;
        if (angle == -15) distance = 650;
        sample(&state, &in, angle, distance, AX_XURSC_READY);
    }
    out = finish_fine_scan(&state, &in, 45, 1000, 0, 0);
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(state.fine_selected_angle_deg == 45);
    assert(state.detour_angle_deg == 45);
    in.range_angle_deg = 0;
    in.range_distance_mm = 330;
    in.yaw_rate_mrad_s = -500;
    for (unsigned n = 0; n < 6; ++n) {
        in.now_ms += 100;
        in.range_triggered_ms = in.now_ms - 20;
        ++in.range_sequence;
        out = AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.turn_progress_mrad == -300);
    in.yaw_rate_mrad_s = 500;
    for (unsigned n = 0; n < 22; ++n) {
        in.now_ms += 100;
        in.range_triggered_ms = in.now_ms - 20;
        ++in.range_sequence;
        out = AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.turn_progress_mrad == 800);
    assert(state.phase == AX_AVOID_TURN_OUT);
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.yaw_rate_mrad_s = 150;
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.yaw_rate_mrad_s = 0;
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    assert(out.pan_angle_deg == 90);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    /* Failed or short readings are recorded but cannot become a route. */
    in = input();
    start_scan(&state, &in);
    for (angle = 90; angle >= -90; angle -= 15)
        out = sample(&state, &in, angle, 0, AX_XURSC_TIMEOUT);
    assert(state.phase == AX_AVOID_RESCAN_WAIT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    /* A scan that stalls must stop instead of using partial results. */
    in = input();
    start_scan(&state, &in);
    sample(&state, &in, 90, 900, AX_XURSC_READY);
    in.now_ms = 8120;
    in.range_valid = 0;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_RESCAN_WAIT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    config.turn_outward_offset_deg = 10;
    in = input();
    start_scan(&state, &in);
    in.aoa_azimuth_deg = 20;
    for (angle = 90; angle >= -90; angle -= 15) {
        uint16_t distance = angle == 30 ? 800 : 350;
        sample(&state, &in, angle, distance, AX_XURSC_READY);
    }
    out = finish_fine_scan(&state, &in, 30, 800, 0, 0);
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(state.fine_selected_angle_deg == 30);
    assert(state.detour_angle_deg == 40);
    return 0;
}
