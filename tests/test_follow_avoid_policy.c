#include <assert.h>
#include <stdint.h>

#include "ax_follow_avoid_policy.h"
#include "openctr_avoid_config.h"

static const AX_AVOID_Config config = {
    .front_stop_mm = 350,
    .front_release_mm = 650,
    .side_min_mm = 650,
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
    .max_distance_mm = 0, /* Unlimited cumulative detour distance. */
    .max_time_ms = 0, /* Unlimited cumulative detour time. */
    .stall_timeout_ms = 1000,
    .stall_min_mm = 20,
    .rejoin_clear_samples = 2,
    .rejoin_stop_turn_deg = 12,
    .azimuth_to_yaw_sign = 1,
    .rescan_wait_ms = 2000,
    .side_scan_timeout_ms = 8000
};

static AX_AVOID_Input input(void)
{
    AX_AVOID_Input in = {0};
    in.aoa_valid = 1;
    in.aoa_distance_cm = 130;
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

static void assert_recenter_then_rescan(AX_AVOID_State *state,
                                       AX_AVOID_Input *in,
                                       AX_AVOID_DiagReason reason)
{
    AX_AVOID_Output out;
    assert(state->phase == AX_AVOID_WAIT_FRONT);
    assert(state->hold_reason == reason);
    assert(state->pan_angle_deg == 0);

    /* Neither an old echo nor a newly delivered pre-recovery echo may
     * start the side scan. A centered echo triggered afterwards may. */
    in->now_ms += 20;
    in->range_valid = 1;
    in->range_angle_deg = 0;
    in->range_distance_mm = 900;
    out = AX_AVOID_Step(state, &config, in);
    assert(state->phase == AX_AVOID_WAIT_FRONT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in->now_ms += 20;
    in->range_triggered_ms = in->now_ms - 40;
    ++in->range_sequence;
    out = AX_AVOID_Step(state, &config, in);
    assert(state->phase == AX_AVOID_WAIT_FRONT);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in->now_ms += 100;
    in->range_triggered_ms = in->now_ms - 20;
    ++in->range_sequence;
    out = AX_AVOID_Step(state, &config, in);
    assert(state->phase == AX_AVOID_SCAN_SIDE);
    assert(out.pan_angle_deg == 90);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
}

static void test_track_probes_while_moving(void)
{
    static const int16_t path[] = {
        5, 10, 15, 20, 25, 20, 15, 10, 5, 0,
        -5, -10, -15, -20, -25, -20, -15, -10, -5, 0, 5
    };
    AX_AVOID_State state;
    AX_AVOID_Input in = input();
    AX_AVOID_Output out;

    AX_AVOID_Reset(&state);
    AX_AVOID_Step(&state, &config, &in); /* Initial clear 0-degree echo. */
    assert(state.phase == AX_AVOID_TRACK);

    in.now_ms = 20;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(out.pan_angle_deg == path[0]);
    assert(out.forward_mm_s == 300);

    for (unsigned i = 0; i + 1 < sizeof(path) / sizeof(path[0]); ++i) {
        in.now_ms += 20;
        in.range_valid = 0; /* Keep following while the servo settles. */
        out = AX_AVOID_Step(&state, &config, &in);
        assert(state.phase == AX_AVOID_TRACK);
        assert(out.pan_angle_deg == path[i] && out.forward_mm_s == 300);

        in.now_ms += 80;
        in.range_valid = 1;
        in.range_angle_deg = path[i];
        in.range_distance_mm = 350; /* Exactly the limit is still clear. */
        ++in.range_sequence;
        out = AX_AVOID_Step(&state, &config, &in);
        assert(state.phase == AX_AVOID_TRACK);
        assert(out.pan_angle_deg == path[i + 1] && out.forward_mm_s == 300);
    }

    in.now_ms += 20;
    out = AX_AVOID_Step(&state, &config, &in); /* Duplicate echo. */
    assert(out.pan_angle_deg == 5 && out.forward_mm_s == 300);

    in.now_ms += 80;
    in.range_angle_deg = 5;
    in.range_distance_mm = 349;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    assert(out.pan_angle_deg == 90);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
}

static void test_track_stops_if_probe_echo_never_arrives(void)
{
    AX_AVOID_State state;
    AX_AVOID_Input in = input();
    AX_AVOID_Output out;

    AX_AVOID_Reset(&state);
    AX_AVOID_Step(&state, &config, &in);
    in.now_ms = 20;
    AX_AVOID_Step(&state, &config, &in); /* Request +5 degrees. */
    in.range_valid = 0;
    in.now_ms = 271; /* Beyond the configured sample age. */
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_WAIT_FRONT);
    assert(out.pan_angle_deg == 0);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    in.now_ms = 280;
    in.range_valid = 1;
    in.range_angle_deg = 0;
    out = AX_AVOID_Step(&state, &config, &in); /* Old 0-degree echo. */
    assert(state.phase == AX_AVOID_WAIT_FRONT);
    assert(out.forward_mm_s == 0);
    in.now_ms = 300;
    in.range_triggered_ms = 270; /* Delivered after stop, fired before. */
    ++in.range_sequence;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_WAIT_FRONT);
    in.now_ms = 400;
    in.range_triggered_ms = 380;
    ++in.range_sequence;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRACK);
}

static void test_track_keeps_pan_centered_at_target_distance(void)
{
    AX_AVOID_State state;
    AX_AVOID_Input in = input();
    AX_AVOID_Output out;

    in.aoa_distance_cm = 100;
    in.follow_target.forward_mm_s = 0;
    AX_AVOID_Reset(&state);
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRACK);
    in.now_ms = 20;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRACK);
    assert(out.pan_angle_deg == 0 && out.forward_mm_s == 0);
}

static void test_scan_skips_center_that_cannot_release(void)
{
    AX_AVOID_State state;
    AX_AVOID_Input in = input();

    start_scan(&state, &in);
    for (int16_t angle = 90; angle >= -90; angle -= 15) {
        in.now_ms += 350;
        in.range_angle_deg = angle;
        in.range_distance_mm = angle == 0 ? 620 :
                               angle == -45 ? 900 : 350;
        in.range_triggered_ms = in.now_ms - 20;
        ++in.range_sequence;
        AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(state.detour_angle_deg == -45);
}

static void test_project_400mm_clearance(void)
{
    AX_AVOID_Config tuned = config;
    AX_AVOID_State state;
    AX_AVOID_Input in = input();

    tuned.front_release_mm = OPENCTR_AVOID_FRONT_RELEASE_MM;
    tuned.side_min_mm = OPENCTR_AVOID_SIDE_MIN_MM;
    in.range_distance_mm = 400;
    AX_AVOID_Reset(&state);
    AX_AVOID_Step(&state, &tuned, &in);
    assert(state.phase == AX_AVOID_TRACK);
    in.now_ms = 20;
    in.range_distance_mm = 330;
    ++in.range_sequence;
    AX_AVOID_Step(&state, &tuned, &in);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    for (int16_t angle = 90; angle >= -90; angle -= 15) {
        in.now_ms += 350;
        in.range_angle_deg = angle;
        in.range_distance_mm = angle == 45 ? 400 : 350;
        in.range_triggered_ms = in.now_ms - 20;
        ++in.range_sequence;
        AX_AVOID_Step(&state, &tuned, &in);
    }
    assert(state.phase == AX_AVOID_TURN_OUT);
    assert(state.detour_angle_deg == 45);
}

int main(void)
{
    test_project_400mm_clearance();
    AX_AVOID_State state;
    AX_AVOID_Input in = input();
    AX_AVOID_Output out;

    test_track_probes_while_moving();
    test_track_stops_if_probe_echo_never_arrives();
    test_track_keeps_pan_centered_at_target_distance();
    test_scan_skips_center_that_cannot_release();

    /* A nearby target does not justify scanning or driving around an
     * obstacle. This applies both on entry and after a clear TRACK fix. */
    in.aoa_distance_cm = 100;
    in.follow_target.forward_mm_s = 0;
    in.follow_target.yaw_mrad_s = 0;
    in.range_distance_mm = 330;
    AX_AVOID_Reset(&state);
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_WAIT_FRONT);
    assert(out.pan_angle_deg == 0);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.now_ms = 20;
    in.range_distance_mm = 900;
    ++in.range_sequence;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRACK);
    in.now_ms = 40;
    in.range_distance_mm = 330;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_WAIT_FRONT);
    assert(out.pan_angle_deg == 0);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.now_ms = 60;
    in.aoa_distance_cm = 105;
    in.range_distance_mm = 550;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_WAIT_FRONT);
    assert(out.pan_angle_deg == 0);
    in.now_ms = 80;
    in.aoa_distance_cm = 110;
    in.follow_target.yaw_mrad_s = 500;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    /* Turning priority can make forward speed zero even when the target
     * is far away, so distance must still authorize obstacle scanning. */
    in = input();
    in.follow_target.forward_mm_s = 0;
    in.follow_target.yaw_mrad_s = 500;
    in.range_distance_mm = 330;
    AX_AVOID_Reset(&state);
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    assert(out.pan_angle_deg == 90);

    /* Reaching the target during a detour cancels the planned motion. */
    in = input();
    start_scan(&state, &in);
    in.now_ms += 20;
    in.aoa_distance_cm = 105;
    in.follow_target.forward_mm_s = 0;
    in.follow_target.yaw_mrad_s = 500;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    in.now_ms += 20;
    in.aoa_distance_cm = 100;
    in.follow_target.forward_mm_s = 0;
    in.follow_target.yaw_mrad_s = 0;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_WAIT_FRONT);
    assert(state.detour_active == 0);
    assert(out.pan_angle_deg == 0);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    in = input();
    enter_advance(&state, &in);
    in.now_ms += 20;
    in.aoa_distance_cm = 100;
    in.follow_target.forward_mm_s = 0;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_WAIT_FRONT);
    assert(state.detour_active == 0);
    assert(out.pan_angle_deg == 0);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    in = input();

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
    /* An echo completed after alignment can still have been triggered
     * before alignment; its new sequence alone is insufficient. */
    in.now_ms += 20;
    in.range_triggered_ms = in.now_ms - 40;
    ++in.range_sequence;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    in.now_ms += 20;
    AX_AVOID_Step(&state, &config, &in); /* Duplicate echo cannot rejoin. */
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRACK);
    assert(state.detour_active == 0 && state.total_distance_mm == 0);

    /* Rejoin must keep turning past 90 degrees and five seconds when the
     * latest AOA still points away from the target. At 12 degrees it stops
     * and waits for two distinct clear front samples before tracking. */
    in = input();
    enter_advance(&state, &in);
    in.forward_mm_s = 100;
    for (unsigned n = 0; n < 15; ++n) {
        in.now_ms += 100;
        ++in.range_sequence;
        AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    in.forward_mm_s = 0;
    in.aoa_azimuth_deg = 85;
    in.yaw_rate_mrad_s = 500;
    for (unsigned n = 0; n < 65; ++n) {
        in.now_ms += 100;
        ++in.range_sequence;
        out = AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    assert(state.rejoin_progress_mrad > 1570u);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 500);
    in.aoa_azimuth_deg = 13;
    in.now_ms += 100;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    assert(out.yaw_mrad_s == 500);
    in.aoa_azimuth_deg = 12;
    in.now_ms += 100;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRACK);

    /* Rejoining is a stationary turn. A close or missing front echo must
     * not divert it; front clearance matters once alignment is reached. */
    in = input();
    enter_advance(&state, &in);
    in.forward_mm_s = 100;
    for (unsigned n = 0; n < 15; ++n) {
        in.now_ms += 100;
        ++in.range_sequence;
        AX_AVOID_Step(&state, &config, &in);
    }
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    in.forward_mm_s = 0;
    in.aoa_azimuth_deg = -85;
    in.range_valid = 0;
    in.now_ms += 150;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == -500);
    in.range_valid = 1;
    in.range_distance_mm = 330;
    in.now_ms += 20;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == -500);
    in.aoa_azimuth_deg = -12;
    in.now_ms += 20;
    out = AX_AVOID_Step(&state, &config, &in);
    /* The 330 mm echo was triggered while turning, so it cannot decide
     * whether the newly aligned direction is blocked. */
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

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
    assert(state.phase == AX_AVOID_TRY_REJOIN);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.now_ms += 100;
    in.range_triggered_ms = in.now_ms - 20;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    assert(out.pan_angle_deg == 90);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    /* A commanded advance without encoder motion recenters, then rescans. */
    in = input();
    enter_advance(&state, &in);
    in.forward_mm_s = 0;
    for (unsigned n = 0; n < 10; ++n) {
        in.now_ms += 100;
        ++in.range_sequence;
        out = AX_AVOID_Step(&state, &config, &in);
    }
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    assert_recenter_then_rescan(&state, &in, AX_AVOID_DIAG_ADVANCE_STALL);

    /* Reaching the former 1200 mm total must not stop another segment. */
    in = input();
    enter_advance(&state, &in);
    state.total_distance_mm = 1190u;
    state.segment_distance_mm = 0;
    in.now_ms += 100;
    in.forward_mm_s = 100;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_ADVANCE);
    assert(state.total_distance_mm == 1200u);
    assert(out.forward_mm_s == 100 && out.yaw_mrad_s == 0);
    in.now_ms += 100;
    in.range_distance_mm = 330;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_SCAN_SIDE);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    in.forward_mm_s = 0;
    finish_scan(&state, &in, 45);
    assert(state.phase == AX_AVOID_TURN_OUT);

    /* A timed-out turn stops, recenters, and waits for a new front echo. */
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
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    assert_recenter_then_rescan(&state, &in, AX_AVOID_DIAG_TURN_TIMEOUT);
    finish_scan(&state, &in, 45);
    assert(state.phase == AX_AVOID_TURN_OUT);

    /* A late control loop during motion follows the same recovery path. */
    in = input();
    enter_advance(&state, &in);
    in.now_ms += 101;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    assert_recenter_then_rescan(&state, &in, AX_AVOID_DIAG_LOOP_LATE);

    /* Optional cumulative limits, when enabled, also start a fresh
     * stationary scan rather than leaving the car locked forever. */
    AX_AVOID_Config capped = config;
    capped.max_time_ms = 1;
    in = input();
    start_scan(&state, &in);
    finish_scan(&state, &in, 45);
    in.now_ms += 20;
    in.range_angle_deg = 0;
    out = AX_AVOID_Step(&state, &capped, &in);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    assert_recenter_then_rescan(&state, &in, AX_AVOID_DIAG_TIME_LIMIT);

    capped = config;
    capped.max_distance_mm = 1;
    in = input();
    enter_advance(&state, &in);
    in.now_ms += 100;
    in.forward_mm_s = 100;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &capped, &in);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    assert_recenter_then_rescan(&state, &in, AX_AVOID_DIAG_DISTANCE_LIMIT);

    /* Invalid configuration cannot be corrected by resampling. */
    capped = config;
    capped.azimuth_to_yaw_sign = 0;
    in = input();
    AX_AVOID_Reset(&state);
    out = AX_AVOID_Step(&state, &capped, &in);
    assert(state.phase == AX_AVOID_HOLD);
    assert(state.hold_reason == AX_AVOID_DIAG_BAD_CONFIG);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);

    /* Passing the former 25 s total must not stop an active segment. */
    in = input();
    enter_advance(&state, &in);
    state.detour_start_ms = in.now_ms - 25000u;
    in.now_ms += 100;
    in.forward_mm_s = 100;
    ++in.range_sequence;
    out = AX_AVOID_Step(&state, &config, &in);
    assert(state.phase == AX_AVOID_ADVANCE);
    assert(out.forward_mm_s == 100 && out.yaw_mrad_s == 0);
    return 0;
}
