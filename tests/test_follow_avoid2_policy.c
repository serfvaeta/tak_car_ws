#include <assert.h>
#include <stdint.h>

#include "ax_follow_avoid2_policy.h"
#include "openctr_avoid2_config.h"

static AX_AVOID2_Config config = {
    .safe_distance_mm = 600,
    .emergency_distance_mm = 250,
    .scan_max_angle_deg = 90,
    .coarse_step_deg = 6,
    .fine_range_deg = 10,
    .fine_step_deg = 5,
    .max_candidates = 5,
    .max_adjust_count = 10,
    .range_max_age_ms = 250,
    .approach_start_cm = 110,
    .approach_stop_cm = 100,
    .advance_max_mm_s = 100,
    .yaw_max_mrad_s = 500,
    .turn_timeout_ms = 5000,
    .turn_settle_max_yaw_mrad_s = 100,
    .advance_range_cap_mm = 2000,
    .advance_clearance_mm = 400,
    .stall_timeout_ms = 1000,
    .stall_min_mm = 20,
    .rescan_wait_ms = 2000,
    .coarse_scan_timeout_ms = 10000,
    .fine_scan_timeout_ms = 5000,
    .verify_timeout_ms = 2000,
    .aoa_to_yaw_sign = 1
};

static AX_AVOID2_Input MakeInput(void)
{
    AX_AVOID2_Input in = {0};
    in.aoa_valid = 1;
    in.aoa_distance_cm = 130;
    in.aoa_azimuth_deg = 20;
    in.range_valid = 1;
    in.range_status = AX_XURSC_READY;
    in.range_distance_mm = 1000;
    in.range_sequence = 1;
    in.range_triggered_ms = 0;
    return in;
}

static int16_t CoarseAngle(uint8_t index)
{
    if (index == 0) return 0;
    return (index & 1u) ?
        (int16_t)(((index + 1u) / 2u) * 6u) :
        (int16_t)(-((int16_t)(index / 2u) * 6));
}

static AX_AVOID2_Output Tick(AX_AVOID2_State *state, AX_AVOID2_Input *in)
{
    return AX_AVOID2_Step(state, &config, in);
}

static AX_AVOID2_Output FreshRange(AX_AVOID2_State *state,
                                   AX_AVOID2_Input *in,
                                   int16_t angle, uint16_t distance_mm,
                                   uint32_t dt_ms)
{
    in->now_ms += dt_ms;
    in->range_valid = 1;
    in->range_angle_deg = angle;
    in->range_status = AX_XURSC_READY;
    in->range_distance_mm = distance_mm;
    in->range_triggered_ms = in->now_ms;
    ++in->range_sequence;
    return Tick(state, in);
}

static void StartCoarse(AX_AVOID2_State *state, AX_AVOID2_Input *in)
{
    AX_AVOID2_Reset(state);
    Tick(state, in);
    assert(state->phase == AX_AVOID2_COARSE_SCAN);
    assert(state->pan_angle_deg == 0);
}

static AX_AVOID2_Output RunCoarse(AX_AVOID2_State *state,
                                  AX_AVOID2_Input *in,
                                  const int16_t *open_angles,
                                  const uint16_t *distances,
                                  uint8_t open_count)
{
    AX_AVOID2_Output out = {0};
    for (uint8_t i = 0; i < AX_AVOID2_COARSE_POINT_COUNT; ++i) {
        int16_t angle = CoarseAngle(i);
        uint16_t distance = 350;
        for (uint8_t j = 0; j < open_count; ++j)
            if (angle == open_angles[j]) distance = distances[j];
        assert(state->phase == AX_AVOID2_COARSE_SCAN);
        assert(state->pan_angle_deg == angle);
        out = FreshRange(state, in, angle, distance, 100);
        if (i + 1u < AX_AVOID2_COARSE_POINT_COUNT &&
            state->phase == AX_AVOID2_COARSE_SCAN)
            assert(out.pan_angle_deg == CoarseAngle((uint8_t)(i + 1u)));
    }
    return out;
}

static AX_AVOID2_Output RunFineWithDelay(AX_AVOID2_State *state,
                                         AX_AVOID2_Input *in,
                                         const uint16_t distances[5],
                                         uint32_t delay_ms)
{
    AX_AVOID2_Output out = {0};
    int16_t center = state->fine_center_angle_deg;
    for (uint8_t i = 0; i < 5u; ++i) {
        int16_t angle = (int16_t)(center - config.fine_range_deg +
                                  config.fine_step_deg * i);
        assert(state->phase == AX_AVOID2_FINE_SCAN);
        assert(state->pan_angle_deg == angle);
        out = FreshRange(state, in, angle, distances[i], delay_ms);
        if (i < 4u && state->phase == AX_AVOID2_FINE_SCAN)
            assert(out.pan_angle_deg == (int16_t)(angle +
                                                   config.fine_step_deg));
    }
    return out;
}

static AX_AVOID2_Output RunFine(AX_AVOID2_State *state,
                                AX_AVOID2_Input *in,
                                const uint16_t distances[5])
{
    return RunFineWithDelay(state, in, distances, 100);
}

static void TestCoarseOrderAndCandidateRanking(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    const int16_t open[] = {12, 18, 24};
    const uint16_t distances[] = {900, 700, 1200};
    AX_AVOID2_Output out;

    StartCoarse(&state, &in);
    out = RunCoarse(&state, &in, open, distances, 3);
    assert(state.phase == AX_AVOID2_FINE_SCAN);
    assert(state.candidate_count == 3);
    assert(state.candidate_angles[0] == 18);
    assert(state.candidate_angles[1] == 24);
    assert(state.candidate_angles[2] == 12);
    assert(state.fine_center_angle_deg == 18);
    assert(out.pan_angle_deg == 8);
}

static void TestCoarseScanAllowsSensorSchedulingMargin(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    uint16_t previous_timeout = config.coarse_scan_timeout_ms;

    config.coarse_scan_timeout_ms = OPENCTR_AVOID2_COARSE_SCAN_TIMEOUT_MS;
    StartCoarse(&state, &in);
    for (uint8_t i = 0; i < AX_AVOID2_COARSE_POINT_COUNT; ++i) {
        int16_t angle = CoarseAngle(i);
        assert(state.phase == AX_AVOID2_COARSE_SCAN);
        FreshRange(&state, &in, angle, angle == 0 ? 900 : 350, 340);
    }
    assert(state.phase == AX_AVOID2_FINE_SCAN);
    assert(state.fine_center_angle_deg == 0);
    config.coarse_scan_timeout_ms = previous_timeout;
}

static void TestFineScanCanTryAllConfiguredCandidates(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    const int16_t open[] = {0, 6, 12, 18, 24};
    const uint16_t distances[] = {900, 900, 900, 900, 900};
    const uint16_t blocked_center[] = {700, 700, 500, 800, 800};
    uint32_t previous_timeout = config.fine_scan_timeout_ms;

    in.aoa_azimuth_deg = 0;
    config.fine_scan_timeout_ms = OPENCTR_AVOID2_FINE_SCAN_TIMEOUT_MS;
    StartCoarse(&state, &in);
    RunCoarse(&state, &in, open, distances, 5);
    for (uint8_t candidate = 0; candidate < 5u; ++candidate) {
        assert(state.candidate_index == candidate);
        for (uint8_t cycle = 0; cycle < 11u; ++cycle)
            RunFineWithDelay(&state, &in, blocked_center, 340);
    }
    assert(state.phase == AX_AVOID2_WAIT_RESCAN);
    assert(state.candidate_index == 5u);
    assert(state.reason == AX_AVOID2_REASON_NO_SAFE_FINE);
    config.fine_scan_timeout_ms = previous_timeout;
}

static void TestCoarseTimeoutExplainsRescan(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();

    StartCoarse(&state, &in);
    in.now_ms += config.coarse_scan_timeout_ms;
    Tick(&state, &in);
    assert(state.phase == AX_AVOID2_WAIT_RESCAN);
    assert(state.reason == AX_AVOID2_REASON_COARSE_TIMEOUT);
}

static void TestNoCoarseCandidateExplainsRescan(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();

    StartCoarse(&state, &in);
    RunCoarse(&state, &in, 0, 0, 0);
    assert(state.phase == AX_AVOID2_WAIT_RESCAN);
    assert(state.reason == AX_AVOID2_REASON_NO_COARSE_CANDIDATE);
}

static void TestWaitTargetReasonUpdatesWhenAoARecovers(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();

    AX_AVOID2_Reset(&state);
    in.aoa_valid = 0;
    Tick(&state, &in);
    assert(state.reason == AX_AVOID2_REASON_AOA_INVALID);
    in.aoa_valid = 1;
    in.aoa_distance_cm = 95;
    Tick(&state, &in);
    assert(state.phase == AX_AVOID2_WAIT_TARGET);
    assert(state.reason == AX_AVOID2_REASON_TARGET_NEAR);
}

static void TestOneSidedObstaclesCorrectAwayFromObstacle(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    const int16_t open[] = {18};
    const uint16_t distances[] = {900};
    const uint16_t left_blocked[] = {300, 400, 900, 900, 900};
    const uint16_t right_blocked[] = {900, 900, 900, 400, 300};

    StartCoarse(&state, &in);
    RunCoarse(&state, &in, open, distances, 1);
    RunFine(&state, &in, left_blocked);
    assert(state.fine_center_angle_deg == 23);
    assert(state.adjust_count == 1);

    StartCoarse(&state, &in);
    RunCoarse(&state, &in, open, distances, 1);
    RunFine(&state, &in, right_blocked);
    assert(state.fine_center_angle_deg == 13);
    assert(state.adjust_count == 1);
}

static void TestBothSidesBlockedMovesToNextCandidate(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    const int16_t open[] = {18, 24};
    const uint16_t distances[] = {900, 950};
    const uint16_t blocked_both[] = {300, 400, 900, 400, 300};

    StartCoarse(&state, &in);
    RunCoarse(&state, &in, open, distances, 2);
    RunFine(&state, &in, blocked_both);
    assert(state.phase == AX_AVOID2_FINE_SCAN);
    assert(state.candidate_index == 1);
    assert(state.fine_center_angle_deg == 24);
}

static void TestCenterObstacleChoosesSaferSide(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    const int16_t open[] = {18};
    const uint16_t distances[] = {900};
    const uint16_t center_blocked[] = {700, 800, 500, 1000, 900};

    StartCoarse(&state, &in);
    RunCoarse(&state, &in, open, distances, 1);
    RunFine(&state, &in, center_blocked);
    assert(state.fine_center_angle_deg == 23);
}

static void TestInvalidAndStaleSamplesDoNotAdvanceCoarseScan(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();

    StartCoarse(&state, &in);
    in.now_ms += 100;
    ++in.range_sequence;
    in.range_valid = 0;
    Tick(&state, &in);
    assert(state.scan_index == 0);

    in.now_ms += 100;
    in.range_valid = 1;
    in.range_angle_deg = 0;
    in.range_status = AX_XURSC_READY;
    in.range_distance_mm = 1000;
    in.range_triggered_ms = in.now_ms - 251;
    ++in.range_sequence;
    Tick(&state, &in);
    assert(state.scan_index == 0);

    FreshRange(&state, &in, 0, 1000, 20);
    assert(state.scan_index == 1);
}

static void TestCandidateAtServoBoundaryNeedsCompleteWindow(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    const int16_t open[] = {78, 90};
    const uint16_t distances[] = {800, 1000};

    in.aoa_azimuth_deg = 90;
    StartCoarse(&state, &in);
    RunCoarse(&state, &in, open, distances, 2);
    assert(state.candidate_index == 1);
    assert(state.fine_center_angle_deg == 78);
    assert(state.pan_angle_deg == 68);
}

static void TestCorrectionLimitAndVisitedAngleGuard(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    const int16_t open[] = {18, 24};
    const uint16_t distances[] = {900, 950};
    const uint16_t center_blocked[] = {700, 700, 500, 800, 800};

    config.fine_scan_timeout_ms = 20000;
    StartCoarse(&state, &in);
    RunCoarse(&state, &in, open, distances, 2);
    for (uint8_t n = 0; n < 11u; ++n) {
        RunFine(&state, &in, center_blocked);
        if (n < 10u) assert(state.adjust_count == (uint8_t)(n + 1u));
    }
    assert(state.candidate_index == 1);
    assert(state.fine_center_angle_deg == 24);

    in = MakeInput();
    in.aoa_azimuth_deg = 1;
    const int16_t open_loop[] = {0, 6};
    const uint16_t open_loop_distances[] = {900, 900};
    const uint16_t tied_center_block[] = {800, 800, 500, 800, 800};
    StartCoarse(&state, &in);
    RunCoarse(&state, &in, open_loop, open_loop_distances, 2);
    RunFine(&state, &in, tied_center_block);
    assert(state.fine_center_angle_deg == 5);
    in.aoa_azimuth_deg = -1;
    RunFine(&state, &in, tied_center_block);
    assert(state.candidate_index == 1);
    assert(state.fine_center_angle_deg == 6);

    config.fine_scan_timeout_ms = 5000;
    in = MakeInput();
    const int16_t open_timeout[] = {18, 24};
    const uint16_t open_timeout_distances[] = {900, 900};
    StartCoarse(&state, &in);
    RunCoarse(&state, &in, open_timeout, open_timeout_distances, 2);
    uint32_t fine_start_ms = state.phase_start_ms;
    RunFine(&state, &in, tied_center_block);
    assert(state.phase == AX_AVOID2_FINE_SCAN);
    assert(state.phase_start_ms == fine_start_ms);
    in.now_ms = fine_start_ms + config.fine_scan_timeout_ms;
    Tick(&state, &in);
    assert(state.phase == AX_AVOID2_WAIT_RESCAN);
}

static void TestExhaustedCandidatesWaitsStopped(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    const int16_t open[] = {18};
    const uint16_t distances[] = {900};
    const uint16_t blocked[] = {300, 300, 700, 300, 300};
    AX_AVOID2_Output out;

    StartCoarse(&state, &in);
    RunCoarse(&state, &in, open, distances, 1);
    out = RunFine(&state, &in, blocked);
    assert(state.phase == AX_AVOID2_WAIT_RESCAN);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
}

static void ReachMoveAfterTurn(AX_AVOID2_State *state,
                               AX_AVOID2_Input *in)
{
    const int16_t open[] = {6};
    const uint16_t distances[] = {900};
    const uint16_t clear_window[] = {900, 900, 900, 900, 900};
    StartCoarse(state, in);
    RunCoarse(state, in, open, distances, 1);
    RunFine(state, in, clear_window);
    assert(state->phase == AX_AVOID2_TURN);

    assert(state->turn_direction == -1); /* positive-right candidate */
    in->yaw_rate_mrad_s = -500;
    FreshRange(state, in, 0, 900, 20);
    for (uint8_t n = 0; n < 20u && state->phase == AX_AVOID2_TURN; ++n) {
        if (state->turn_progress_mrad >= state->turn_target_mrad)
            in->yaw_rate_mrad_s = 0;
        in->now_ms += 20;
        if ((n % 4u) == 0u) {
            in->range_angle_deg = 0;
            in->range_status = AX_XURSC_READY;
            in->range_distance_mm = 900;
            in->range_triggered_ms = in->now_ms;
            ++in->range_sequence;
        }
        Tick(state, in);
    }
    assert(state->phase == AX_AVOID2_VERIFY_FRONT);
}

static void TestTurnRequiresFreshCenteredClearanceAndMovesShortSegment(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    AX_AVOID2_Output out;

    ReachMoveAfterTurn(&state, &in);
    out = Tick(&state, &in); /* Same echo cannot authorize motion. */
    assert(state.phase == AX_AVOID2_VERIFY_FRONT);
    assert(out.forward_mm_s == 0);

    FreshRange(&state, &in, 0, 700, 20);
    assert(state.phase == AX_AVOID2_MOVE);
    assert(state.segment_target_mm == 300);
    assert(out.forward_mm_s == 0);
    in.forward_mm_s = 100;
    for (uint16_t n = 0; n < 170u && state.phase == AX_AVOID2_MOVE; ++n) {
        in.now_ms += 20;
        if ((n % 4u) == 0u) {
            in.range_angle_deg = 0;
            in.range_status = AX_XURSC_READY;
            in.range_distance_mm = 700;
            in.range_triggered_ms = in.now_ms;
            ++in.range_sequence;
        }
        out = Tick(&state, &in);
    }
    assert(state.phase == AX_AVOID2_COARSE_SCAN);
    assert(out.forward_mm_s == 0);

    in = MakeInput();
    ReachMoveAfterTurn(&state, &in);
    FreshRange(&state, &in, 0, 900, 20);
    assert(state.phase == AX_AVOID2_MOVE);
    assert(state.segment_target_mm == 500);

    in = MakeInput();
    ReachMoveAfterTurn(&state, &in);
    FreshRange(&state, &in, 0, 2500, 20);
    assert(state.phase == AX_AVOID2_MOVE);
    assert(state.segment_target_mm == 1600);
}

static void TestUnsafePostTurnAndEmergencyFrontStop(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    AX_AVOID2_Output out;

    ReachMoveAfterTurn(&state, &in);
    out = FreshRange(&state, &in, 0, 500, 20);
    assert(state.phase == AX_AVOID2_WAIT_RESCAN);
    assert(out.forward_mm_s == 0);

    in = MakeInput();
    ReachMoveAfterTurn(&state, &in);
    FreshRange(&state, &in, 0, 700, 20);
    assert(state.phase == AX_AVOID2_MOVE);
    out = FreshRange(&state, &in, 0, 500, 20);
    assert(state.phase == AX_AVOID2_MOVE);
    assert(out.forward_mm_s == 100);
    out = FreshRange(&state, &in, 0, 399, 20);
    assert(state.phase == AX_AVOID2_COARSE_SCAN);
    assert(out.forward_mm_s == 0);

    in = MakeInput();
    ReachMoveAfterTurn(&state, &in);
    FreshRange(&state, &in, 0, 700, 20);
    out = FreshRange(&state, &in, 0, 249, 20);
    assert(state.phase == AX_AVOID2_WAIT_RESCAN);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
}

static void TestMoveStopsAfterProgressStallsLater(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    AX_AVOID2_Output out = {0};

    ReachMoveAfterTurn(&state, &in);
    FreshRange(&state, &in, 0, 900, 20);
    assert(state.phase == AX_AVOID2_MOVE);
    in.forward_mm_s = 100;
    for (uint8_t n = 0; n < 3u; ++n)
        FreshRange(&state, &in, 0, 900, 100);
    assert(state.segment_distance_mm >= 20u);
    in.forward_mm_s = 0;
    for (uint8_t n = 0; n < 15u && state.phase == AX_AVOID2_MOVE; ++n)
        out = FreshRange(&state, &in, 0, 900, 100);
    assert(state.phase == AX_AVOID2_WAIT_RESCAN);
    assert(out.forward_mm_s == 0);
}

static void TestSlowMeasuredMotionAccumulatesFractionalDistance(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    AX_AVOID2_Output out = {0};

    ReachMoveAfterTurn(&state, &in);
    FreshRange(&state, &in, 0, 900, 20);
    assert(state.phase == AX_AVOID2_MOVE);
    in.forward_mm_s = 25; /* 0.5 mm per 20 ms control tick. */
    for (uint8_t n = 0; n < 60u; ++n)
        out = FreshRange(&state, &in, 0, 900, 20);
    assert(state.phase == AX_AVOID2_MOVE);
    assert(state.segment_distance_mm == 30u);
    assert(out.forward_mm_s == 100);
}

static void TestOversizedCandidateConfigStopsSafely(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    AX_AVOID2_Config bad = config;
    AX_AVOID2_Output out;

    StartCoarse(&state, &in);
    bad.max_candidates = AX_AVOID2_CANDIDATE_CAPACITY + 1u;
    out = AX_AVOID2_Step(&state, &bad, &in);
    assert(state.phase == AX_AVOID2_WAIT_TARGET);
    assert(out.pan_angle_deg == 0);
    assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
}

static void TestMalformedScanConfigStopsSafely(void)
{
    AX_AVOID2_State state;
    AX_AVOID2_Input in = MakeInput();
    AX_AVOID2_Config bad[4] = {config, config, config, config};

    bad[0].coarse_step_deg = 0;
    bad[1].coarse_step_deg = 1; /* 181 points exceed the coarse buffer. */
    bad[2].fine_step_deg = 4; /* Five samples no longer span +/-10 deg. */
    bad[3].max_adjust_count = AX_AVOID2_VISITED_CAPACITY;
    for (uint8_t i = 0; i < 4u; ++i) {
        AX_AVOID2_Output out;
        StartCoarse(&state, &in);
        out = AX_AVOID2_Step(&state, &bad[i], &in);
        assert(state.phase == AX_AVOID2_WAIT_TARGET);
        assert(out.pan_angle_deg == 0);
        assert(out.forward_mm_s == 0 && out.yaw_mrad_s == 0);
    }
}

int main(void)
{
    assert(OPENCTR_AVOID2_FINE_RANGE_DEG == 10);
    assert(OPENCTR_AVOID2_FINE_STEP_DEG == 5);
    TestCoarseOrderAndCandidateRanking();
    TestCoarseScanAllowsSensorSchedulingMargin();
    TestFineScanCanTryAllConfiguredCandidates();
    TestCoarseTimeoutExplainsRescan();
    TestNoCoarseCandidateExplainsRescan();
    TestWaitTargetReasonUpdatesWhenAoARecovers();
    TestOneSidedObstaclesCorrectAwayFromObstacle();
    TestBothSidesBlockedMovesToNextCandidate();
    TestCenterObstacleChoosesSaferSide();
    TestInvalidAndStaleSamplesDoNotAdvanceCoarseScan();
    TestCandidateAtServoBoundaryNeedsCompleteWindow();
    TestCorrectionLimitAndVisitedAngleGuard();
    TestExhaustedCandidatesWaitsStopped();
    TestTurnRequiresFreshCenteredClearanceAndMovesShortSegment();
    TestUnsafePostTurnAndEmergencyFrontStop();
    TestMoveStopsAfterProgressStallsLater();
    TestSlowMeasuredMotionAccumulatesFractionalDistance();
    TestOversizedCandidateConfigStopsSafely();
    TestMalformedScanConfigStopsSafely();
    return 0;
}
