#include "ax_follow_avoid_policy.h"

#include <string.h>

static AX_AVOID_Output StopOutput(const AX_AVOID_State *state)
{
    AX_AVOID_Output output = {0, 0, 0};
    if (state != 0) output.pan_angle_deg = state->pan_angle_deg;
    return output;
}

static uint8_t ConfigValid(const AX_AVOID_Config *config)
{
    return config != 0 && config->front_stop_mm > 0u &&
           config->front_release_mm > config->front_stop_mm &&
           config->side_min_mm > 0u && config->scan_max_angle_deg > 0 &&
           config->scan_max_angle_deg <= 90 &&
           config->scan_step_deg > 0 &&
           config->coarse_scan_radius_deg > 0 &&
           config->coarse_scan_radius_deg <= config->scan_max_angle_deg &&
           config->fine_scan_radius_deg > 0 &&
           config->fine_scan_radius_deg <=
               config->coarse_scan_radius_deg &&
           config->fine_scan_step_deg > 0 &&
           config->fine_scan_radius_deg % config->fine_scan_step_deg == 0 &&
           (2 * config->fine_scan_radius_deg /
                config->fine_scan_step_deg + 1) <= AX_AVOID_SCAN_MAX_POINTS &&
           config->track_probe_angle_deg > 0 &&
           config->track_probe_angle_deg <= 90 &&
           config->track_probe_step_deg > 0 &&
           config->track_probe_angle_deg % config->track_probe_step_deg == 0 &&
           config->scan_max_angle_deg % config->scan_step_deg == 0 &&
           (2 * config->scan_max_angle_deg / config->scan_step_deg + 1) <=
               AX_AVOID_SCAN_MAX_POINTS &&
           config->follow_max_mm_s > 0 &&
           config->advance_max_mm_s > 0 && config->yaw_max_mrad_s > 0 &&
           config->turn_timeout_ms > 0u &&
           config->turn_settle_max_yaw_mrad_s > 0u &&
           config->turn_settle_max_yaw_mrad_s <=
               (uint16_t)config->yaw_max_mrad_s &&
           config->turn_overshoot_mrad > 0u &&
           config->front_recovery_ms > 0u &&
           config->advance_segment_cap_mm > 0u &&
           config->advance_clearance_mm > 0u &&
           config->stall_timeout_ms > 0u &&
           config->rescan_wait_ms > 0u &&
           config->coarse_scan_timeout_ms > 0u &&
           config->fine_scan_timeout_ms > 0u &&
           config->approach_stop_cm > 0u &&
           config->approach_start_cm > config->approach_stop_cm &&
           config->align_stop_turn_deg > 0u &&
           config->align_stop_turn_deg <= 90u &&
           (config->azimuth_to_yaw_sign == 1 ||
            config->azimuth_to_yaw_sign == -1);
}

static uint8_t RangeReadyAt(const AX_AVOID_Input *input, int16_t angle_deg)
{
    return input->range_valid && input->range_status == AX_XURSC_READY &&
           input->range_angle_deg == angle_deg;
}

static uint8_t TickAfter(uint32_t candidate, uint32_t reference)
{
    uint32_t delta = candidate - reference;
    return delta != 0u && delta < 0x80000000u;
}

static void EnterHold(AX_AVOID_State *state, AX_AVOID_DiagReason reason)
{
    state->phase = AX_AVOID_HOLD;
    state->hold_reason = reason;
}

static void EnterAlign(AX_AVOID_State *state, const AX_AVOID_Input *input)
{
    state->phase = AX_AVOID_ALIGN_TARGET;
    state->phase_start_ms = input->now_ms;
    state->pan_angle_deg = 0;
}

static void EnterForward(AX_AVOID_State *state, const AX_AVOID_Input *input)
{
    state->phase = AX_AVOID_FORWARD;
    state->hold_reason = AX_AVOID_DIAG_NONE;
    state->pan_angle_deg = 0;
    state->track_probe_index = 0;
    state->track_sample_sequence = input->range_sequence;
    state->track_last_sample_ms = input->now_ms;
    state->phase_start_ms = input->now_ms;
    state->track_wait_fresh_front = 1;
}

static void AdvanceTrackProbe(AX_AVOID_State *state,
                              const AX_AVOID_Config *config)
{
    uint16_t quarter = (uint16_t)(config->track_probe_angle_deg /
                                  config->track_probe_step_deg);
    uint16_t cycle = (uint16_t)(4u * quarter);
    uint16_t index = (uint16_t)((state->track_probe_index + 1u) % cycle);
    int16_t offset = (int16_t)index;

    /* 0 -> +max -> 0 -> -max -> 0, measuring at every step. */
    if (index > quarter && index <= 3u * quarter)
        offset = (int16_t)(2 * quarter - index);
    else if (index > 3u * quarter)
        offset = (int16_t)(index - 4 * quarter);
    state->track_probe_index = index;
    state->pan_angle_deg = (int16_t)(offset * config->track_probe_step_deg);
}

static void EnterRescanWait(AX_AVOID_State *state, const AX_AVOID_Input *input)
{
    state->phase = AX_AVOID_RESCAN_WAIT;
    state->phase_start_ms = input->now_ms;
    state->pan_angle_deg = 0;
}

static void ClearDetour(AX_AVOID_State *state)
{
    state->detour_active = 0;
    state->turn_front_ready_seen = 0;
    state->front_invalid_active = 0;
    state->front_invalid_since_ms = 0;
    state->turn_settle_pending = 0;
    state->detour_angle_deg = 0;
    state->coarse_candidate_angle_deg = 0;
    state->fine_selected_angle_deg = 0;
    state->detour_side = 0;
    state->scan_index = 0;
    state->scan_stage = AX_AVOID_SCAN_COARSE;
    state->scan_center_angle_deg = 0;
    state->detour_start_ms = 0;
    state->segment_distance_mm = 0;
    state->advance_target_mm = 0;
    state->total_distance_mm = 0;
    state->turn_progress_mrad = 0;
    state->turn_target_mrad = 0;
    state->turn_settle_ms = 0;
    state->turn_settle_sequence = 0;
    state->scan_valid_mask = 0;
    memset(state->scan_distance_mm, 0, sizeof(state->scan_distance_mm));
    state->coarse_valid_mask = 0;
    state->coarse_attempted_mask = 0;
    memset(state->coarse_distance_mm, 0, sizeof(state->coarse_distance_mm));
}

static uint8_t ScanPointCount(const AX_AVOID_State *state,
                              const AX_AVOID_Config *config)
{
    if (state->scan_stage == AX_AVOID_SCAN_FINE) {
        int16_t low = (int16_t)(state->scan_center_angle_deg -
                                config->fine_scan_radius_deg);
        int16_t high = (int16_t)(state->scan_center_angle_deg +
                                 config->fine_scan_radius_deg);
        if (low < -config->scan_max_angle_deg)
            low = (int16_t)-config->scan_max_angle_deg;
        if (high > config->scan_max_angle_deg)
            high = config->scan_max_angle_deg;
        return (uint8_t)((high - low) / config->fine_scan_step_deg + 1);
    }
    return (uint8_t)(2 * config->scan_max_angle_deg /
                     config->scan_step_deg + 1);
}

static int16_t ScanAngleAt(const AX_AVOID_State *state,
                           const AX_AVOID_Config *config, uint8_t index)
{
    if (state->scan_stage == AX_AVOID_SCAN_FINE) {
        int16_t high = (int16_t)(state->scan_center_angle_deg +
                                 config->fine_scan_radius_deg);
        if (high > config->scan_max_angle_deg)
            high = config->scan_max_angle_deg;
        return (int16_t)(high - index * config->fine_scan_step_deg);
    }
    return (int16_t)(config->scan_max_angle_deg -
                     index * config->scan_step_deg);
}

static int32_t AngleError(int16_t angle, int16_t azimuth)
{
    int32_t error = (int32_t)angle - azimuth;
    return error < 0 ? -error : error;
}

static void EnterFineScan(AX_AVOID_State *state,
                          const AX_AVOID_Config *config,
                          const AX_AVOID_Input *input,
                          int16_t center_angle)
{
    int16_t high;
    state->scan_stage = AX_AVOID_SCAN_FINE;
    state->scan_center_angle_deg = center_angle;
    state->phase_start_ms = input->now_ms;
    state->scan_index = 0;
    state->scan_valid_mask = 0;
    memset(state->scan_distance_mm, 0, sizeof(state->scan_distance_mm));
    state->scan_wait_sequence = input->range_sequence;
    high = (int16_t)(center_angle + config->fine_scan_radius_deg);
    if (high > config->scan_max_angle_deg)
        high = config->scan_max_angle_deg;
    state->pan_angle_deg = high;
}

static void EnterTurnOut(AX_AVOID_State *state, const AX_AVOID_Input *input);

static void EnterDetour(AX_AVOID_State *state,
                        const AX_AVOID_Config *config,
                        const AX_AVOID_Input *input,
                        int16_t selected_angle, uint16_t selected_distance)
{
    uint16_t planning_distance_mm = selected_distance;

    if (selected_angle == 0) {
        ClearDetour(state);
        EnterAlign(state, input);
        return;
    }
    if (planning_distance_mm > config->advance_range_cap_mm)
        planning_distance_mm = config->advance_range_cap_mm;
    if (planning_distance_mm <= config->advance_clearance_mm) {
        EnterRescanWait(state, input);
        return;
    }
    state->fine_selected_angle_deg = selected_angle;
    state->detour_angle_deg = selected_angle;
    state->detour_side = state->detour_angle_deg < 0 ? -1 : 1;
    state->advance_target_mm = (uint32_t)(planning_distance_mm -
                                          config->advance_clearance_mm);
    if (state->advance_target_mm > config->advance_segment_cap_mm)
        state->advance_target_mm = config->advance_segment_cap_mm;
    state->turn_target_mrad =
        ((uint32_t)(state->detour_angle_deg < 0 ?
                    -state->detour_angle_deg : state->detour_angle_deg) *
         3142u + 90u) / 180u;
    EnterTurnOut(state, input);
}

static void EnterRecovery(AX_AVOID_State *state,
                          const AX_AVOID_Input *input,
                          AX_AVOID_DiagReason reason)
{
    ClearDetour(state);
    EnterAlign(state, input);
    state->hold_reason = reason;
}

static void PauseForFront(AX_AVOID_State *state,
                          const AX_AVOID_Config *config,
                          const AX_AVOID_Input *input)
{
    if (!state->front_invalid_active) {
        state->front_invalid_active = 1;
        state->front_invalid_since_ms = input->now_ms;
    }
    if ((uint32_t)(input->now_ms - state->front_invalid_since_ms) >=
        config->front_recovery_ms) {
        ClearDetour(state);
        EnterHold(state, AX_AVOID_DIAG_FRONT_STALE);
    }
}

static uint8_t FrontRecovered(AX_AVOID_State *state,
                              const AX_AVOID_Input *input)
{
    if (!state->front_invalid_active) return 0;
    state->phase_start_ms += input->now_ms - state->front_invalid_since_ms;
    state->front_invalid_active = 0;
    state->front_invalid_since_ms = 0;
    return 1;
}

static void EnterScan(AX_AVOID_State *state, const AX_AVOID_Config *config,
                      const AX_AVOID_Input *input)
{
    state->hold_reason = AX_AVOID_DIAG_NONE;
    if (!state->detour_active) {
        state->detour_active = 1;
        state->detour_start_ms = input->now_ms;
        state->total_distance_mm = 0;
    }
    state->phase = AX_AVOID_SCAN_SIDE;
    state->scan_stage = AX_AVOID_SCAN_COARSE;
    state->phase_start_ms = input->now_ms;
    state->scan_index = 0;
    state->coarse_candidate_angle_deg = 0;
    state->fine_selected_angle_deg = 0;
    state->detour_angle_deg = 0;
    state->detour_side = 0;
    state->advance_target_mm = 0;
    state->scan_center_angle_deg = 0;
    state->scan_valid_mask = 0;
    memset(state->scan_distance_mm, 0, sizeof(state->scan_distance_mm));
    state->coarse_valid_mask = 0;
    state->coarse_attempted_mask = 0;
    memset(state->coarse_distance_mm, 0, sizeof(state->coarse_distance_mm));
    state->scan_wait_sequence = input->range_sequence;
    state->pan_angle_deg = config->scan_max_angle_deg;
}

static int16_t ClampSpeed(int32_t value, int16_t limit)
{
    if (value > limit) value = limit;
    if (value < -limit) value = -limit;
    return (int16_t)value;
}

static void AddTurnProgress(int32_t *progress, int16_t yaw_rate_mrad_s,
                            uint32_t dt_ms, int8_t direction)
{
    int64_t projected = (int64_t)yaw_rate_mrad_s * direction * dt_ms;
    int64_t signed_delta = projected / 1000;
    int64_t next = (int64_t)*progress + signed_delta;
    if (next > INT32_MAX) *progress = INT32_MAX;
    else if (next < INT32_MIN) *progress = INT32_MIN;
    else *progress = (int32_t)next;
}

static void EnterTurnOut(AX_AVOID_State *state, const AX_AVOID_Input *input)
{
    state->phase = AX_AVOID_TURN_OUT;
    state->phase_start_ms = input->now_ms;
    state->turn_progress_mrad = 0;
    state->turn_front_ready_seen = 0;
    state->front_invalid_active = 0;
    state->front_invalid_since_ms = 0;
    state->turn_settle_pending = 0;
    state->turn_settle_ms = 0;
    state->turn_settle_sequence = 0;
    state->pan_angle_deg = 0;
}

static void SelectCoarseDirection(AX_AVOID_State *state,
                                  const AX_AVOID_Config *config,
                                  const AX_AVOID_Input *input)
{
    uint8_t count = (uint8_t)(2 * config->scan_max_angle_deg /
                              config->scan_step_deg + 1);
    uint8_t found = 0;
    uint16_t best_clearance = 0;
    int16_t best_angle = 0;
    uint8_t best_index = 0;
    uint8_t radius_steps = (uint8_t)((config->coarse_scan_radius_deg +
                                      config->scan_step_deg - 1) /
                                     config->scan_step_deg);
    for (uint8_t i = radius_steps; i + radius_steps < count; ++i) {
        int16_t angle = (int16_t)(config->scan_max_angle_deg -
                                  i * config->scan_step_deg);
        uint16_t clearance = UINT16_MAX;
        uint32_t required_mask =
            (((uint32_t)1u << (2u * radius_steps + 1u)) - 1u) <<
            (i - radius_steps);
        if ((state->coarse_valid_mask & required_mask) != required_mask ||
            (state->coarse_attempted_mask & ((uint32_t)1u << i)) != 0u ||
            angle - config->coarse_scan_radius_deg <
                -config->scan_max_angle_deg ||
            angle + config->coarse_scan_radius_deg >
                config->scan_max_angle_deg)
            continue;
        for (uint8_t j = (uint8_t)(i - radius_steps);
             j <= i + radius_steps; ++j)
            if (state->coarse_distance_mm[j] < clearance)
                clearance = state->coarse_distance_mm[j];
        if (!found || AngleError(angle, input->aoa_azimuth_deg) <
                          AngleError(best_angle, input->aoa_azimuth_deg) ||
            (AngleError(angle, input->aoa_azimuth_deg) ==
             AngleError(best_angle, input->aoa_azimuth_deg) &&
             clearance > best_clearance)) {
            found = 1;
            best_clearance = clearance;
            best_angle = angle;
            best_index = i;
        }
    }
    if (!found) {
        EnterRescanWait(state, input);
        return;
    }
    state->coarse_attempted_mask |= (uint32_t)1u << best_index;
    state->coarse_candidate_angle_deg = best_angle;
    EnterFineScan(state, config, input, best_angle);
}

static void SelectFineDirection(AX_AVOID_State *state,
                                const AX_AVOID_Config *config,
                                const AX_AVOID_Input *input)
{
    uint8_t count = ScanPointCount(state, config);
    uint16_t clearance = UINT16_MAX;
    /* The entire measured +/- radius must be clear around the turn angle. */
    for (uint8_t i = 0; i < count; ++i) {
        if ((state->scan_valid_mask & ((uint32_t)1u << i)) == 0u) {
            SelectCoarseDirection(state, config, input);
            return;
        }
        if (state->scan_distance_mm[i] < clearance)
            clearance = state->scan_distance_mm[i];
    }
    EnterDetour(state, config, input, state->scan_center_angle_deg,
                clearance);
}

static AX_AVOID_Output StepTurnOut(AX_AVOID_State *state,
                                   const AX_AVOID_Config *config,
                                   const AX_AVOID_Input *input,
                                   uint32_t dt_ms)
{
    AX_AVOID_Output output;
    uint32_t now_ms = input->now_ms;
    int8_t direction;

    /* Keep counting actual rotation while stopped for a missing echo. */
    direction = (int8_t)(state->detour_side * config->azimuth_to_yaw_sign);
    if (state->turn_front_ready_seen)
        AddTurnProgress(&state->turn_progress_mrad,
                        input->yaw_rate_mrad_s, dt_ms, direction);
    if (!RangeReadyAt(input, 0)) {
        PauseForFront(state, config, input);
        return StopOutput(state);
    }
    FrontRecovered(state, input);
    if ((uint32_t)(now_ms - state->phase_start_ms) >=
        config->turn_timeout_ms) {
        EnterRecovery(state, input, AX_AVOID_DIAG_TURN_TIMEOUT);
        return StopOutput(state);
    }
    if (!state->turn_front_ready_seen)
        AddTurnProgress(&state->turn_progress_mrad,
                        input->yaw_rate_mrad_s, dt_ms, direction);
    state->turn_front_ready_seen = 1;
    if (state->turn_progress_mrad >
        state->turn_target_mrad + config->turn_overshoot_mrad) {
        EnterScan(state, config, input);
        return StopOutput(state);
    }
    if (state->turn_progress_mrad >= state->turn_target_mrad) {
        int32_t yaw_rate = input->yaw_rate_mrad_s;
        if (yaw_rate < 0) yaw_rate = -yaw_rate;
        if (yaw_rate > config->turn_settle_max_yaw_mrad_s) {
            state->turn_settle_pending = 0;
            return StopOutput(state);
        }
        if (!state->turn_settle_pending) {
            state->turn_settle_pending = 1;
            state->turn_settle_ms = now_ms;
            state->turn_settle_sequence = input->range_sequence;
            return StopOutput(state);
        }
        if (input->range_sequence == state->turn_settle_sequence ||
            !TickAfter(input->range_triggered_ms, state->turn_settle_ms))
            return StopOutput(state);
        if (input->range_distance_mm >= config->front_release_mm &&
            state->advance_target_mm != 0u) {
            state->phase = AX_AVOID_ADVANCE;
            state->phase_start_ms = now_ms;
            state->segment_distance_mm = 0;
            output = StopOutput(state);
            output.forward_mm_s = config->advance_max_mm_s;
            return output;
        }
        EnterRecovery(state, input, AX_AVOID_DIAG_TURN_BLOCKED);
        return StopOutput(state);
    }
    state->turn_settle_pending = 0;
    output = StopOutput(state);
    output.yaw_mrad_s = (int16_t)(state->detour_side *
                                  config->azimuth_to_yaw_sign *
                                  config->yaw_max_mrad_s);
    return output;
}

static AX_AVOID_Output StepAlignTarget(AX_AVOID_State *state,
                                       const AX_AVOID_Config *config,
                                       const AX_AVOID_Input *input)
{
    AX_AVOID_Output output = StopOutput(state);
    int32_t azimuth = input->aoa_azimuth_deg;
    uint32_t absolute_azimuth = (uint32_t)(azimuth < 0 ? -azimuth : azimuth);

    if (absolute_azimuth > config->align_stop_turn_deg ||
        input->follow_target.yaw_mrad_s != 0) {
        int16_t yaw = input->follow_target.yaw_mrad_s;
        if (yaw == 0)
            yaw = (int16_t)((azimuth > 0 ? 1 : -1) *
                            config->azimuth_to_yaw_sign *
                            config->yaw_max_mrad_s);
        output.yaw_mrad_s = ClampSpeed(yaw, config->yaw_max_mrad_s);
        return output;
    }
    if (!state->approach_needed) return output;
    EnterForward(state, input);
    return output;
}

static AX_AVOID_Output StepForward(AX_AVOID_State *state,
                                   const AX_AVOID_Config *config,
                                   const AX_AVOID_Input *input)
{
    AX_AVOID_Output output;
    if (!state->approach_needed) {
        EnterAlign(state, input);
        return StopOutput(state);
    }
    if ((uint32_t)(input->aoa_azimuth_deg < 0 ?
                   -input->aoa_azimuth_deg : input->aoa_azimuth_deg) >
        config->align_stop_turn_deg ||
        input->follow_target.yaw_mrad_s != 0) {
        EnterAlign(state, input);
        return StepAlignTarget(state, config, input);
    }
    if (state->track_wait_fresh_front) {
        if (!RangeReadyAt(input, 0) ||
            input->range_sequence == state->track_sample_sequence ||
            !TickAfter(input->range_triggered_ms, state->phase_start_ms))
            return StopOutput(state);
        state->track_wait_fresh_front = 0;
        state->track_sample_sequence = input->range_sequence;
        state->track_last_sample_ms = input->now_ms;
        if (input->range_distance_mm < config->front_release_mm) {
            EnterScan(state, config, input);
            return StopOutput(state);
        }
        AdvanceTrackProbe(state, config);
    }
    if (input->range_valid &&
               input->range_angle_deg == state->pan_angle_deg &&
               input->range_sequence != state->track_sample_sequence) {
        if (input->range_status != AX_XURSC_READY) {
            EnterAlign(state, input);
            return StopOutput(state);
        }
        state->track_sample_sequence = input->range_sequence;
        state->track_last_sample_ms = input->now_ms;
        if (input->range_distance_mm < config->front_stop_mm) {
            EnterScan(state, config, input);
            return StopOutput(state);
        }
        AdvanceTrackProbe(state, config);
    } else if ((uint32_t)(input->now_ms - state->track_last_sample_ms) >=
               config->range_max_age_ms) {
        EnterAlign(state, input);
        return StopOutput(state);
    }
    output = StopOutput(state);
    output.forward_mm_s = input->follow_target.forward_mm_s;
    if (output.forward_mm_s < 0) output.forward_mm_s = 0;
    if (output.forward_mm_s > config->follow_max_mm_s)
        output.forward_mm_s = config->follow_max_mm_s;
    output.yaw_mrad_s = 0;
    return output;
}

static AX_AVOID_Output StepScanSide(AX_AVOID_State *state,
                                    const AX_AVOID_Config *config,
                                    const AX_AVOID_Input *input)
{
    uint16_t timeout_ms = state->scan_stage == AX_AVOID_SCAN_FINE ?
        config->fine_scan_timeout_ms : config->coarse_scan_timeout_ms;
    if ((uint32_t)(input->now_ms - state->phase_start_ms) >=
        timeout_ms) {
        EnterRescanWait(state, input);
        return StopOutput(state);
    }
    if (input->range_valid &&
        input->range_sequence != state->scan_wait_sequence) {
        int16_t expected_angle = ScanAngleAt(state, config,
                                             state->scan_index);
        if (input->range_angle_deg == expected_angle) {
            uint8_t count = ScanPointCount(state, config);
            state->scan_distance_mm[state->scan_index] =
                input->range_distance_mm;
            if (input->range_status == AX_XURSC_READY &&
                input->range_distance_mm >= config->side_min_mm &&
                ((expected_angle == 0 && input->range_distance_mm >=
                                          config->front_release_mm) ||
                 (expected_angle != 0 && input->range_distance_mm >
                                          config->advance_clearance_mm)))
                state->scan_valid_mask |=
                    (uint32_t)1u << state->scan_index;
            state->scan_wait_sequence = input->range_sequence;
            ++state->scan_index;
            if (state->scan_index < count)
                state->pan_angle_deg = ScanAngleAt(state, config,
                                                   state->scan_index);
            else if (state->scan_stage == AX_AVOID_SCAN_COARSE) {
                state->coarse_valid_mask = state->scan_valid_mask;
                memcpy(state->coarse_distance_mm, state->scan_distance_mm,
                       sizeof(state->coarse_distance_mm));
                SelectCoarseDirection(state, config, input);
            } else
                SelectFineDirection(state, config, input);
        }
    }
    return StopOutput(state);
}

static AX_AVOID_Output StepAdvance(AX_AVOID_State *state,
                                   const AX_AVOID_Config *config,
                                   const AX_AVOID_Input *input,
                                   uint32_t dt_ms)
{
    AX_AVOID_Output output;
    uint8_t front_recovered;
    if (input->forward_mm_s > 0 && dt_ms != 0u) {
        uint64_t increment = (uint64_t)(uint16_t)input->forward_mm_s *
                             dt_ms / 1000u;
        if (increment > UINT32_MAX - state->segment_distance_mm)
            state->segment_distance_mm = UINT32_MAX;
        else state->segment_distance_mm += (uint32_t)increment;
        if (increment > UINT32_MAX - state->total_distance_mm)
            state->total_distance_mm = UINT32_MAX;
        else state->total_distance_mm += (uint32_t)increment;
    }
    if (!RangeReadyAt(input, 0)) {
        PauseForFront(state, config, input);
        return StopOutput(state);
    }
    front_recovered = FrontRecovered(state, input);
    if (front_recovered &&
        input->range_distance_mm < config->front_release_mm) {
        EnterRecovery(state, input, AX_AVOID_DIAG_TURN_BLOCKED);
        return StopOutput(state);
    }
    if (input->range_distance_mm < config->front_stop_mm) {
        EnterScan(state, config, input);
        return StopOutput(state);
    }
    if (config->max_distance_mm != 0u &&
        state->total_distance_mm >= config->max_distance_mm) {
        EnterRecovery(state, input, AX_AVOID_DIAG_DISTANCE_LIMIT);
        return StopOutput(state);
    }
    if ((uint32_t)(input->now_ms - state->phase_start_ms) >=
            config->stall_timeout_ms &&
        state->segment_distance_mm < config->stall_min_mm) {
        EnterRecovery(state, input, AX_AVOID_DIAG_ADVANCE_STALL);
        return StopOutput(state);
    }
    if (state->segment_distance_mm >= state->advance_target_mm) {
        ClearDetour(state);
        EnterAlign(state, input);
        return StopOutput(state);
    }
    output = StopOutput(state);
    output.forward_mm_s = config->advance_max_mm_s;
    return output;
}

void AX_AVOID_Reset(AX_AVOID_State *state)
{
    if (state == 0) return;
    memset(state, 0, sizeof(*state));
    state->phase = AX_AVOID_ALIGN_TARGET;
    state->initialized = 1;
}

AX_AVOID_Output AX_AVOID_Step(AX_AVOID_State *state,
                              const AX_AVOID_Config *config,
                              const AX_AVOID_Input *input)
{
    uint32_t dt_ms = 0;
    uint32_t now_ms;

    if (state == 0 || input == 0) return StopOutput(0);
    if (!state->initialized) AX_AVOID_Reset(state);
    if (!ConfigValid(config)) {
        EnterHold(state, AX_AVOID_DIAG_BAD_CONFIG);
        return StopOutput(state);
    }

    now_ms = input->now_ms;
    if (state->have_tick)
        dt_ms = now_ms - state->last_tick_ms;
    state->last_tick_ms = now_ms;
    state->have_tick = 1;

    /* Stationary observation must not exhaust the moving-detour timer. */
    if (state->detour_active &&
        (state->phase == AX_AVOID_ALIGN_TARGET ||
         state->phase == AX_AVOID_SCAN_SIDE ||
         state->phase == AX_AVOID_RESCAN_WAIT))
        state->detour_start_ms += dt_ms;

    if (state->phase == AX_AVOID_HOLD) {
        if (state->hold_reason == AX_AVOID_DIAG_FRONT_STALE &&
            input->aoa_valid && RangeReadyAt(input, 0)) {
            if (input->aoa_distance_cm >= config->approach_start_cm)
                state->approach_needed = 1;
            else if (input->aoa_distance_cm <= config->approach_stop_cm)
                state->approach_needed = 0;
            if (state->approach_needed)
                EnterScan(state, config, input);
            else
                EnterAlign(state, input);
        }
        return StopOutput(state);
    }
    if (!input->aoa_valid) {
        state->approach_needed = 0;
        ClearDetour(state);
        EnterAlign(state, input);
        return StopOutput(state);
    } else if (input->aoa_distance_cm >= config->approach_start_cm) {
        state->approach_needed = 1;
    } else if (input->aoa_distance_cm <= config->approach_stop_cm) {
        state->approach_needed = 0;
    }
    if (!state->approach_needed && state->detour_active) {
        ClearDetour(state);
        EnterAlign(state, input);
        return StopOutput(state);
    }
    if (state->detour_active && config->max_time_ms != 0u &&
        (uint32_t)(now_ms - state->detour_start_ms) >= config->max_time_ms) {
        EnterRecovery(state, input, AX_AVOID_DIAG_TIME_LIMIT);
        return StopOutput(state);
    }
    if ((state->phase == AX_AVOID_TURN_OUT ||
         state->phase == AX_AVOID_ADVANCE) && dt_ms > 100u) {
        EnterRecovery(state, input, AX_AVOID_DIAG_LOOP_LATE);
        return StopOutput(state);
    }

    switch (state->phase) {
    case AX_AVOID_ALIGN_TARGET:
        return StepAlignTarget(state, config, input);
    case AX_AVOID_FORWARD:
        return StepForward(state, config, input);
    case AX_AVOID_SCAN_SIDE:
        return StepScanSide(state, config, input);
    case AX_AVOID_RESCAN_WAIT:
        state->pan_angle_deg = 0;
        if ((uint32_t)(now_ms - state->phase_start_ms) >=
            config->rescan_wait_ms)
            EnterAlign(state, input);
        return StopOutput(state);
    case AX_AVOID_TURN_OUT:
        return StepTurnOut(state, config, input, dt_ms);
    case AX_AVOID_ADVANCE:
        return StepAdvance(state, config, input, dt_ms);
    case AX_AVOID_HOLD:
    default:
        EnterHold(state, AX_AVOID_DIAG_HOLD_UNKNOWN);
        return StopOutput(state);
    }
}
