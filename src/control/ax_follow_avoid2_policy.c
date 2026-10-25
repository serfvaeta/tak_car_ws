#include "ax_follow_avoid2_policy.h"

#include <limits.h>
#include <string.h>

static AX_AVOID2_Output StopOutput(const AX_AVOID2_State *state)
{
    AX_AVOID2_Output output = {0, 0, 0};
    if (state != 0) output.pan_angle_deg = state->pan_angle_deg;
    return output;
}

static uint8_t ConfigValid(const AX_AVOID2_Config *config)
{
    return config->emergency_distance_mm > 0u &&
           config->safe_distance_mm > config->emergency_distance_mm &&
           config->scan_max_angle_deg > 0 &&
           config->scan_max_angle_deg <= 90 &&
           config->coarse_step_deg > 0 &&
           config->scan_max_angle_deg % config->coarse_step_deg == 0 &&
           (2 * config->scan_max_angle_deg / config->coarse_step_deg + 1) <=
               (int)AX_AVOID2_COARSE_POINT_COUNT &&
           config->fine_step_deg > 0 &&
           config->fine_range_deg == 2 * config->fine_step_deg &&
           config->fine_range_deg <= config->scan_max_angle_deg &&
           config->max_candidates > 0u &&
           config->max_candidates <= AX_AVOID2_CANDIDATE_CAPACITY &&
           config->max_adjust_count > 0u &&
           config->max_adjust_count < AX_AVOID2_VISITED_CAPACITY &&
           config->range_max_age_ms > 0u &&
           config->approach_stop_cm > 0u &&
           config->approach_start_cm > config->approach_stop_cm &&
           config->advance_max_mm_s > 0 &&
           config->yaw_max_mrad_s > 0 &&
           config->turn_timeout_ms > 0u &&
           config->turn_settle_max_yaw_mrad_s <=
               (uint16_t)config->yaw_max_mrad_s &&
           config->advance_clearance_mm >= config->emergency_distance_mm &&
           config->safe_distance_mm > config->advance_clearance_mm &&
           config->advance_range_cap_mm > config->advance_clearance_mm &&
           config->stall_timeout_ms > 0u &&
           config->stall_min_mm > 0u &&
           config->rescan_wait_ms > 0u &&
           config->coarse_scan_timeout_ms > 0u &&
           config->fine_scan_timeout_ms > 0u &&
           config->verify_timeout_ms > 0u &&
           (config->aoa_to_yaw_sign == 1 ||
            config->aoa_to_yaw_sign == -1);
}

static uint8_t TickAfter(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) > 0;
}

static uint16_t AbsAngleDiff(int16_t a, int16_t b)
{
    int32_t difference = (int32_t)a - b;
    if (difference < 0) difference = -difference;
    return (uint16_t)difference;
}

static int8_t AngleSide(int16_t angle)
{
    return angle < 0 ? -1 : angle > 0 ? 1 : 0;
}

static void RequestPan(AX_AVOID2_State *state,
                       const AX_AVOID2_Input *input, int16_t angle)
{
    state->pan_angle_deg = angle;
    state->range_wait_sequence = input->range_sequence;
    state->range_after_ms = input->now_ms;
}

static void EnterWaitTarget(AX_AVOID2_State *state,
                            const AX_AVOID2_Input *input,
                            AX_AVOID2_Reason reason)
{
    state->phase = AX_AVOID2_WAIT_TARGET;
    state->reason = reason;
    state->phase_start_ms = input->now_ms;
    state->approach_needed = 0;
    state->candidate_count = 0;
    state->candidate_index = 0;
    state->scan_index = 0;
    state->coarse_valid_mask = 0;
    RequestPan(state, input, 0);
}

static void EnterWaitRescan(AX_AVOID2_State *state,
                            const AX_AVOID2_Input *input,
                            AX_AVOID2_Reason reason)
{
    state->phase = AX_AVOID2_WAIT_RESCAN;
    state->reason = reason;
    state->phase_start_ms = input->now_ms;
    state->segment_distance_mm = 0;
    state->segment_target_mm = 0;
    RequestPan(state, input, 0);
}

static void BeginCoarseScan(AX_AVOID2_State *state,
                            const AX_AVOID2_Input *input)
{
    memset(state->coarse_distances_mm, 0,
           sizeof(state->coarse_distances_mm));
    memset(state->candidate_angles, 0, sizeof(state->candidate_angles));
    memset(state->candidate_distances_mm, 0,
           sizeof(state->candidate_distances_mm));
    state->coarse_valid_mask = 0;
    state->candidate_count = 0;
    state->candidate_index = 0;
    state->scan_index = 0;
    state->adjust_count = 0;
    state->visited_count = 0;
    state->segment_distance_mm = 0;
    state->segment_target_mm = 0;
    state->phase = AX_AVOID2_COARSE_SCAN;
    state->reason = AX_AVOID2_REASON_NONE;
    state->phase_start_ms = input->now_ms;
    RequestPan(state, input, 0);
}

static int16_t CoarseAngleAt(uint8_t index, int16_t step)
{
    uint16_t magnitude;
    if (index == 0u) return 0;
    magnitude = (uint16_t)((index + 1u) / 2u) * (uint16_t)step;
    return (index & 1u) != 0u ? (int16_t)magnitude :
                               (int16_t)-(int16_t)magnitude;
}

static uint8_t CoarsePointCount(const AX_AVOID2_Config *config)
{
    return (uint8_t)(2 * config->scan_max_angle_deg /
                     config->coarse_step_deg + 1);
}

static uint8_t WindowInRange(const AX_AVOID2_Config *config, int16_t center)
{
    return center - config->fine_range_deg >= -config->scan_max_angle_deg &&
           center + config->fine_range_deg <= config->scan_max_angle_deg;
}

static uint8_t CandidateBefore(const AX_AVOID2_State *state,
                               const AX_AVOID2_Input *input,
                               int16_t angle, uint16_t distance,
                               uint8_t other)
{
    uint16_t error = AbsAngleDiff(angle, input->aoa_azimuth_deg);
    uint16_t other_error = AbsAngleDiff(state->candidate_angles[other],
                                        input->aoa_azimuth_deg);
    int8_t side;
    int8_t other_side;
    if (error != other_error) return error < other_error;
    if (distance != state->candidate_distances_mm[other])
        return distance > state->candidate_distances_mm[other];
    side = AngleSide(angle);
    other_side = AngleSide(state->candidate_angles[other]);
    return state->last_safe_side != 0 && side == state->last_safe_side &&
           other_side != state->last_safe_side;
}

static void AddCandidate(AX_AVOID2_State *state,
                         const AX_AVOID2_Config *config,
                         const AX_AVOID2_Input *input,
                         int16_t angle, uint16_t distance)
{
    uint8_t position = 0;
    while (position < state->candidate_count &&
           !CandidateBefore(state, input, angle, distance, position))
        ++position;
    if (position >= config->max_candidates) return;
    if (state->candidate_count < config->max_candidates)
        ++state->candidate_count;
    for (uint8_t i = state->candidate_count; i > position + 1u; --i) {
        state->candidate_angles[i - 1u] = state->candidate_angles[i - 2u];
        state->candidate_distances_mm[i - 1u] =
            state->candidate_distances_mm[i - 2u];
    }
    state->candidate_angles[position] = angle;
    state->candidate_distances_mm[position] = distance;
}

static void StartFineCycle(AX_AVOID2_State *state,
                           const AX_AVOID2_Config *config,
                           const AX_AVOID2_Input *input)
{
    memset(state->fine_distances_mm, 0, sizeof(state->fine_distances_mm));
    state->fine_index = 0;
    state->fine_known_mask = 0;
    state->fine_has_unknown = 0;
    state->phase = AX_AVOID2_FINE_SCAN;
    RequestPan(state, input,
               (int16_t)(state->fine_center_angle_deg -
                         config->fine_range_deg));
}

static void StartNextCandidate(AX_AVOID2_State *state,
                               const AX_AVOID2_Config *config,
                               const AX_AVOID2_Input *input,
                               uint8_t candidate_index)
{
    state->candidate_index = candidate_index;
    if (candidate_index == 0u) state->phase_start_ms = input->now_ms;
    while (state->candidate_index < state->candidate_count) {
        int16_t center = state->candidate_angles[state->candidate_index];
        if (WindowInRange(config, center)) {
            state->fine_center_angle_deg = center;
            state->adjust_count = 0;
            state->visited_count = 1;
            state->visited_centers_deg[0] = center;
            StartFineCycle(state, config, input);
            return;
        }
        ++state->candidate_index;
    }
    EnterWaitRescan(state, input, state->candidate_count == 0u ?
                    AX_AVOID2_REASON_NO_COARSE_CANDIDATE :
                    AX_AVOID2_REASON_NO_SAFE_FINE);
}

static void SelectCoarseCandidates(AX_AVOID2_State *state,
                                   const AX_AVOID2_Config *config,
                                   const AX_AVOID2_Input *input)
{
    uint8_t count = CoarsePointCount(config);
    state->candidate_count = 0;
    for (uint8_t i = 0; i < count; ++i) {
        if ((state->coarse_valid_mask & ((uint32_t)1u << i)) != 0u)
            AddCandidate(state, config, input,
                         CoarseAngleAt(i, config->coarse_step_deg),
                         state->coarse_distances_mm[i]);
    }
    StartNextCandidate(state, config, input, 0);
}

static void FailCandidate(AX_AVOID2_State *state,
                          const AX_AVOID2_Config *config,
                          const AX_AVOID2_Input *input)
{
    StartNextCandidate(state, config, input,
                       (uint8_t)(state->candidate_index + 1u));
}

static void EnterTurn(AX_AVOID2_State *state,
                      const AX_AVOID2_Config *config,
                      const AX_AVOID2_Input *input)
{
    int16_t angle = state->fine_center_angle_deg;
    uint32_t absolute_angle = (uint32_t)(angle < 0 ? -angle : angle);
    state->phase = AX_AVOID2_TURN;
    state->phase_start_ms = input->now_ms;
    state->turn_progress_mrad = 0;
    state->turn_target_mrad =
        (int32_t)((absolute_angle * 3142u + 90u) / 180u);
    /* Planning angles are positive-right; raw positive AOA uses the
     * opposite side convention, so a right candidate needs negative yaw
     * when the calibrated AOA-to-yaw sign is positive. */
    state->turn_direction = (int8_t)(-AngleSide(angle) *
                                      config->aoa_to_yaw_sign);
    if (AngleSide(angle) != 0) state->last_safe_side = AngleSide(angle);
    RequestPan(state, input, 0);
}

static int16_t FineAngleAt(const AX_AVOID2_State *state,
                           const AX_AVOID2_Config *config, uint8_t index)
{
    int16_t offset = (int16_t)(-config->fine_range_deg +
                               (int16_t)index * config->fine_step_deg);
    return (int16_t)(state->fine_center_angle_deg + offset);
}

static uint8_t RangeSampleAfterRequest(const AX_AVOID2_State *state,
                                       const AX_AVOID2_Config *config,
                                       const AX_AVOID2_Input *input,
                                       int16_t angle)
{
    return input->range_valid &&
           input->range_sequence != state->range_wait_sequence &&
           input->range_angle_deg == angle &&
           TickAfter(input->range_triggered_ms, state->range_after_ms) &&
           (uint32_t)(input->now_ms - input->range_triggered_ms) <=
               config->range_max_age_ms;
}

static uint8_t RangeReadyAt(const AX_AVOID2_Config *config,
                            const AX_AVOID2_Input *input, int16_t angle)
{
    return input->range_valid && input->range_angle_deg == angle &&
           input->range_status == AX_XURSC_READY &&
           (uint32_t)(input->now_ms - input->range_triggered_ms) <=
               config->range_max_age_ms;
}

static void CorrectFineCenter(AX_AVOID2_State *state,
                              const AX_AVOID2_Config *config,
                              const AX_AVOID2_Input *input,
                              int8_t direction)
{
    int16_t next_center = (int16_t)(state->fine_center_angle_deg +
                                    direction * config->fine_step_deg);
    if (state->adjust_count >= config->max_adjust_count ||
        !WindowInRange(config, next_center)) {
        FailCandidate(state, config, input);
        return;
    }
    for (uint8_t i = 0; i < state->visited_count; ++i) {
        if (state->visited_centers_deg[i] == next_center) {
            FailCandidate(state, config, input);
            return;
        }
    }
    if (state->visited_count >= AX_AVOID2_VISITED_CAPACITY) {
        FailCandidate(state, config, input);
        return;
    }
    ++state->adjust_count;
    state->visited_centers_deg[state->visited_count++] = next_center;
    state->fine_center_angle_deg = next_center;
    StartFineCycle(state, config, input);
}

static void EvaluateFineWindow(AX_AVOID2_State *state,
                               const AX_AVOID2_Config *config,
                               const AX_AVOID2_Input *input)
{
    uint8_t left_bad;
    uint8_t right_bad;
    uint8_t center_bad;
    int8_t correction;
    if (state->fine_has_unknown ||
        state->fine_known_mask != (uint8_t)((1u << AX_AVOID2_FINE_POINT_COUNT) -
                                            1u)) {
        FailCandidate(state, config, input);
        return;
    }

    left_bad = state->fine_distances_mm[0] < config->safe_distance_mm ||
               state->fine_distances_mm[1] < config->safe_distance_mm;
    center_bad = state->fine_distances_mm[2] < config->safe_distance_mm;
    right_bad = state->fine_distances_mm[3] < config->safe_distance_mm ||
                state->fine_distances_mm[4] < config->safe_distance_mm;
    if (!left_bad && !center_bad && !right_bad) {
        EnterTurn(state, config, input);
        return;
    }
    if (left_bad && right_bad) {
        FailCandidate(state, config, input);
        return;
    }
    if (left_bad) {
        CorrectFineCenter(state, config, input, 1);
        return;
    }
    if (right_bad) {
        CorrectFineCenter(state, config, input, -1);
        return;
    }

    {
        uint16_t left_score = state->fine_distances_mm[0] <
                              state->fine_distances_mm[1] ?
                              state->fine_distances_mm[0] :
                              state->fine_distances_mm[1];
        uint16_t right_score = state->fine_distances_mm[3] <
                               state->fine_distances_mm[4] ?
                               state->fine_distances_mm[3] :
                               state->fine_distances_mm[4];
        if (right_score != left_score) {
            correction = right_score > left_score ? 1 : -1;
        } else {
            int16_t right_center = (int16_t)(state->fine_center_angle_deg +
                                             config->fine_step_deg);
            int16_t left_center = (int16_t)(state->fine_center_angle_deg -
                                            config->fine_step_deg);
            uint16_t right_error = AbsAngleDiff(right_center,
                                                input->aoa_azimuth_deg);
            uint16_t left_error = AbsAngleDiff(left_center,
                                               input->aoa_azimuth_deg);
            if (right_error != left_error)
                correction = right_error < left_error ? 1 : -1;
            else if (state->last_safe_side != 0)
                correction = state->last_safe_side;
            else
                correction = 1;
        }
    }
    CorrectFineCenter(state, config, input, correction);
}

static void IntegrateTurn(AX_AVOID2_State *state, int16_t yaw_rate,
                          uint32_t dt_ms)
{
    int64_t change = (int64_t)yaw_rate * state->turn_direction * dt_ms / 1000;
    int64_t progress = (int64_t)state->turn_progress_mrad + change;
    if (progress > INT32_MAX) progress = INT32_MAX;
    if (progress < INT32_MIN) progress = INT32_MIN;
    state->turn_progress_mrad = (int32_t)progress;
}

static void EnterVerifyFront(AX_AVOID2_State *state,
                             const AX_AVOID2_Input *input)
{
    state->phase = AX_AVOID2_VERIFY_FRONT;
    state->phase_start_ms = input->now_ms;
    RequestPan(state, input, 0);
}

static AX_AVOID2_Output StepCoarse(AX_AVOID2_State *state,
                                   const AX_AVOID2_Config *config,
                                   const AX_AVOID2_Input *input)
{
    uint8_t count = CoarsePointCount(config);
    int16_t expected_angle = CoarseAngleAt(state->scan_index,
                                           config->coarse_step_deg);
    if ((uint32_t)(input->now_ms - state->phase_start_ms) >=
        config->coarse_scan_timeout_ms) {
        EnterWaitRescan(state, input, AX_AVOID2_REASON_COARSE_TIMEOUT);
        return StopOutput(state);
    }
    if (RangeSampleAfterRequest(state, config, input, expected_angle)) {
        if (input->range_status == AX_XURSC_READY &&
            input->range_distance_mm >= config->safe_distance_mm) {
            state->coarse_valid_mask |= (uint32_t)1u << state->scan_index;
            state->coarse_distances_mm[state->scan_index] =
                input->range_distance_mm;
        }
        state->range_wait_sequence = input->range_sequence;
        ++state->scan_index;
        if (state->scan_index >= count) {
            SelectCoarseCandidates(state, config, input);
        } else {
            RequestPan(state, input,
                       CoarseAngleAt(state->scan_index,
                                     config->coarse_step_deg));
        }
    }
    return StopOutput(state);
}

static AX_AVOID2_Output StepFine(AX_AVOID2_State *state,
                                 const AX_AVOID2_Config *config,
                                 const AX_AVOID2_Input *input)
{
    int16_t expected_angle;
    if ((uint32_t)(input->now_ms - state->phase_start_ms) >=
        config->fine_scan_timeout_ms) {
        EnterWaitRescan(state, input, AX_AVOID2_REASON_FINE_TIMEOUT);
        return StopOutput(state);
    }
    expected_angle = FineAngleAt(state, config, state->fine_index);
    if (RangeSampleAfterRequest(state, config, input, expected_angle)) {
        if (input->range_status == AX_XURSC_READY &&
            input->range_distance_mm > 0u) {
            state->fine_known_mask |= (uint8_t)(1u << state->fine_index);
            state->fine_distances_mm[state->fine_index] =
                input->range_distance_mm;
        } else {
            state->fine_has_unknown = 1;
        }
        state->range_wait_sequence = input->range_sequence;
        ++state->fine_index;
        if (state->fine_index >= AX_AVOID2_FINE_POINT_COUNT) {
            EvaluateFineWindow(state, config, input);
        } else {
            RequestPan(state, input,
                       FineAngleAt(state, config, state->fine_index));
        }
    }
    return StopOutput(state);
}

static AX_AVOID2_Output StepTurn(AX_AVOID2_State *state,
                                 const AX_AVOID2_Config *config,
                                 const AX_AVOID2_Input *input,
                                 uint32_t dt_ms)
{
    AX_AVOID2_Output output;
    int32_t yaw_rate = input->yaw_rate_mrad_s;
    if ((uint32_t)(input->now_ms - state->phase_start_ms) >=
        config->turn_timeout_ms) {
        EnterWaitRescan(state, input, AX_AVOID2_REASON_TURN_TIMEOUT);
        return StopOutput(state);
    }
    if (!RangeReadyAt(config, input, 0) ||
        input->range_sequence == state->range_wait_sequence ||
        !TickAfter(input->range_triggered_ms, state->range_after_ms))
        return StopOutput(state);
    if (input->range_distance_mm < config->emergency_distance_mm) {
        EnterWaitRescan(state, input, AX_AVOID2_REASON_FRONT_EMERGENCY);
        return StopOutput(state);
    }
    IntegrateTurn(state, input->yaw_rate_mrad_s, dt_ms);
    if (state->turn_progress_mrad >= state->turn_target_mrad) {
        if (yaw_rate < 0) yaw_rate = -yaw_rate;
        if ((uint32_t)yaw_rate > config->turn_settle_max_yaw_mrad_s) {
            return StopOutput(state);
        }
        EnterVerifyFront(state, input);
        return StopOutput(state);
    }
    output = StopOutput(state);
    output.yaw_mrad_s = (int16_t)(state->turn_direction *
                                  config->yaw_max_mrad_s);
    return output;
}

static AX_AVOID2_Output StepVerifyFront(AX_AVOID2_State *state,
                                        const AX_AVOID2_Config *config,
                                        const AX_AVOID2_Input *input)
{
    if ((uint32_t)(input->now_ms - state->phase_start_ms) >=
        config->verify_timeout_ms) {
        EnterWaitRescan(state, input, AX_AVOID2_REASON_VERIFY_TIMEOUT);
        return StopOutput(state);
    }
    if (RangeSampleAfterRequest(state, config, input, 0)) {
        state->range_wait_sequence = input->range_sequence;
        if (input->range_status != AX_XURSC_READY ||
            input->range_distance_mm < config->safe_distance_mm) {
            EnterWaitRescan(state, input, AX_AVOID2_REASON_VERIFY_BLOCKED);
            return StopOutput(state);
        }
        {
            uint16_t planning_distance_mm = input->range_distance_mm;
            if (planning_distance_mm > config->advance_range_cap_mm)
                planning_distance_mm = config->advance_range_cap_mm;
            if (planning_distance_mm <= config->advance_clearance_mm) {
                EnterWaitRescan(state, input, AX_AVOID2_REASON_VERIFY_BLOCKED);
                return StopOutput(state);
            }
            state->segment_target_mm = (uint32_t)(planning_distance_mm -
                                                   config->advance_clearance_mm);
        }
        state->phase = AX_AVOID2_MOVE;
        state->phase_start_ms = input->now_ms;
        state->segment_distance_mm = 0;
        state->segment_distance_fraction = 0;
        state->stall_check_ms = input->now_ms;
        state->stall_check_distance_mm = 0;
    }
    return StopOutput(state);
}

static AX_AVOID2_Output StepMove(AX_AVOID2_State *state,
                                 const AX_AVOID2_Config *config,
                                 const AX_AVOID2_Input *input,
                                 uint32_t dt_ms)
{
    AX_AVOID2_Output output;
    uint32_t speed;
    uint64_t millimm;
    uint64_t increment;
    if (!RangeReadyAt(config, input, 0)) {
        EnterWaitRescan(state, input, AX_AVOID2_REASON_FRONT_INVALID);
        return StopOutput(state);
    }
    if (input->range_distance_mm < config->emergency_distance_mm) {
        EnterWaitRescan(state, input, AX_AVOID2_REASON_FRONT_EMERGENCY);
        return StopOutput(state);
    }
    if (input->range_distance_mm <= config->advance_clearance_mm) {
        BeginCoarseScan(state, input);
        return StopOutput(state);
    }
    speed = input->forward_mm_s > 0 ? (uint16_t)input->forward_mm_s : 0u;
    millimm = (uint64_t)speed * dt_ms +
              state->segment_distance_fraction;
    increment = millimm / 1000u;
    state->segment_distance_fraction = (uint16_t)(millimm % 1000u);
    if (increment > UINT32_MAX - state->segment_distance_mm)
        state->segment_distance_mm = UINT32_MAX;
    else state->segment_distance_mm += (uint32_t)increment;
    if (state->segment_distance_mm - state->stall_check_distance_mm >=
        config->stall_min_mm) {
        state->stall_check_ms = input->now_ms;
        state->stall_check_distance_mm = state->segment_distance_mm;
    } else if ((uint32_t)(input->now_ms - state->stall_check_ms) >=
               config->stall_timeout_ms) {
        EnterWaitRescan(state, input, AX_AVOID2_REASON_MOVE_STALL);
        return StopOutput(state);
    }
    if (state->segment_distance_mm >= state->segment_target_mm) {
        BeginCoarseScan(state, input);
        return StopOutput(state);
    }
    output = StopOutput(state);
    output.forward_mm_s = config->advance_max_mm_s;
    return output;
}

void AX_AVOID2_Reset(AX_AVOID2_State *state)
{
    if (state == 0) return;
    memset(state, 0, sizeof(*state));
    state->phase = AX_AVOID2_WAIT_TARGET;
    state->initialized = 1;
}

AX_AVOID2_Output AX_AVOID2_Step(AX_AVOID2_State *state,
                                const AX_AVOID2_Config *config,
                                const AX_AVOID2_Input *input)
{
    uint32_t dt_ms = 0;
    if (state == 0 || input == 0 || config == 0) return StopOutput(state);
    if (!ConfigValid(config)) {
        AX_AVOID2_Reset(state);
        state->reason = AX_AVOID2_REASON_BAD_CONFIG;
        return StopOutput(state);
    }
    if (!state->initialized) AX_AVOID2_Reset(state);
    if (state->have_tick) dt_ms = input->now_ms - state->last_tick_ms;
    state->last_tick_ms = input->now_ms;
    state->have_tick = 1;
    if (!input->aoa_valid) {
        EnterWaitTarget(state, input, AX_AVOID2_REASON_AOA_INVALID);
        return StopOutput(state);
    }
    if (input->aoa_distance_cm >= config->approach_start_cm)
        state->approach_needed = 1;
    else if (input->aoa_distance_cm <= config->approach_stop_cm)
        state->approach_needed = 0;
    if (!state->approach_needed) {
        if (state->phase != AX_AVOID2_WAIT_TARGET)
            EnterWaitTarget(state, input, AX_AVOID2_REASON_TARGET_NEAR);
        else state->reason = AX_AVOID2_REASON_TARGET_NEAR;
        return StopOutput(state);
    }
    if (state->phase == AX_AVOID2_WAIT_TARGET) {
        BeginCoarseScan(state, input);
        return StopOutput(state);
    }
    switch (state->phase) {
    case AX_AVOID2_COARSE_SCAN:
        return StepCoarse(state, config, input);
    case AX_AVOID2_FINE_SCAN:
        return StepFine(state, config, input);
    case AX_AVOID2_TURN:
        return StepTurn(state, config, input, dt_ms);
    case AX_AVOID2_VERIFY_FRONT:
        return StepVerifyFront(state, config, input);
    case AX_AVOID2_MOVE:
        return StepMove(state, config, input, dt_ms);
    case AX_AVOID2_WAIT_RESCAN:
        if ((uint32_t)(input->now_ms - state->phase_start_ms) >=
            config->rescan_wait_ms)
            BeginCoarseScan(state, input);
        return StopOutput(state);
    case AX_AVOID2_WAIT_TARGET:
        return StopOutput(state);
    default:
        EnterWaitRescan(state, input, AX_AVOID2_REASON_INVALID_STATE);
        return StopOutput(state);
    }
}
