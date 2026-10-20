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
           config->advance_segment_mm > 0u && config->max_distance_mm > 0u &&
           config->max_time_ms > 0u && config->stall_timeout_ms > 0u &&
           config->rejoin_clear_samples > 0u &&
           config->rescan_wait_ms > 0u &&
           config->side_scan_timeout_ms > 0u &&
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

static void EnterWaitFront(AX_AVOID_State *state)
{
    state->phase = AX_AVOID_WAIT_FRONT;
    state->pan_angle_deg = 0;
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
    state->turn_settle_pending = 0;
    state->detour_angle_deg = 0;
    state->detour_side = 0;
    state->scan_index = 0;
    state->detour_start_ms = 0;
    state->segment_distance_mm = 0;
    state->total_distance_mm = 0;
    state->turn_progress_mrad = 0;
    state->turn_target_mrad = 0;
    state->turn_settle_ms = 0;
    state->turn_settle_sequence = 0;
    state->rejoin_progress_mrad = 0;
    state->rejoin_clear_count = 0;
    state->last_rejoin_clear_sequence = 0;
    state->scan_valid_mask = 0;
    memset(state->scan_distance_mm, 0, sizeof(state->scan_distance_mm));
}

static void EnterScan(AX_AVOID_State *state, const AX_AVOID_Config *config,
                      const AX_AVOID_Input *input)
{
    if (!state->detour_active) {
        state->detour_active = 1;
        state->detour_start_ms = input->now_ms;
        state->total_distance_mm = 0;
    }
    state->phase = AX_AVOID_SCAN_SIDE;
    state->phase_start_ms = input->now_ms;
    state->scan_index = 0;
    state->scan_valid_mask = 0;
    memset(state->scan_distance_mm, 0, sizeof(state->scan_distance_mm));
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

static void AddAbsoluteYaw(uint32_t *progress, int16_t yaw_rate_mrad_s,
                           uint32_t dt_ms)
{
    int32_t rate = yaw_rate_mrad_s;
    uint64_t delta;
    if (rate < 0) rate = -rate;
    delta = (uint64_t)(uint32_t)rate * dt_ms / 1000u;
    if (delta > UINT32_MAX - *progress) *progress = UINT32_MAX;
    else *progress += (uint32_t)delta;
}

static void SelectDirection(AX_AVOID_State *state,
                            const AX_AVOID_Config *config,
                            const AX_AVOID_Input *input)
{
    uint8_t count = (uint8_t)(2 * config->scan_max_angle_deg /
                              config->scan_step_deg + 1);
    uint8_t found = 0;
    uint16_t best_distance = 0;
    int16_t best_angle = 0;
    for (uint8_t i = 0; i < count; ++i) {
        int16_t angle;
        int16_t absolute_angle;
        int16_t absolute_best;
        if ((state->scan_valid_mask & (uint16_t)(1u << i)) == 0u) continue;
        angle = (int16_t)(config->scan_max_angle_deg -
                          i * config->scan_step_deg);
        absolute_angle = angle < 0 ? (int16_t)-angle : angle;
        absolute_best = best_angle < 0 ? (int16_t)-best_angle : best_angle;
        if (!found || state->scan_distance_mm[i] < best_distance ||
            (state->scan_distance_mm[i] == best_distance &&
             absolute_angle < absolute_best)) {
            found = 1;
            best_distance = state->scan_distance_mm[i];
            best_angle = angle;
        }
    }
    if (!found) {
        EnterRescanWait(state, input);
        return;
    }
    if (best_angle == 0) {
        ClearDetour(state);
        EnterWaitFront(state);
        return;
    }
    state->detour_angle_deg = best_angle;
    state->detour_side = best_angle < 0 ? -1 : 1;
    state->turn_target_mrad =
        ((uint32_t)(best_angle < 0 ? -best_angle : best_angle) * 3142u + 90u)
        / 180u;
    state->phase = AX_AVOID_TURN_OUT;
    state->phase_start_ms = input->now_ms;
    state->turn_progress_mrad = 0;
    state->turn_front_ready_seen = 0;
    state->turn_settle_pending = 0;
    state->turn_settle_ms = 0;
    state->turn_settle_sequence = 0;
    state->pan_angle_deg = 0;
}

void AX_AVOID_Reset(AX_AVOID_State *state)
{
    if (state == 0) return;
    memset(state, 0, sizeof(*state));
    state->phase = AX_AVOID_WAIT_FRONT;
    state->initialized = 1;
}

AX_AVOID_Output AX_AVOID_Step(AX_AVOID_State *state,
                              const AX_AVOID_Config *config,
                              const AX_AVOID_Input *input)
{
    AX_AVOID_Output output;
    uint32_t dt_ms = 0;
    uint32_t now_ms;

    if (state == 0 || input == 0) return StopOutput(0);
    if (!state->initialized) AX_AVOID_Reset(state);
    output = StopOutput(state);
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
        (state->phase == AX_AVOID_WAIT_FRONT ||
         state->phase == AX_AVOID_SCAN_SIDE ||
         state->phase == AX_AVOID_RESCAN_WAIT))
        state->detour_start_ms += dt_ms;

    if (state->phase == AX_AVOID_HOLD) return output;
    if (!input->aoa_valid && state->phase != AX_AVOID_WAIT_FRONT) {
        EnterWaitFront(state);
        return StopOutput(state);
    }
    if (state->detour_active &&
        (uint32_t)(now_ms - state->detour_start_ms) >= config->max_time_ms) {
        EnterHold(state, AX_AVOID_DIAG_TIME_LIMIT);
        return StopOutput(state);
    }
    if ((state->phase == AX_AVOID_TURN_OUT ||
         state->phase == AX_AVOID_ADVANCE ||
         state->phase == AX_AVOID_TRY_REJOIN) && dt_ms > 100u) {
        EnterHold(state, AX_AVOID_DIAG_LOOP_LATE);
        return StopOutput(state);
    }

    switch (state->phase) {
    case AX_AVOID_WAIT_FRONT:
        state->pan_angle_deg = 0;
        if (!input->aoa_valid) return StopOutput(state);
        if (RangeReadyAt(input, 0)) {
            if (input->range_distance_mm >= config->front_release_mm) {
                state->phase = AX_AVOID_TRACK;
                ClearDetour(state);
            } else {
                EnterScan(state, config, input);
            }
        }
        return StopOutput(state);

    case AX_AVOID_TRACK:
        state->pan_angle_deg = 0;
        if (!RangeReadyAt(input, 0)) {
            EnterWaitFront(state);
            return StopOutput(state);
        }
        if (input->range_distance_mm < config->front_stop_mm) {
            EnterScan(state, config, input);
            return StopOutput(state);
        }
        output.forward_mm_s = input->follow_target.forward_mm_s;
        if (output.forward_mm_s < 0) output.forward_mm_s = 0;
        if (output.forward_mm_s > config->follow_max_mm_s)
            output.forward_mm_s = config->follow_max_mm_s;
        output.yaw_mrad_s = ClampSpeed(input->follow_target.yaw_mrad_s,
                                       config->yaw_max_mrad_s);
        return output;

    case AX_AVOID_SCAN_SIDE:
        if ((uint32_t)(now_ms - state->phase_start_ms) >=
            config->side_scan_timeout_ms) {
            EnterRescanWait(state, input);
            return StopOutput(state);
        }
        if (input->range_valid && input->range_sequence !=
                                  state->scan_wait_sequence) {
            int16_t expected_angle = (int16_t)(config->scan_max_angle_deg -
                                state->scan_index * config->scan_step_deg);
            if (input->range_angle_deg == expected_angle) {
                uint8_t count = (uint8_t)(2 * config->scan_max_angle_deg /
                                          config->scan_step_deg + 1);
                state->scan_distance_mm[state->scan_index] =
                    input->range_distance_mm;
                if (input->range_status == AX_XURSC_READY &&
                    input->range_distance_mm >= config->side_min_mm)
                    state->scan_valid_mask |=
                        (uint16_t)(1u << state->scan_index);
                state->scan_wait_sequence = input->range_sequence;
                ++state->scan_index;
                if (state->scan_index < count)
                    state->pan_angle_deg = (int16_t)(expected_angle -
                                                     config->scan_step_deg);
                else
                    SelectDirection(state, config, input);
            }
        }
        return StopOutput(state);

    case AX_AVOID_RESCAN_WAIT:
        state->pan_angle_deg = 0;
        if ((uint32_t)(now_ms - state->phase_start_ms) >=
            config->rescan_wait_ms)
            EnterWaitFront(state);
        return StopOutput(state);

    case AX_AVOID_TURN_OUT: {
        /* Wait for the first centered echo while the servo returns from its
         * side scan. Once turning starts, losing that echo stops the turn. */
        if ((uint32_t)(now_ms - state->phase_start_ms) >=
            config->turn_timeout_ms) {
            EnterHold(state, AX_AVOID_DIAG_TURN_TIMEOUT);
            return StopOutput(state);
        }
        if (!RangeReadyAt(input, 0)) {
            if (state->turn_front_ready_seen) EnterWaitFront(state);
            return StopOutput(state);
        }
        state->turn_front_ready_seen = 1;
        int8_t direction = (int8_t)(state->detour_side *
                                    config->azimuth_to_yaw_sign);
        AddTurnProgress(&state->turn_progress_mrad,
                        input->yaw_rate_mrad_s, dt_ms, direction);
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
                !TickAfter(input->range_triggered_ms,
                           state->turn_settle_ms))
                return StopOutput(state);
            if (input->range_distance_mm >= config->front_release_mm) {
                state->phase = AX_AVOID_ADVANCE;
                state->phase_start_ms = now_ms;
                state->segment_distance_mm = 0;
                output = StopOutput(state);
                output.forward_mm_s = config->advance_max_mm_s;
                return output;
            }
            EnterScan(state, config, input);
            return StopOutput(state);
        }
        state->turn_settle_pending = 0;
        output.yaw_mrad_s = (int16_t)(state->detour_side *
                                      config->azimuth_to_yaw_sign *
                                      config->yaw_max_mrad_s);
        return output;
    }

    case AX_AVOID_ADVANCE:
        if (!RangeReadyAt(input, 0)) {
            EnterWaitFront(state);
            return StopOutput(state);
        }
        if (input->range_distance_mm < config->front_stop_mm) {
            EnterScan(state, config, input);
            return StopOutput(state);
        }
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
        if (state->total_distance_mm >= config->max_distance_mm) {
            EnterHold(state, AX_AVOID_DIAG_DISTANCE_LIMIT);
            return StopOutput(state);
        }
        if ((uint32_t)(now_ms - state->phase_start_ms) >=
                config->stall_timeout_ms &&
            state->segment_distance_mm < config->stall_min_mm) {
            EnterHold(state, AX_AVOID_DIAG_ADVANCE_STALL);
            return StopOutput(state);
        }
        if (state->segment_distance_mm >= config->advance_segment_mm) {
            state->phase = AX_AVOID_TRY_REJOIN;
            state->phase_start_ms = now_ms;
            state->rejoin_progress_mrad = 0;
            state->rejoin_clear_count = 0;
            state->last_rejoin_clear_sequence = input->range_sequence;
            return StopOutput(state);
        }
        output.forward_mm_s = config->advance_max_mm_s;
        return output;

    case AX_AVOID_TRY_REJOIN: {
        int32_t azimuth = input->aoa_azimuth_deg;
        uint32_t absolute_azimuth;
        int8_t direction;
        if (!RangeReadyAt(input, 0)) {
            EnterWaitFront(state);
            return StopOutput(state);
        }
        if (input->range_distance_mm < config->front_release_mm) {
            EnterScan(state, config, input);
            return StopOutput(state);
        }
        absolute_azimuth = (uint32_t)(azimuth < 0 ? -azimuth : azimuth);
        if (absolute_azimuth > config->rejoin_stop_turn_deg) {
            if ((uint32_t)(now_ms - state->phase_start_ms) >=
                    config->turn_timeout_ms) {
                EnterHold(state, AX_AVOID_DIAG_REJOIN_TIMEOUT);
                return StopOutput(state);
            }
            AddAbsoluteYaw(&state->rejoin_progress_mrad,
                           input->yaw_rate_mrad_s, dt_ms);
            if (state->rejoin_progress_mrad >= 1570u) {
                EnterHold(state, AX_AVOID_DIAG_REJOIN_TURN_LIMIT);
                return StopOutput(state);
            }
            direction = (int8_t)((azimuth > 0 ? 1 : -1) *
                                 config->azimuth_to_yaw_sign);
            output.yaw_mrad_s = (int16_t)(direction *
                                          config->yaw_max_mrad_s);
            return output;
        }
        if (input->range_distance_mm >= config->front_release_mm &&
            input->range_sequence != state->last_rejoin_clear_sequence) {
            state->last_rejoin_clear_sequence = input->range_sequence;
            if (state->rejoin_clear_count < UINT8_MAX)
                ++state->rejoin_clear_count;
        }
        if (state->rejoin_clear_count >= config->rejoin_clear_samples) {
            state->phase = AX_AVOID_TRACK;
            ClearDetour(state);
        }
        return StopOutput(state);
    }

    case AX_AVOID_HOLD:
    default:
        EnterHold(state, AX_AVOID_DIAG_HOLD_UNKNOWN);
        return StopOutput(state);
    }
}
