#include "ax_follow_avoid_diag.h"

static void Put16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)value;
}

static uint16_t Limit16(uint32_t value)
{
    return value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}

static AX_AVOID_DiagReason Explain(const AX_AVOID_State *state,
                                    const AX_AVOID_Input *input,
                                    const AX_AVOID_Output *output,
                                    uint8_t chassis_fwd)
{
    if (!chassis_fwd) return AX_AVOID_DIAG_CHASSIS;
    if (state->phase == AX_AVOID_HOLD)
        return state->hold_reason == AX_AVOID_DIAG_NONE ?
               AX_AVOID_DIAG_HOLD_UNKNOWN : state->hold_reason;
    if (!input->aoa_valid) return AX_AVOID_DIAG_AOA_STALE;
    if (state->phase == AX_AVOID_ALIGN_TARGET &&
        state->hold_reason != AX_AVOID_DIAG_NONE &&
        output->forward_mm_s == 0 && output->yaw_mrad_s == 0)
        return state->hold_reason;
    if (state->phase == AX_AVOID_FORWARD && state->track_wait_fresh_front)
        return AX_AVOID_DIAG_FRONT_STALE;
    if (state->phase == AX_AVOID_RESCAN_WAIT)
        return AX_AVOID_DIAG_WAIT_RESCAN;
    if (state->phase == AX_AVOID_SCAN_SIDE)
        return state->pan_angle_deg < 0 ? AX_AVOID_DIAG_WAIT_LEFT :
                                          AX_AVOID_DIAG_WAIT_RIGHT;
    if (output->forward_mm_s != 0 || output->yaw_mrad_s != 0)
        return AX_AVOID_DIAG_NONE;
    if (state->phase == AX_AVOID_TURN_OUT ||
        state->phase == AX_AVOID_FORWARD ||
        state->phase == AX_AVOID_ADVANCE) {
        if (!input->range_valid || input->range_angle_deg != 0 ||
            input->range_status != AX_XURSC_READY)
            return AX_AVOID_DIAG_FRONT_STALE;
    }
    if (state->phase == AX_AVOID_TURN_OUT)
        return AX_AVOID_DIAG_TURN_CENTER;
    if (state->phase == AX_AVOID_ALIGN_TARGET)
        return AX_AVOID_DIAG_FOLLOW_STOP;
    if (state->phase == AX_AVOID_FORWARD)
        return AX_AVOID_DIAG_FOLLOW_STOP;
    return AX_AVOID_DIAG_NONE;
}

AX_AVOID_DiagReason AX_AVOID_DiagEncode(const AX_AVOID_State *state,
                                         const AX_AVOID_Input *input,
                                         const AX_AVOID_Output *output,
                                         uint8_t chassis_fwd,
                                         uint16_t aoa_distance_cm,
                                         uint8_t payload[AX_AVOID_DIAG_LENGTH])
{
    AX_AVOID_DiagReason reason = Explain(state, input, output,
                                         chassis_fwd);
    uint8_t flags = 0;
    uint32_t progress = 0;
    uint16_t elapsed = 0;
    if (input->aoa_valid) flags |= 1u;
    if (input->range_valid) flags |= 2u;
    if (chassis_fwd) flags |= 4u;
    if (input->range_valid && input->range_status == AX_XURSC_READY)
        flags |= 8u;
    if (output->forward_mm_s != 0 || output->yaw_mrad_s != 0) flags |= 16u;
    if (state->detour_active) {
        elapsed = Limit16(input->now_ms - state->phase_start_ms);
        if (state->phase == AX_AVOID_TURN_OUT)
            progress = state->turn_progress_mrad > 0 ?
                       (uint32_t)state->turn_progress_mrad : 0u;
        else if (state->phase == AX_AVOID_ADVANCE)
            progress = state->segment_distance_mm;
    }
    payload[0] = (uint8_t)state->phase;
    payload[1] = (uint8_t)reason;
    payload[2] = flags;
    payload[3] = (uint8_t)state->pan_angle_deg;
    Put16(payload + 4, (uint16_t)input->aoa_azimuth_deg);
    Put16(payload + 6, aoa_distance_cm);
    payload[8] = (uint8_t)input->range_angle_deg;
    payload[9] = input->range_valid ? (uint8_t)input->range_status : 0u;
    Put16(payload + 10, input->range_distance_mm);
    Put16(payload + 12, (uint16_t)output->forward_mm_s);
    Put16(payload + 14, (uint16_t)input->forward_mm_s);
    Put16(payload + 16, (uint16_t)output->yaw_mrad_s);
    Put16(payload + 18, (uint16_t)input->yaw_rate_mrad_s);
    Put16(payload + 20, elapsed);
    Put16(payload + 22, Limit16(progress));
    Put16(payload + 24, Limit16(state->total_distance_mm));
    return reason;
}
