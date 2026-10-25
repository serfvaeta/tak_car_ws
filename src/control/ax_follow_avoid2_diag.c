#include "ax_follow_avoid2_diag.h"

#include <limits.h>

static void Put16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)(value >> 8);
    out[1] = (uint8_t)value;
}

static int16_t LimitSigned16(int32_t value)
{
    if (value > INT16_MAX) return INT16_MAX;
    if (value < INT16_MIN) return INT16_MIN;
    return (int16_t)value;
}

static uint16_t LimitUnsigned16(uint32_t value)
{
    return value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
}

void AX_AVOID2_DiagEncode(const AX_AVOID2_State *state,
                          const AX_AVOID2_Input *input,
                          const AX_AVOID2_Output *output,
                          uint8_t payload[AX_AVOID2_DIAG_LENGTH])
{
    int32_t progress = 0;
    uint32_t target = 0;
    if (state->phase == AX_AVOID2_TURN) {
        progress = state->turn_progress_mrad;
        target = (uint32_t)state->turn_target_mrad;
    } else if (state->phase == AX_AVOID2_MOVE) {
        progress = state->segment_distance_mm > INT16_MAX ?
                   INT16_MAX : (int32_t)state->segment_distance_mm;
        target = state->segment_target_mm;
    }
    payload[0] = (uint8_t)state->phase;
    payload[1] = (uint8_t)state->reason;
    payload[2] = (input->aoa_valid ? 1u : 0u) |
                 (input->range_valid ? 2u : 0u);
    payload[3] = (uint8_t)state->pan_angle_deg;
    Put16(payload + 4, (uint16_t)input->aoa_azimuth_deg);
    Put16(payload + 6, input->aoa_distance_cm);
    payload[8] = (uint8_t)input->range_angle_deg;
    payload[9] = input->range_valid ? (uint8_t)input->range_status : 0u;
    Put16(payload + 10, input->range_distance_mm);
    Put16(payload + 12, (uint16_t)output->forward_mm_s);
    Put16(payload + 14, (uint16_t)input->forward_mm_s);
    Put16(payload + 16, (uint16_t)output->yaw_mrad_s);
    Put16(payload + 18, (uint16_t)input->yaw_rate_mrad_s);
    Put16(payload + 20,
          LimitUnsigned16(input->now_ms - state->phase_start_ms));
    Put16(payload + 22, (uint16_t)LimitSigned16(progress));
    Put16(payload + 24, LimitUnsigned16(target));
    payload[26] = state->scan_index;
    payload[27] = state->candidate_index;
    payload[28] = state->candidate_count;
    payload[29] = state->fine_index;
}
