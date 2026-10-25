#include <assert.h>
#include <stdint.h>

#include "ax_follow_avoid2_diag.h"

int main(void)
{
    AX_AVOID2_State state = {0};
    AX_AVOID2_Input input = {0};
    AX_AVOID2_Output output = {0};
    uint8_t payload[AX_AVOID2_DIAG_LENGTH];

    state.phase = AX_AVOID2_WAIT_RESCAN;
    state.reason = AX_AVOID2_REASON_COARSE_TIMEOUT;
    state.scan_index = 29;
    state.candidate_index = 0;
    state.pan_angle_deg = 0;
    state.phase_start_ms = 1000;
    input.now_ms = 1200;
    input.aoa_valid = 1;
    input.aoa_distance_cm = 130;
    input.aoa_azimuth_deg = -20;
    input.range_valid = 1;
    input.range_angle_deg = 84;
    input.range_status = AX_XURSC_READY;
    input.range_distance_mm = 900;
    AX_AVOID2_DiagEncode(&state, &input, &output, payload);
    assert(payload[0] == AX_AVOID2_WAIT_RESCAN);
    assert(payload[1] == AX_AVOID2_REASON_COARSE_TIMEOUT);
    assert(payload[2] == 3u);
    assert(payload[4] == 0xff && payload[5] == 0xec);
    assert(payload[8] == 84 && payload[9] == AX_XURSC_READY);
    assert(payload[10] == 3 && payload[11] == 132);
    assert(payload[20] == 0 && payload[21] == 200);
    assert(payload[26] == 29);

    state.phase = AX_AVOID2_TURN;
    state.reason = AX_AVOID2_REASON_NONE;
    state.turn_progress_mrad = -40;
    state.turn_target_mrad = 300;
    output.yaw_mrad_s = -500;
    input.yaw_rate_mrad_s = 100;
    AX_AVOID2_DiagEncode(&state, &input, &output, payload);
    assert(payload[16] == 0xfe && payload[17] == 0x0c);
    assert(payload[18] == 0 && payload[19] == 100);
    assert(payload[22] == 0xff && payload[23] == 0xd8);
    assert(payload[24] == 1 && payload[25] == 44);

    state.phase = AX_AVOID2_MOVE;
    state.segment_distance_mm = UINT32_MAX;
    state.segment_target_mm = 1600;
    AX_AVOID2_DiagEncode(&state, &input, &output, payload);
    assert(payload[22] == 0x7f && payload[23] == 0xff);
    return 0;
}
