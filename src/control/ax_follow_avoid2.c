#include "ax_follow_avoid2.h"

#include "ax_aoa.h"
#include "ax_follow_avoid2_diag.h"
#include "ax_follow_avoid2_policy.h"
#include "ax_range_pan.h"
#include "ax_robot.h"
#include "ax_uart3.h"
#include "openctr_avoid2_config.h"
#include "openctr_follow_config.h"

#if OPENCTR_AVOID2_ADVANCE_MAX_MM_S > R_VX_LIMIT
#error "AVOID2 advance speed exceeds the chassis X speed limit"
#endif

#if OPENCTR_AVOID2_YAW_MAX_MRAD_S > R_VW_LIMIT
#error "AVOID2 yaw speed exceeds the chassis yaw speed limit"
#endif

static AX_AVOID2_State avoid2_state;
static uint8_t diag_active;
static AX_AVOID2_Phase last_diag_phase;
static AX_AVOID2_Reason last_diag_reason;
static uint32_t last_diag_ms;

static const AX_AVOID2_Config avoid2_config = {
    .safe_distance_mm = OPENCTR_AVOID2_SAFE_DISTANCE_MM,
    .emergency_distance_mm = OPENCTR_AVOID2_EMERGENCY_DISTANCE_MM,
    .scan_max_angle_deg = OPENCTR_AVOID2_SCAN_MAX_ANGLE_DEG,
    .coarse_step_deg = OPENCTR_AVOID2_COARSE_STEP_DEG,
    .fine_range_deg = OPENCTR_AVOID2_FINE_RANGE_DEG,
    .fine_step_deg = OPENCTR_AVOID2_FINE_STEP_DEG,
    .max_candidates = OPENCTR_AVOID2_MAX_CANDIDATES,
    .max_adjust_count = OPENCTR_AVOID2_MAX_ADJUST_COUNT,
    .range_max_age_ms = OPENCTR_AVOID2_RANGE_MAX_AGE_MS,
    .approach_start_cm = OPENCTR_AVOID2_APPROACH_START_CM,
    .approach_stop_cm = OPENCTR_AVOID2_APPROACH_STOP_CM,
    .advance_max_mm_s = OPENCTR_AVOID2_ADVANCE_MAX_MM_S,
    .yaw_max_mrad_s = OPENCTR_AVOID2_YAW_MAX_MRAD_S,
    .turn_timeout_ms = OPENCTR_AVOID2_TURN_TIMEOUT_MS,
    .turn_settle_max_yaw_mrad_s =
        OPENCTR_AVOID2_TURN_SETTLE_MAX_YAW_MRAD_S,
    .advance_range_cap_mm = OPENCTR_AVOID2_ADVANCE_RANGE_CAP_MM,
    .advance_clearance_mm = OPENCTR_AVOID2_ADVANCE_CLEARANCE_MM,
    .stall_timeout_ms = OPENCTR_AVOID2_STALL_TIMEOUT_MS,
    .stall_min_mm = OPENCTR_AVOID2_STALL_MIN_MM,
    .rescan_wait_ms = OPENCTR_AVOID2_RESCAN_WAIT_MS,
    .coarse_scan_timeout_ms = OPENCTR_AVOID2_COARSE_SCAN_TIMEOUT_MS,
    .fine_scan_timeout_ms = OPENCTR_AVOID2_FINE_SCAN_TIMEOUT_MS,
    .verify_timeout_ms = OPENCTR_AVOID2_VERIFY_TIMEOUT_MS,
    .aoa_to_yaw_sign = OPENCTR_AVOID2_AZIMUTH_TO_YAW_SIGN
};

void AX_FOLLOW_AVOID2_Reset(void)
{
    AX_AVOID2_Reset(&avoid2_state);
    diag_active = 0;
}

static void SendAvoid2Diag(const AX_AVOID2_Input *input,
                           const AX_AVOID2_Output *output)
{
    uint8_t payload[AX_AVOID2_DIAG_LENGTH];
    if (diag_active && avoid2_state.phase == last_diag_phase &&
        avoid2_state.reason == last_diag_reason &&
        (uint32_t)(input->now_ms - last_diag_ms) < 200u)
        return;
    AX_AVOID2_DiagEncode(&avoid2_state, input, output, payload);
    AX_UART3_SendPacket(payload, AX_AVOID2_DIAG_LENGTH,
                        ID_UTX_AVOID2_DIAG);
    diag_active = 1;
    last_diag_phase = avoid2_state.phase;
    last_diag_reason = avoid2_state.reason;
    last_diag_ms = input->now_ms;
}

void AX_FOLLOW_AVOID2_Step(void)
{
    AX_AOA_Sample aoa;
    AX_RANGE_PAN_Sample range;
    AX_AVOID2_Input input = {0};
    AX_AVOID2_Output output;

    input.now_ms = (uint32_t)xTaskGetTickCount();
    if (ax_robot_type != ROBOT_FWD) {
        AX_AVOID2_Reset(&avoid2_state);
        avoid2_state.reason = AX_AVOID2_REASON_CHASSIS;
        AX_RANGE_PAN_AvoidRequestAngle(0);
        AX_ROBOT_SetTargetIfActive(CTL_PS2, 0, 0, 0);
        output = (AX_AVOID2_Output){0, 0, 0};
        SendAvoid2Diag(&input, &output);
        return;
    }

    if (AX_AOA_GetLatest(&aoa, OPENCTR_AVOID2_AOA_MAX_AGE_MS) &&
        (OPENCTR_AVOID2_AZIMUTH_TO_YAW_SIGN == 1 ||
         OPENCTR_AVOID2_AZIMUTH_TO_YAW_SIGN == -1) &&
        aoa.measurement.distance_cm > OPENCTR_FOLLOW_MIN_VALID_DISTANCE_CM &&
        aoa.measurement.distance_cm <= OPENCTR_FOLLOW_MAX_DISTANCE_CM) {
        input.aoa_valid = 1;
        input.aoa_distance_cm = aoa.measurement.distance_cm;
        /* The current AOA installation reports positive-left; policy angles
         * use positive-right to match the pan-servo search coordinates. */
        input.aoa_azimuth_deg =
            (int16_t)-aoa.measurement.azimuth_deg;
    }

    if (AX_RANGE_PAN_GetAvoidSample(OPENCTR_AVOID2_RANGE_MAX_AGE_MS,
                                    &range)) {
        input.range_valid = 1;
        input.range_angle_deg = range.angle_deg;
        input.range_status = range.status;
        input.range_distance_mm = range.distance_mm;
        input.range_sequence = range.sequence;
        input.range_triggered_ms = range.triggered_tick_ms;
    }

    input.yaw_rate_mrad_s = (int16_t)(((int32_t)R_Imu.GYRO_Z * 8727) /
                                      32768);
    input.forward_mm_s = R_Vel.RT_IX;
    output = AX_AVOID2_Step(&avoid2_state, &avoid2_config, &input);
    AX_RANGE_PAN_AvoidRequestAngle(output.pan_angle_deg);
    AX_ROBOT_SetTargetIfActive(CTL_PS2, output.forward_mm_s, 0,
                               output.yaw_mrad_s);
    SendAvoid2Diag(&input, &output);
}
