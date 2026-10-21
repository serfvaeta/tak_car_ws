#include "ax_follow_avoid.h"

#include "ax_aoa.h"
#include "ax_follow_policy.h"
#include "ax_follow_avoid_policy.h"
#include "ax_follow_avoid_diag.h"
#include "ax_range_pan.h"
#include "ax_robot.h"
#include "ax_uart3.h"
#include "openctr_avoid_config.h"
#include "openctr_follow_config.h"

#if OPENCTR_AVOID_FOLLOW_MAX_MM_S > R_VX_LIMIT
#error "AVOID follow speed exceeds the chassis X speed limit"
#endif

#if OPENCTR_AVOID_FORWARD_MAX_MM_S > R_VX_LIMIT
#error "AVOID forward speed exceeds the chassis X speed limit"
#endif

#if OPENCTR_AVOID_YAW_MAX_MRAD_S > R_VW_LIMIT
#error "AVOID yaw speed exceeds the chassis yaw speed limit"
#endif

static AX_AVOID_State avoid_state;
static AX_FOLLOW_State follow_state;
static uint8_t diag_active;
static AX_AVOID_Phase last_diag_phase;
static AX_AVOID_DiagReason last_diag_reason;
static uint32_t last_diag_ms;

static const AX_FOLLOW_Config follow_config = {
    OPENCTR_FOLLOW_START_REVERSE_CM,
    OPENCTR_FOLLOW_START_FORWARD_CM,
    OPENCTR_FOLLOW_STOP_FORWARD_CM,
    OPENCTR_FOLLOW_MAX_DISTANCE_CM,
    OPENCTR_FOLLOW_START_TURN_DEG,
    OPENCTR_FOLLOW_STOP_TURN_DEG,
    OPENCTR_AVOID_FOLLOW_MAX_MM_S,
    OPENCTR_FOLLOW_TURN_MAX_MRAD_S,
    OPENCTR_FOLLOW_AZIMUTH_TO_YAW_SIGN,
    OPENCTR_FOLLOW_FORWARD_MIN_MM_S,
    OPENCTR_FOLLOW_TURN_MIN_MRAD_S,
    OPENCTR_FOLLOW_FORWARD_KP_MM_S_PER_CM,
    OPENCTR_FOLLOW_TURN_KP_MRAD_S_PER_DEG,
    OPENCTR_FOLLOW_MIN_VALID_DISTANCE_CM,
    OPENCTR_FOLLOW_STOP_REVERSE_CM,
    OPENCTR_FOLLOW_REVERSE_MIN_MM_S,
    OPENCTR_FOLLOW_REVERSE_MAX_MM_S,
    OPENCTR_FOLLOW_REVERSE_KP_MM_S_PER_CM
};

static const AX_AVOID_Config avoid_config = {
    .front_stop_mm = OPENCTR_AVOID_FRONT_STOP_MM,
    .front_release_mm = OPENCTR_AVOID_FRONT_RELEASE_MM,
    .side_min_mm = OPENCTR_AVOID_SIDE_MIN_MM,
    .scan_max_angle_deg = OPENCTR_AVOID_SCAN_MAX_ANGLE_DEG,
    .scan_step_deg = OPENCTR_AVOID_SCAN_STEP_DEG,
    .track_probe_angle_deg = OPENCTR_AVOID_TRACK_PROBE_ANGLE_DEG,
    .range_max_age_ms = OPENCTR_AVOID_RANGE_MAX_AGE_MS,
    .aoa_max_age_ms = OPENCTR_AVOID_AOA_MAX_AGE_MS,
    .approach_start_cm = OPENCTR_FOLLOW_START_FORWARD_CM,
    .approach_stop_cm = OPENCTR_FOLLOW_STOP_FORWARD_CM,
    .follow_max_mm_s = OPENCTR_AVOID_FOLLOW_MAX_MM_S,
    .advance_max_mm_s = OPENCTR_AVOID_FORWARD_MAX_MM_S,
    .yaw_max_mrad_s = OPENCTR_AVOID_YAW_MAX_MRAD_S,
    .turn_timeout_ms = OPENCTR_AVOID_TURN_TIMEOUT_MS,
    .turn_settle_max_yaw_mrad_s =
        OPENCTR_AVOID_TURN_SETTLE_MAX_YAW_MRAD_S,
    .turn_overshoot_mrad = OPENCTR_AVOID_TURN_OVERSHOOT_MRAD,
    .advance_segment_mm = OPENCTR_AVOID_ADVANCE_SEGMENT_MM,
    .max_distance_mm = OPENCTR_AVOID_MAX_DISTANCE_MM,
    .max_time_ms = OPENCTR_AVOID_MAX_TIME_MS,
    .stall_timeout_ms = OPENCTR_AVOID_STALL_TIMEOUT_MS,
    .stall_min_mm = OPENCTR_AVOID_STALL_MIN_MM,
    .rejoin_clear_samples = OPENCTR_AVOID_REJOIN_CLEAR_SAMPLES,
    .rejoin_stop_turn_deg = OPENCTR_FOLLOW_START_TURN_DEG,
    .azimuth_to_yaw_sign = OPENCTR_AVOID_AZIMUTH_TO_YAW_SIGN,
    .rescan_wait_ms = OPENCTR_AVOID_RESCAN_WAIT_MS,
    .side_scan_timeout_ms = OPENCTR_AVOID_SIDE_SCAN_TIMEOUT_MS
};

void AX_FOLLOW_AVOID_Reset(void)
{
    AX_AVOID_Reset(&avoid_state);
    AX_FOLLOW_Reset(&follow_state);
    diag_active = 0;
}

static void SendAvoidDiag(const AX_AVOID_Input *input,
                          const AX_AVOID_Output *output,
                          uint8_t chassis_fwd,
                          uint16_t aoa_distance_cm)
{
    uint8_t payload[AX_AVOID_DIAG_LENGTH];
    AX_AVOID_DiagReason reason = AX_AVOID_DiagEncode(
        &avoid_state, &avoid_config, input, output, chassis_fwd,
        aoa_distance_cm, payload);
    if (!diag_active || avoid_state.phase != last_diag_phase ||
        reason != last_diag_reason ||
        (uint32_t)(input->now_ms - last_diag_ms) >= 200u) {
        AX_UART3_SendPacket(payload, AX_AVOID_DIAG_LENGTH, ID_UTX_AVOID_DIAG);
        diag_active = 1;
        last_diag_phase = avoid_state.phase;
        last_diag_reason = reason;
        last_diag_ms = input->now_ms;
    }
}

void AX_FOLLOW_AVOID_Step(void)
{
    AX_AOA_Sample aoa;
    AX_RANGE_PAN_Sample range;
    AX_FOLLOW_Observation observation;
    AX_AVOID_Input input = {0};
    AX_AVOID_Output output;
    uint8_t aoa_valid = 0;
    uint16_t aoa_distance_cm = 0;

    input.now_ms = (uint32_t)xTaskGetTickCount();

    if (ax_robot_type != ROBOT_FWD) {
        AX_AVOID_Reset(&avoid_state);
        AX_FOLLOW_Reset(&follow_state);
        AX_RANGE_PAN_AvoidRequestAngle(0);
        AX_ROBOT_SetTargetIfActive(CTL_PS2, 0, 0, 0);
        output = (AX_AVOID_Output){0, 0, 0};
        SendAvoidDiag(&input, &output, 0, 0);
        return;
    }

    if (AX_AOA_GetLatest(&aoa, OPENCTR_AVOID_AOA_MAX_AGE_MS) &&
        (OPENCTR_AVOID_AZIMUTH_TO_YAW_SIGN == 1 ||
         OPENCTR_AVOID_AZIMUTH_TO_YAW_SIGN == -1) &&
        aoa.measurement.distance_cm > OPENCTR_FOLLOW_MIN_VALID_DISTANCE_CM &&
        aoa.measurement.distance_cm <= OPENCTR_FOLLOW_MAX_DISTANCE_CM) {
        observation.distance_cm = aoa.measurement.distance_cm;
        aoa_distance_cm = observation.distance_cm;
        input.aoa_distance_cm = observation.distance_cm;
        observation.azimuth_deg = aoa.measurement.azimuth_deg;
        input.follow_target = AX_FOLLOW_Compute(&follow_state,
                                                &follow_config,
                                                &observation);
        input.aoa_azimuth_deg = observation.azimuth_deg;
        aoa_valid = 1;
    } else {
        AX_FOLLOW_Reset(&follow_state);
    }
    input.aoa_valid = aoa_valid;

    if (AX_RANGE_PAN_GetAvoidSample(OPENCTR_AVOID_RANGE_MAX_AGE_MS, &range)) {
        input.range_valid = 1;
        input.range_angle_deg = range.angle_deg;
        input.range_status = range.status;
        input.range_distance_mm = range.distance_mm;
        input.range_sequence = range.sequence;
        input.range_triggered_ms = range.triggered_tick_ms;
    }

    input.yaw_rate_mrad_s = (int16_t)(((int32_t)R_Imu.GYRO_Z * 8727) / 32768);
    input.forward_mm_s = R_Vel.RT_IX;
    output = AX_AVOID_Step(&avoid_state, &avoid_config, &input);
    if (avoid_state.phase == AX_AVOID_TRACK)
        AX_RANGE_PAN_AvoidRequestProbeAngle(output.pan_angle_deg);
    else
        AX_RANGE_PAN_AvoidRequestAngle(output.pan_angle_deg);
    AX_ROBOT_SetTargetIfActive(CTL_PS2, output.forward_mm_s, 0,
                               output.yaw_mrad_s);
    SendAvoidDiag(&input, &output, 1, aoa_distance_cm);
}
