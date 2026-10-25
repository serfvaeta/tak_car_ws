#include <assert.h>
#include <stdint.h>

#include "ax_aoa.h"
#include "ax_follow_avoid2.h"
#include "ax_follow_avoid2_diag.h"
#include "ax_range_pan.h"
#include "ax_robot.h"

static uint32_t now_ms;
static uint8_t aoa_available;
static uint8_t range_available;
static uint32_t requested_age_ms;
static AX_AOA_Sample mock_aoa;
static AX_RANGE_PAN_Sample mock_range;
static int16_t requested_pan;
static uint8_t target_source;
static int16_t target_forward;
static int16_t target_lateral;
static int16_t target_yaw;
static unsigned diag_count;
static uint8_t last_diag_phase;
static uint8_t last_diag_reason;

int ax_robot_type = ROBOT_FWD;
TestImu R_Imu = {0};
TestVelocity R_Vel = {0};

uint32_t xTaskGetTickCount(void)
{
    return now_ms;
}

uint8_t AX_AOA_GetLatest(AX_AOA_Sample *out, uint32_t max_age_ms)
{
    (void)max_age_ms;
    if (!aoa_available) return 0;
    *out = mock_aoa;
    return 1;
}

uint8_t AX_RANGE_PAN_GetAvoidSample(uint32_t max_age_ms,
                                   AX_RANGE_PAN_Sample *out)
{
    requested_age_ms = max_age_ms;
    if (!range_available) return 0;
    *out = mock_range;
    return 1;
}

void AX_RANGE_PAN_AvoidRequestAngle(int16_t degrees)
{
    requested_pan = degrees;
}

void AX_ROBOT_SetTargetIfActive(uint8_t source, int16_t forward_mm_s,
                                int16_t lateral_mm_s, int16_t yaw_mrad_s)
{
    target_source = source;
    target_forward = forward_mm_s;
    target_lateral = lateral_mm_s;
    target_yaw = yaw_mrad_s;
}

void AX_UART3_SendPacket(uint8_t *payload, uint8_t length, uint8_t id)
{
    assert(id == ID_UTX_AVOID2_DIAG);
    assert(length == AX_AVOID2_DIAG_LENGTH);
    last_diag_phase = payload[0];
    last_diag_reason = payload[1];
    ++diag_count;
}

static int16_t CoarseAngle(uint8_t index)
{
    if (index == 0) return 0;
    return (index & 1u) ?
        (int16_t)(((index + 1u) / 2u) * 6u) :
        (int16_t)(-((int16_t)(index / 2u) * 6));
}

static void FeedRange(int16_t angle, uint16_t distance_mm)
{
    now_ms += 100;
    mock_range.angle_deg = angle;
    mock_range.status = AX_XURSC_READY;
    mock_range.distance_mm = distance_mm;
    mock_range.triggered_tick_ms = now_ms;
    mock_range.received_tick_ms = now_ms;
    ++mock_range.sequence;
    range_available = 1;
    AX_FOLLOW_AVOID2_Step();
}

int main(void)
{
    AX_FOLLOW_AVOID2_Reset();
    now_ms = 1000;
    aoa_available = 1;
    mock_aoa.measurement.distance_cm = 130;
    mock_aoa.measurement.azimuth_deg = -20; /* Sensor right is negative. */
    range_available = 0;
    AX_FOLLOW_AVOID2_Step();
    assert(target_source == CTL_PS2);
    assert(target_forward == 0 && target_lateral == 0 && target_yaw == 0);
    assert(requested_pan == 0);
    assert(requested_age_ms == 250);
    assert(diag_count == 1 && last_diag_phase == AX_AVOID2_COARSE_SCAN);

    for (uint8_t i = 0; i < 31u; ++i) {
        int16_t angle = CoarseAngle(i);
        uint16_t distance = angle == 18 ? 700 :
                            angle == 24 ? 1000 :
                            angle == 12 ? 800 : 350;
        FeedRange(angle, distance);
        if (i + 1u < 31u) assert(requested_pan == CoarseAngle(i + 1u));
    }
    assert(requested_pan == 8); /* +18° center, then start at center -10°. */

    FeedRange(8, 900);
    FeedRange(13, 900);
    FeedRange(18, 900);
    FeedRange(23, 900);
    FeedRange(28, 900);
    assert(requested_pan == 0);
    assert(last_diag_phase == AX_AVOID2_TURN);
    assert(last_diag_reason == AX_AVOID2_REASON_NONE);

    R_Imu.GYRO_Z = -1878; /* Right turn is negative yaw in this installation. */
    uint8_t turn_command_seen = 0;
    for (uint8_t n = 0; n < 10u; ++n) {
        FeedRange(0, 900);
        if (target_yaw == 0) break;
        assert(target_yaw == -500);
        turn_command_seen = 1;
    }
    assert(turn_command_seen);
    R_Imu.GYRO_Z = 0;
    FeedRange(0, 900);
    assert(target_yaw == 0);
    FeedRange(0, 900);
    assert(target_forward == 0); /* fresh post-turn sample authorizes move */

    R_Vel.RT_IX = 100;
    uint8_t move_command_seen = 0;
    for (uint8_t n = 0; n < 12u; ++n) {
        FeedRange(0, 900);
        if (target_forward == 0) break;
        assert(target_forward == 100);
        move_command_seen = 1;
    }
    assert(move_command_seen);

    aoa_available = 0;
    now_ms += 20;
    AX_FOLLOW_AVOID2_Step();
    assert(target_forward == 0 && target_lateral == 0 && target_yaw == 0);
    assert(requested_pan == 0);

    ax_robot_type = 99;
    aoa_available = 1;
    now_ms += 20;
    AX_FOLLOW_AVOID2_Step();
    assert(target_forward == 0 && target_yaw == 0);
    return 0;
}
