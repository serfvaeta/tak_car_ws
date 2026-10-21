#include <assert.h>
#include <stdint.h>

#include "ax_aoa.h"
#include "ax_follow.h"
#include "ax_robot.h"

static AX_AOA_Sample sample;
static uint8_t fix_valid;
static int16_t target_forward;
static int16_t target_yaw;

uint8_t AX_AOA_GetLatest(AX_AOA_Sample *out, uint32_t max_age_ms)
{
    assert(max_age_ms == 300u);
    if (!fix_valid) return 0;
    *out = sample;
    return 1;
}

void AX_ROBOT_SetTargetIfActive(uint8_t source, int16_t x, int16_t y, int16_t w)
{
    assert(source == CTL_PS2);
    assert(y == 0);
    target_forward = x;
    target_yaw = w;
}

static void follow_at(uint32_t distance_cm, int16_t azimuth_deg,
                      int16_t elevation_deg)
{
    sample.measurement.distance_cm = distance_cm;
    sample.measurement.azimuth_deg = azimuth_deg;
    sample.measurement.elevation_deg = elevation_deg;
    fix_valid = 1;
    AX_FOLLOW_Step();
}

int main(void)
{
    AX_FOLLOW_ModeReset();
    fix_valid = 0;
    AX_FOLLOW_Step();
    assert(target_forward == 0 && target_yaw == 0);

    follow_at(130, 0, 0);
    assert(target_forward == 300 && target_yaw == 0);
    follow_at(130, 0, 60); /* use reported distance, without projection */
    assert(target_forward == 300 && target_yaw == 0);
    follow_at(130, 0, 76); /* elevation is ignored */
    assert(target_forward == 300 && target_yaw == 0);
    follow_at(200, 0, 0);
    assert(target_forward == 300 && target_yaw == 0);
    follow_at(105, 0, 0); /* slow down before the 100 cm stop line */
    assert(target_forward == 150 && target_yaw == 0);

    follow_at(130, 38, 0);
    assert(target_forward == 0 && target_yaw == 1900);
    follow_at(130, 6, 0); /* keep turning just outside the stop angle */
    assert(target_forward == 0 && target_yaw == 300);
    follow_at(130, 5, 0); /* resume distance control at 5 degrees */
    assert(target_forward == 300 && target_yaw == 0);

    follow_at(55, 38, 47);
    assert(target_forward == 0 && target_yaw == 1900);
    follow_at(55, 0, 47);
    assert(target_forward == 0 && target_yaw == 0); /* no rear sensor */
    follow_at(10, 0, 0);
    assert(target_forward == 0 && target_yaw == 0);
    follow_at(0, 0, 0);
    assert(target_forward == 0 && target_yaw == 0);

    follow_at(130, 0, 0);
    assert(target_forward == 300 && target_yaw == 0);
    follow_at(130, 38, 0);
    assert(target_forward == 0 && target_yaw == 1900);
    return 0;
}
