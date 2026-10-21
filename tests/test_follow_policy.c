#include <assert.h>
#include <stdint.h>

#include "ax_follow_policy.h"

static const AX_FOLLOW_Config config = {
    80u, 110u, 100u, 300u, /* stop-close, start, stop, max distance (cm) */
    12u, 5u,               /* turn start/stop (deg) */
    400, 800, 1,            /* speed ceilings and angle sign */
    150, 300, 5u, 20u,      /* forward/turn speed floors and proportional gains */
    0u, 95u, 100, 400, 10u /* reverse validity, stop, speed floor/cap, gain */
};

static AX_FOLLOW_Observation observe(uint32_t distance_cm, int16_t azimuth_deg)
{
    AX_FOLLOW_Observation result = {distance_cm, azimuth_deg};
    return result;
}

int main(void)
{
    AX_FOLLOW_State state;
    AX_FOLLOW_Target target;
    AX_FOLLOW_Observation sample;
    AX_FOLLOW_Config unknown_sign = config;
    AX_FOLLOW_Config reversed_sign = config;

    AX_FOLLOW_Reset(&state);
    sample = observe(95, 0);
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 0);

    sample = observe(130, 0);
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 150 && target.yaw_mrad_s == 0);
    sample.distance_cm = 105; /* advance continues until the inner threshold */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 150);
    sample.distance_cm = 100;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0);

    sample.distance_cm = 130;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 150);
    AX_FOLLOW_Reset(&state); /* leaving FOLLOW or losing the location frame */
    sample.distance_cm = 105;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0);

    sample = observe(130, 20);
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 400);
    sample.azimuth_deg = -20;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.yaw_mrad_s == -400);

    reversed_sign.azimuth_to_yaw_sign = -1;
    sample.azimuth_deg = 20;
    target = AX_FOLLOW_Compute(&state, &reversed_sign, &sample);
    assert(target.yaw_mrad_s == -400);
    sample.azimuth_deg = 6;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 300);
    sample.azimuth_deg = 5;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 150 && target.yaw_mrad_s == 0);

    sample = observe(70, 20); /* align first, even when close */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 400);
    sample.azimuth_deg = 5;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == -250 && target.yaw_mrad_s == 0);
    sample = observe(90, 0); /* keep reversing through the deadband */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == -100 && target.yaw_mrad_s == 0);
    sample.distance_cm = 95; /* stop reverse before forward can start */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 0);
    sample.distance_cm = 0; /* invalid location must never trigger reverse */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 0);
    sample.distance_cm = 10; /* a genuine very close fix still backs away */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == -400 && target.yaw_mrad_s == 0);
    sample = observe(130, 61); /* keep aligning at large angles */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 800);
    sample = observe(130, 0);
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 150 && target.yaw_mrad_s == 0);

    sample = observe(300, 0); /* upper distance boundary still follows */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 400 && target.yaw_mrad_s == 0);
    sample = observe(301, 0);
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 0);

    AX_FOLLOW_Reset(&state);
    sample = observe(110, 0); /* start threshold still yields usable torque */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 150);
    sample = observe(200, 0); /* farther target reaches the speed ceiling */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 400);
    AX_FOLLOW_Reset(&state);
    sample = observe(130, 12);
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.yaw_mrad_s == 300);
    sample.azimuth_deg = 38;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.yaw_mrad_s == 760);
    sample.azimuth_deg = 60;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.yaw_mrad_s == 800);
    sample = observe(55, 38); /* turning takes priority even when close */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 760);
    sample.azimuth_deg = 0;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == -400 && target.yaw_mrad_s == 0);

    AX_FOLLOW_Reset(&state);
    sample = observe(70, 0);
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == -250);
    sample = observe(85, 12); /* a turn pauses reversing before 95 cm */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 300);
    sample.azimuth_deg = 5;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == -100 && target.yaw_mrad_s == 0);

    AX_FOLLOW_Reset(&state);
    sample = observe(130, 0);
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 150);
    sample = observe(105, 12); /* a turn pauses advancing before 100 cm */
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 300);
    sample.azimuth_deg = 5;
    target = AX_FOLLOW_Compute(&state, &config, &sample);
    assert(target.forward_mm_s == 150 && target.yaw_mrad_s == 0);

    unknown_sign.azimuth_to_yaw_sign = 0; /* uncalibrated installation */
    sample = observe(130, 0);
    target = AX_FOLLOW_Compute(&state, &unknown_sign, &sample);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 0);

    return 0;
}
