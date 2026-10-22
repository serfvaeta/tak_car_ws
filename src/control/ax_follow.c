#include "ax_follow.h"
#include "ax_follow_policy.h"
#include "ax_follow_profile.h"
#include "ax_aoa.h"
#include "ax_robot.h"
#include "openctr_follow_config.h"

#if OPENCTR_FOLLOW_FORWARD_MAX_MM_S > R_VX_LIMIT
#error "FOLLOW forward speed exceeds the chassis X speed limit"
#endif

#if OPENCTR_FOLLOW_REVERSE_MAX_MM_S > R_VX_LIMIT
#error "FOLLOW reverse speed exceeds the chassis X speed limit"
#endif

#if OPENCTR_FOLLOW_TURN_MAX_MRAD_S > R_VW_LIMIT
#error "FOLLOW turn speed exceeds the chassis yaw speed limit"
#endif

static AX_FOLLOW_State follow_state;

void AX_FOLLOW_ModeReset(void)
{
    AX_FOLLOW_Reset(&follow_state);
}

void AX_FOLLOW_Step(void)
{
    AX_AOA_Sample sample;
    AX_FOLLOW_Observation observation;
    AX_FOLLOW_Target target;
    AX_FOLLOW_Config config;

    if (!AX_AOA_GetLatest(&sample, OPENCTR_FOLLOW_FIX_MAX_AGE_MS)) {
        AX_FOLLOW_ModeReset();
        AX_ROBOT_SetTargetIfActive(CTL_PS2, 0, 0, 0);
        return;
    }

    observation.distance_cm = sample.measurement.distance_cm;
    observation.azimuth_deg = sample.measurement.azimuth_deg;
    config = AX_FOLLOW_Profile(OPENCTR_FOLLOW_FORWARD_MAX_MM_S);
    target = AX_FOLLOW_Compute(&follow_state, &config, &observation);
    /* Keep the current no-reverse behavior without depending on a range sensor. */
    if (target.forward_mm_s < 0) target.forward_mm_s = 0;
    AX_ROBOT_SetTargetIfActive(CTL_PS2, target.forward_mm_s, 0, target.yaw_mrad_s);
}
