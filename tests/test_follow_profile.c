#include <assert.h>

#include "ax_follow_policy.h"
#include "ax_follow_profile.h"

int main(void)
{
    AX_FOLLOW_Config normal = AX_FOLLOW_Profile(300);
    AX_FOLLOW_Config limited = AX_FOLLOW_Profile(200);
    AX_FOLLOW_State state;
    AX_FOLLOW_Observation observation = {130, 0};
    AX_FOLLOW_Target target;

    AX_FOLLOW_Reset(&state);
    target = AX_FOLLOW_Compute(&state, &normal, &observation);
    assert(target.forward_mm_s == 300 && target.yaw_mrad_s == 0);

    AX_FOLLOW_Reset(&state);
    target = AX_FOLLOW_Compute(&state, &limited, &observation);
    assert(target.forward_mm_s == 200 && target.yaw_mrad_s == 0);

    observation.azimuth_deg = 20;
    target = AX_FOLLOW_Compute(&state, &limited, &observation);
    assert(target.forward_mm_s == 0 && target.yaw_mrad_s == 1000);
    return 0;
}
