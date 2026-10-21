#include <assert.h>
#include <stdint.h>

#include "ax_obstacle_guard.h"

static const AX_OBSTACLE_Config config = {500u, 650u, 300};

static AX_FOLLOW_Target target(int16_t forward, int16_t yaw)
{
    AX_FOLLOW_Target result = {forward, yaw};
    return result;
}

static void close_range_stops_all_motion_and_holds_until_clear(void)
{
    AX_OBSTACLE_State state;
    AX_FOLLOW_Target result;

    AX_OBSTACLE_Reset(&state);
    result = AX_OBSTACLE_Apply(&state, &config, 1, 499u, target(250, 400));
    assert(result.forward_mm_s == 0 && result.yaw_mrad_s == 0);
    result = AX_OBSTACLE_Apply(&state, &config, 1, 600u, target(250, 400));
    assert(result.forward_mm_s == 0 && result.yaw_mrad_s == 0);
    result = AX_OBSTACLE_Apply(&state, &config, 1, 650u, target(250, 400));
    assert(result.forward_mm_s == 250 && result.yaw_mrad_s == 400);
    result = AX_OBSTACLE_Apply(&state, &config, 1, 500u, target(250, 400));
    assert(result.forward_mm_s == 250 && result.yaw_mrad_s == 400);
}

static void invalid_range_stops_and_does_not_release_a_block(void)
{
    AX_OBSTACLE_State state;
    AX_FOLLOW_Target result;

    AX_OBSTACLE_Reset(&state);
    result = AX_OBSTACLE_Apply(&state, &config, 0, 0u, target(250, 400));
    assert(result.forward_mm_s == 0 && result.yaw_mrad_s == 0);
    result = AX_OBSTACLE_Apply(&state, &config, 1, 600u, target(250, 400));
    assert(result.forward_mm_s == 0 && result.yaw_mrad_s == 0);
    result = AX_OBSTACLE_Apply(&state, &config, 1, 700u, target(250, 400));
    assert(result.forward_mm_s == 250 && result.yaw_mrad_s == 400);
}

static void clear_range_caps_forward_and_disables_blind_reverse(void)
{
    AX_OBSTACLE_State state;
    AX_FOLLOW_Target result;

    AX_OBSTACLE_Reset(&state);
    result = AX_OBSTACLE_Apply(&state, &config, 1, 800u, target(900, 0));
    assert(result.forward_mm_s == 300 && result.yaw_mrad_s == 0);
    result = AX_OBSTACLE_Apply(&state, &config, 1, 800u, target(-200, 0));
    assert(result.forward_mm_s == 0 && result.yaw_mrad_s == 0);
    result = AX_OBSTACLE_Apply(&state, &config, 1, 800u, target(0, -300));
    assert(result.forward_mm_s == 0 && result.yaw_mrad_s == -300);
}

int main(void)
{
    close_range_stops_all_motion_and_holds_until_clear();
    invalid_range_stops_and_does_not_release_a_block();
    clear_range_caps_forward_and_disables_blind_reverse();
    return 0;
}
