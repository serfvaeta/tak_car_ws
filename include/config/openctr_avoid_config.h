#ifndef OPENCTR_AVOID_CONFIG_H
#define OPENCTR_AVOID_CONFIG_H

#include "openctr_follow_config.h"

/* 超声波避障距离，单位 mm。调试时优先修改这三项。 */
/* TRACK 任一探测方向或绕行前方 < 停车距离：停车并扫描。 */
#define OPENCTR_AVOID_FRONT_STOP_MM 350u
/* 正前方 >= 放行距离：允许继续；必须大于停车距离。 */
#define OPENCTR_AVOID_FRONT_RELEASE_MM 650u
/* 侧扫读数 >= 此值：该方向可作为绕行候选。 */
#define OPENCTR_AVOID_SIDE_MIN_MM 800u
#define OPENCTR_AVOID_TRACK_PROBE_ANGLE_DEG 15

/* 测距节奏。完整换角等待也作用于手动扫描；测量间隔作用于全部超声波模式。 */
#ifndef OPENCTR_AVOID_PAN_SETTLE_MS
/* 舵机换角后等待稳定。 */
#define OPENCTR_AVOID_PAN_SETTLE_MS 250u
#endif
#ifndef OPENCTR_AVOID_TRACK_PROBE_SETTLE_MS
/* FOLLOW_AVOID TRACK 中 0 / +/-15 度探测换角的等待时间。 */
#define OPENCTR_AVOID_TRACK_PROBE_SETTLE_MS 60u
#endif
#ifndef OPENCTR_AVOID_ULTRASONIC_INTERVAL_MS
/* 两次触发的最短间隔；底层还要求至少 60 ms。 */
#define OPENCTR_AVOID_ULTRASONIC_INTERVAL_MS 100u
#endif

#define OPENCTR_AVOID_SCAN_MAX_ANGLE_DEG 90
#define OPENCTR_AVOID_SCAN_STEP_DEG 15
#define OPENCTR_AVOID_RANGE_MAX_AGE_MS 250u /* 超过此年龄的测距不用于运动决策。 */
#define OPENCTR_AVOID_AOA_MAX_AGE_MS 300u

#define OPENCTR_AVOID_FOLLOW_MAX_MM_S 300
#define OPENCTR_AVOID_FORWARD_MAX_MM_S 100
#define OPENCTR_AVOID_YAW_MAX_MRAD_S 500
#define OPENCTR_AVOID_TURN_TIMEOUT_MS 5000u
#define OPENCTR_AVOID_TURN_SETTLE_MAX_YAW_MRAD_S 100u
#define OPENCTR_AVOID_TURN_OVERSHOOT_MRAD 100u
#define OPENCTR_AVOID_ADVANCE_SEGMENT_MM 150u
/* Zero disables the cumulative distance/time caps across repeated detours. */
#define OPENCTR_AVOID_MAX_DISTANCE_MM 0u
#define OPENCTR_AVOID_MAX_TIME_MS 0u
#define OPENCTR_AVOID_STALL_TIMEOUT_MS 1000u
#define OPENCTR_AVOID_STALL_MIN_MM 20u
#define OPENCTR_AVOID_REJOIN_CLEAR_SAMPLES 2u
#define OPENCTR_AVOID_RESCAN_WAIT_MS 2000u /* 无合格方向时，停车多久再重扫。 */
#define OPENCTR_AVOID_SIDE_SCAN_TIMEOUT_MS 8000u /* 一轮侧扫的超时。 */

#define OPENCTR_AVOID_AZIMUTH_TO_YAW_SIGN \
    OPENCTR_FOLLOW_AZIMUTH_TO_YAW_SIGN

#endif
