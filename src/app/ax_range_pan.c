#include "ax_range_pan.h"

#include "openctr_avoid_config.h"
#include "openctr_features.h"
#include "ax_robot.h"
#include "ax_sg90_pan.h"
#include "ax_xursc.h"

#define RANGE_PAN_SCAN_STEP_DEG 15

#if OPENCTR_ENABLE_SG90_PAN
static volatile int16_t requested_angle;
static volatile uint8_t angle_pending;
static volatile uint8_t requested_fast_settle;
static TickType_t angle_changed_tick;
static uint16_t angle_settle_ms;
static uint32_t angle_generation;
static volatile uint8_t scan_enabled;
static volatile uint8_t manual_allowed = 1;
static volatile uint8_t avoid_enabled;
static uint32_t avoid_mode_generation;
static uint32_t next_avoid_sequence;
static uint8_t avoid_sample_valid;
static AX_RANGE_PAN_Sample latest_avoid_sample;
static int16_t scan_angle;
static uint8_t scan_x_was_down;
static uint8_t scan_b_was_down;
static uint8_t scan_first_buttons;
#if OPENCTR_ENABLE_XURSC
static uint8_t manual_enabled;
#endif
#endif

#if OPENCTR_ENABLE_XURSC
static uint8_t waiting_for_echo;
static uint8_t triggered_once;
static TickType_t last_trigger_tick;
#if OPENCTR_ENABLE_SG90_PAN
    static uint32_t trigger_angle_generation;
    static uint32_t trigger_avoid_mode_generation;
#endif
#endif

void AX_RANGE_PAN_RequestAngle(int16_t degrees)
{
#if OPENCTR_ENABLE_SG90_PAN
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!scan_enabled && !avoid_enabled && manual_allowed) {
        requested_angle = degrees;
        angle_pending = 1;
        requested_fast_settle = 0;
#if OPENCTR_ENABLE_XURSC
        manual_enabled = 1;
#endif
    }
    __set_PRIMASK(primask);
#else
    (void)degrees;
#endif
}

void AX_RANGE_PAN_SetManualAllowed(uint8_t allowed)
{
#if OPENCTR_ENABLE_SG90_PAN
    uint32_t primask = __get_PRIMASK();
    uint8_t release_pan = 0;
    __disable_irq();
    if (!allowed) {
        if (!scan_enabled && !avoid_enabled) {
            if (manual_allowed) release_pan = 1;
            angle_pending = 0;
        }
#if OPENCTR_ENABLE_XURSC
        manual_enabled = 0;
#endif
    }
    manual_allowed = allowed ? 1u : 0u;
    __set_PRIMASK(primask);
    if (release_pan) AX_SG90_PanRelease();
#else
    (void)allowed;
#endif
}

void AX_RANGE_PAN_SetScanEnabled(uint8_t enabled)
{
#if OPENCTR_ENABLE_SG90_PAN && OPENCTR_ENABLE_XURSC
    uint32_t primask = __get_PRIMASK();
    uint8_t release_pan = 0;
    __disable_irq();
    if (avoid_enabled) enabled = 0;
    if (enabled && !scan_enabled) {
        scan_angle = 0;
        scan_x_was_down = 0;
        scan_b_was_down = 0;
        scan_first_buttons = 1;
        requested_angle = scan_angle;
        angle_pending = 1;
        requested_fast_settle = 0;
        manual_enabled = 0;
    } else if (!enabled && scan_enabled) {
        angle_pending = 0;
        manual_enabled = 0;
        release_pan = 1;
    }
    scan_enabled = enabled ? 1u : 0u;
    __set_PRIMASK(primask);
    if (release_pan) AX_SG90_PanRelease();
#else
    (void)enabled;
#endif
}

void AX_RANGE_PAN_SetAvoidEnabled(uint8_t enabled)
{
#if OPENCTR_ENABLE_SG90_PAN && OPENCTR_ENABLE_XURSC
    uint32_t primask = __get_PRIMASK();
    uint8_t release_pan = 0;
    enabled = enabled ? 1u : 0u;
    __disable_irq();
    if (enabled && !avoid_enabled) {
        avoid_enabled = 1;
        scan_enabled = 0;
        manual_enabled = 0;
        requested_angle = 0;
        angle_pending = 1;
        requested_fast_settle = 0;
        avoid_sample_valid = 0;
        ++avoid_mode_generation;
    } else if (!enabled && avoid_enabled) {
        avoid_enabled = 0;
        angle_pending = 0;
        manual_enabled = 0;
        avoid_sample_valid = 0;
        ++avoid_mode_generation;
        release_pan = 1;
    }
    __set_PRIMASK(primask);
    if (release_pan) AX_SG90_PanRelease();
#else
    (void)enabled;
#endif
}

static void AvoidRequestAngleWithSettle(int16_t degrees, uint8_t fast_settle)
{
#if OPENCTR_ENABLE_SG90_PAN && OPENCTR_ENABLE_XURSC
    uint32_t primask = __get_PRIMASK();
    if (degrees > 90) degrees = 90;
    if (degrees < -90) degrees = -90;
    __disable_irq();
    if (avoid_enabled && requested_angle != degrees) {
        requested_angle = degrees;
        angle_pending = 1;
        requested_fast_settle = fast_settle;
        avoid_sample_valid = 0;
    }
    __set_PRIMASK(primask);
#else
    (void)degrees;
    (void)fast_settle;
#endif
}

void AX_RANGE_PAN_AvoidRequestAngle(int16_t degrees)
{
    AvoidRequestAngleWithSettle(degrees, 0);
}

void AX_RANGE_PAN_AvoidRequestProbeAngle(int16_t degrees)
{
    if (degrees == -OPENCTR_AVOID_TRACK_PROBE_ANGLE_DEG ||
        degrees == 0 ||
        degrees == OPENCTR_AVOID_TRACK_PROBE_ANGLE_DEG)
        AvoidRequestAngleWithSettle(degrees, 1);
}

uint8_t AX_RANGE_PAN_GetAvoidSample(uint32_t max_age_ms,
                                    AX_RANGE_PAN_Sample *out)
{
#if OPENCTR_ENABLE_SG90_PAN && OPENCTR_ENABLE_XURSC
    AX_RANGE_PAN_Sample snapshot;
    uint8_t valid;
    uint32_t primask;
    if (out == 0) return 0;
    primask = __get_PRIMASK();
    __disable_irq();
    valid = avoid_enabled && avoid_sample_valid;
    snapshot = latest_avoid_sample;
    __set_PRIMASK(primask);
    if (!valid || (uint32_t)(xTaskGetTickCount() -
                             (TickType_t)snapshot.triggered_tick_ms) >
                  (uint32_t)pdMS_TO_TICKS(max_age_ms))
        return 0;
    *out = snapshot;
    return 1;
#else
    (void)max_age_ms;
    (void)out;
    return 0;
#endif
}

void AX_RANGE_PAN_ScanButtons(uint8_t x_down, uint8_t b_down)
{
#if OPENCTR_ENABLE_SG90_PAN && OPENCTR_ENABLE_XURSC
    uint32_t primask = __get_PRIMASK();
    int16_t next_angle;
    __disable_irq();
    if (!scan_enabled) {
        __set_PRIMASK(primask);
        return;
    }
    next_angle = scan_angle;
    x_down = x_down ? 1u : 0u;
    b_down = b_down ? 1u : 0u;
    if (scan_first_buttons) {
        scan_first_buttons = 0;
    } else if (x_down && !b_down && !scan_x_was_down) {
        next_angle -= RANGE_PAN_SCAN_STEP_DEG;
    } else if (b_down && !x_down && !scan_b_was_down) {
        next_angle += RANGE_PAN_SCAN_STEP_DEG;
    }
    scan_x_was_down = x_down;
    scan_b_was_down = b_down;
    if (next_angle > 90) next_angle = 90;
    if (next_angle < -90) next_angle = -90;
    if (next_angle != scan_angle) {
        scan_angle = next_angle;
        requested_angle = next_angle;
        angle_pending = 1;
        requested_fast_settle = 0;
    }
    __set_PRIMASK(primask);
#else
    (void)x_down;
    (void)b_down;
#endif
}

void AX_RANGE_PAN_TaskStep(void)
{
    TickType_t now_tick = xTaskGetTickCount();
#if OPENCTR_ENABLE_SG90_PAN
    uint32_t primask;
    uint8_t apply_angle;
    uint8_t fast_settle;
    int16_t angle;
    uint8_t angle_data[2];

    primask = __get_PRIMASK();
    __disable_irq();
    apply_angle = angle_pending;
    angle = requested_angle;
    fast_settle = requested_fast_settle;
    angle_pending = 0;
    __set_PRIMASK(primask);

    if (apply_angle) {
        if (angle > 90) angle = 90;
        if (angle < -90) angle = -90;
        AX_SG90_PanInit();
        AX_SG90_PanSetAngle(angle);
        angle_changed_tick = now_tick;
        angle_settle_ms = fast_settle ?
            OPENCTR_AVOID_TRACK_PROBE_SETTLE_MS :
            OPENCTR_AVOID_PAN_SETTLE_MS;
        ++angle_generation;
        angle_data[0] = (uint8_t)((uint16_t)angle >> 8);
        angle_data[1] = (uint8_t)angle;
        AX_UART3_SendPacket(angle_data, 2, ID_UTX_SG90_PAN);
    }
#endif

#if OPENCTR_ENABLE_XURSC
    AX_XURSC_Status status;
    uint16_t distance_mm = 0;
    uint8_t range_data[3];
#if OPENCTR_ENABLE_SG90_PAN
    uint8_t scan_data[5];
#endif

    if (waiting_for_echo) {
        status = AX_XURSC_GetStatus(&distance_mm);
        if (status == AX_XURSC_READY || status == AX_XURSC_TIMEOUT ||
            status == AX_XURSC_OUT_OF_RANGE) {
#if OPENCTR_ENABLE_SG90_PAN
            if (trigger_angle_generation == angle_generation && !angle_pending &&
                (scan_enabled || manual_enabled || avoid_enabled) &&
                (!avoid_enabled || trigger_avoid_mode_generation ==
                                   avoid_mode_generation))
#endif
            {
                range_data[0] = (uint8_t)status;
                range_data[1] = (uint8_t)(distance_mm >> 8);
                range_data[2] = (uint8_t)distance_mm;
                AX_UART3_SendPacket(range_data, 3, ID_UTX_XURSC);
#if OPENCTR_ENABLE_SG90_PAN
                if (avoid_enabled) {
                    latest_avoid_sample.angle_deg = requested_angle;
                    latest_avoid_sample.status = status;
                    latest_avoid_sample.distance_mm = distance_mm;
                    latest_avoid_sample.triggered_tick_ms =
                        (uint32_t)last_trigger_tick;
                    latest_avoid_sample.received_tick_ms =
                        (uint32_t)xTaskGetTickCount();
                    latest_avoid_sample.sequence = ++next_avoid_sequence;
                    avoid_sample_valid = 1;
                }
                if (scan_enabled) {
                    scan_data[0] = (uint8_t)((uint16_t)scan_angle >> 8);
                    scan_data[1] = (uint8_t)scan_angle;
                    scan_data[2] = (uint8_t)status;
                    scan_data[3] = (uint8_t)(distance_mm >> 8);
                    scan_data[4] = (uint8_t)distance_mm;
                    AX_UART3_SendPacket(scan_data, 5, ID_UTX_SCAN);
                }
#endif
            }
            waiting_for_echo = 0;
        }
    }

    if (!waiting_for_echo &&
#if OPENCTR_ENABLE_SG90_PAN
        (scan_enabled || manual_enabled || avoid_enabled) &&
#endif
        (!triggered_once ||
         (TickType_t)(now_tick - last_trigger_tick) >=
             pdMS_TO_TICKS(OPENCTR_AVOID_ULTRASONIC_INTERVAL_MS))
#if OPENCTR_ENABLE_SG90_PAN
        && !angle_pending
        && (TickType_t)(now_tick - angle_changed_tick) >=
            pdMS_TO_TICKS(angle_settle_ms)
#endif
        ) {
        if (AX_XURSC_Trigger()) {
            last_trigger_tick = now_tick;
            triggered_once = 1;
            waiting_for_echo = 1;
#if OPENCTR_ENABLE_SG90_PAN
            trigger_angle_generation = angle_generation;
            trigger_avoid_mode_generation = avoid_mode_generation;
#endif
        }
    }
#endif
}
