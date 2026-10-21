#include <assert.h>
#include <stdint.h>

#include "ax_range_pan.h"
#include "ax_xursc.h"

static TickType_t fake_tick;
static AX_XURSC_Status fake_status = AX_XURSC_IDLE;
static uint16_t fake_distance_mm;
static unsigned trigger_count;
static int16_t applied_angle = -100;
static unsigned release_count;
static unsigned range_packet_count;
static uint8_t last_range_packet[3];
static unsigned scan_packet_count;
static uint8_t last_scan_packet[5];

uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __set_PRIMASK(uint32_t mask) { (void)mask; }
TickType_t xTaskGetTickCount(void) { return fake_tick; }
void AX_UART3_SendPacket(uint8_t *data, uint8_t len, uint8_t id)
{
    if (id == ID_UTX_XURSC) {
        assert(len == 3);
        last_range_packet[0] = data[0];
        last_range_packet[1] = data[1];
        last_range_packet[2] = data[2];
        ++range_packet_count;
    }
    if (id == ID_UTX_SCAN) {
        assert(len == 5);
        for (unsigned i = 0; i < 5; ++i) last_scan_packet[i] = data[i];
        ++scan_packet_count;
    }
}
void AX_SG90_PanInit(void) {}
void AX_SG90_PanSetAngle(int16_t angle) { applied_angle = angle; }
void AX_SG90_PanRelease(void) { ++release_count; }
uint8_t AX_XURSC_Trigger(void)
{
    ++trigger_count;
    fake_status = AX_XURSC_WAITING;
    return 1;
}
AX_XURSC_Status AX_XURSC_GetStatus(uint16_t *distance_mm)
{
    if (fake_status == AX_XURSC_READY) *distance_mm = fake_distance_mm;
    return fake_status;
}

int main(void)
{
#ifdef TEST_RANGE_PAN_TIMING_OVERRIDE
    /* A tuning build must honor the configured pan wait and ping interval. */
    AX_RANGE_PAN_RequestAngle(30);
    AX_RANGE_PAN_TaskStep();
    fake_tick = 299;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == 0);
    fake_tick = 300;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == 1);
    fake_status = AX_XURSC_READY;
    fake_distance_mm = 800;
    fake_tick = 310;
    AX_RANGE_PAN_TaskStep();
    fake_tick = 419;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == 1);
    fake_tick = 420;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == 2);
    return 0;
#endif
    AX_RANGE_PAN_RequestAngle(30);
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 30);
    assert(trigger_count == 0);
    fake_tick = 249;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == 0);
    fake_tick = 250;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == 1);
    fake_status = AX_XURSC_READY;
    fake_distance_mm = 800;
    fake_tick = 270;
    AX_RANGE_PAN_TaskStep();
    assert(range_packet_count == 1);
    assert(last_range_packet[0] == AX_XURSC_READY);
    assert(last_range_packet[1] == 3 && last_range_packet[2] == 32);

    AX_RANGE_PAN_RequestAngle(-45);
    fake_tick = 280;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == -45);
    fake_tick = 529;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == 1);
    fake_tick = 530;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == 2);
    fake_status = AX_XURSC_TIMEOUT;
    fake_tick = 550;
    AX_RANGE_PAN_TaskStep();
    assert(range_packet_count == 2);
    assert(last_range_packet[0] == AX_XURSC_TIMEOUT);

    fake_tick = 650;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == 3);
    AX_RANGE_PAN_RequestAngle(45);
    fake_tick = 660;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 45);
    fake_status = AX_XURSC_READY;
    fake_distance_mm = 900;
    fake_tick = 680;
    AX_RANGE_PAN_TaskStep();
    assert(range_packet_count == 2); /* Echo belongs to the old angle. */
    fake_tick = 910;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == 4);
    fake_status = AX_XURSC_READY;
    fake_distance_mm = 700;
    fake_tick = 930;
    AX_RANGE_PAN_TaskStep();
    assert(range_packet_count == 3);
    assert(last_range_packet[0] == AX_XURSC_READY);
    assert(last_range_packet[1] == 2 && last_range_packet[2] == 188);

    AX_RANGE_PAN_SetManualAllowed(0);
    AX_RANGE_PAN_SetScanEnabled(1);
    AX_RANGE_PAN_ScanButtons(0, 0);
    fake_tick = 1000;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 0);
    fake_tick = 1249;
    AX_RANGE_PAN_TaskStep();
    assert(scan_packet_count == 0);
    fake_tick = 1250;
    AX_RANGE_PAN_TaskStep();
    fake_status = AX_XURSC_READY;
    fake_distance_mm = 600;
    fake_tick = 1270;
    AX_RANGE_PAN_TaskStep();
    assert(scan_packet_count == 1);
    assert(last_scan_packet[0] == 0 && last_scan_packet[1] == 0);
    assert(last_scan_packet[2] == AX_XURSC_READY);
    assert(last_scan_packet[3] == 2 && last_scan_packet[4] == 88);
    fake_tick = 1290;
    AX_RANGE_PAN_SetManualAllowed(0);
    AX_RANGE_PAN_ScanButtons(0, 0);
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 0); /* A completed echo does not advance the pan. */

    AX_RANGE_PAN_ScanButtons(0, 1); /* B pressed. */
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 15);
    AX_RANGE_PAN_ScanButtons(0, 1); /* Held B moves only once. */
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 15);
    AX_RANGE_PAN_ScanButtons(0, 0);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    AX_RANGE_PAN_ScanButtons(0, 1);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 30);
    AX_RANGE_PAN_ScanButtons(0, 0);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    AX_RANGE_PAN_RequestAngle(45); /* USB cannot interrupt SCAN. */
    fake_tick += 250;
    AX_RANGE_PAN_TaskStep();
    fake_status = AX_XURSC_TIMEOUT;
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(scan_packet_count == 2);
    assert(last_scan_packet[0] == 0 && last_scan_packet[1] == 30);
    assert(last_scan_packet[2] == AX_XURSC_TIMEOUT);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 30);

    for (int16_t expected = 15; expected >= -90; expected -= 15) {
        AX_RANGE_PAN_ScanButtons(1, 0); /* X pressed. */
        fake_tick += 20;
        AX_RANGE_PAN_TaskStep();
        assert(applied_angle == expected);
        AX_RANGE_PAN_ScanButtons(0, 0); /* X released. */
        fake_tick += 20;
        AX_RANGE_PAN_TaskStep();
    }
    AX_RANGE_PAN_ScanButtons(1, 0); /* Cannot pass -90. */
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == -90);
    AX_RANGE_PAN_ScanButtons(0, 0);
    AX_RANGE_PAN_ScanButtons(1, 1); /* Simultaneous buttons do nothing. */
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == -90);
    AX_RANGE_PAN_ScanButtons(0, 0);
    AX_RANGE_PAN_ScanButtons(0, 1);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == -75);
    AX_RANGE_PAN_ScanButtons(0, 0);
    for (int16_t expected = -60; expected <= 90; expected += 15) {
        AX_RANGE_PAN_ScanButtons(0, 1);
        fake_tick += 20;
        AX_RANGE_PAN_TaskStep();
        assert(applied_angle == expected);
        AX_RANGE_PAN_ScanButtons(0, 0);
        fake_tick += 20;
        AX_RANGE_PAN_TaskStep();
    }
    AX_RANGE_PAN_ScanButtons(0, 1); /* Cannot pass +90. */
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 90);
    AX_RANGE_PAN_SetScanEnabled(0);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 90);
    unsigned count_at_exit = trigger_count;
    fake_tick += 1000;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_at_exit);
    assert(release_count == 2);

    AX_RANGE_PAN_SetScanEnabled(1);
    AX_RANGE_PAN_ScanButtons(0, 1); /* Held on entry does not move. */
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 0); /* Re-entry starts centered. */
    AX_RANGE_PAN_ScanButtons(0, 1);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 0);
    AX_RANGE_PAN_ScanButtons(0, 0);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    AX_RANGE_PAN_ScanButtons(0, 1);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 15);
    fake_tick += 250;
    AX_RANGE_PAN_TaskStep();
    unsigned scan_count_at_exit = scan_packet_count;
    unsigned range_count_at_exit = range_packet_count;
    AX_RANGE_PAN_SetScanEnabled(0);
    fake_status = AX_XURSC_READY;
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(scan_packet_count == scan_count_at_exit);
    assert(range_packet_count == range_count_at_exit);
    AX_RANGE_PAN_SetManualAllowed(1); /* Return to USB control. */
    AX_RANGE_PAN_RequestAngle(30);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 30);
    fake_tick += 250;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_at_exit + 2);
    unsigned range_count_before_ps2 = range_packet_count;
    unsigned trigger_count_before_ps2 = trigger_count;
    AX_RANGE_PAN_SetManualAllowed(0); /* Enter PS2 DRIVE or FOLLOW. */
    AX_RANGE_PAN_RequestAngle(45);
    fake_status = AX_XURSC_READY;
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    fake_tick += 1000;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 30);
    assert(trigger_count == trigger_count_before_ps2);
    assert(range_packet_count == range_count_before_ps2);
    assert(release_count == 4);

    AX_RANGE_PAN_SetAvoidEnabled(1);
    AX_RANGE_PAN_SetScanEnabled(1); /* AVOID owns the pan over SCAN. */
    AX_RANGE_PAN_ScanButtons(0, 1);
    AX_RANGE_PAN_RequestAngle(45); /* USB cannot move it during AVOID. */
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 0);
    fake_tick += 250;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == trigger_count_before_ps2 + 1);
    TickType_t first_avoid_trigger_tick = fake_tick;
    fake_status = AX_XURSC_READY;
    fake_distance_mm = 1200;
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    AX_RANGE_PAN_Sample sample;
    assert(AX_RANGE_PAN_GetAvoidSample(250, &sample));
    assert(sample.angle_deg == 0);
    assert(sample.status == AX_XURSC_READY);
    assert(sample.distance_mm == 1200);
    assert(sample.triggered_tick_ms == first_avoid_trigger_tick);
    assert(sample.received_tick_ms == fake_tick);
    assert(sample.sequence != 0);

    AX_RANGE_PAN_AvoidRequestAngle(-45);
    assert(!AX_RANGE_PAN_GetAvoidSample(250, &sample));
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == -45);
    fake_tick += 250;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == trigger_count_before_ps2 + 2);
    TickType_t second_avoid_trigger_tick = fake_tick;
    fake_status = AX_XURSC_READY;
    fake_distance_mm = 900;
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(AX_RANGE_PAN_GetAvoidSample(250, &sample));
    assert(sample.angle_deg == -45 && sample.distance_mm == 900);
    assert(sample.triggered_tick_ms == second_avoid_trigger_tick);
    fake_tick += 230;
    assert(AX_RANGE_PAN_GetAvoidSample(250, &sample));
    fake_tick += 1;
    assert(!AX_RANGE_PAN_GetAvoidSample(250, &sample));

    /* Only TRACK's probe requests use the 60 ms settle time. */
    AX_RANGE_PAN_AvoidRequestAngle(0);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    unsigned count_before_center = trigger_count;
    fake_tick += 249;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_before_center);
    fake_tick += 1;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_before_center + 1);
    fake_status = AX_XURSC_READY;
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();

    AX_RANGE_PAN_AvoidRequestProbeAngle(15);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 15);
    unsigned count_before_probe = trigger_count;
    fake_tick += 59;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_before_probe);
    fake_tick += 1;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_before_probe + 1);
    fake_status = AX_XURSC_READY;
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(AX_RANGE_PAN_GetAvoidSample(250, &sample));
    assert(sample.angle_deg == 15);

    AX_RANGE_PAN_AvoidRequestProbeAngle(0);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == 0);
    count_before_probe = trigger_count;
    fake_tick += 59;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_before_probe);
    fake_tick += 1;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_before_probe + 1);
    fake_status = AX_XURSC_READY;
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();

    AX_RANGE_PAN_AvoidRequestProbeAngle(-15);
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    assert(applied_angle == -15);
    count_before_probe = trigger_count;
    fake_tick += 59;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_before_probe);
    fake_tick += 1;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_before_probe + 1);
    fake_status = AX_XURSC_READY;
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();

    AX_RANGE_PAN_AvoidRequestAngle(15); /* A normal request stays at 250 ms. */
    fake_tick += 20;
    AX_RANGE_PAN_TaskStep();
    count_before_probe = trigger_count;
    fake_tick += 60;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_before_probe);
    fake_tick += 189;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_before_probe);
    fake_tick += 1;
    AX_RANGE_PAN_TaskStep();
    assert(trigger_count == count_before_probe + 1);
    AX_RANGE_PAN_SetAvoidEnabled(0);
    assert(!AX_RANGE_PAN_GetAvoidSample(250, &sample));
    return 0;
}
