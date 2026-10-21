# PS2 Obstacle-Avoiding Follow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fourth PS2 mode that follows AOA and detours around small static obstacles using the SG90/XURSC module.

**Architecture:** Keep DRIVE, FOLLOW and SCAN intact. Give the new mode a small pan/range API and a pure avoidance state machine; one adapter submits its chassis target. Use the existing AOA follow policy while the front is clear.

**Tech Stack:** C99, STM32F407 Standard Peripheral Library, FreeRTOS, PlatformIO; three focused host C tests.

**Spec:** `docs/superpowers/specs/2026-10-19-follow-avoid-design.md`

## Global Constraints

- SELECT release cycles `DRIVE → FOLLOW → FOLLOW_AVOID → SCAN → DRIVE`; entry remains DRIVE.
- Differential chassis: forward and in-place yaw only. Never command automatic reverse or lateral travel.
- Initial thresholds: front stop 500 mm, release 650 mm, side candidate 800 mm, side scan ±45°, sensor settle 250 ms, fresh range ≤250 ms, AOA age ≤300 ms.
- Initial motion limits: normal follow 300 mm/s, detour 100 mm/s, yaw 500 mrad/s, turn 45° within 3 s, advance 150 mm per segment, detour cap 1200 mm or 25 s.
- Stop on invalid AOA/range and return to `WAIT_FRONT` when those inputs recover; lock `HOLD` for blocked routes, lost PS2, movement timeout or exhausted detour budget. Keep the existing USB-COM packet formats; add no diagnostic protocol.

## Review Focus

- A measurement from an earlier pan angle or mode cannot authorize movement; Task 1 checks generation invalidation.
- Holding SELECT during entry or switching cannot advance modes or drive; Task 3 checks mode actions.
- A blocked front reading cannot let the FOLLOW target pass through; Task 2 checks zero forward output.
- A stale/missing centered sample cannot authorize `TURN_OUT`; the policy stops and reacquires from `WAIT_FRONT`.
- Two identical copies of one clear reading cannot finish rejoin; Task 2 checks distinct sequence numbers.
- A stalled wheel or turn timeout cannot keep commanding motion; Task 2 checks the limits.

---

### Task 1: Expose fresh pan/range samples to the fourth mode

**Files:** Modify `src/app/ax_range_pan.h`, `src/app/ax_range_pan.c`, `tests/test_range_pan.c`.

**Interfaces:** Add `void AX_RANGE_PAN_SetAvoidEnabled(uint8_t enabled)`, `void AX_RANGE_PAN_AvoidRequestAngle(int16_t degrees)`, `uint8_t AX_RANGE_PAN_GetAvoidSample(uint32_t max_age_ms, AX_RANGE_PAN_Sample *out)`. The sample has `int16_t angle_deg`, `AX_XURSC_Status status`, `uint16_t distance_mm`, `uint32_t received_tick_ms`, `uint32_t sequence`. A successful getter means a fresh sample exists, even if its status is timeout/out-of-range; callers require READY before movement. Preserve the existing USB and SCAN APIs; avoid mode takes priority while enabled.

- [x] **Step 1: Add failing assertions to `tests/test_range_pan.c`.** Assert avoid entry requests 0°, USB/SCAN cannot move it, a sample reports angle/distance/status/tick, changing angle or leaving mode removes the old sample, and a 251 ms sample is rejected.
- [x] **Step 2: Run `cc -std=c99 -Wall -Wextra -Werror -include tests/range_pan_stubs/platform.h -Isrc/app -Isrc/sensors -Isrc/actuators -Iinclude/config tests/test_range_pan.c src/app/ax_range_pan.c -o /tmp/test_range_pan && /tmp/test_range_pan`; expect a missing API failure.**
- [x] **Step 3: Implement the three APIs.** Reuse current 250 ms settle and 100 ms trigger timing. Keep one accepted sample with angle generation and mode generation; invalidate it on angle/mode changes. Preserve existing 0x11/0x12/0x15 frames and SCAN button steps.
- [x] **Step 4: Run the focused test; expect exit 0.** Check `git diff --check`; keep the work in the shared checkout per the ledger ruling.

### Task 2: Implement one testable detour decision module

**Files:** Create `src/control/ax_follow_avoid_policy.h`, `src/control/ax_follow_avoid_policy.c`, `include/config/openctr_avoid_config.h`, `tests/test_follow_avoid_policy.c`.

**Interfaces:** Define `AX_AVOID_Input` with current time, AOA validity/azimuth, follow target, current range angle/status/distance/sequence, measured yaw rate and forward speed. Define `AX_AVOID_Output` with requested pan angle and `{forward_mm_s, yaw_mrad_s}`. Expose `void AX_AVOID_Reset(AX_AVOID_State *state)` and `AX_AVOID_Output AX_AVOID_Step(AX_AVOID_State *state, const AX_AVOID_Config *config, const AX_AVOID_Input *input)`. State is `WAIT_FRONT → TRACK → SCAN_SIDE → TURN_OUT → ADVANCE → TRY_REJOIN`, with HOLD for blocked or failed cases. Use sample sequence to count two distinct clear front readings; locked side prevents oscillation. The module does not call hardware.

- [x] **Step 1: Create one focused C test.** Assert clear 0° range permits capped FOLLOW output, 499 mm stops, -45°/+45° samples select a valid ≥800 mm side, both blocked stop, stale centered range stops turning, temporary AOA/range loss recovers through WAIT_FRONT, 45° turn and 150 mm advance are bounded, two distinct ≥650 mm front samples restore TRACK, and stalled advance, 3 s turn timeout, 1200 mm distance or 25 s detour budget stop.
- [x] **Step 2: Run `cc -std=c99 -Wall -Wextra -Werror -Isrc/control -Isrc/sensors -Iinclude/config tests/test_follow_avoid_policy.c src/control/ax_follow_avoid_policy.c -o /tmp/test_follow_avoid_policy && /tmp/test_follow_avoid_policy`; expect a missing module failure.**
- [x] **Step 3: Implement the state machine and initial config.** Integrate measured yaw/forward speed over `now_ms` deltas using wide intermediates; clamp each valid step to 100 ms and stop if the loop gap exceeds it. Use zero motion while scanning and waiting; cap detour to 100 mm/s and 500 mrad/s, with no reverse/lateral target.
- [x] **Step 4: Run the focused test; expect exit 0.** Check `git diff --check`; keep the work in the shared checkout per the ledger ruling.

### Task 3: Connect mode, sensor and motion

**Files:** Create `src/control/ax_follow_avoid.h`, `src/control/ax_follow_avoid.c`; modify `src/control/ax_ps2_mode.h`, `src/control/ax_ps2_mode.c`, `src/control/ax_control.h`, `src/control/ax_control.c`, `src/app/ax_robot.c`, `tests/test_ps2_mode.c`, `README.md`.

**Interfaces:** Add `AX_PS2_MODE_FOLLOW_AVOID` and `AX_PS2_INPUT_FOLLOW_AVOID`. Change `AX_CTL_Ps2(JOYSTICK_TypeDef)` to return the action; it commands zero for the new mode before sensing. Add `void AX_FOLLOW_AVOID_Reset(void)` and `void AX_FOLLOW_AVOID_Step(void)`; the step reads AOA ≤300 ms, fresh range ≤250 ms, `R_Imu.GYRO_Z` and `R_Vel.RT_IX`, computes a separate follow target, calls `AX_AVOID_Step`, requests its pan angle and commits one PS2 target. Convert raw gyro to mrad/s with `raw * 8727 / 32768`.

- [x] **Step 1: Update `tests/test_ps2_mode.c` to expect the four-mode release cycle, initial-held SELECT suppression and invalid-sample reset.** Run `cc -std=c99 -Wall -Wextra -Werror -Isrc/control tests/test_ps2_mode.c src/control/ax_ps2_mode.c -o /tmp/test_ps2_mode && /tmp/test_ps2_mode`; expect failure before mode changes.
- [x] **Step 2: Implement mode routing and the adapter.** In `Robot_Task`, after PS2 mode update, enable avoid ranging, then run range/AOA/IMU steps before the avoid decision and kinematics. Only valid AVOID action runs the adapter; switching, stale input and source exit leave zero target and reset avoid state. OLED shows `Mode: AVOID`; README explains the new SELECT cycle and low-speed limitations.
- [x] **Step 3: Run all three host tests and `/home/fhc/.platformio/penv/bin/pio run`; expect exit 0 and `[SUCCESS]`.** Also run the existing `tests/test_follow_step.c` command from README to confirm old FOLLOW, then `git diff --check`.
- [x] **Step 4: Complete final review and keep the work in the shared checkout per the ledger ruling.**

## Physical validation handoff

Software completion does not establish real-world clearance. When the board is available, confirm the four-wheel differential switch, pan direction and distance against known targets with wheels raised; then measure braking distance and 45°/150 mm movement at ≤100 mm/s in a wide open area. Adjust thresholds from those measurements before testing a static obstacle. No firmware upload is part of this host-only plan.
