# FOLLOW_AVOID2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a selectable PS2 `FOLLOW_AVOID2` mode that scans 6° coarse candidates, validates five neighboring angles across ±10°, corrects a candidate by 5° with bounded retries, then turns and advances by the post-turn front range minus a 400 mm clearance (capped at 2000 mm range).

**Architecture:** Keep the existing `FOLLOW_AVOID` implementation unchanged. Add a pure `ax_follow_avoid2_policy` state machine and a small runtime adapter that uses the same fresh AOA/range, gyro, encoder, pan scheduler, chassis and follow-distance inputs as `FOLLOW_AVOID`. Extend the PS2 mode cycle and route AVOID2 to its own state, config and controller.

**Tech Stack:** C99, STM32F407/FreeRTOS, PlatformIO, host-side C assertion tests.

**Spec:** `docs/superpowers/specs/2026-10-23-follow-avoid2-design.md`

## Global Constraints

- Keep `FOLLOW_AVOID` behavior and its config unchanged.
- Add a separate PS2 mode after `FOLLOW_AVOID`: `DRIVE → FOLLOW → FOLLOW_AVOID → FOLLOW_AVOID2 → SCAN → DRIVE`.
- Use robot-relative angles: 0° forward, positive right, negative left, bounded to ±90°.
- Coarse scan spacing is 6°; retain at most 5 safe candidates ordered by target-angle error.
- Local validation samples are candidate center ±10°, ±5° and center; every sample must be fresh, valid and at least 600 mm.
- Correct planning angle in 5° steps; limit to 10 corrections per candidate and prevent revisiting an angle.
- Never authorize movement from invalid, stale, mismatched-angle or pre-turn range samples.
- Use 250 mm as the moving emergency threshold; require a fresh centered sample of at least 600 mm after turning.
- Advance by `min(post-turn front range, 2000 mm) - 400 mm`, then obtain a new target bearing and replan; no blind reverse or lateral motion.

## Review Focus

- Positive-right AOA and gyro signs disagree: test signed left/right candidate turns against the configured yaw sign.
- A scan point is missing or stale: test it fails the local window and never becomes a clear path.
- Coarse candidates do not fit the ±10° verification window at a servo limit: test they are skipped.
- Correction revisits a center or exceeds its bound: test the current candidate terminates and tries the next one.
- A target comes within the existing AOA stop hysteresis or sensor data expires during movement: test motion stops without reusing old samples.

---

### Task 1: Implement and test the isolated AVOID2 policy

**Files:**
- Create: `include/config/openctr_avoid2_config.h`
- Create: `src/control/ax_follow_avoid2_policy.h`
- Create: `src/control/ax_follow_avoid2_policy.c`
- Create: `tests/test_follow_avoid2_policy.c`

**Interfaces:**
- Consumes `AX_XURSC_Status` from `ax_xursc.h` and a per-tick snapshot of AOA, the latest range sample, gyro rate, forward velocity and current time.
- Produces `AX_AVOID2_Output AX_AVOID2_Step(AX_AVOID2_State *, const AX_AVOID2_Config *, const AX_AVOID2_Input *)`; output contains requested pan angle, forward speed and yaw speed.
- Policy phases: `WAIT_TARGET`, `COARSE_SCAN`, `FINE_SCAN`, `TURN`, `VERIFY_FRONT`, `MOVE`, `WAIT_RESCAN`.

- [x] **Step 1: Write failing policy tests** for center-out coarse angle sequence, range threshold and AOA candidate ordering; five-point acceptance; one-sided correction sign; both-side candidate rejection; center-only safer-side selection; invalid/stale sample rejection; candidate window at ±90°; 10-correction/revisited-angle termination; candidate exhaustion stop; turn completion requiring a fresh centered ≥600 mm reading; post-turn range minus clearance with a 2000 mm cap; emergency-front stop.
- [x] **Step 2: Run the policy test and verify it fails** because the AVOID2 policy API does not exist.

  Run: `cc -std=c99 -Wall -Wextra -Werror -Isrc/control -Isrc/sensors -Iinclude/config tests/test_follow_avoid2_policy.c src/control/ax_follow_avoid2_policy.c -o /tmp/test_follow_avoid2_policy && /tmp/test_follow_avoid2_policy`

- [x] **Step 3: Implement the independent AVOID2 configuration and policy.** Defaults: ±90° range, 6° coarse interval, ±10° local window, 5° local interval/correction, 600 mm safe distance, 250 mm emergency distance, at most 5 candidates, 10 corrections per candidate, 2000 mm range cap and 400 mm clearance. Use the existing AVOID AOA start/stop distance, sample-age, yaw and low-speed advance defaults where applicable, but define AVOID2-only values independently. Retain all coarse samples, sort only safe candidates by absolute AOA error, then distance and previous safe side. Any invalid local sample rejects that candidate. Only a fully validated candidate can start a turn.
- [x] **Step 4: Run the policy test and verify it passes.** Re-run the command from Step 2; expected exit code 0.

### Task 2: Add the runtime adapter and connect shared sensors

**Files:**
- Create: `src/control/ax_follow_avoid2.h`
- Create: `src/control/ax_follow_avoid2.c`
- Modify: `src/app/ax_robot.c`

**Interfaces:**
- `void AX_FOLLOW_AVOID2_Step(void)` runs only for `AX_PS2_INPUT_FOLLOW_AVOID2`.
- `void AX_FOLLOW_AVOID2_Reset(void)` clears AVOID2 policy and follow-distance hysteresis on exit.
- The adapter uses `AX_AOA_GetLatest`, `AX_RANGE_PAN_GetAvoidSample`, `AX_RANGE_PAN_AvoidRequestAngle`, `AX_ROBOT_SetTargetIfActive`, `R_Imu.GYRO_Z`, and `R_Vel.RT_IX`, matching the existing AVOID controller's input path.

- [x] **Step 1: Add an integration test or compile-time harness** proving the adapter maps AOA freshness, sample angle/status/sequence/timestamps, gyro and encoder speed into `AX_AVOID2_Input`; prove pan and chassis outputs come only from the AVOID2 result.
- [x] **Step 2: Run the focused host test and verify it fails** before adding the adapter.
- [x] **Step 3: Implement the adapter** with independent policy/follow state. Enable AVOID pan ownership through the existing shared range-pan scheduler; request the policy angle every tick, use the normal 250 ms settle for scan/turn checks, and let the common scheduler discard samples from old angles. On chassis mismatch or invalid AOA, request stop and reset the policy. Do not alter `ax_range_pan` unless the integration test demonstrates the existing API cannot express a required behavior.
- [x] **Step 4: Run focused integration tests and verify they pass.**

### Task 3: Add PS2 mode selection and route dispatch

**Files:**
- Modify: `src/control/ax_ps2_mode.h`
- Modify: `src/control/ax_ps2_mode.c`
- Modify: `src/control/ax_control.c`
- Modify: `src/app/ax_robot.c`
- Modify: `tests/test_ps2_mode.c`

**Interfaces:**
- Add `AX_PS2_MODE_FOLLOW_AVOID2` and `AX_PS2_INPUT_FOLLOW_AVOID2` after the existing AVOID values.
- SELECT release cycles through `DRIVE`, `FOLLOW`, `FOLLOW_AVOID`, `FOLLOW_AVOID2`, `SCAN`.
- `AX_CTL_Ps2` stops the chassis and returns the new action; `Robot_Task` enables shared avoid sensor ownership for either avoid mode, dispatches the matching step function and resets the inactive controller.

- [x] **Step 1: Extend `tests/test_ps2_mode.c`** to assert the complete five-mode cycle, held-select behavior, invalid input reset and new mode action.
- [x] **Step 2: Run the mode test and verify it fails** on the current four-mode cycle.

  Run: `cc -std=c99 -Wall -Wextra -Werror -Isrc/control tests/test_ps2_mode.c src/control/ax_ps2_mode.c -o /tmp/test_ps2_mode && /tmp/test_ps2_mode`

- [x] **Step 3: Implement mode enum/action, stop behavior and app dispatch.** Preserve one bottom-chassis command owner per active mode. Add the OLED label `Mode: AVOID2`.
- [x] **Step 4: Run mode and policy tests; verify both pass.**

### Task 4: Document the new selectable mode and run regression checks

**Files:**
- Modify: `README.md`
- Modify: `docs/superpowers/specs/2026-10-23-follow-avoid2-design.md`

- [x] **Step 1: Update README** with the five-mode cycle, AVOID2 workflow and separate config, clearly distinguishing existing `FOLLOW_AVOID` from `FOLLOW_AVOID2`.
- [x] **Step 2: Run all relevant host tests**, including `test_follow_avoid2_policy`, `test_ps2_mode`, `test_follow_avoid_policy`, `test_follow_avoid_sweep`, `test_follow_avoid_diag`, and `test_range_pan` using the repository's established commands.
- [x] **Step 3: Build firmware** with `/home/fhc/.platformio/penv/bin/pio run`; expected `[SUCCESS]`.
- [x] **Step 4: Run `git diff --check`**; expected no whitespace errors. Review the final diff to confirm only the new mode and its docs changed.
