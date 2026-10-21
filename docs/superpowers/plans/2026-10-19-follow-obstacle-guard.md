# FOLLOW Obstacle Guard Implementation Plan

> Historical plan: FOLLOW was later decoupled from sonar and pan at the user's request. See the current README and source for active behavior.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a testable, independent front ultrasonic guard to PS2 FOLLOW.

**Architecture:** The range/pan application publishes only fresh, centered measurements. A pure guard module filters the AOA follow target before `ax_follow` performs the sole chassis target submission.

**Tech Stack:** STM32F407 SPL, FreeRTOS, C99 host tests, PlatformIO.

**Spec:** `docs/superpowers/specs/2026-10-19-follow-obstacle-guard-design.md`

## Global Constraints

- Stop below 500 mm; resume at or above 650 mm.
- Limit FOLLOW forward travel to 300 mm/s during initial hardware validation.
- Stop on samples older than 250 ms, invalid echoes, or a moving/noncentered pan.
- Do not add reverse motion or active route planning.
- Current Git repository has no initial commit; do not make a history-wide commit as part of this change.

## Review Focus

- Mode changes while an echo is pending must not publish a sample for the wrong pan angle.
- A USB pan command during FOLLOW must not move the sensor away from front.
- A stale earlier `READY` reading must not survive a later timeout.
- AOA timeout must still stop the chassis.
- An invalid reading must not release an already blocked guard.

---

### Task 1: Pure obstacle guard

**Files:** Create `src/control/ax_obstacle_guard.h`, `src/control/ax_obstacle_guard.c`, `tests/test_obstacle_guard.c`; modify `include/config/openctr_follow_config.h`.

**Interfaces:** `AX_OBSTACLE_GuardStep(state, config, range_valid, distance_mm, desired)` produces a filtered `AX_FOLLOW_Target`.

- [x] Write tests for close stop, hysteresis, invalid input, forward speed cap, and no reverse.
- [x] Run the host test and observe failure because the module is absent.
- [x] Implement the smallest pure guard that passes.
- [x] Run the guard test again and verify success.

### Task 2: Publish valid front range

**Files:** Modify `src/app/ax_range_pan.h`, `src/app/ax_range_pan.c`; create `tests/test_range_pan.c` and host stubs if needed.

**Interfaces:** `AX_RANGE_PAN_SetFollowActive(active)` claims/releases center-facing pan; `AX_RANGE_PAN_GetFrontDistance(&mm, max_age_ms)` returns only a fresh valid centered measurement.

- [x] Write tests for startup, pan settling, fresh range, timeout, staleness, and USB pan ownership.
- [x] Run the test and observe expected missing-interface failure.
- [x] Implement sample publication and angle ownership.
- [x] Run the range test again and verify success.

### Task 3: Integrate in FOLLOW

**Files:** Modify `src/control/ax_follow.c`, `src/control/ax_control.c`, `src/app/ax_robot.c`, `tests/test_follow_step.c` and stubs.

**Interfaces:** FOLLOW calls the pure guard before its single target submission; mode transitions activate/deactivate front range ownership.

- [x] Extend the FOLLOW step test for clear, blocked, invalid, and AOA timeout cases; observe failure.
- [x] Wire the guard and pan ownership into the existing control flow.
- [x] Run FOLLOW and guard tests to verify success.

### Task 4: Documentation and verification

**Files:** Modify `README.md`.

- [x] Document guarded FOLLOW behavior and current hardware limits.
- [x] Run every host test in `tests/` and inspect failures.
- [x] Run `~/.platformio/penv/bin/pio run` and inspect the full build result.
- [x] Review all changed code for sample freshness, single-writer control, and mode transitions.
