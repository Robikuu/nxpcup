# Racing Corner Approach with Straight-Line Threshold Pre-Braking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement Option 1: Uncapped 100% straightaway sprint power with an active straight-line threshold brake pulse (3 ticks / 60 ms) upon front-camera turn preview, transitioning into recalibrated DRV8871 corner apex setpoints (58% outside / 38% inside) and 6° differential deadband.

**Architecture:** Remove output `drive_scale` so 100% commanded duty reaches motors directly. In `main.c`, monitor front-camera lookahead urgency while sprinting straight. When confirmed for $\ge 2$ ticks, trigger a 3-tick `HbridgeBrake()` pulse in a straight line, latched directly to corner speed setpoints on release, with immediate differential pivot drag.

**Tech Stack:** C99, NXP MCXN947 Cortex-M33, MCUXpresso IDE GCC Toolchain, DRV8871 H-Bridges.

---

### Task 1: Revert Output Drive Scaling in `source/hbridge.c`

**Files:**
- Modify: [`source/hbridge.c:45-55`](file:///d:/nxpcup/source/hbridge.c#L45-L55)

- [ ] **Step 1: Remove `g_params.drive_scale` multiplication in `HbridgeSpeed`**
  Directly pass clamped `speed_left` and `speed_right` into duty calculations without scaling.

- [ ] **Step 2: Commit `source/hbridge.c`**
  ```bash
  git add source/hbridge.c
  git commit -m "refactor: remove drive_scale multiplication to restore 100% straight power"
  ```

---

### Task 2: Update Configuration Constants in `include/Config.h`

**Files:**
- Modify: [`include/Config.h:26-36`](file:///d:/nxpcup/include/Config.h#L26-L36)

- [ ] **Step 1: Update corner speeds and add Pre-Brake definitions**
  ```c
  #define SPEED_STRAIGHT               100.0f
  #define SPEED_OUTSIDE_CORNER         58.0f
  #define SPEED_INSIDE_MIN             38.0f
  #define DIFFERENTIAL_DEADZONE_DEG    6.0f

  #define PREBRAKE_PULSE_TICKS         3U
  #define PREBRAKE_MIN_SPEED           80.0f
  #define PREBRAKE_MAX_HEADING_DEG     10.0f
  #define PREBRAKE_MAX_STEER_DEG       12.0f
  #define PREBRAKE_LOOKAHEAD_THRESH    0.35f
  #define PREBRAKE_CONFIRM_TICKS       2U
  ```

- [ ] **Step 2: Commit `include/Config.h`**
  ```bash
  git add include/Config.h
  git commit -m "feat: define corner apex setpoints and prebrake pulse constants"
  ```

---

### Task 3: Implement Pre-Brake State & Trigger Logic in `source/main.c`

**Files:**
- Modify: [`source/main.c:116-160`](file:///d:/nxpcup/source/main.c#L116-L160)
- Modify: [`source/main.c:560-638`](file:///d:/nxpcup/source/main.c#L560-L638)

- [ ] **Step 1: Add pre-brake tracking variables**
  ```c
  static uint8_t s_prebrake_confirm_count = 0;
  static uint8_t s_prebrake_active_ticks  = 0;
  static bool    s_prebrake_armed         = true;
  ```

- [ ] **Step 2: Add trigger detection in normal racing state**
  Check conditions: high speed, straight alignment, confirmed lookahead urgency $\ge 2$ ticks. Trigger active brake pulse.

- [ ] **Step 3: Execute brake pulse and seed corner speed on exit**
  When `s_prebrake_active_ticks > 0`, call `HbridgeBrake()`, clamp applied speed to `SPEED_OUTSIDE_CORNER`, and set telemetry state indicator to `'B'`.

- [ ] **Step 4: Commit `source/main.c`**
  ```bash
  git add source/main.c
  git commit -m "feat: implement straight-line threshold pre-brake trigger and execution"
  ```

---

### Task 4: Compilation, Sync & Verification

**Files:**
- Sync: `source/*`, `include/*` to `D:\robotica\workspace\nxpcup-ciorba_curata`
- Output: `d:\nxpcup\Debug\nxpcup.axf`

- [ ] **Step 1: Sync files to workspace**
- [ ] **Step 2: Build with MCUXpresso make tool**
- [ ] **Step 3: Copy binary back to `d:\nxpcup\Debug\`**
- [ ] **Step 4: Confirm 0 errors, 0 warnings**
