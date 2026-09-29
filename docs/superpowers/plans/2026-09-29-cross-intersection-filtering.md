# Gated Cross-Intersection Filtering Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement horizontal line rejection in `vectors.c` and a full-speed straight-through intersection transit state machine in `main.c` to navigate standard $90^\circ$ cross-junctions without steering pull or speed reduction.

**Architecture:**
- `vectors.c`: Vector angle filter ($|\text{angle}| > 60^\circ$) discards perpendicular crossing lines from lane boundary selection and flags `has_transverse_tape`.
- `main.c`: Gated state machine enters `MACRO_STATE_INTERSECTION` on transverse tape while aligned straight, holds $0^\circ$ steering and $100\%$ full speed across the box for 160–440 ms, and exits when two longitudinal lines are reacquired.

**Tech Stack:** C99, NXP MCXN947 Cortex-M33, MCUXpresso GCC Toolchain, Dual Pixy2.

---

### Task 1: Update Interface in `source/vectors.h`

**Files:**
- Modify: [`source/vectors.h`](file:///d:/nxpcup/source/vectors.h)

- [ ] **Step 1: Add `has_transverse_tape` field to `steering_result_t`**
  ```c
  typedef struct {
      float   cte;
      float   heading_deg;
      uint8_t left_found;
      uint8_t right_found;
      uint8_t has_transverse_tape;
      uint8_t found;
  } steering_result_t;
  ```

- [ ] **Step 2: Commit `source/vectors.h`**
  ```bash
  git add source/vectors.h
  git commit -m "feat: add has_transverse_tape to steering_result_t"
  ```

---

### Task 2: Implement Transverse Line Rejection in `source/vectors.c`

**Files:**
- Modify: [`source/vectors.c`](file:///d:/nxpcup/source/vectors.c)

- [ ] **Step 1: Add angle filter in `compute_top_camera_lane()`**
  Calculate `angle = atan2f(dx, dy) * (180.0f / M_PI)`.
  If `fabsf(angle) > 60.0f`:
  - Flag `out->has_transverse_tape = 1;` if `fabsf(dx) >= 12.0f && y_near >= 16.0f`.
  - Execute `continue;` to reject the vector from being chosen as left/right boundary line.

- [ ] **Step 2: Commit `source/vectors.c`**
  ```bash
  git add source/vectors.c
  git commit -m "feat: reject horizontal crossbar vectors in compute_top_camera_lane"
  ```

---

### Task 3: Implement Full-Speed Straight-Through State in `source/main.c`

**Files:**
- Modify: [`source/main.c`](file:///d:/nxpcup/source/main.c)

- [ ] **Step 1: Define macro states and tracking variables**
  ```c
  #define MACRO_STATE_NORMAL       0
  #define MACRO_STATE_INTERSECTION 1
  uint8_t  macro_state                   = MACRO_STATE_NORMAL;
  uint32_t intersection_ticks           = 0;
  uint32_t intersection_reacquire_streak = 0;
  ```

- [ ] **Step 2: Implement state transition and control logic**
  - Trigger into `MACRO_STATE_INTERSECTION` on `sr.has_transverse_tape` with straight alignment.
  - In intersection: force `steer_cmd = 0.0f`, `target_speed = g_params.speed_straight` (100%), bypass corner braking.
  - Exit after min 8 ticks on `sr.found == 2` (streak $\ge 2$) or 22 ticks timeout.

- [ ] **Step 3: Update telemetry stream**
  Output `ST: I` when in intersection.

- [ ] **Step 4: Commit `source/main.c`**
  ```bash
  git add source/main.c
  git commit -m "feat: implement full speed straight through intersection transit in main.c"
  ```

---

### Task 4: Compilation, Sync & Verification

**Files:**
- Output: `d:\nxpcup\Debug\nxpcup.axf`

- [ ] **Step 1: Sync all files to MCUXpresso workspace**
- [ ] **Step 2: Build with make tool and verify 0 errors, 0 warnings**
- [ ] **Step 3: Copy binary back to `d:\nxpcup\Debug\`**
