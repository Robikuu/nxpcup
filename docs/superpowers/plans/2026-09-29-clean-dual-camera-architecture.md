# Clean Dual-Camera Architecture & Vector Rewrite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Completely replace the bloated 939-line `vectors.c` with a clean, ~150-line geometric vector engine (Top Camera: closest left/right lane centering; Front Camera: length-weighted horizon angle with direct $>30^\circ$ corner trigger) and streamline `main.c` for 100% straight sprint and immediate corner braking/differential pivot.

**Architecture:** 
- Top camera identifies closest left line ($x < 39$) and closest right line ($x > 39$) near bumper ($y \ge 20$) and centers between them.
- Front camera calculates length-weighted horizon angle $\theta_{\text{front}}$. If $|\theta_{\text{front}}| > 30^\circ$, it activates pre-steer, drops speed to $50\%$, and applies inside-wheel differential pivot ($32\%$). Otherwise, 100% straightaway speed is commanded.

**Tech Stack:** C99, NXP MCXN947 Cortex-M33, MCUXpresso GCC Toolchain, Dual Pixy2 Cameras.

---

### Task 1: Clean Interface in `source/vectors.h`

**Files:**
- Modify: [`source/vectors.h`](file:///d:/nxpcup/source/vectors.h)

- [ ] **Step 1: Define `steering_result_t` and `horizon_result_t`**
  ```c
  typedef struct {
      float   cte;             // Cross-track error in pixels (center_x - 39.0f)
      float   heading_deg;     // Road heading angle in degrees relative to vertical
      uint8_t left_found;      // 1 if closest left line detected
      uint8_t right_found;     // 1 if closest right line detected
      uint8_t found;           // Total lines found (0, 1, or 2)
  } steering_result_t;

  typedef struct {
      float   horizon_angle_deg; // Length-weighted average forward road angle (-90 to +90 deg)
      uint8_t is_sharp_corner;   // 1 if |horizon_angle_deg| > 30.0f
      uint8_t found;             // Number of valid horizon vectors detected
  } horizon_result_t;

  void compute_top_camera_lane(const uint16_t *vectors, size_t num_vectors, steering_result_t *out);
  void compute_front_camera_horizon(const uint16_t *vectors, size_t num_vectors, horizon_result_t *out);
  ```

- [ ] **Step 2: Commit `source/vectors.h`**
  ```bash
  git add source/vectors.h
  git commit -m "refactor: simplify vectors.h interface for clean dual camera architecture"
  ```

---

### Task 2: Implement Clean Geometric Engine in `source/vectors.c`

**Files:**
- Rewrite: [`source/vectors.c`](file:///d:/nxpcup/source/vectors.c)

- [ ] **Step 1: Implement `compute_top_camera_lane()`**
  - Loop over vectors, find near point ($y_{\text{near}} = \max(y_0, y_1)$).
  - Filter: $y_{\text{near}} \ge 18$, $|\Delta y| \ge 4$.
  - Closest left line: $\max(x_{\text{near}})$ for $x_{\text{near}} < 39.0f$.
  - Closest right line: $\min(x_{\text{near}})$ for $x_{\text{near}} > 39.0f$.
  - If both found: `center_x = (l_x + r_x) / 2.0f; heading = (l_ang + r_ang) / 2.0f;`
  - If single line found: offset by `TRACK_HALF_WIDTH_PX`.
  - Compute `cte = center_x - 39.0f;`.

- [ ] **Step 2: Implement `compute_front_camera_horizon()`**
  - Loop over front camera vectors.
  - Compute $\Delta x = x_{\text{far}} - x_{\text{near}}$, $\Delta y = y_{\text{near}} - y_{\text{far}}$.
  - Compute length $L = \sqrt{\Delta x^2 + \Delta y^2}$ and angle $\theta = \operatorname{atan2}(\Delta x, \Delta y) \times \frac{180}{\pi}$.
  - Compute weighted average: $\theta_{\text{front}} = \sum (L \times \theta) / \sum L$.
  - Flag `is_sharp_corner = (fabsf(theta_front) > FRONT_CORNER_ANGLE_THRESH_DEG);`.

- [ ] **Step 3: Commit `source/vectors.c`**
  ```bash
  git add source/vectors.c
  git commit -m "feat: complete rewrite of vectors.c with clean geometric logic"
  ```

---

### Task 3: Update Racing Parameters in `include/Config.h`

**Files:**
- Modify: [`include/Config.h`](file:///d:/nxpcup/include/Config.h)

- [ ] **Step 1: Define clear constants for the new architecture**
  ```c
  #define SPEED_STRAIGHT               100.0f
  #define SPEED_OUTSIDE_CORNER         50.0f
  #define SPEED_INSIDE_MIN             32.0f
  #define FRONT_CORNER_ANGLE_THRESH_DEG 30.0f
  #define TRACK_HALF_WIDTH_PX          25.0f
  #define DIFFERENTIAL_DEADZONE_DEG    5.0f
  ```

- [ ] **Step 2: Commit `include/Config.h`**
  ```bash
  git add include/Config.h
  git commit -m "feat: update Config.h for 30 deg corner trigger and clean speed targets"
  ```

---

### Task 4: Streamline Main Loop in `source/main.c`

**Files:**
- Modify: [`source/main.c`](file:///d:/nxpcup/source/main.c)

- [ ] **Step 1: Replace vector calls in camera polling**
  Call `compute_top_camera_lane()` on `vectors_top` and `compute_front_camera_horizon()` on `vectors_front`.

- [ ] **Step 2: Implement direct steering and speed governance**
  - If `horizon.is_sharp_corner`:
    - Target speed = `g_params.speed_outside_corner` ($50\%$)
    - Inside wheel pivot drag = `g_params.speed_inside_min` ($32\%$)
    - Steer = `steer_top + 0.40f * (horizon.horizon_angle_deg - sign * 30.0f)`
  - Else:
    - Target speed = `g_params.speed_straight` ($100\%$)
    - Steer = `steer_top` (centered between closest lines)

- [ ] **Step 3: Update telemetry stream**
  Log `L:%u/%u`, `CTE`, `Head`, `FrontAngle`, `Steer`, `SpeedL`, `SpeedR`, `CornerFlag`.

- [ ] **Step 4: Commit `source/main.c`**
  ```bash
  git add source/main.c
  git commit -m "refactor: simplify main.c control loop to use direct 30 deg horizon trigger"
  ```

---

### Task 5: Compilation, Sync & Verification

**Files:**
- Output: `d:\nxpcup\Debug\nxpcup.axf`

- [ ] **Step 1: Sync all files to MCUXpresso workspace**
- [ ] **Step 2: Run make and verify 0 errors, 0 warnings**
- [ ] **Step 3: Copy binary back to `d:\nxpcup\Debug\`**
