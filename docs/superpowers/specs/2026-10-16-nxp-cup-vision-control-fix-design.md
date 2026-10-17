# NXP Cup Dual-Camera Vision, Control & Driver Upgrade Design Spec

**Date:** 2026-10-16  
**Target Platform:** NXP FRDM-MCXN947 (ARM Cortex-M33 @ 150 MHz)  
**Architecture:** Dual Pixy2 I2C eDMA (Top Near-Field + Front Horizon) + DRV8871 Dual H-Bridge + ESP32 UART/Wi-Fi Telemetry

---

## 1. Overview & Goals

Upgrade the NXP Cup autonomous car firmware across three modules (**Vision & Perception**, **Steering & Drivetrain Control**, and **Peripherals & Reliability**) while keeping 100% compatibility with the existing Wi-Fi tuning workflow and telemetry format.

All 7 design refinements from review are incorporated:
1. **Fast Deceleration Ramp Preserved:** `DECEL_RAMP_STEP = 30.0f` (`%/tick`, 40–50 ms braking/pivot response) and `ACCEL_RAMP_STEP = 6.0f` (`%/tick`, smooth traction-preserving corner exit).
2. **Direct `drive_scale` Multiplier in Blind-Gap Timer:** `g_params.drive_scale` is a `0.10f..1.00f` multiplier (`DEFAULT_DRIVE_SCALE = 1.00f`), so `current_spd = fmaxf(applied_speed_left, applied_speed_right) * g_params.drive_scale` (no `/ 100.0f`).
3. **Unblocked `kd_s` / `kd_c` in `wifi_config.c` + Filtered Derivative (`d_cte`):** Remove `kd_straight`/`kd_s` and `kd_corner`/`kd_c` from the inactive-parameter error block in `source/wifi_config.c`, display them in `status`, compute `d_cte_rate = (filtered_cte - prev_filtered_cte) / g_params.dt` (in pixels/sec), and low-pass filter it with `g_params.alpha` (`0.25f`) before multiplying by `active_kd`.
4. **Conditional Single-Line Perspective Compensation in Front Camera:** Subtract the `±18.0f` natural perspective convergence baseline in `compute_front_camera_horizon()` **only** when `valid_count == 1` and the vector's `x_far` remains on the same side of `PIXY_CENTER_X (39.5f)` as `x_near`. If `x_far` crosses `PIXY_CENTER_X`, do not subtract `18.0f` so genuine single-line sharp corners still exceed `FRONT_CORNER_ANGLE_THRESH_DEG (30.0f)`.
5. **Non-Dropping Feedforward During Top-Camera Dropout (`sr.found == 0`):** Track `last_valid_steer_top` whenever `sr.found > 0`. When `sr.found == 0 && !s_intersection_ahead`, compute `candidate_steer = last_valid_steer_top + active_feedforward` and take the larger-magnitude turn command between `candidate_steer` and `last_steer_cmd`.
6. **Preserve `uint16_t vectors[4 * i]` Array Layout:** Keep `pixy_get_vectors(pixy_t *cam, uint16_t *vectors, size_t max_vectors, size_t *num_vectors)` writing `[x0, y0, x1, y1]` so `g_dump_vectors` in `source/main.c` and the signatures in `include/pixy.h` and `source/vectors.h` remain unchanged.
7. **Exact Repository File Paths & Symbol Names:**
   - Headers in `include/`: `include/Config.h`, `include/pixy.h`, `include/hbridge.h`, `include/servo.h`, `include/esc.h`
   - Headers in `source/`: `source/vectors.h`, `source/wifi_config.h`, `source/control_theory.h`, `source/utils.h`
   - Board headers in `board/`: `board/peripherals.h`
   - Exact symbols: `reset_steering_tracking()`, `is_intersection_ahead`, `OFF_TRACK_FAILSAFE_TICKS`, `ACCEL_RAMP_STEP`, `DECEL_RAMP_STEP`, `g_params.kd_straight`, `g_params.kd_corner`.

---

## 2. Module 1: Vision & Perception (`source/vectors.c`, `source/vectors.h`, `include/Config.h`)

### 2.1 Bottom-Edge Chassis/Wheel Reflection Exclusion (`compute_top_camera_lane`)
- Use `TOP_CAM_EXCLUDE_Y_MIN (38U)`, `TOP_CAM_EXCLUDE_X_MIN (18U)`, and `TOP_CAM_EXCLUDE_X_MAX (62U)` from `include/Config.h`.
- Any vector with `y_near >= (float)TOP_CAM_EXCLUDE_Y_MIN` and `x_near >= (float)TOP_CAM_EXCLUDE_X_MIN && x_near <= (float)TOP_CAM_EXCLUDE_X_MAX` is rejected as a chassis/wheel reflection **unless** it extends well up into the roadway (`y_far < 28.0f` and `dy >= 16.0f`).

### 2.2 Lookahead Reference Row Projection (`Y_REF = 35.0f`)
- For each candidate vector oriented from bottom (`x_near, y_near`) to top (`x_far, y_far`), compute its projected X position at fixed lookahead row `Y_REF = 35.0f`:
  ```c
  float slope = (x_far - x_near) / (y_near - y_far); // change in X per pixel moving up (toward smaller Y)
  float x_proj = x_near + slope * (y_near - 35.0f);
  ```
- Clamp `x_proj` to `[-20.0f, 100.0f]`. All left/right lane boundary comparisons and CTE calculations use `x_proj` so that vectors starting at different vertical rows are evaluated at the exact same physical lookahead distance.

### 2.3 Stateful Temporal Left/Right Boundary Tracking (`reset_steering_tracking`, `compute_top_camera_lane`)
- Maintain static tracking state in `source/vectors.c`:
  - `static uint8_t s_has_tracking = 0;`
  - `static float s_last_left_x = 14.5f;`
  - `static float s_last_right_x = 64.5f;`
  - `static uint16_t s_blind_frames = 0;`
- `reset_steering_tracking(void)` resets `s_has_tracking = 0`, `s_last_left_x = PIXY_CENTER_X - TRACK_HALF_WIDTH_PX`, `s_last_right_x = PIXY_CENTER_X + TRACK_HALF_WIDTH_PX`, `s_blind_frames = 0`.
- **Candidate Classification:**
  - When `s_has_tracking && s_blind_frames < 8`:
    - Compute distance to tracked boundaries: `d_left = fabsf(x_proj - s_last_left_x)` and `d_right = fabsf(x_proj - s_last_right_x)`.
    - A candidate belongs to the **Left** boundary if `d_left <= d_right && d_left < 24.0f` (even if `x_proj >= PIXY_CENTER_X`), or if `!s_has_tracking && x_proj < PIXY_CENTER_X`.
    - A candidate belongs to the **Right** boundary if `d_right < d_left && d_right < 24.0f` (even if `x_proj <= PIXY_CENTER_X`), or if `!s_has_tracking && x_proj >= PIXY_CENTER_X`.
    - Fallback: if a newly appearing line is more than `24.0f` px from both tracked positions, classify by `x_proj < PIXY_CENTER_X` vs `x_proj >= PIXY_CENTER_X`.
- **Minimum Lane Width Validation (`MIN_LANE_WIDTH_PX = 32.0f`):**
  - If both `left_found` and `right_found` are set, check `best_right_x - best_left_x < MIN_LANE_WIDTH_PX`.
  - If too narrow (two segments of the same line or a noisy inner reflection), keep the side closer to its prior tracked position (or the longer vector) and clear the duplicate side.
- **Perpendicular Single-Line Offset:**
  - When only 1 boundary is found (`out->found == 1`), compute heading in radians `theta = out->heading_deg * (PI / 180.0f)` and perpendicular half-width:
    ```c
    float cos_h = cosf(theta);
    if (cos_h < 0.55f) cos_h = 0.55f;
    float perp_half_width = TRACK_HALF_WIDTH_PX / cos_h;
    ```
  - `center_x = best_left_x + perp_half_width` (if left found) or `best_right_x - perp_half_width` (if right found).
  - Update `s_last_left_x`, `s_last_right_x`, `s_has_tracking = 1`, `s_blind_frames = 0`.

### 2.4 Front Camera Horizon & 90° Corner Pre-Steer (`compute_front_camera_horizon`)
- **Preserve `horizon_angle_deg` on 90° Corners:**
  - When `out->has_transverse_tape` is true:
    - If `has_exit_line` is true (`out->is_intersection_ahead = 1U`), set `out->is_sharp_corner = 0U` and `out->horizon_angle_deg = 0.0f` so the car drives straight through the intersection.
    - If `has_exit_line` is false (`out->is_sharp_corner = 1U`), **do not zero `horizon_angle_deg`**. Instead:
      - If longitudinal approach vectors exist (`valid_count > 0` and `sum_len > 0.0f`), compute `out->horizon_angle_deg = sum_weighted_angle / sum_len`.
      - If `fabsf(out->horizon_angle_deg) < FRONT_CORNER_ANGLE_THRESH_DEG`, use the transverse wall's slope and horizontal midpoint (`trans_mid_x`) relative to `PIXY_CENTER_X` to set `out->horizon_angle_deg` to at least `±FRONT_CORNER_ANGLE_THRESH_DEG` in the direction the corner opens.
- **Single-Line Perspective Convergence Compensation:**
  - When `valid_count == 1` and `!out->has_transverse_tape`:
    - If the single vector starts on the left (`single_x_near < PIXY_CENTER_X`) and its far endpoint stays on the left (`single_x_far < PIXY_CENTER_X`), subtract the natural straight-line convergence baseline (`+18.0f` deg, clamped toward `0.0f`).
    - If the single vector starts on the right (`single_x_near >= PIXY_CENTER_X`) and its far endpoint stays on the right (`single_x_far >= PIXY_CENTER_X`), add `+18.0f` deg toward `0.0f`.
    - If `single_x_far` crosses past `PIXY_CENTER_X` onto the opposite side, do **not** subtract `18.0f`—it is a real curve crossing the center of the camera.

---

## 3. Module 2: Steering & Drivetrain Control (`source/main.c`, `include/Config.h`, `source/wifi_config.c`)

### 3.1 PD Steering Controller & Reduced EMA Lag
- Reduce EMA filter lag in `source/main.c`:
  ```c
  filtered_cte = 0.75f * sr.cte + 0.25f * filtered_cte;
  filtered_heading = 0.75f * sr.heading_deg + 0.25f * filtered_heading;
  ```
- Compute filtered CTE derivative in pixels/second using `g_params.dt` and `g_params.alpha`:
  ```c
  float raw_d_cte = (filtered_cte - prev_filtered_cte) / fmaxf(g_params.dt, 0.005f);
  prev_filtered_cte = filtered_cte;
  filtered_d_cte = g_params.alpha * raw_d_cte + (1.0f - g_params.alpha) * filtered_d_cte;
  ```
- Apply `HEADING_DEADBAND_DEG` (`2.0f` deg) when on a straightaway (`!horizon.is_sharp_corner && s_corner_blend < 0.20f`) so pixel quantization does not cause micro-hunting.
- Compute `steer_top`:
  - In discrete mode (`blend_enable == 0`):
    ```c
    float active_kd = horizon.is_sharp_corner ? g_params.kd_corner : g_params.kd_straight;
    float steer_top = g_params.k_cte * filtered_cte + g_params.k_heading * eff_heading + active_kd * filtered_d_cte;
    ```
  - In continuous blend mode (`blend_enable == 1`):
    ```c
    float active_k_cte = g_params.k_cte + s_corner_blend * (g_params.k_cte_corner - g_params.k_cte);
    float active_k_heading = g_params.k_heading + s_corner_blend * (g_params.k_heading_corner - g_params.k_heading);
    float active_kd = g_params.kd_straight + s_corner_blend * (g_params.kd_corner - g_params.kd_straight);
    float steer_top = active_k_cte * filtered_cte + active_k_heading * eff_heading + active_kd * filtered_d_cte;
    ```
- Save `last_valid_steer_top = steer_top`.
- **Top-Camera Dropout (`sr.found == 0 && !s_intersection_ahead`):**
  - Compute `candidate_steer = last_valid_steer_top + active_feedforward`.
  - If `fabsf(candidate_steer) > fabsf(last_steer_cmd)` (or if `last_steer_cmd` and `candidate_steer` have the same sign and `candidate_steer` turns harder), set `steer_cmd = candidate_steer`; otherwise hold `steer_cmd = last_steer_cmd`.
- **Runtime Trim Clamp Bounds:**
  - Replace `STEER_CMD_MAX_RIGHT` and `STEER_CMD_MAX_LEFT` in `main.c` with runtime expressions using `g_params.steering_offset`:
    ```c
    float max_right_delta = STEERING_LIMIT_RIGHT - g_params.steering_offset;
    float max_left_delta  = STEERING_LIMIT_LEFT  - g_params.steering_offset;
    ```

### 3.2 Differential Braking, Ramps & Intersection Latch
- **Fix `blend=1` Inside-Wheel Lockup:**
  - Replace `float diff_mult = steer_norm + s_corner_blend * (1.0f - steer_norm);` with:
    ```c
    float corner_authority = 0.25f + 0.75f * s_corner_blend;
    float diff_mult = steer_norm * corner_authority;
    ```
- **Reduce Straightaway Differential Drag in Discrete Mode (`blend=0`):**
  - Scale `diff_drag` by `0.35f` when `!horizon.is_sharp_corner && fabsf(steer_cmd) < 22.0f` so minor straight-line corrections do not drag a rear wheel and induce yaw oscillation, while full differential drag engages immediately when `horizon.is_sharp_corner` or `fabsf(steer_cmd) >= 22.0f`.
- **Speed Ramp Defaults (`include/Config.h`):**
  - `DECEL_RAMP_STEP = 30.0f` (`%/tick` — fast corner-entry braking and inside-wheel pivot lock in ~2 ticks / 40 ms).
  - `ACCEL_RAMP_STEP = 6.0f` (`%/tick` — smooth corner-exit throttle ramp over ~160 ms).
- **Scale-Aware Blind-Gap Timer (`source/main.c`):**
  - `float current_spd = fmaxf(applied_speed_left, applied_speed_right) * g_params.drive_scale;`
- **Intersection Latch Hysteresis (`source/main.c`):**
  - Require 3 consecutive ticks of `horizon.is_sharp_corner && !horizon.is_intersection_ahead` (`s_sharp_cancel_count >= 3`) before clearing an active `s_intersection_ahead` latch.

---

## 4. Module 3: Peripherals & Reliability (`source/pixy.c`, `source/wifi_config.c`, `source/utils.c`, `board/peripherals.h`)

1. **`source/pixy.c` Header Validation & 64-Byte Read:**
   - Read `64` bytes instead of `100` bytes in `pixy_get_vectors()` (6-byte header + up to 9 vectors = 60 bytes).
   - Validate `buf[0] == 0xAF && buf[1] == 0xC1 && buf[2] == 49` (or offset by 1 byte if Pixy2 prepends a single dummy byte) before parsing `buf[3]` (`payload_len`). On header mismatch, set `*num_vectors = 0` and return `kStatus_Fail`.
2. **`source/wifi_config.c` Unblock `kd_s`/`kd_c` & Expand `esp_printf` Buffer:**
   - Unblock `kd_straight`/`kd_s` and `kd_corner`/`kd_c` in `command_execute()`, and include `kd_s=` and `kd_c=` in `status` output.
   - Increase `esp_printf` buffer from `char buf[128]` to `char buf[384]`.
3. **`board/peripherals.h`, `source/main.c`, and `source/utils.c`:**
   - Fix `INT_4_IRQN` in `board/peripherals.h` to `EDMA_0_CH3_IRQn`.
   - Enable and set NVIC priority `2U` for `LP_FLEXCOMM2_IRQn`, `EDMA_0_CH0_IRQn`, and `EDMA_0_CH1_IRQn` alongside `LP_FLEXCOMM3_IRQn`, `EDMA_0_CH2_IRQn`, and `EDMA_0_CH3_IRQn` in `source/main.c`.
   - Add a 3-tick (60 ms) debounce filter in `button_pressed_edge()` in `source/utils.c`.
