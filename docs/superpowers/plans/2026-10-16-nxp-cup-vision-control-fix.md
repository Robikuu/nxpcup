# NXP Cup Vision, Control & Driver Upgrade Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the 18 vision, steering/drivetrain control, and peripheral driver bugs in the FRDM-MCXN947 dual-Pixy2 autonomous car firmware while preserving the existing Wi-Fi tuning interface and all 7 review corrections.

**Architecture:** Three focused modules: (1) Vision & Perception in `source/vectors.c` (bottom-edge car reflection filtering, lookahead projection to `Y_REF = 35.0f`, stateful temporal Left/Right tracking with `MIN_LANE_WIDTH_PX` validation and perpendicular single-line projection, 90° corner `horizon_angle_deg` preservation, and conditional single-line perspective compensation); (2) Control & Drivetrain in `source/main.c`, `include/Config.h`, and `source/wifi_config.c` (reduced EMA lag, filtered derivative damping via `kd_s`/`kd_c`, non-dropping feedforward during top-camera dropout, proportional `diff_mult` in `blend=1` mode, fast `DECEL_RAMP_STEP = 30.0f` and smooth `ACCEL_RAMP_STEP = 6.0f`, direct `drive_scale` multiplier in the blind-gap timer, and 3-tick hysteresis on intersection cancellation); (3) Peripherals & Reliability in `source/pixy.c`, `source/wifi_config.c`, `source/utils.c`, and `board/peripherals.h`.

**Tech Stack:** C99, NXP MCUXpresso SDK (FRDM-MCXN947 ARM Cortex-M33), host `gcc` unit test harness (`scratch/test_vision_and_control.c`), MCUXpresso cross-compiler (`arm-none-eabi-gcc`).

---

### Task 1: Create Host Unit Test Suite for Vision & Control Logic (TDD Red Phase)

**Files:**
- Create: `scratch/test_vision_and_control.c`

- [ ] **Step 1: Write the failing test suite covering all vision and control bugs**

Create `scratch/test_vision_and_control.c` to test:
1. Top camera bottom-edge wheel/chassis reflection rejection (`TOP_CAM_EXCLUDE_*` at `x=25, y=46..50`).
2. Top camera lookahead projection to `Y_REF = 35.0f` when left and right vectors start at different `y_near` rows (`y_near=50` vs `y_near=25`).
3. Top camera stateful temporal Left/Right tracking when a single left line crosses `PIXY_CENTER_X (39.5f)` to `x=43` in a right turn (must NOT swap to right boundary).
4. Top camera `MIN_LANE_WIDTH_PX (32.0f)` duplicate boundary rejection.
5. Top camera perpendicular single-line offset (`TRACK_HALF_WIDTH_PX / cos(heading)`).
6. Front camera 90° corner wall (`has_transverse_tape = 1, is_sharp_corner = 1`) preserving non-zero `horizon_angle_deg` in the turn direction.
7. Front camera single-line perspective convergence compensation (`±18°` subtracted when `x_far` stays on its own side of `39.5f`, not subtracted when `x_far` crosses `39.5f`).
8. `blend=1` differential multiplier scaling smoothly with `steer_norm` rather than locking to `1.0f` at small steering angles.

- [ ] **Step 2: Compile and run the host test against current `source/vectors.c` to verify RED failure**

Run: `gcc -Iinclude -Isource scratch/test_vision_and_control.c source/vectors.c -lm -o scratch/test_vision_and_control.exe && .\scratch\test_vision_and_control.exe`
Expected: FAIL on existing `source/vectors.c`.

---

### Task 2: Upgrade Vision & Perception (`include/Config.h` and `source/vectors.c`)

**Files:**
- Modify: `include/Config.h:41-44`
- Modify: `source/vectors.c:12-390`

- [ ] **Step 1: Update `include/Config.h` ramp defaults and `HEADING_DEADBAND_DEG`**

Update `DECEL_RAMP_STEP` to `30.0f`, `ACCEL_RAMP_STEP` to `6.0f`, and `HEADING_DEADBAND_DEG` to `2.0f` in `include/Config.h`.

- [ ] **Step 2: Implement upgraded `compute_top_camera_lane` and `compute_front_camera_horizon` in `source/vectors.c`**

Implement:
- `TOP_CAM_EXCLUDE_*` bottom-edge car reflection filter (`y_near >= TOP_CAM_EXCLUDE_Y_MIN` and `x_near` in `[TOP_CAM_EXCLUDE_X_MIN, TOP_CAM_EXCLUDE_X_MAX]`, unless `y_far < 28.0f && dy >= 16.0f`).
- Lookahead row projection to `Y_REF = 35.0f` (`x_proj = x_near + (dx / dy) * (y_near - 35.0f)`).
- Stateful temporal Left/Right tracking (`s_has_tracking`, `s_last_left_x`, `s_last_right_x`, `s_blind_frames`), `MIN_LANE_WIDTH_PX (32.0f)` duplicate rejection, and perpendicular single-line offset (`TRACK_HALF_WIDTH_PX / fmaxf(cosf(theta), 0.55f)`).
- Non-zero `horizon_angle_deg` on 90° corners (`has_transverse_tape == 1 && is_sharp_corner == 1`) using approach vectors + transverse wall direction.
- Conditional `±18.0f` single-line perspective convergence subtraction when `valid_count == 1` and `x_far` remains on the same side of `PIXY_CENTER_X` as `x_near`.

- [ ] **Step 3: Re-run the host unit test suite to verify GREEN pass**

Run: `gcc -Iinclude -Isource scratch/test_vision_and_control.c source/vectors.c -lm -o scratch/test_vision_and_control.exe && .\scratch\test_vision_and_control.exe`
Expected: PASS (0 failures).

---

### Task 3: Upgrade Control Loop & Wi-Fi Tuning (`source/main.c` and `source/wifi_config.c`)

**Files:**
- Modify: `source/wifi_config.c:165-175, 338-374`
- Modify: `source/main.c:39-516`

- [ ] **Step 1: Unblock `kd_s`/`kd_c` and expand `esp_printf` buffer in `source/wifi_config.c`**

- Allow `kd_straight` / `kd_s` and `kd_corner` / `kd_c` in `command_execute()`.
- Add `kd_s` and `kd_c` to the `help` text.
- Expand `esp_printf()` buffer from `char buf[128]` to `char buf[384]`.

- [ ] **Step 2: Upgrade `source/main.c` control loop, NVIC IRQs, intersection hysteresis, and differential braking**

- Enable NVIC interrupts and set priority `3` for `EDMA_0_CH0_IRQn`, `EDMA_0_CH1_IRQn`, and `LP_FLEXCOMM2_IRQn` alongside `LP_FLEXCOMM3`.
- Reduce EMA filter lag on `filtered_cte` and `filtered_heading` (`0.75f * raw + 0.25f * prev`).
- Compute `raw_d_cte = (filtered_cte - prev_filtered_cte) / fmaxf(g_params.dt, 0.005f)` and low-pass filter into `filtered_d_cte` using `g_params.alpha`.
- Apply `HEADING_DEADBAND_DEG` on straights and include `active_kd * filtered_d_cte` in `steer_top`.
- Track `last_valid_steer_top` and keep front-camera feedforward active during `sr.found == 0 && !s_intersection_ahead` without dropping steering magnitude.
- Use runtime `STEERING_LIMIT_RIGHT - g_params.steering_offset` and `STEERING_LIMIT_LEFT - g_params.steering_offset` for clamping `steer_cmd`.
- Require 3 consecutive ticks of `horizon.is_sharp_corner && !horizon.is_intersection_ahead` (`s_sharp_cancel_count >= 3`) before canceling an active `s_intersection_ahead` latch.
- Multiply by `g_params.drive_scale` directly in the blind-gap distance timer (`float current_spd = fmaxf(applied_speed_left, applied_speed_right) * g_params.drive_scale;`).
- Replace `diff_mult = steer_norm + s_corner_blend * (1.0f - steer_norm)` in `blend=1` mode with `float corner_authority = 0.25f + 0.75f * s_corner_blend; float diff_mult = steer_norm * corner_authority;`, and reduce straightaway differential drag in `blend=0` mode when `!horizon.is_sharp_corner && abs_steer < 22.0f`.

---

### Task 4: Upgrade Peripheral Drivers (`source/pixy.c`, `source/utils.c`, `board/peripherals.h`)

**Files:**
- Modify: `source/pixy.c:109-153`
- Modify: `source/utils.c:15-42`
- Modify: `board/peripherals.h:56-62`

- [ ] **Step 1: Add packet header validation & 64-byte read to `pixy_get_vectors()` in `source/pixy.c`**

- Read `uint8_t buf[64]` instead of `100` bytes.
- Locate the `0xAF, 0xC1` Pixy2 response sync header within the first 2 bytes of `buf`, verify packet type `49` (`0x31`), and return `kStatus_Fail` with `*num_vectors = 0` on corrupted frames.

- [ ] **Step 2: Add 3-tick debounce filter to `button_pressed_edge()` in `source/utils.c` and fix `INT_4_IRQN` in `board/peripherals.h`**

- Filter `GPIO_PinRead(GPIO0, 23U)` across 3 consecutive calls before registering a state transition.
- Update `INT_4_IRQN` and `INT_4_IRQHANDLER` in `board/peripherals.h` from `EDMA_0_CH4_IRQn` to `EDMA_0_CH3_IRQn`.

---

### Task 5: Full Firmware Compilation & Verification

**Files:**
- Test: `scratch/test_vision_and_control.c`
- Build: `Debug/NXP_cup.axf` via `scratch/build_mcuxpresso.py`

- [ ] **Step 1: Run host unit tests**
- [ ] **Step 2: Run full MCUXpresso ARM cross-compilation build and verify zero errors**
