# Design Specification: Output Plant Gain Normalization for DRV8871 Upgrade (Revised)

## 1. Overview & Problem Statement

The vehicle hardware was recently upgraded from a legacy bipolar L298N H-bridge driver to two Adafruit DRV8871 DMOS H-bridge breakout boards.
- The legacy L298N exhibited an internal Darlington transistor saturation drop of approximately $2.0\,\text{V} - 3.0\,\text{V}$, delivering $\approx 9.6\,\text{V} - 10.1\,\text{V}$ to the $12\,\text{V}$ DC drive motors at 100% commanded duty from a $12.6\,\text{V}$ 3S LiPo battery.
- The DRV8871 has a typical combined high-side + low-side on-resistance of $R_{DS(on)} \approx 565\,\text{m}\Omega$ ($660\,\text{m}\Omega$ max). At $2\,\text{A}$ cruise current, the bridge drops $\approx 1.13\,\text{V}$; at $3\,\text{A}$ launch/stall, it drops $\approx 1.7\,\text{V}$. Because voltage drop depends on current, a single linear scale cannot match the old driver identically across all load regimes. The scale will be calibrated at nominal cruising speed, with the expectation that the car will feel somewhat punchier at launch/stall than the old driver.

This hardware transition shifted the vehicle's actuator plant gain ($K_{\text{plant}}$) by roughly $+20\%\dots+26\%$. Consequently, running the codebase with `SPEED_STRAIGHT = 100.0f` caused severe vehicle over-speeding, invalidating prior calibrations for acceleration ramps, deceleration rates, differential torque vectoring, and corner entry timing.

Rather than altering individual speed setpoints and distorting the tuned ramp slopes, this design introduces a unified output-stage plant gain scale (`DRIVE_SCALE`), preserving all upstream control units and providing a single calibrated knob to match the historical physical baseline.

---

## 2. Architecture & Data Flow

```
[Control Laws & State Machine]
  - Normal racing governor
  - Intersection straight-hold
  - Line lost failsafe
              │
              ▼
[Longitudinal & Differential Mixing]
  - Target speed calculation (58% - 100%)
  - Decel/Accel ramp rate limiters (28% / 35% per tick)
  - Differential yaw drag delta (v_target - v_inside)
              │
              ▼ (applied_speed_left, applied_speed_right: float [-100.0f, +100.0f])
[HbridgeSpeed()]
  - Input saturation clamp to [-100.0f, +100.0f]
  - FLOAT-PRECISION PLANT GAIN SCALING (No integer truncation):
      scaled_left  = speed_left  * g_params.drive_scale
      scaled_right = speed_right * g_params.drive_scale
              │
              ▼ (float duty: 0.0f - 100.0f)
[PWM Duty & CTIMER Compare Register Update]
  - Direction logic (GPIO IN2)
  - Single final rounding directly to CTIMER Match Register tick value
```

> [!NOTE]
> **Decay Mode Notice:** Current driver wiring pairs 1 PWM pin with 1 static GPIO pin per motor. Forward drive runs in Drive/Brake mode (`IN2=1`, inverted PWM on `IN1`), while reverse operates in Drive/Coast. Transitioning forward chopping to alternative decay modes is strictly deferred as a separate experiment after plant gain calibration is complete.

---

## 3. Detailed Component Modifications

### 3.1 [`include/Config.h`](file:///d:/nxpcup/include/Config.h)
- Define default scale factor:
  ```c
  #define DEFAULT_DRIVE_SCALE          0.80f     // Output plant gain scale (starting guess: 0.76 - 0.84)
  ```
- Add `float drive_scale;` member to `robot_params_t` struct.
- Keep all upstream parameters (`SPEED_STRAIGHT = 100.0f`, `SPEED_OUTSIDE_CORNER = 78.0f`, `SPEED_INSIDE_MIN = 62.0f`, ramps, and deadbands) strictly untouched.

### 3.2 [`include/hbridge.h`](file:///d:/nxpcup/include/hbridge.h) & [`source/hbridge.c`](file:///d:/nxpcup/source/hbridge.c)
- Upgrade `HbridgeSpeed` signature to accept `float` instead of `int16_t` to preserve sub-percent differential and ramp resolution:
  ```c
  void HbridgeSpeed(Hbridge *h, float speed_left, float speed_right);
  ```
- Inside `HbridgeSpeed()`, scale duty in floating-point and compute CTIMER match tick value directly without intermediate integer-percent truncation:
  ```c
  float scaled_left  = speed_left  * g_params.drive_scale;
  float scaled_right = speed_right * g_params.drive_scale;
  ```
- Calculate timer match value with float rounding:
  ```c
  uint32_t period = h->pwmPeripheral->MR[h->periodChannel];
  uint32_t match_val = (uint32_t)((float)period * (100.0f - duty_float) / 100.0f + 0.5f);
  ```

### 3.3 [`source/wifi_config.c`](file:///d:/nxpcup/source/wifi_config.c)
- Initialize in `params_init()`:
  ```c
  g_params.drive_scale = DEFAULT_DRIVE_SCALE;
  ```
- Add command parser support for live track tuning:
  ```c
  else if ((!strcmp(keyword, "scale") || !strcmp(keyword, "drive_scale")) && has_val) {
      if (val < 0.10f) val = 0.10f;
      if (val > 1.00f) val = 1.00f;
      g_params.drive_scale = val;
      send_ack_dbl("drive_scale=", val);
  }
  ```
- Report `scale` in the `status` command and `help` menu.

### 3.4 Telemetry Stream Logging ([`source/main.c`](file:///d:/nxpcup/source/main.c))
- To ensure telemetry runs before and after the modification remain directly comparable:
  - `VL` and `VR` continue reporting the **pre-scale controller command** (e.g. `VL:100 VR:100` or `VL:78 VR:62`), preserving continuity with historical log analysis scripts.
  - Append the active scale value `SC:%d` (e.g. `SC:80` for 0.80) to the telemetry line so log analyzers know the exact physical scaling applied at the wheels:
    ```text
    ... ST:N VL:100 VR:100 SC:80
    ```

---

## 4. Verification & Track Testing Plan

1. **Software Sanity Verification:**
   - MCUXpresso headless build clean (`0 errors, 0 warnings`).
   - Query `status;\r` $\rightarrow$ confirms `scale=0.80` output.
   - Command `scale 0.75;\r` $\rightarrow$ verifies dynamic update without restart.
   - Command `motor_left 50;\r` $\rightarrow$ verifies code execution path.
2. **Physical Calibration Verification:**
   - **Straightaway Speed Matching:** Time the car over a measured straightaway section (or compare elapsed frames between track landmarks against historical telemetry runs) to verify that vehicle velocity matches the previous physical baseline.
   - **Corner Entry Validation:** Verify that the car no longer approaches the first turn with excessive momentum, entering the corner within the available tire traction limits.
   - **Trim Iteration:** Adjust `scale` via Wi-Fi terminal in $0.02$ increments until the car's dynamic response matches the known baseline before adjusting any control laws.
