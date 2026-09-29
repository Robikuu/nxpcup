# Design Specification: Output Plant Gain Normalization for DRV8871 Upgrade

## 1. Overview & Problem Statement

The vehicle hardware was recently upgraded from a legacy bipolar L298N H-bridge driver to two Adafruit DRV8871 DMOS H-bridge breakout boards. 
- The legacy L298N exhibited an internal Darlington transistor saturation drop of approximately $2.0\,\text{V} - 3.0\,\text{V}$, delivering $\approx 9.6\,\text{V} - 10.1\,\text{V}$ to the $12\,\text{V}$ DC drive motors at 100% commanded duty from a $12.6\,\text{V}$ 3S LiPo battery.
- The new DRV8871 features an ultra-low total bridge on-resistance of $R_{DS(on)} = 365\,\text{m}\Omega$, dropping under $0.8\,\text{V}$ under load and delivering $\approx 11.8\,\text{V} - 12.2\,\text{V}$ to the motors at 100% duty.

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
              ▼ (applied_speed_left, applied_speed_right: [-100, +100])
[HbridgeSpeed()]
  - Input saturation clamp to [-100, +100]
  - OUTPUT PLANT GAIN SCALING:
      speed_left  = speed_left  * g_params.drive_scale
      speed_right = speed_right * g_params.drive_scale
              │
              ▼
[PWM Duty & Direction Translation]
  - Direction logic (GPIO IN2)
  - Quadratic Fast-Decay compensation (if reverse)
  - CTIMER Match compare register update
```

---

## 3. Detailed Component Modifications

### 3.1 [`include/Config.h`](file:///d:/nxpcup/include/Config.h)
- Define default scale factor:
  ```c
  #define DEFAULT_DRIVE_SCALE          0.80f     // Output plant gain scale (starting guess: 0.76 - 0.84)
  ```
- Add `float drive_scale;` member to `robot_params_t` struct.
- Keep all upstream parameters (`SPEED_STRAIGHT = 100.0f`, `SPEED_OUTSIDE_CORNER = 78.0f`, `SPEED_INSIDE_MIN = 62.0f`, ramps, and deadbands) strictly untouched.

### 3.2 [`source/wifi_config.c`](file:///d:/nxpcup/source/wifi_config.c)
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
- Include `scale` in the `status` command report and `help` menu.

### 3.3 [`source/hbridge.c`](file:///d:/nxpcup/source/hbridge.c)
- In `HbridgeSpeed()`, scale commanded speed values before duty calculation:
  ```c
  /* Output plant gain scaling: normalizes DRV8871 output to match original driver baseline */
  speed_left  = (int16_t)((float)speed_left  * g_params.drive_scale);
  speed_right = (int16_t)((float)speed_right * g_params.drive_scale);
  ```

---

## 4. Verification & Track Testing Plan

1. **Compilation Verification:**
   - MCUXpresso headless build clean (`0 errors, 0 warnings`).
   - Binary size and flash integrity verified.
2. **Wi-Fi Shell Verification:**
   - Query `status;\r` $\rightarrow$ confirms `scale=0.80` output.
   - Command `scale 0.75;\r` $\rightarrow$ verifies dynamic update without restart.
   - Command `motor_left 50;\r` $\rightarrow$ verifies output duty is scaled to $40\%$ ($50 \times 0.80$).
3. **Track Speed Verification:**
   - Place car on straightaway; send `start;\r`.
   - Verify visually and via telemetry that straightaway speed matches historical benchmark before the driver upgrade.
   - Trim `scale` in small increments ($0.02$) if necessary.
