# Design Specification: Racing Corner Approach with Straight-Line Threshold Pre-Braking

## 1. Overview & Problem Statement

To maximize lap times in competitive time-attack racing, straightaway speed must remain uncapped at $100\%$ commanded duty (delivering the full $12.6\,\text{V}$ battery potential through the low-resistance DRV8871 H-bridges). 

However, approaching tight turns at full sprint ($\approx 2.5\,\text{m/s}$) with legacy cornering setpoints ($78\%$ outside / $62\%$ inside) causes severe understeer and track departures because:
1. Corner entry speeds are too high for tire adhesion limits ($\mu \approx 0.6$).
2. The preview speed governor quadratic squashing ($\kappa^2$) practically ignores early lookahead detection.
3. Deceleration ramping is too slow to shed speed before the turn apex.

This specification implements **Option 1: Straight-Line Threshold Active Braking**:
- Reverts the global plant gain scaling (`drive_scale` scaling removed, delivering 100% duty on straights).
- Detects upcoming corners early via the Front Lookahead camera while the car is still traveling straight.
- Fires a brief, high-force active dynamic brake pulse ($60\,\text{ms}$, 3 ticks) in a pure straight line to rapidly dump momentum before turn-in.
- Seamlessly transitions out of the pulse into recalibrated corner apex cruising speeds ($58\%$ outside / $38\%$ inside) with aggressive inside-wheel differential pivot drag biting at $6^\circ$ steering angle.

---

## 2. Architecture & State Machine

```
              [HIGH-SPEED SPRINT]
            applied_speed = 100%
        |Heading| <= 10°, |Steer| <= 12°
                     │
                     │  Front camera confirms upcoming turn:
                     │  lookahead_turn_factor >= 0.35 for >= 2 ticks
                     ▼
        [ACTIVE THRESHOLD BRAKE PULSE]
            State: ST:B
            Duration: Exactly 3 ticks (60 ms)
            Action: HbridgeBrake() (active low-side coil short)
            Safety: Only executes while |Heading| <= 10°
                     │
                     │  Timer expires (3 ticks completed)
                     ▼
          [CORNER APEX CRUISE & PIVOT]
            State: ST:N
            Seed: applied_speed = 58% (instant latch)
            SPEED_OUTSIDE_CORNER = 58.0%
            SPEED_INSIDE_MIN     = 38.0%
            DIFFERENTIAL_DEADZONE = 6.0°
                     │
                     │  Turn completed: lookahead < 0.20, |Heading| < 8°
                     ▼
            [ACCELERATION OUT OF TURN]
            Ramp back to 100% sprint via ACCEL_RAMP_STEP
            Re-arm threshold pre-brake trigger
```

---

## 3. Detailed Component Modifications

### 3.1 [`include/Config.h`](file:///d:/nxpcup/include/Config.h)
- Set racing corner setpoints:
  ```c
  #define SPEED_STRAIGHT               100.0f    // 100% full power sprint
  #define SPEED_OUTSIDE_CORNER         58.0f     // Calibrated DRV8871 corner apex speed
  #define SPEED_INSIDE_MIN             38.0f     // Calibrated inside-wheel pivot anchor duty
  #define DIFFERENTIAL_DEADZONE_DEG    6.0f      // Differential drag begins at 6° (down from 12°)
  ```
- Define Pre-Brake Constants:
  ```c
  #define PREBRAKE_PULSE_TICKS         3U        // 3 ticks @ 50Hz = 60ms active brake pulse
  #define PREBRAKE_MIN_SPEED           80.0f     // Only brake if sprinting at or above 80%
  #define PREBRAKE_MAX_HEADING_DEG     10.0f     // Straight line alignment guard
  #define PREBRAKE_MAX_STEER_DEG       12.0f     // Wheels centered alignment guard
  #define PREBRAKE_LOOKAHEAD_THRESH    0.35f     // Front camera turn urgency threshold
  #define PREBRAKE_CONFIRM_TICKS       2U        // Consecutive ticks required to confirm turn
  ```

### 3.2 [`source/hbridge.c`](file:///d:/nxpcup/source/hbridge.c)
- Remove `g_params.drive_scale` multiplication in `HbridgeSpeed()`, allowing commanded speed to pass $100\%$ duty directly to CTIMER PWM compare registers while retaining sub-percent float tick accuracy.

### 3.3 [`source/main.c`](file:///d:/nxpcup/source/main.c)
- Add state variables:
  ```c
  static uint8_t  s_prebrake_confirm_count = 0;
  static uint8_t  s_prebrake_active_ticks  = 0;
  static bool     s_prebrake_armed         = true;
  ```
- **Trigger Evaluation (executed before motor speed calculation):**
  - If car is in `MACRO_STATE_NORMAL`:
    - If `applied_speed_left >= PREBRAKE_MIN_SPEED` and `fabsf(filtered_heading) <= PREBRAKE_MAX_HEADING_DEG` and `fabsf(steer_cmd) <= PREBRAKE_MAX_STEER_DEG`:
      - If `lookahead_turn_factor >= PREBRAKE_LOOKAHEAD_THRESH`:
        - `s_prebrake_confirm_count++`;
        - If `s_prebrake_confirm_count >= PREBRAKE_CONFIRM_TICKS && s_prebrake_armed`:
          - `s_prebrake_active_ticks = PREBRAKE_PULSE_TICKS`;
          - `s_prebrake_armed = false`;
      - Else: `s_prebrake_confirm_count = 0;`
    - Else:
      - `s_prebrake_confirm_count = 0;`
      - If road is straight and speed recovered (`lookahead_turn_factor < 0.20f && fabsf(filtered_heading) < 6.0f`):
        - `s_prebrake_armed = true`;
- **Execution:**
  - If `s_prebrake_active_ticks > 0`:
    - `s_prebrake_active_ticks--`;
    - Call `HbridgeBrake(&g_hbridge)`;
    - Clamp `applied_speed_left = g_params.speed_outside_corner`;
    - Clamp `applied_speed_right = g_params.speed_outside_corner`;
    - Telemetry logs `ST:B`.
  - Else:
    - Normal differential control and speed governor.

---

## 4. Verification Plan

1. **Clean Compilation:** Headless MCUXpresso build with 0 errors and 0 warnings.
2. **Wi-Fi Status:** Verify `spd_s=100`, `spd_out=58`, `spd_in=38`.
3. **Track Testing:**
   - Car sprints at 100% on straightaway.
   - At corner approach, telemetry prints 3 frames of `ST:B` with active braking.
   - Car enters corner at 58% with inside wheel pulling to 38% at 6°+ steering.
   - Car makes apex cleanly without understeering into wall.
