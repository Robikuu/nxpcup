# Design Specification: Continuous Dual-Camera Steering Blend & Gated Intersection Navigation

- **Date:** 2026-09-24
- **Target Platform:** NXP FRDM-MCXN947 (dual Cortex-M33, FreeRTOS, dual Pixy2 cameras)
- **Status:** Approved with Engineering Review Fixes (v4 - Production Ready), Ready for Implementation Planning

---

## 1. Executive Summary & Problem Formulation

Telemetry from the `lookahead-prebrake-v1` track run revealed a critical asymmetry between the vehicle's two vision sensors:
* **Top Camera (~25 cm lookahead):**
  - Valid 2-vector lane pair: **56.8%** of the run (84 frames).
  - Single boundary vector: **22.3%** of the run (33 frames).
  - Completely blind (0 vectors): **20.9% of the run (31 frames)!**
* **Front Horizon Camera (~1.0 m lookahead):**
  - 2+ vectors visible: **81.7% of the run** (121 frames).
  - 1 vector visible: **14.9% of the run** (22 frames).
  - Completely blind (0 vectors): **3.4% of the run** (5 frames).

### Key Architectural Fixes
1. **Continuous Dual-Camera Steering Blend with Shadow Mode Verification:** Replace the top-only steering paradigm with smooth, asymmetric blending ($\alpha$). Includes a zero-risk **Shadow Mode** (`shadow 1`) to log and compare $\delta_{\text{front}}$ alongside $\delta_{\text{top}}$ in real track noise before enabling active blend control.
2. **Gated Intersection Navigation:** Enforce the NXP Cup rule ("exit on the opposite side it entered") by strictly gating heading-aligned candidate pair selection to confirmed intersections (transverse tape signature while roughly straight), with approach-aware timeout scaling (~1.5 m total transit).
3. **Self-Healing Temporal Plausibility & Complete PID Reseed:** Temporal jump limits ($\Delta \theta \le 25^\circ$/tick) with a 3-frame auto-reseed to prevent permanent lockouts; full PID state reset (`prev_error = 0`, `integral` frozen) on reacquisition; output servo slew limiting ($\le 12^\circ$/tick).

---

## 2. System Architecture

```
                 +-----------------------+           +-----------------------+
                 |    Top Camera (Near)  |           |   Front Camera (Far)  |
                 +-----------------------+           +-----------------------+
                             |                                   |
                             v                                   v
                    Confidence C_top                    Confidence C_front
                    (2 vec = 1.0, 1 vec = 0.5)          (2 vec = 1.0, 1 vec = 0.6)
                    (0 vec = 0.0)                       Temporal jump check with
                             |                          3-frame auto-reseed
                             v                                   |
                 +-----------------------+                       |
                 | Asymmetric Slew Limit |                       |
                 | Fall: 0.50, Rise: 0.08|                       |
                 | Forced to 1 if FV=0   |                       |
                 +-----------------------+                       |
                             |                                   |
                             v                                   v
                     Blend Weight alpha                  Front Validity Flag (FV)
                             \                                   /
                              \-----> [ Steering Fusion ] <-----/
                                      If Shadow Mode: delta_raw = delta_top
                                      Else: delta_raw = alpha*delta_top + (1-alpha)*delta_fr
                                         |
                                         v
                       +----------------------------------------+
                       | Output Slew Limiter (<= 12.0 deg/tick) |
                       +----------------------------------------+
                                         |
                                         v
                                     delta_cmd
```

---

## 3. Subsystem 1: Continuous Dual-Camera Steering Blend

### 3.1 Top Camera Confidence & Front-Aware Asymmetric Blend Slew
When the Top camera goes blind, authority transfers to the Front camera within 2 ticks (40 ms). Reacquisition returns smoothly over ~12 ticks (240 ms). If the Front camera is invalid (`front_plausible == false`) and Top has any data, $\alpha$ is forced to 1.0:

```c
/* Top camera instantaneous confidence */
float c_top = 0.0f;
if (num_vectors_top >= 2) {
    c_top = 1.0f;
} else if (num_vectors_top == 1) {
    c_top = 0.50f;
} else {
    c_top = 0.0f;
}

/* Target blend weight: if front is invalid and top has any signal, rely entirely on top */
float target_alpha = c_top;
if (!front_plausible && c_top > 0.0f) {
    target_alpha = 1.0f;
}

/* Asymmetric slew rate */
#define BLEND_FALL_SLEW_MAX  0.50f  /* 2 ticks to drop to 0.0 on blindness */
#define BLEND_RISE_SLEW_MAX  0.08f  /* ~12 ticks smooth return to top camera */

float delta_alpha = target_alpha - s_blend_weight;
if (delta_alpha > BLEND_RISE_SLEW_MAX) {
    delta_alpha = BLEND_RISE_SLEW_MAX;
} else if (delta_alpha < -BLEND_FALL_SLEW_MAX) {
    delta_alpha = -BLEND_FALL_SLEW_MAX;
}
s_blend_weight += delta_alpha;
if (s_blend_weight > 1.0f) s_blend_weight = 1.0f;
if (s_blend_weight < 0.0f) s_blend_weight = 0.0f;
```

### 3.2 Complete Top PID & Filter Reseed
Freezes integrator while blind and resets error history on reacquisition to eliminate derivative kick:
```c
if (c_top == 0.0f) {
    /* Freeze Top PID integrator: preserve accumulated value without windup */
    /* Do not update filtered_cte or filtered_heading with empty data */
} else if (s_last_c_top == 0.0f && c_top > 0.0f) {
    /* Reacquisition re-seeding: initialize filters and clear previous error */
    filtered_cte = current_cte;
    filtered_heading = current_heading;
    steer_pid.prev_error = current_cte;  /* Eliminates derivative spike on first tick */
    cte_derivative = 0.0f;
}
s_last_c_top = c_top;
```

### 3.3 Self-Healing Front Camera Plausibility Filter
Rejects frame-to-frame jump anomalies while preventing permanent lockout via a 3-frame auto-reseed:
```c
bool front_plausible = false;
float c_front = 0.0f;

if (num_vectors_front > 0) {
    if (s_front_dropout_ticks > 0) {
        /* Immediate re-seed after camera recovery from 0 vectors */
        s_last_front_angle  = g_traj.trajectory_angle_deg;
        s_last_front_offset = g_traj.trajectory_offset_px;
        s_consecutive_rejections = 0;
        front_plausible = true;
    } else {
        float angle_jump  = fabsf(g_traj.trajectory_angle_deg - s_last_front_angle);
        float offset_jump = fabsf(g_traj.trajectory_offset_px - s_last_front_offset);

        if (fabsf(g_traj.trajectory_angle_deg) <= 75.0f &&
            angle_jump <= 25.0f &&
            offset_jump <= 18.0f) {
            front_plausible = true;
            s_consecutive_rejections = 0;
            s_last_front_angle  = g_traj.trajectory_angle_deg;
            s_last_front_offset = g_traj.trajectory_offset_px;
        } else {
            s_consecutive_rejections++;
            if (s_consecutive_rejections >= 3) {
                /* Self-healing reseed: prevent permanent lockout in real sharp corners */
                front_plausible = true;
                s_consecutive_rejections = 0;
                s_last_front_angle  = g_traj.trajectory_angle_deg;
                s_last_front_offset = g_traj.trajectory_offset_px;
            }
        }
    }

    if (front_plausible) {
        c_front = (num_vectors_front >= 2) ? 1.0f : 0.60f;
    }
}
```

### 3.4 Steering Synthesis & Output Slew Limiting
$$\delta_{\text{top}} = -\left( K_{p,\text{top}} \cdot e_{\text{cte}} + K_{d,\text{top}} \cdot \dot{e}_{\text{cte}} + K_{\text{heading}} \cdot \psi_{\text{filtered}} \right)$$

* Front steering calculation with invalid-decay handling:
```c
if (front_plausible) {
    delta_front = -(K_p_front * filtered_traj_angle + K_offset_front * g_traj.trajectory_offset_px);
    s_last_valid_delta_front = delta_front;
    s_front_hold_ticks = 0;
} else {
    /* Hold last valid front command for 3 ticks (60ms), then decay gently */
    s_front_hold_ticks++;
    if (s_front_hold_ticks <= 3) {
        delta_front = s_last_valid_delta_front;
    } else {
        delta_front = s_last_valid_delta_front * 0.95f;
        s_last_valid_delta_front = delta_front;
    }
}
```

* Baseline front gains: $K_{p,\text{front}} = 1.00\text{ [deg/deg]}$, $K_{\text{offset,front}} = 0.35\text{ [deg/px]}$, $K_{d,\text{front}} = 0.0$.
* Low-pass filter on front trajectory: $\theta_k = 0.40 \cdot \theta_{\text{raw}} + 0.60 \cdot \theta_{k-1}$.

**Shadow Mode & Servo Slew Rate Limiter:**
```c
float delta_raw;
if (g_params.shadow_mode_enable) {
    /* Shadow mode: vehicle steers 100% on top camera while front camera logs for calibration */
    delta_raw = delta_top;
} else {
    delta_raw = s_blend_weight * delta_top + (1.0f - s_blend_weight) * delta_front;
}

/* Servo physical slew rate limiter */
float delta_diff = delta_raw - s_last_servo_cmd;
if (delta_diff > 12.0f) delta_diff = 12.0f;
if (delta_diff < -12.0f) delta_diff = -12.0f;

servo_cmd = s_last_servo_cmd + delta_diff;
if (servo_cmd > 77.5f)  servo_cmd = 77.5f;
if (servo_cmd < -77.5f) servo_cmd = -77.5f;
s_last_servo_cmd = servo_cmd;
```

---

## 4. Subsystem 2: Gated Intersection Navigation

### 4.1 Approach-Aware Trigger & Straightness Logging
```c
bool is_roughly_straight = (fabsf(filtered_heading) <= 15.0f) && (fabsf(s_last_servo_cmd) <= 18.0f);
bool has_transverse_tape = (g_traj.has_transverse_tape || top_has_transverse_tape);

if (has_transverse_tape && !is_roughly_straight) {
    s_curved_intersection_approaches++; /* Logged in telemetry for audit */
}

if (!g_intersection_active && has_transverse_tape && is_roughly_straight) {
    g_intersection_active = true;
    s_intersection_ticks = 0;
    s_latched_steer = s_last_servo_cmd; /* Latch pre-entry steering angle */
    s_latched_heading = filtered_heading;
}
```

### 4.2 Gated Candidate Pair Filtering Across Intersection
During `g_intersection_active == true`:
* Candidate pairs in [`source/vectors.c`](file:///d:/nxpcup/source/vectors.c) are filtered against latched heading $\psi_{\text{latched}}$:
  $$\Delta \psi = |\theta_{\text{pair}} - \psi_{\text{latched}}|$$
* If $\Delta \psi > 35.0^\circ$, candidate pair is **hard-rejected** (eliminating $\pm 45^\circ..\pm 90^\circ$ cross-branches).
* **Fallback Steering Across Intersection:** If no forward longitudinal pair is visible, hold latched pre-entry angle for 5 ticks (100 ms), then decay gently ($0.97^k$).

### 4.3 Approach-Aware Distance Timeout Backstop
The front camera triggers detection $\approx 1.0\text{ m}$ ahead of the box, and the box width is $\approx 0.45\text{ m}$, totaling $\approx 1.45\text{ m}$ of transit.
At corner speed $v \approx 1.2\text{ m/s}$ (PWM 50%), this requires $\approx 60\text{ ticks}$ (1.2 s):
$$\text{timeout\_ticks} = \text{clamp}\left( \frac{1.45\text{ m}}{v_{\text{est}} \cdot 0.020\text{ s}}, 35, 75 \right)$$
* Intersection deactivates when clear longitudinal lines ($\ge 2$ vectors) are tracked on the opposite side for 4 consecutive ticks, or when `s_intersection_ticks >= timeout_ticks`.

---

## 5. Subsystem 3: Speed Governor & Confidence Coupling

### 5.1 Preview Speed Governor with Asymmetric Filter
Speed targets are computed from lookahead curvature $\kappa$ with fast attack (0.70) into corners and slow decay (0.15) on exit:
$$v_{\text{target}} = v_{\text{straight}} - \kappa \cdot (v_{\text{straight}} - v_{\text{corner}})$$

* $v_{\text{straight}} = 92\%$ (reduced from 100% for straightaway stability).
* $v_{\text{corner}} = 45\%$ for sharp corners.

### 5.2 Confidence & Intersection Speed Caps
* **Fallback Speed Cap:** When Top camera is blind and Front dominates ($\alpha < 0.30$), cap speed to $\le 52\%$ to prevent cutting inside curbs on far lookahead.
* **Intersection Speed Cap:** When `g_intersection_active == true`, cap speed to $\le 55\%$.

---

## 6. Subsystem 4: Dual-Blind `STATE_LOST_LINE`
`STATE_LOST_LINE` triggers only if **both** cameras fail simultaneously:
```c
bool line_lost = (c_top == 0.0f) && (!front_plausible || c_front == 0.0f);
if (line_lost) {
    s_lost_line_ticks++;
    if (s_lost_line_ticks > LOST_LINE_TOLERANCE_TICKS) {
        state = STATE_LOST_LINE;
    }
} else {
    s_lost_line_ticks = 0;
}
```

---

## 7. Subsystem 5: Telemetry Stream Expansion (No Key Collisions)

Updated telemetry stream with collision-free keys:
```
L:2/3 R:-10 F:-11 H:6 TS:-15 FS:-12 S:-14 BW:85 DOM:T FV:1 IX:0 ST:N VL:88 VR:88
```
* **`TS:xx`**: Top steering command $\delta_{\text{top}}$ (deg $\times 10$).
* **`FS:xx`**: Front steering command $\delta_{\text{front}}$ (deg $\times 10$).
* **`S:xx`**: Blended, slew-limited servo command $\delta_{\text{cmd}}$ (deg $\times 10$).
* **`BW:xx`**: Blend weight $\alpha$ percentage (`0` to `100`).
* **`DOM:x`**: Dominant source (`T` = Top, `F` = Front, `B` = Blended).
* **`FV:x`**: Front validity flag (`1` = valid, `0` = implausible).
* **`IX:x`**: Intersection active flag (`1` = active, `0` = normal).
* **`ST:x`**: Vehicle state (`N` = Normal, `L` = Lost Line).

---

## 8. Implementation Roadmap

1. **Phase 1A: Telemetry & Shadow Mode Deployment (Zero-Risk):**
   - Implement `TS:`, `FS:`, `BW:`, `DOM:`, `FV:`, `IX:` in telemetry parser.
   - Implement $\delta_{\text{front}}$ calculation and temporal plausibility check with 3-frame auto-reseed.
   - Enable `shadow_mode_enable = 1` by default (car steers on $\delta_{\text{top}}$ as before).
   - Test on track: log and compare `TS:` vs. `FS:` side-by-side in the ~57% dual-camera frames.

2. **Phase 1B: Enable Active Blending & Fallback:**
   - Set `shadow_mode_enable = 0`.
   - Enable asymmetric blend weight $\alpha$ ($\Delta_{\text{fall}} = 0.50, \Delta_{\text{rise}} = 0.08$).
   - Enable full Top PID reseed (`prev_error` and `derivative` reset).
   - Enable servo slew limiter ($\le 12^\circ/\text{tick}$) and dual-blind `STATE_LOST_LINE`.

3. **Phase 2: Gated Intersection Navigation:**
   - Transverse tape + straight-approach intersection trigger.
   - Gated candidate pair $\Delta \psi \le 35^\circ$ filter and latched steering fallback.
   - Approach-aware timeout backstop (~1.45 m transit).

4. **Phase 3: Track Validation & Speed Governor Dial-In:**
   - Verify inner loop navigation without boundary loss and clean intersection crossing.
   - Dial in corner speeds starting at 45%.
