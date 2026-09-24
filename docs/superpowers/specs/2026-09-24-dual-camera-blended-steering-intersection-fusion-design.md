# Design Specification: Continuous Dual-Camera Steering Blend & Gated Intersection Navigation

- **Date:** 2026-09-24
- **Target Platform:** NXP FRDM-MCXN947 (dual Cortex-M33, FreeRTOS, dual Pixy2 cameras)
- **Status:** Approved with Engineering Review Revisions, Ready for Implementation Planning

---

## 1. Executive Summary & Root Cause Analysis

Telemetry from the `lookahead-prebrake-v1` track run revealed a critical asymmetry between the vehicle's two vision sensors:
* **Top Camera (~25 cm lookahead):**
  - Valid 2-vector lane pair: **56.8%** of the run (84 frames).
  - Single boundary vector: **22.3%** of the run (33 frames).
  - Completely blind (0 vectors): **20.9% of the run (31 frames)!**
* **Front Horizon Camera (~1.0 m lookahead):**
  - 2+ vectors visible: **81.7% of the run** (121 frames).
  - 1 vector visible: **14.9% of the run** (22 frames).
  - Completely blind (0 vectors): **3.4% of the run** (5 frames).

### Key Failure Modes Addressed
1. **Corner Blindness & Wall Crossing:** In sharp turns, tape rotates out of the Top camera's narrow FOV (`Top: 0`). Forbidding the Front camera from steering caused the car to enter `STATE_LOST_LINE`, freeze steering, and coast through boundaries. The Front camera must provide closed-loop steering fallback with smooth blending.
2. **Intersection Wrong-Way Turning ("Ring Around the Rosie"):** 4-way intersections presented side branches at $\pm 45^\circ$ to $\pm 90^\circ$ that outscored the straight-through exit lane. We must actively reject side branches and track straight across the intersection to the opposite exit.

---

## 2. System Architecture

```
                 +-----------------------+           +-----------------------+
                 |    Top Camera (Near)  |           |   Front Camera (Far)  |
                 +-----------------------+           +-----------------------+
                             |                                   |
                             v                                   v
                    Confidence C_top                    Confidence C_front
                    (2 vec = 1.0, 1 vec = 0.5)          (2 vec = 1.0, 1 vec = 0.5,
                     0 vec = 0.0)                        Plausibility check)
                             |                                   |
                             v                                   |
                 +-----------------------+                       |
                 | Asymmetric Slew Limit |                       |
                 | Fall: 0.50, Rise: 0.08|                       |
                 +-----------------------+                       |
                             |                                   |
                             v                                   v
                     Blend Weight alpha                  Front Validity Flag
                             \                                   /
                              \-----> [ Steering Fusion ] <-----/
                                      delta_cmd = alpha*delta_top + (1-alpha)*delta_fr
```

---

## 3. Subsystem 1: Continuous Dual-Camera Steering Blend

### 3.1 Top Camera Confidence & Asymmetric Blend Slew
When the Top camera goes blind, we cannot afford to wait 133 ms to transfer authority (which would traverse ~30 cm blind at speed). The blend weight $\alpha$ must **drop fast** on degradation and **ramp up conservatively** on reacquisition:

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

/* Asymmetric slew rate: fast attack into fallback, smooth conservative return */
#define BLEND_FALL_SLEW_MAX  0.50f  /* Max 2 ticks (40ms) to drop to 0.0 on blindness */
#define BLEND_RISE_SLEW_MAX  0.08f  /* ~12 ticks (240ms) smooth return to top camera */

float delta_alpha = c_top - s_blend_weight;
if (delta_alpha > BLEND_RISE_SLEW_MAX) {
    delta_alpha = BLEND_RISE_SLEW_MAX;
} else if (delta_alpha < -BLEND_FALL_SLEW_MAX) {
    delta_alpha = -BLEND_FALL_SLEW_MAX;
}
s_blend_weight += delta_alpha;
if (s_blend_weight > 1.0f) s_blend_weight = 1.0f;
if (s_blend_weight < 0.0f) s_blend_weight = 0.0f;
```

### 3.2 Top PID & Filter Freeze During Blindness
To prevent steering kicks and integrator windup upon reacquisition after a blind corner:
```c
if (c_top == 0.0f) {
    /* Freeze Top PID integrator and lock CTE derivative while blind */
    steer_pid.integral = 0.0f; 
    /* filtered_cte and filtered_heading maintain their previous values without updating */
} else if (s_last_c_top == 0.0f && c_top > 0.0f) {
    /* Reacquisition re-seeding: initialize filter directly to prevent derivative spike */
    filtered_cte = current_cte;
    filtered_heading = current_heading;
}
s_last_c_top = c_top;
```

### 3.3 Front Camera Confidence & Plausibility Validation
The Front camera is granted steering authority only when its vector geometry is physically plausible:
```c
float c_front = 0.0f;
bool front_plausible = false;

if (num_vectors_front > 0) {
    /* Plausibility check: reject optical anomalies, mast vibration artifacts, and adjacent loops */
    if (fabsf(g_traj.trajectory_angle_deg) <= 38.0f && fabsf(g_traj.trajectory_offset_px) <= 28.0f) {
        front_plausible = true;
        c_front = (num_vectors_front >= 2) ? 1.0f : 0.60f;
    }
}
```

### 3.4 Steering Synthesis & Front Steering Authority
$$\delta_{\text{cmd}} = \alpha \cdot \delta_{\text{top}} + (1.0 - \alpha) \cdot \delta_{\text{front}}$$

* **$\delta_{\text{top}}$ Calculation:**
  $$\delta_{\text{top}} = -\left( K_{p,\text{top}} \cdot e_{\text{cte}} + K_{d,\text{top}} \cdot \dot{e}_{\text{cte}} + K_{\text{heading}} \cdot \psi_{\text{filtered}} \right)$$
* **$\delta_{\text{front}}$ Calculation (Lookahead Pursuit):**
  $$\delta_{\text{front}} = -\left( K_{p,\text{front}} \cdot \theta_{\text{traj,filtered}} + K_{\text{offset,front}} \cdot \Delta x_{\text{traj}} \right)$$
  - Initial baseline gains: $K_{p,\text{front}} = 1.00\text{ [deg steer / deg traj]}$, $K_{\text{offset,front}} = 0.35\text{ [deg steer / px offset]}$.
  - $K_{d,\text{front}} = 0.0$ to eliminate noise amplification from quantized Pixy vector endpoints.
  - $\theta_{\text{traj,filtered}}$ uses a 50 Hz low-pass filter: $\theta_k = 0.40 \cdot \theta_{\text{raw}} + 0.60 \cdot \theta_{k-1}$.

### 3.5 Redefined `STATE_LOST_LINE`
`STATE_LOST_LINE` is triggered only if **both** cameras are untrustworthy:
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

## 4. Subsystem 2: Gated Intersection Navigation

### 4.1 The Intersection Gating Architecture
* **Critical Design Rule:** Do **not** penalize candidate vector headings during normal open-track driving or in standard 90° corners.
* The heading alignment constraint is **gated**: it activates **only** upon detecting an intersection, latches the entry heading, and deactivates once clear.

### 4.2 Intersection State Machine
```
   [ NORMAL_DRIVING ] --(Intersection Trigger)--> [ CROSSING_INTERSECTION ]
          ^                                                    |
          |--------(Exit Detected / Timeout 500ms)-------------|
```

1. **Trigger Conditions:**
   - Top or Front camera detects transverse cross-tape ($\Delta X \ge 16\text{ px}, \Delta Y \le 10\text{ px}$), OR
   - Vector count $\ge 3$ with candidate pairs containing diverging headings ($|h_i - h_j| \ge 40^\circ$).
2. **On Entry to Intersection:**
   - Set `g_intersection_active = true`.
   - Latch entry heading: $\psi_{\text{latch}} = \text{filtered\_heading}$.
   - Reset intersection timer (`intersection_ticks = 0`).
3. **During Intersection Crossing (`g_intersection_active == true`):**
   - Candidate vector pairs are scored with a hard heading rejection filter against $\psi_{\text{latch}}$:
     $$\Delta \psi = |\theta_{\text{pair}} - \psi_{\text{latch}}|$$
     - If $\Delta \psi > 35.0^\circ$: **Hard reject pair** (disqualifies $\pm 45^\circ$ to $\pm 90^\circ$ cross branches).
   - If no candidate pair satisfies $\Delta \psi \le 35.0^\circ$, use single vector matching $\psi_{\text{latch}}$ or project $\delta_{\text{cmd}} = -K_{\text{heading}} \cdot \psi_{\text{latch}}$.
4. **On Exit from Intersection:**
   - Deactivates when valid longitudinal lane pair is detected clear of transverse tape for 3 consecutive ticks, OR
   - Timeout backstop: 25 ticks (500 ms) maximum duration.
   - Set `g_intersection_active = false`.

---

## 5. Subsystem 3: Continuous Speed Governor Integration

Speed is governed continuously by curvature preview $\kappa$ from [`compute_lookahead_curvature`](file:///d:/nxpcup/source/vectors.c#L400):
$$v_{\text{target}} = v_{\text{straight}} - \kappa \cdot (v_{\text{straight}} - v_{\text{corner}})$$

* $v_{\text{straight}} = 100\%$ (configurable via serial `speed_straight`).
* $v_{\text{corner}} = 45\%$ for tight corners (reduced from 58% to prevent pushing wide on sharp radii).
* Dynamic PID gain scheduling scales seamlessly with $\kappa$ across both straight and corner regimes.

---

## 6. Subsystem 4: Telemetry Stream Expansion

To provide full observability into the fusion controller and intersection gating:
```
L:2/3 R:-10 F:-11 H:6 TJ:27 S:-5 LA:0 BW:95 DOM:T FV:1 IX:0 ST:N VL:98 VR:98
```
* **`BW:xx`**: Blend weight $\alpha$ percentage (`0` to `100`).
* **`DOM:x`**: Dominant steering source:
  - `T`: Top camera dominant ($\alpha \ge 0.70$).
  - `F`: Front camera dominant ($\alpha \le 0.30$).
  - `B`: Blended ($0.30 < \alpha < 0.70$).
* **`FV:x`**: Front camera validity flag (`1` = plausible & usable, `0` = invalid/implausible).
* **`IX:x`**: Intersection gating active flag (`1` = active crossing, `0` = normal driving).

---

## 7. Phased Implementation Roadmap

1. **Phase 1: Dual-Camera Blending & Steering Fallback (Immediate):**
   - Asymmetric slew-limited blend weight ($\Delta_{\text{fall}} = 0.50, \Delta_{\text{rise}} = 0.08$).
   - Top PID state & EMA freeze during blindness.
   - Front camera plausibility filter and lookahead steering synthesis ($K_d = 0$).
   - Dual-blind `STATE_LOST_LINE` logic.
   - Updated telemetry stream (`BW`, `DOM`, `FV`).
   - Unit test verification in simulation.

2. **Phase 2: Gated Intersection Navigation:**
   - Intersection entry detection and heading latching.
   - Branch rejection filter ($\Delta \psi \le 35^\circ$) gated strictly to intersection state.
   - Exit detection and 500 ms backstop timeout.
   - Unit test verification with synthetic 4-way intersection scenarios.

3. **Phase 3: Track Validation & Speed Governor Tuning:**
   - Flash firmware to vehicle.
   - Verify on track at baseline corner speed (45–50%).
   - Inspect telemetry for smooth `DOM` transitions and clean intersection pass-through.
