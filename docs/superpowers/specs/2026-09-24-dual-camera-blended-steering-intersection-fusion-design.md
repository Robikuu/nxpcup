# Design Specification: Continuous Dual-Camera Steering Blend & Gated Intersection Navigation

- **Date:** 2026-09-24
- **Target Platform:** NXP FRDM-MCXN947 (dual Cortex-M33, FreeRTOS, dual Pixy2 cameras)
- **Status:** Approved with Engineering Review Fixes (v3), Ready for Implementation Planning

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
1. **Continuous Dual-Camera Steering Blend:** Replace the top-only steering paradigm with smooth, asymmetric blending ($\alpha$). In sharp corners where the near camera goes blind ($C_{\text{top}} = 0$), the Front camera takes steering authority without dropping into `STATE_LOST_LINE`.
2. **Gated Intersection Navigation:** Enforce the NXP Cup rule ("exit on the opposite side it entered") by strictly gating heading-aligned candidate pair selection to confirmed intersections (transverse tape signature while roughly straight), preventing false triggers in real corners.
3. **Temporal Plausibility & Servo Slew Limiting:** Replace static angle clamps with temporal jump limits ($\Delta \theta \le 25^\circ$/tick), freeze the Top PID state during dropouts, limit $\delta_{\text{cmd}}$ slew rate, and log $\delta_{\text{top}}$ vs. $\delta_{\text{front}}$ side-by-side.

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
                    (0 vec = 0.0)                       Temporal jump check:
                             |                          |d_theta| <= 25 deg/tick
                             v                          |d_offset| <= 18 px/tick
                 +-----------------------+                       |
                 | Asymmetric Slew Limit |                       |
                 | Fall: 0.50, Rise: 0.08|                       |
                 +-----------------------+                       |
                             |                                   |
                             v                                   v
                     Blend Weight alpha                  Front Validity Flag (FV)
                             \                                   /
                              \-----> [ Steering Fusion ] <-----/
                                      delta_raw = alpha*delta_top + (1-alpha)*delta_fr
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

### 3.1 Top Camera Confidence & Asymmetric Blend Slew
When the Top camera goes blind, authority transfers to the Front camera within 2 ticks (40 ms, traversing < 10 cm). Reacquisition returns smoothly over ~12 ticks (240 ms):

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

/* Asymmetric slew rate */
#define BLEND_FALL_SLEW_MAX  0.50f  /* 2 ticks to drop to 0.0 on blindness */
#define BLEND_RISE_SLEW_MAX  0.08f  /* ~12 ticks smooth return to top camera */

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
```c
if (c_top == 0.0f) {
    /* Freeze Top PID integrator: preserve accumulated value without windup */
    /* Do not update filtered_cte or filtered_heading with empty data */
} else if (s_last_c_top == 0.0f && c_top > 0.0f) {
    /* Reacquisition re-seeding: initialize filters directly to current measurement */
    filtered_cte = current_cte;
    filtered_heading = current_heading;
}
s_last_c_top = c_top;
```

### 3.3 Front Camera Temporal Plausibility
Rather than rejecting large angles (which naturally occur in tight 90° corners), the Front camera checks **temporal jump consistency** to reject glitches or latching onto adjacent parallel tracks:

```c
bool front_plausible = false;
float c_front = 0.0f;

if (num_vectors_front > 0) {
    float angle_jump  = fabsf(g_traj.trajectory_angle_deg - s_last_front_angle);
    float offset_jump = fabsf(g_traj.trajectory_offset_px - s_last_front_offset);

    /* Allow wide angles up to physical camera limit (75 deg), but reject unphysical jumps */
    if (fabsf(g_traj.trajectory_angle_deg) <= 75.0f &&
        angle_jump <= 25.0f &&
        offset_jump <= 18.0f) {
        front_plausible = true;
        c_front = (num_vectors_front >= 2) ? 1.0f : 0.60f;
        s_last_front_angle  = g_traj.trajectory_angle_deg;
        s_last_front_offset = g_traj.trajectory_offset_px;
    }
}
```

### 3.4 Steering Synthesis & Output Slew Limiting
$$\delta_{\text{top}} = -\left( K_{p,\text{top}} \cdot e_{\text{cte}} + K_{d,\text{top}} \cdot \dot{e}_{\text{cte}} + K_{\text{heading}} \cdot \psi_{\text{filtered}} \right)$$
$$\delta_{\text{front}} = -\left( K_{p,\text{front}} \cdot \theta_{\text{traj,filtered}} + K_{\text{offset,front}} \cdot \Delta x_{\text{traj}} \right)$$

* Baseline front gains: $K_{p,\text{front}} = 1.00\text{ [deg/deg]}$, $K_{\text{offset,front}} = 0.35\text{ [deg/px]}$, $K_{d,\text{front}} = 0.0$.
* Low-pass filter on front trajectory: $\theta_k = 0.40 \cdot \theta_{\text{raw}} + 0.60 \cdot \theta_{k-1}$.
* **Handling Invalid Front Camera:** If Top is blind ($C_{\text{top}} == 0$) and Front is implausible (`front_plausible == false`), decay the previous steering command: $\delta_{\text{front}} = 0.90 \cdot \delta_{\text{cmd}, k-1}$.

**Servo Slew Rate Limiter:**
$$\delta_{\text{raw}} = \alpha \cdot \delta_{\text{top}} + (1.0 - \alpha) \cdot \delta_{\text{front}}$$
$$\Delta \delta = \text{clamp}(\delta_{\text{raw}} - \delta_{\text{cmd}, k-1}, -12.0^\circ, +12.0^\circ)$$
$$\delta_{\text{cmd}} = \text{clamp}(\delta_{\text{cmd}, k-1} + \Delta \delta, -77.5^\circ, +77.5^\circ)$$

---

## 4. Subsystem 2: Gated Intersection Navigation

### 4.1 Trigger Conditions (Strictly Gated)
An intersection state is activated **only** when the specific transverse tape signature is observed while the vehicle is traveling roughly straight (preventing false corner triggers):

```c
bool is_roughly_straight = (fabsf(filtered_heading) <= 15.0f) && (fabsf(steer_output) <= 18.0f);
bool has_transverse_tape = (g_traj.has_transverse_tape || top_has_transverse_tape);

if (!g_intersection_active && has_transverse_tape && is_roughly_straight) {
    g_intersection_active = true;
    s_intersection_ticks = 0;
    s_latched_steer = steer_output; /* Latch pre-entry steering angle */
    s_latched_heading = filtered_heading;
}
```

### 4.2 Gated Candidate Pair Filtering Across Intersection
During `g_intersection_active == true`:
* Candidate pairs are filtered against latched heading $\psi_{\text{latched}}$:
  $$\Delta \psi = |\theta_{\text{pair}} - \psi_{\text{latched}}|$$
* If $\Delta \psi > 35.0^\circ$, candidate pair is **hard-rejected** (eliminating $\pm 45^\circ..\pm 90^\circ$ cross-branches).
* **Fallback Steering Across Intersection:** If no forward longitudinal pair is visible, steer with latched entry angle: $\delta = s\_latched\_steer \cdot 0.95^k$.

### 4.3 Exit & Speed-Scaled Timeout Backstop
The intersection box is $\sim 0.40\text{ m}$ to $0.50\text{ m}$ wide. At target speed $v$, the maximum timeout ticks are scaled:
$$\text{timeout\_ticks} = \text{clamp}\left( \frac{D_{\text{box}}}{v \cdot \Delta t \cdot k_{\text{speed}}}, 12, 28 \right)$$
* Intersection deactivates when clear longitudinal lines ($\ge 2$ vectors) are tracked for 3 consecutive ticks, or when `s_intersection_ticks >= timeout_ticks`.

---

## 5. Subsystem 3: Speed Governor & Confidence Coupling

### 5.1 Preview Speed Governor with Asymmetric Filter
Speed targets are computed from lookahead curvature $\kappa$ with fast attack (0.70) into corners and slow decay (0.15) on exit:
$$v_{\text{target}} = v_{\text{straight}} - \kappa \cdot (v_{\text{straight}} - v_{\text{corner}})$$

* $v_{\text{straight}} = 92\%$ (reduced from 100% for stability).
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

## 7. Subsystem 5: Telemetry Stream Expansion

To enable complete side-by-side calibration of $\delta_{\text{top}}$ vs. $\delta_{\text{front}}$ and observability into fusion and gating:
```
L:2/3 R:-10 F:-11 H:6 ST:-15 SF:-12 S:-14 BW:85 DOM:T FV:1 IX:0 ST:N VL:88 VR:88
```
* **`ST:xx`**: Top steering command $\delta_{\text{top}}$ (deg $\times 10$).
* **`SF:xx`**: Front steering command $\delta_{\text{front}}$ (deg $\times 10$).
* **`S:xx`**: Blended, slew-limited servo command $\delta_{\text{cmd}}$ (deg $\times 10$).
* **`BW:xx`**: Blend weight $\alpha$ percentage (`0` to `100`).
* **`DOM:x`**: Dominant source (`T` = Top, `F` = Front, `B` = Blended).
* **`FV:x`**: Front validity flag (`1` = valid, `0` = implausible).
* **`IX:x`**: Intersection active flag (`1` = active, `0` = normal).

---

## 8. Implementation Roadmap

1. **Phase 1: Dual-Camera Blending, Slew Limiting & Calibration Telemetry:**
   - Asymmetric blend weight $\alpha$ ($\Delta_{\text{fall}} = 0.50, \Delta_{\text{rise}} = 0.08$).
   - Top PID freeze/re-seed and Front temporal plausibility filter.
   - Output servo slew limiter ($\le 12^\circ/\text{tick}$).
   - Confidence-based speed caps and dual-blind `STATE_LOST_LINE`.
   - Side-by-side telemetry stream (`ST:`, `SF:`, `S:`, `BW:`, `DOM:`, `FV:`).
   - Simulation verification of calibration and transition smoothness.

2. **Phase 2: Gated Intersection Navigation:**
   - Transverse tape + straight-approach intersection trigger.
   - Gated candidate pair $\Delta \psi \le 35^\circ$ filter and latched steering fallback.
   - Speed-scaled timeout backstop.
   - Simulation verification on synthetic 4-way crossroad.

3. **Phase 3: Hardware Track Validation & Speed Governor Dial-In:**
   - Bench test: verify $\delta_{\text{top}}$ and $\delta_{\text{front}}$ track closely on straightaway and turns.
   - Track test: verify inner loop navigation without boundary loss and clean intersection crossing.
   - Dial in corner speeds starting at 45%.
