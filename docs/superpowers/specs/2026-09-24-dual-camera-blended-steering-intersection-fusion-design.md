# Design Specification: Continuous Dual-Camera Steering Blend & Heading-Aligned Intersection Navigation

- **Date:** 2026-09-24
- **Target Platform:** NXP FRDM-MCXN947 (dual Cortex-M33, FreeRTOS, dual Pixy2 cameras)
- **Status:** Approved by User, Ready for Implementation Planning

---

## 1. Executive Summary & Root Cause Analysis

Telemetry from the `lookahead-prebrake-v1` track run revealed a critical asymmetry between the vehicle's two vision sensors:
* **Top Camera (~25 cm lookahead):**
  - Valid 2-vector lane pair: **56.8%** of the run (84 frames).
  - Single boundary vector: **22.3%** of the run (33 frames).
  - Completely blind (0 vectors): **20.9% of the run (31 frames)!**
* **Front Horizon Camera (~1.0 m lookahead):**
  - Uptime (1 to 5 vectors visible): **96.6% of the run** (143 frames).
  - 2+ vectors visible: **81.7% of the run** (121 frames).
  - Completely blind (0 vectors): **Only 3.4% of the run** (5 frames).

### The Two Core Failure Modes
1. **Corner Blindness & Wall Crossing:** In sharp turns (the inner loop and hairpins), the tape rotates out of the Top camera's narrow near-field field-of-view. Because lateral control previously gave the Top camera 100% ownership of steering and completely forbade the Front camera from steering, `Top: 0` triggered an immediate transition into `STATE_LOST_LINE`. The steering froze/decayed, motor throttle cut to zero, and the vehicle coasted across boundary lines — despite the Front camera having 2–4 vectors in clear view 1 meter ahead.
2. **Intersection Wrong-Way Turning ("Ring Around the Rosie"):** At 4-way intersections and branches, the vector pair-scorer previously selected candidate pairs based solely on proximity to the bumper and nominal width. When the vehicle approached an intersection, side branches turning $\pm 45^\circ$ to $\pm 90^\circ$ entered the camera view and outscored the forward continuation lane. The car turned 90° into the inner loop instead of crossing straight through to the opposite side as required by NXP Cup competition rules.

---

## 2. System Architecture

```
                       +----------------------------------------+
                       |    Top Camera Pixy2 (~25 cm near)      |
                       +----------------------------------------+
                                         |
                                         v
                         Top Confidence: C_top in [0.0, 1.0]
                         (2 vectors = 1.0, 1 vector = 0.50, 0 = 0.0)
                                         |
                                         v
                       +----------------------------------------+
                       |      Slew-Rate Limiter (Delta = 0.15)  |
                       +----------------------------------------+
                                         |
                                         v
                         Smooth Blend Weight: alpha in [0.0, 1.0]
                                         |
               +-------------------------+-------------------------+
               |                                                   |
               v                                                   v
   +-----------------------+                           +-----------------------+
   |  Top Steer (delta_top)|                           | Front Steer (delta_fr)|
   |  PID CTE + Heading FF |                           | Lookahead Trajectory  |
   +-----------------------+                           +-----------------------+
               \                                                   /
                \-----> [ delta_cmd = alpha*delta_top + (1-alpha)*delta_fr ] <---/
                                         |
                                         v
                       +----------------------------------------+
                       |  Servo Output Limiter (-77.5 .. +77.5) |
                       +----------------------------------------+
```

---

## 3. Subsystem 1: Continuous Dual-Camera Steering Blend

### 3.1 Sensor Confidence Metric ($C_{\text{top}}$)
At each 20 ms control tick:
```c
float c_top = 0.0f;
if (num_vectors_top >= 2) {
    c_top = 1.0f;
} else if (num_vectors_top == 1) {
    c_top = 0.50f;
} else {
    c_top = 0.0f;
}
```

### 3.2 Slew-Rate Limiter for Smooth Blend Weight ($\alpha$)
To prevent steering jerks or oscillations during rapid camera transitions:
```c
#define BLEND_WEIGHT_SLEW_RATE_MAX 0.15f /* Max change per 20ms tick (~133ms full transition) */

float delta_alpha = c_top - s_blend_weight;
if (delta_alpha > BLEND_WEIGHT_SLEW_RATE_MAX) {
    delta_alpha = BLEND_WEIGHT_SLEW_RATE_MAX;
} else if (delta_alpha < -BLEND_WEIGHT_SLEW_RATE_MAX) {
    delta_alpha = -BLEND_WEIGHT_SLEW_RATE_MAX;
}
s_blend_weight += delta_alpha;
if (s_blend_weight > 1.0f) s_blend_weight = 1.0f;
if (s_blend_weight < 0.0f) s_blend_weight = 0.0f;
```

### 3.3 Steering Command Synthesis
The commanded steering angle $\delta_{\text{cmd}}$ combines both sources:
$$\delta_{\text{cmd}} = \alpha \cdot \delta_{\text{top}} + (1.0 - \alpha) \cdot \delta_{\text{front}}$$

* **$\delta_{\text{top}}$ Calculation:**
  $$\delta_{\text{top}} = -\left( \text{PID}_{\text{cte}}(e_{\text{cte}}) + K_{\text{heading}} \cdot \psi_{\text{filtered}} \right)$$
* **$\delta_{\text{front}}$ Calculation:**
  $$\delta_{\text{front}} = -\left( K_{p,\text{front}} \cdot \theta_{\text{traj}} + K_{d,\text{front}} \cdot \dot{\theta}_{\text{traj}} + K_{\text{offset}} \cdot \Delta x_{\text{traj}} \right)$$
  - Nominal gains: $K_{p,\text{front}} = 1.20$, $K_{d,\text{front}} = 0.08$, $K_{\text{offset}} = 0.40$.

### 3.4 Dual-Blind Safety (`STATE_LOST_LINE`)
A line loss event is re-scoped to require **both** cameras to be blind:
```c
bool dual_camera_blind = (num_vectors_top == 0) && (num_vectors_front == 0);
if (dual_camera_blind) {
    s_lost_line_ticks++;
} else {
    s_lost_line_ticks = 0;
}

if (s_lost_line_ticks > LOST_LINE_TOLERANCE_TICKS) {
    state = STATE_LOST_LINE;
}
```
If the Top camera loses tracking but the Front camera sees lines, `s_lost_line_ticks` resets to 0 and the Front camera steers the car through the turn without dropping into recovery mode.

---

## 4. Subsystem 2: Heading-Aligned Intersection Navigation

### 4.1 Competition Rule
> "Every time the vehicle encounters an intersection, it must exit on the opposite side it entered (straight through)."

### 4.2 Heading Alignment Penalty in Candidate Pair Selection
In [`source/vectors.c`](file:///d:/nxpcup/source/vectors.c#L157-L195), candidate vector pairs are currently evaluated based on width error and length. We introduce a heading-continuity term comparing candidate pair angle $\theta_{\text{pair}}$ against vehicle entry heading $\psi_{\text{car}}$:

$$\theta_{\text{pair}} = \frac{1}{2} (\text{heading}_{\text{left}} + \text{heading}_{\text{right}})$$
$$\Delta \theta = |\theta_{\text{pair}} - \psi_{\text{car}}|$$

```c
/* Heavily penalize candidate pairs turning off into crossroad branches (e.g. 45 to 90 deg) */
float heading_alignment_penalty = 0.0f;
if (delta_theta > 15.0f) {
    heading_alignment_penalty = (delta_theta - 15.0f) * 12.0f;
}

float score = min_ny * 10.0f - width_err * 8.0f + len_sum * 0.5f - heading_alignment_penalty;
```

* **Straight-through lane:** $\Delta \theta \approx 0^\circ \to$ penalty $= 0.0$.
* **Side branch turning $45^\circ$ to $90^\circ$:** $\Delta \theta \ge 45^\circ \to$ penalty $\ge 360.0$, disqualifying cross branches.

### 4.3 Lookahead Horizon Projection
Because the Front camera surveys $1.0\text{ m}$ ahead, it captures the exit corridor of a standard NXP Cup intersection (box width $\approx 0.35\text{ m}$ to $0.50\text{ m}$) before the front bumper crosses the entrance line. By filtering candidate pairs for alignment with $\psi_{\text{car}}$, the Front camera locks onto the exit lane on the opposite side and guides the vehicle cleanly across in closed loop.

---

## 5. Subsystem 3: Telemetry Stream Expansion

To enable real-time observability and post-run tuning of the fusion controller, the telemetry string in [`source/wifi_config.c`](file:///d:/nxpcup/source/wifi_config.c#L110) will be updated:

```
L:2/3 R:-10 F:-11 H:6 TJ:27 S:-5 LA:0 BW:95 DOM:T ST:N VL:98 VR:98
```
* **`BW:xx`**: Blend weight $\alpha$ percentage (`0` to `100`).
* **`DOM:x`**: Dominant steering authority:
  - `T`: Top camera dominant ($\alpha \ge 0.70$).
  - `F`: Front camera dominant ($\alpha \le 0.30$).
  - `B`: Blended ($0.30 < \alpha < 0.70$).

---

## 6. Implementation & Verification Roadmap

### Phase 1: Dual-Camera Blending & Steering Fallback
1. Implement confidence calculation and slew-limited blend weight $\alpha$ in [`source/main.c`](file:///d:/nxpcup/source/main.c).
2. Synthesize front-camera steering command $\delta_{\text{front}}$.
3. Update `STATE_LOST_LINE` condition to require dual-camera blindness.
4. Add `BW:` and `DOM:` to serial telemetry stream.
5. Unit test in simulation: verify transition smoothness and absence of steering discontinuities when Top vectors drop from 2 to 0.

### Phase 2: Heading-Aligned Intersection Exit Selection
1. Update vector pair scoring in [`source/vectors.c`](file:///d:/nxpcup/source/vectors.c) with heading alignment penalty.
2. Unit test with synthetic 4-way intersection vector data: verify candidate selection strictly prefers the straight-through exit lane and rejects 90° cross-branches.

### Phase 3: Track Validation & Speed Governor Tuning
1. Deploy to hardware and test on inner loop at conservative speed (50–60%).
2. Verify via telemetry that during Top camera dropouts, `DOM` switches to `F` and vehicle stays centered on the track.
3. Verify clean 4-way intersection crossing without turning into crossroad branches.
4. Iteratively dial in corner speeds (starting from 45% on tight turns) based on track boundary margins.
