# Design Spec: Dual-Camera Lookahead Pre-Braking & Speed Governor

- **Date:** 2026-09-24
- **Status:** Approved
- **Target:** NXP FRDM-MCXN947 (RoboCar NXP Cup)
- **Primary Workspaces:** `d:/nxpcup` and `D:/robotica/workspace/nxpcup-ciorba_curata`

---

## 1. Problem Statement & Context

In high-speed sprint runs, the vehicle enters Corner 1 at excessive velocity ($\approx 95\% - 98\%$ duty, $\approx 2.5\text{ m/s}$), fails to make the turn, and overshoots the outer boundary into `STATE_LOST_LINE`.

### Root Cause Analysis
1. **Sensor Range Mismatch:** Lateral steering is commanded by the top/downward camera (`cam_pid`), which looks only $\approx 25\text{ cm}$ ahead of the front bumper. At $2.5\text{ m/s}$, $25\text{ cm}$ corresponds to only $100\text{ ms}$ of reaction time—far too short to shed chassis momentum.
2. **Artificial Horizon Restriction:** The front camera (`cam_front`), positioned to look $\approx 1.0\text{ m}$ down the track, had its vector processing gated by `ny[i] >= FORWARD_CAM_ROOT_ANCHOR_Y_MIN (30U)`. Any curve located $0.5\text{ m} - 1.2\text{ m}$ ahead appears in the upper/middle frame ($Y \in [13, 30]$) and was **discarded** as background clutter until the bumper was nearly touching the tape.
3. **Speed Governor Lag:** Because upcoming curvature was invisible to the firmware until $Y \ge 30$, target speed did not begin decelerating until the car had already crossed into the curve.

---

## 2. Architecture & Separation of Concerns

The dual-camera system follows a strict **Option B: Separation of Concerns** design:

```
[ Front Camera (~1.0m Lookahead) ]
              │
              ▼
   Preview Vector Analysis
   (Horizon Window Y: 13..45)
              │
              ▼
   Curvature Preview (κ_preview) ──────┐
                                       │
[ Top Camera (~0.25m Ground View) ]    ▼
              │                [ Speed Governor ]
              ▼                        │
   Precise CTE & Heading Feedforward   ▼
              │                Target Speed (100% -> 58%)
              ▼                        │
       [ Steering Servo ]       [ H-Bridge Motors ]
```

- **Top Camera (~25 cm)**: 100% owner of lateral control (lane position `CTE`, vehicle alignment `Heading`, and steering PID). It does **not** command vehicle speed.
- **Front Camera (~1.0 m)**: 100% owner of longitudinal control (early curve preview $\to \kappa_{\text{preview}} \to$ continuous speed governor). It does **not** steer the servo.

---

## 3. Detailed Component Design

### 3.1 Front Camera Preview Vector Pipeline (`source/vectors.c`)
A dedicated preview function, `compute_lookahead_curvature`, evaluates vectors in the extended horizon zone:

1. **Horizon Window ($Y \in [13, 45]$)**:
   - Discards ceiling lights and room walls ($Y < 13$).
   - Discards near-bumper vectors ($Y > 48$) to prevent front bumper shadows/chassis reflection.
   - **Removes the $Y \ge 30$ bumper anchor constraint.**
2. **Curvature Extraction**:
   - **C1: Transverse Vector Detection**: Sharp $90^\circ$ turns appear in the forward view as diagonal or horizontal tape lines ($\Delta X \ge 16\text{px}, \Delta Y \le 10\text{px}$). Detecting a transverse tape segment immediately commands maximum curvature $\kappa = 1.0$.
   - **C2: Track Midpoint Lateral Displacement**: Computes the lateral center of visible lane boundaries at horizon $Y=22$. Lateral displacement off center ($|\Delta X| > 4.0\text{px}$) scales curvature proportionally up to $\pm 14.0\text{px}$.
   - **C3: Vector Angle Convergence**: Angles of boundary vectors in the preview zone tilt in the direction of upcoming curves, reinforcing the curvature metric.

### 3.2 Continuous Speed Governor (`source/main.c`)
1. **Curvature Metric Fusion**:
   $$\kappa = \max(\kappa_{\text{preview}}, \kappa_{\text{local}})$$
   where $\kappa_{\text{preview}}$ is derived from the front camera and $\kappa_{\text{local}}$ is derived from the top camera heading.
2. **Speed Mapping**:
   $$v_{\text{target}} = v_{\text{straight}} - \kappa \cdot (v_{\text{straight}} - v_{\text{outside\_corner}})$$
   - On straightaways ($\kappa = 0$): $v_{\text{target}} = 100.0\%$.
   - In corners ($\kappa \to 1.0$): $v_{\text{target}} = 58.0\%$.
3. **Deceleration & Acceleration Timing**:
   - `DECEL_RAMP_STEP = 25.0f` per 20ms tick: Commands the $100\% \to 58\%$ drop in 40 ms.
   - With $1.0\text{ m}$ preview at $2.5\text{ m/s}$, preview warning arrives $\approx 350 - 400\text{ ms}$ before corner entry.
   - Chassis kinetic energy bleeds down smoothly, ensuring the car enters the apex **already at $58\%$ speed**.
   - `ACCEL_RAMP_STEP = 18.0f` per tick ensures smooth corner exit without breaking tire traction.

### 3.3 Lateral Steering Loop Preservation (`source/main.c`)
- The top camera maintains full ownership of lateral steering.
- The steering loop uses dynamic PID gain scheduling ($K_{p,\text{straight}} = 0.75$, $K_{p,\text{corner}} = 2.60$) based on $\kappa$.
- Apex protection attenuates counter-steering CTE if it opposes the curve direction upon corner entry.

---

## 4. Telemetry & Verification Protocol

### Live Serial Verification Stream
```text
L:2/3 R:0 F:0 H:0 TJ:-25 S:0 LA:85 ST:N VL:70 VR:70
```
- **Metric 1 (Early Warning):** `LA` ($\kappa_{\text{preview}}$) must jump from $0\%$ to $70\%-100\%$ while $H$ and $R$ are still $\approx 0$ (vehicle still on the straightaway, $\approx 1\text{m}$ before the turn).
- **Metric 2 (Pre-Braking):** `VL` and `VR` must ramp down from $100$ to $58$ **before** $H$ swings into the turn.
- **Metric 3 (Track Retention):** The vehicle must complete Corner 1 without crossing the outer boundary, with zero transitions into `STATE_LOST_LINE` (`ST: L`).

---

## 5. Rollout Plan
1. Commit this design specification to `docs/superpowers/specs/2026-09-24-dual-camera-lookahead-braking-design.md`.
2. Transition to the `writing-plans` skill to generate a structured implementation plan.
3. Implement `compute_lookahead_curvature` in `source/vectors.c`.
4. Connect preview curvature to the speed governor in `source/main.c`.
5. Verify on hardware and evaluate telemetry logs.
