# Decoupled Dual-Camera Architecture & Lookahead Speed Governor Design

**Date**: 2026-09-23  
**Status**: Approved (Ready for Implementation Planning)  
**Target Hardware**: NXP MCX-N947 (ARM Cortex-M33 @ 150MHz)  
**Target Vehicle**: NXP Cup 1/10 Autonomous Model Car  

---

## 1. Executive Summary & Problem Diagnosis

### 1.1 Root Cause of Driving Degradation
In previous iterations, the vehicle suffered from severe heuristic coupling between its two cameras:
1. **Camera Cross-Contamination**: On short straightaways (1.5m – 2.0m), the front lookahead camera previewed the *next* upcoming curve 1.5m – 2.5m ahead. Feeding this angle directly into the steering servo caused the wheels to angle $15^\circ–30^\circ$ into the corner prematurely while still on the straight, cutting diagonally across the lane and destabilizing the base camera.
2. **Speed Throttling on Straights**: The front camera's corner intensity factor dragged longitudinal speed down from 100% to 74% and dragged the inside wheel down to 50% while driving down straightaways.
3. **Bloated PID Gains Fighting Lookahead**: Because steering was fighting early lookahead angles, gains were ratcheted up to extreme levels ($K_p = 3.50, K_d = 0.20, K_{heading} = 1.30$), causing violent steering twitches.
4. **False Pairing on Complex Curves**: Comparing horizontal separation between arbitrary vector endpoints caused peeling branches and curved lines at disparate $Y$ heights to be mistakenly paired with lane markings, creating erratic lookahead midpoints.
5. **Vector Flicker Fragility**: When Pixy2 momentarily dropped a vector for 1–2 frames due to chassis vibration or glare, the car would interpret this as line loss and trigger emergency behaviors or state chatter.

### 1.2 The Solution
This design introduces a **strictly decoupled dual-camera architecture** running at **50 Hz**:
* **Base Camera (`cam_pid`, 50 Hz)**: 100% exclusive authority over the steering servo. Zero lookahead steering feedforward.
* **Front Camera (`cam_front`, 50 Hz)**: 100% exclusive authority over longitudinal velocity planning and state transitions (`DRIVE_FORWARD` vs `DRIVE_TURNING`).
* **Foreground Root Anchor Filtering**: Bounded bottom-up vector selection that ignores distant branches, room clutter, and center dashed lines.
* **3-Tick Hold-Last-Valid & Top-Cam Sanity Veto**: Complete immunity to 1–3 frame optical flicker dropouts.

---

## 2. System Architecture & Timing Profile

```
+-----------------------------------------------------------------------------------+
|                                   NXP MCX-N947                                    |
|                                                                                   |
|    [ Base Camera: 50 Hz (20ms) ]                [ Front Camera: 50 Hz (20ms) ]    |
|    LP_FLEXCOMM3 (eDMA Ch 2-3) @ 400kHz          LP_FLEXCOMM2 (eDMA Ch 0-1) @ 400kHz|
|    • Tilted down ~45°                           • Tilted down ~15°                |
|    • Surveys 0.1m - 0.4m ahead                  • Surveys 1.0m - 2.0m ahead       |
|    • Clean floor / homography                   • Foreground Root Anchor          |
|    • Computes CTE & Heading                     • Computes X_lookahead @ Y=20     |
|                 │                                              │                  |
|                 │ (CTE, Heading)                               │ (State, Speed)   |
|                 ▼                                              ▼                  |
|        [ Steering Pipeline ]                         [ Velocity Pipeline ]        |
|        • PD Centering (Rationalized Kp)              • 100% Symmetrical Sprint    |
|        • Heading Deadband Feedforward                • Lookahead Pre-Brake        |
|        • Slew Limiter (650 deg/s)                    • Differential Pivot Drag    |
|        • ZERO Lookahead FF                           • Top-Cam Sanity Guard       |
|                 │                                              │                  |
|                 ▼                                              ▼                  |
|        [ Steering Servo PWM ]                        [ Dual Motor H-Bridge PWM ]  |
+-----------------------------------------------------------------------------------+
```

### 2.1 Hardware Timing & 50 Hz Synchronization
* **Clock & Loop**: SysTick configured at **50 Hz ($dt = 20\text{ms}$)**.
* **Camera Polling**: Both cameras are polled **every tick (50 Hz)**. The legacy alternating divider (`front_cam_phase++ & 1`) is removed.
* **Bus Isolation**: `LP_FLEXCOMM2` and `LP_FLEXCOMM3` operate on independent hardware I2C peripherals with dedicated eDMA channels, preventing any bus contention.
* **Sensor Latency**: Pixy2 runs at 60 FPS (~16.6ms). 50 Hz polling guarantees that the freshest frame is consumed within 20ms.

---

## 3. Front Camera Computer Vision Pipeline

```
Y=0 (Furthest Horizon)
       ───────────────────────────────────  Clip upper 25% (Y < 13: walls, lights)
       \                                 /
        \        Target Horizon         /   Sample Lookahead Point: Y = 20 (~1.5m ahead)
         \                             /
          \                           /
           \                         /
            \                       /
Y=51 ────────*─────────────────────*──────  Foreground Anchor: Must start at bumper (Y >= 30)
          Left Root            Right Root
          (X <= 34)            (X >= 45)
          
      [ Center Dash Zone X in [35, 44] is strictly excluded at bumper ]
```

### 3.1 Vector Filtering & Foreground Root Anchor
Raw vectors from `pixy_get_vectors(&cam_front)` undergo the following stages:
1. **Horizon Discard**: Any vector whose endpoints lie entirely within $Y < 13$ is discarded.
2. **Horizontal Rejection**: Any vector with $\Delta y < 6\text{px}$ is rejected to eliminate 4-way intersection crossing lines.
3. **Foreground Anchor Gate**: A boundary vector must touch the near road directly in front of the vehicle:
   $$Y_{near} \ge 30$$
   Any floating segment or branch starting higher up ($Y_{near} < 30$) is excluded.
4. **Bumper Spatial Classification**:
   * **Left Root Candidate**: Vector with $X_{near} \le 34.0$ having maximum $Y_{near}$ (deepest into the foreground bumper zone) and maximum length.
   * **Right Root Candidate**: Vector with $X_{near} \ge 45.0$ having maximum $Y_{near}$ and maximum length.
   * **Center Dash Exclusion**: Any line anchored in $X_{near} \in (34.0, 45.0)$ at $Y_{near} \ge 30$ is strictly rejected, preventing center dashed lines from being picked as outer boundaries.

### 3.2 Lookahead Point Evaluation
Once Left Root and Right Root are identified:
1. Both vectors are projected to the target lookahead horizon at $Y = 20$:
   $$X_{L,20} = X_{n,L} + (X_{f,L} - X_{n,L}) \cdot \frac{20 - Y_{n,L}}{Y_{f,L} - Y_{n,L}}$$
   $$X_{R,20} = X_{n,R} + (X_{f,R} - X_{n,R}) \cdot \frac{20 - Y_{n,R}}{Y_{f,R} - Y_{n,R}}$$
2. The lookahead centerline point is:
   $$X_{lookahead} = \frac{X_{L,20} + X_{R,20}}{2}$$
3. **$\pm 5.0\text{px}$ Deadband**:
   * $\text{offset} = |X_{lookahead} - 39.5|$
   * If $\text{offset} \le 5.0\text{px} \implies \text{urgency} = 0.0$ (**Straightaway Sprint**).
   * If $\text{offset} > 5.0\text{px} \implies$ corner urgency scales linearly:
     $$\text{urgency} = \text{clamp}\left(\frac{\text{offset} - 5.0\text{px}}{20.0\text{px} - 5.0\text{px}}, 0.0, 1.0\right)$$

---

## 4. Dropout Immunity & State Transition Logic

### 4.1 3-Tick "Hold-Last-Valid" Filter
To absorb Pixy2 optical flicker (1–2 frame dropouts caused by vibration or lighting changes):
* **When both Root Vectors are detected**:
  * $X_{lookahead}$ is updated normally.
  * `last_valid_lookahead_x = X_lookahead`.
  * `dropout_counter = 0`.
* **When 1 Root Vector is missing**:
  * Increment `dropout_counter++`.
  * If `dropout_counter <= 3` (within $60\text{ms}$ at 50 Hz):
    * **COAST**: Retain `last_valid_lookahead_x`.
    * Do **not** trigger emergency braking or state transition.
  * If `dropout_counter >= 4` ($> 80\text{ms}$ sustained single-line loss):
    * Evaluate **Top Camera Sanity Veto** (§4.2).

### 4.2 Top Camera Sanity Veto
Before confirming a corner based on front camera line loss:
* Inspect base camera:
  $$\text{Is } |H_{base}| \le 3.0^\circ \quad \text{AND} \quad |CTE_{base}| \le 4.0\text{px}?$$
* **If TRUE**: The vehicle is physically centered on a straight track. The front camera line dropout is a lighting/shadow artifact $\implies$ **VETO the dropout; remain in `DRIVE_FORWARD`**.
* **If FALSE**: The car is drifting, turning, or the road is curving $\implies$ **CONFIRM `DRIVE_TURNING`**.

### 4.3 Hysteresis State Machine
* **Enter `DRIVE_TURNING`**: Requires **2 consecutive ticks (40ms)** of confirmed corner urgency ($X_{lookahead}$ offset $> 6.0\text{px}$ or confirmed line loss).
* **Return to `DRIVE_FORWARD`**: Requires **2 consecutive ticks (40ms)** of confirmed centered road ($X_{lookahead}$ offset $< 4.0\text{px}$).

---

## 5. Control Pipeline & Actuator Mapping

### 5.1 Steering Authority (100% Base Camera)
$$\text{raw\_steer} = u_{pid}(CTE) + FF_H(Heading)$$

* **Cross-Track Error (PID)**:
  * Low-pass filtered CTE: $CTE_{filt} = 0.40 \cdot CTE + 0.60 \cdot CTE_{filt, prev}$.
  * $u_{pid} = -K_p \cdot k_{cte} \cdot CTE_{filt} - K_d \cdot D_{filt}$.
  * **Rationalized Gains**:
    * $K_{p,straight} = 0.75$, $K_{p,corner} = 1.60$ (scheduled by corner urgency).
    * $K_{d} = 0.180$ (constant derivative damping).
    * $k_{cte} = 0.60$.
* **Heading Feedforward ($FF_H$)**:
  * Continuous deadband:
    $$H_{eff} = \begin{cases} (|H_{filt}| - 4.0^\circ) \cdot \text{sgn}(H_{filt}) & \text{if } |H_{filt}| > 4.0^\circ \\ 0.0 & \text{otherwise} \end{cases}$$
    $$FF_H = k_{heading} \cdot H_{eff} \quad (k_{heading} = 1.10)$$
* **Zero Lookahead Steering Feedforward**:
  $$FF_{LA} = 0.0 \quad \text{(Strictly decoupled)}$$
* **Slew Rate Limiter**:
  $$\Delta \text{steer} = \text{clamp}(\text{raw\_steer} - \text{last\_steer}, -13.0^\circ, +13.0^\circ) \quad (650^\circ/\text{s} \text{ at } 50\text{Hz})$$
  $$\text{steer\_cmd} = \text{clamp}(\text{last\_steer} + \Delta \text{steer}, -77.5^\circ, +52.5^\circ)$$
  $$\text{Servo Angle} = \text{steer\_cmd} + 12.5^\circ \quad (\text{STEERING\_OFFSET})$$

### 5.2 Motor Speed & Differential Torque Vectoring
* **In `DRIVE_FORWARD`**:
  $$V_{left} = 100.0\%, \quad V_{right} = 100.0\%$$
  * Zero differential drag. 100% symmetrical full-power sprint.
* **In `DRIVE_TURNING`**:
  * Base longitudinal speed modulates based on lookahead corner urgency:
    $$V_{base} = \text{SPEED\_STRAIGHT} - \text{urgency} \cdot (\text{SPEED\_STRAIGHT} - \text{SPEED\_OUTSIDE\_CORNER})$$
    *(At calibrated 12.6V: $\text{SPEED\_STRAIGHT} = 100.0\%$, $\text{SPEED\_OUTSIDE\_CORNER} = 86.0\%$)*.
  * Differential pivot drag applies strictly to the inside wheel based on physical steering angle:
    $$\Delta V_{diff} = \text{steer\_norm} \cdot (V_{base} - \text{SPEED\_INSIDE\_MIN}) \quad (\text{SPEED\_INSIDE\_MIN} = 65.0\%)$$
    * Right Turn ($\text{steer} > 0$): $V_{left} = V_{base}$, $V_{right} = V_{base} - \Delta V_{diff}$.
    * Left Turn ($\text{steer} < 0$): $V_{right} = V_{base}$, $V_{left} = V_{base} - \Delta V_{diff}$.

---

## 6. Verification & Test Suite Plan

The standalone host simulator (`scratch/test_comprehensive_suite.c`) will be refactored to validate these exact requirements without hardware:

| Test # | Test Name | Scenario & Input Stimulus | Pass / Fail Criteria |
|---|---|---|---|
| **Test 1** | **Camera Decoupling on Straights** | Straight track ($CTE=0, H=0$) with a sharp 90° curve ($X=20$) visible ahead on the front camera. | `steer_cmd == 0.0°` (zero early steering); vehicle remains in `DRIVE_FORWARD` at $VL=100, VR=100$. |
| **Test 2** | **1–3 Frame Optical Flicker Immunity** | 100% straight sprint; simulate 1, 2, and 3 frame dropouts of the right vector. | Zero state change (`DRIVE_FORWARD`); zero speed drop ($VL=100, VR=100$); zero steering twitch. |
| **Test 3** | **Top-Camera Sanity Veto on Dropouts** | Front camera drops line for 6 frames; base camera reports $H=0^\circ, CTE=0\text{px}$. | Top camera vetoes dropout; state remains `DRIVE_FORWARD`; no false braking. |
| **Test 4** | **Foreground Root Anchor & False-Pair Rejection** | 4 vectors present: Foreground Left, Foreground Right, Center Dash, and Peeling Horizon Branch (User Sketch). | Root anchor locks strictly to Foreground Left and Right; center dash and branch completely ignored; lookahead midpoint centered within $\pm 1\text{px}$. |
| **Test 5** | **Real Corner Pre-Braking & 50 Hz Transition** | Lookahead point shifts to $X=22\text{px}$ ($> 5\text{px}$ deadband). | State transitions to `DRIVE_TURNING` within $\le 2$ ticks (40ms); $V_{base}$ smoothly pre-brakes to 86%. |
| **Test 6** | **Chicane Reversal & Slew Limiter Anti-Windup** | Rapid reversal from full right ($+52.5^\circ$) to full left ($-77.5^\circ$). | Slew rate capped at $\le 13.0^\circ/\text{tick}$; motor speeds transition without inside wheel spinout. |
| **Test 7** | **Numerical Fuzzing Immunity** | Extreme inputs ($0.0, \pm 10^6$, NaN, Inf, empty vector arrays). | Zero crashes, zero NaNs, all commands strictly clamped to physical actuator limits. |
