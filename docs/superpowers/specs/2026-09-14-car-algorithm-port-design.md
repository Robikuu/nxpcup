# Advanced Vision & Asymmetric Differential Drag Port Design

## 1. Overview & Objectives
Port the advanced vision pipeline (geometric pair validation, perspective single-boundary synthesis, temporal tracking) and dynamic vehicle control algorithms (corner urgency gating, asymmetric differential drag with inside-wheel pivot braking, bilateral traction guard, and yaw-conflict chicane override) from `nxpcup_official` onto the current car (`nxpcup-ciorba_curata`).

The current car's specific hardware mappings, peripherals, button start/stop logic, and ESP32 telemetry/tuning bridge are fully preserved.

---

## 2. Hardware & Steering Kinematics Specification

### 2.1 Steering Geometry & Limits
* Physical Servo Left Stop: `STEERING_LIMIT_LEFT = -70.0f`
* Physical Servo Right Stop: `STEERING_LIMIT_RIGHT = +87.0f`
* Centerline Trim Offset: `STEERING_OFFSET = +17.0f`
* Relative Command Limits:
  * Maximum Right Turn Command: `+70.0f` (`87.0f - 17.0f`)
  * Maximum Left Turn Command: `-87.0f` (`-70.0f - 17.0f`)
  * Straight Ahead: `0.0f` (`17.0f - 17.0f`)
* Actuation: `Steer(steer_cmd + STEERING_OFFSET)`
  * `Steer()` clamps input angle to `[-70.0f, +87.0f]` and drives PWM duty cycle ($5\% \dots 10\%$).
  * Duplicate offset bug in `servo.c` is removed.
* Normalization for Differential Drag:
  * Deadzone: `DIFFERENTIAL_DEADZONE_DEG = 4.0f`
  * Right: `steer_norm = (steer_cmd - 4.0) / (70.0 - 4.0)`
  * Left: `steer_norm = (|steer_cmd| - 4.0) / (87.0 - 4.0)`

### 2.2 Preserved Peripherals
* Dual H-Bridge Motors: `CTIMER0` (Match 1 `P0_25`, Match 2 `P0_26`), DIR pins `GPIO0` pin 27 (`P0_27`) & pin 24 (`P0_24`).
* Steering Servo: `CTIMER2` Match 2 (`P1_12`).
* Pixy2 I2C Camera: `LP_FLEXCOMM2` on `P4_0` / `P4_1` via eDMA0.
* ESP32 Wi-Fi / ESP-NOW Bridge: `LP_FLEXCOMM9` on `P2_3` / `P2_4` (115200 baud).
* Start/Stop Push Button: `GPIO0` pin 23 (`P0_23`).

---

## 3. Vision Pipeline Specification (`vectors.c`, `vectors.h`)

### 3.1 Geometric Pair Selection (`vector_filtering`)
1. **Slope Filtering**: Rejects vectors where $|dx| > 2|dy|$.
2. **Endpoint Extraction**: Sorts each vector so $(x_{\text{near}}, y_{\text{near}})$ has higher $Y$ (closest to front bumper).
3. **Pair Evaluation**: Evaluates all combinations of $(i, j)$:
   * Rejects pairs where near-$X$ separation is less than `MIN_LANE_WIDTH_PX = 20.0f`.
   * Rejects pairs where any endpoints (near-near, near-far, far-near, far-far) are within `CONNECT_DIST_PX = 15.0f` (identifies fragmented segments of the same curving boundary).
   * Ranks candidates by $\max(\min(y_{\text{near}, i}, y_{\text{near}, j}))$ and closeness to nominal lane width ($50\text{ px}$).
4. **Ordering & Fallback**:
   * If pair found: places left boundary ($x_{\text{near}}$ smaller) in `vectors[0..3]` and right boundary in `vectors[4..7]`, `num_vectors = 2`.
   * If no pair found: selects the single vector with highest $y_{\text{near}}$ (tie-broken by squared length), `num_vectors = 1`.

### 3.2 Error Calculation & Perspective Synthesis (`compute_steering_error`)
1. **Temporal Memory**:
   * `reset_steering_tracking()` resets `s_last_left_near_x` ($14\text{ px}$) and `s_last_right_near_x` ($64\text{ px}$).
2. **Two Boundaries Visible**:
   * Computes near and far midpoints.
   * Cross-track error: $\text{CTE} = \text{mid}_{\text{near}, x} - \text{PIXY\_CENTER\_X}$.
   * Heading: $\theta = \operatorname{atan2}(dx, dy) \cdot \frac{180}{\pi}$.
   * Updates temporal memory with left/right near-$X$.
3. **Single Boundary Visible**:
   * Compares $x_{\text{near}}$ to `s_last_left_near_x` and `s_last_right_near_x` to reliably assign side.
   * Synthesizes centerline using perspective expansion:
     $w_{\text{near}} = 15.0 + (y_{\text{near}} / 51.0) \cdot 10.0$
     $w_{\text{far}} = 15.0 + (y_{\text{far}} / 51.0) \cdot 10.0$
   * Computes synthesized midpoint CTE and heading angle.

---

## 4. Drive & Dynamics Control Specification (`main.c`, `Config.h`)

### 4.1 Corner Urgency Gating & Lookahead Gain Scheduling
* Heading urgency: $|\theta| / 30^\circ$.
* CTE urgency: $|\text{CTE}_{\text{filtered}}| / 20\text{ px}$.
* Persistence filter: Corner urgency must persist for $2\text{ ticks}$ (`CORNER_URGENCY_STREAK`) to rise, but drops immediately.
* Scheduled gains:
  $$\text{schedule\_ratio} = \min(\text{speed\_ratio}, 1.0 - \text{trusted\_urgency})$$
  $$K_p = K_{p,\text{corner}} - \text{schedule\_ratio} \cdot (K_{p,\text{corner}} - K_{p,\text{straight}})$$
  $$K_d = K_{d,\text{corner}} + \text{schedule\_ratio} \cdot (K_{d,\text{straight}} - K_{d,\text{corner}})$$
* Default gains: $K_{p,\text{corner}} = 3.2$, $K_{p,\text{straight}} = 1.5$, $K_{d,\text{corner}} = 0.20$, $K_{d,\text{straight}} = 0.28$.

### 4.2 Asymmetric Differential Drag (Inside-Wheel Pivot Braking)
* Normalized steering error and heading error combined:
  $\text{corner\_intensity} = \max(\text{steer\_norm}, \text{heading\_norm})$.
* Target wheel speeds:
  * Outside wheel: $V_{\text{outside}} = \text{SPEED\_STRAIGHT} - \text{intensity} \cdot (\text{SPEED\_STRAIGHT} - \text{SPEED\_OUTSIDE\_CORNER})$ ($100 \to 88$).
  * Inside wheel: $V_{\text{inside}} = \text{SPEED\_STRAIGHT} - \text{intensity} \cdot (\text{SPEED\_STRAIGHT} - \text{SPEED\_INSIDE\_MIN})$ ($100 \to 25$).

### 4.3 Bilateral Anti-Spin Traction Guard & Yaw Conflict Chicane Override
* Standard rate limits:
  * Acceleration ramp: `ACCEL_RAMP_STEP = 6.0f` per tick.
  * Deceleration ramp: `DECEL_RAMP_STEP = 8.5f` per tick.
* Yaw conflict detection:
  * If $\text{steer\_cmd} > 3.0^\circ$ but left wheel is slower than right wheel (or vice versa), car is reversing turn direction across a chicane.
  * Overrides rate limits with `REVERSAL_RELEASE_STEP = 50.0f` (rapidly dumps dragging torque on old inside wheel) and `REVERSAL_DECEL_STEP = 22.0f` (rapidly anchors new inside wheel).
