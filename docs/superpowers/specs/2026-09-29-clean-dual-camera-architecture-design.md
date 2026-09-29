# Design Specification: Clean Dual-Camera Architecture & Rewrite

## 1. Overview & Problem Statement

The existing `source/vectors.c` has grown to 939 lines with over 15 accumulated heuristic layers (virtual lane adaptation, complex clustering, split-line connection, reject streaks, shadow blending, and lookahead urgency deadbands). In real-world track testing with the new DRV8871 H-bridges, this complexity caused two critical failure modes:
1. **Phantom Urgency Suppression:** Lookahead urgency based on `mid_dx` divisions and deadbands failed to detect incoming turns on straight approaches, preventing pre-braking before corner entry.
2. **Top Camera Blindness in Sharp Yaw:** When entering turns, the car yawed past $25^\circ$, causing lane lines to exit the downward camera's narrow lateral FOV (`L: 0/1`), while the front camera was still tracking the horizon road angle at $-42^\circ$.

This design completely replaces `vectors.c` and simplifies `main.c` with a direct, geometric 2-camera architecture:
- **Top Camera (Immediate Road):** Finds the closest left line ($x < 39$) and closest right line ($x > 39$) to the car bumper, centering the car between them.
- **Front Camera (Horizon Lookahead):** Measures direct horizon road angle $\theta_{\text{front}}$. When $|\theta_{\text{front}}| > 30^\circ$, it commands feedforward steering, cuts speed to corner speed, and activates inside-wheel differential braking.
- **Straights:** When $|\theta_{\text{front}}| \le 30^\circ$, speed is uncapped at $100\%$ full power.

---

## 2. Mathematical & Geometric Formulation

### 2.1 Top Camera: Closest-Line Lane Centering
In Pixy2 coordinates, $x \in [0, 78]$, $y \in [0, 51]$, center $X_c = 39.0$.
Near bumper is at $y \approx 40\dots 51$.

For each vector $i$ with endpoints $(x_0, y_0)$ and $(x_1, y_1)$:
- Near point: $y_{\text{near}} = \max(y_0, y_1)$, $x_{\text{near}}$ is the matching $x$.
- Far point: $y_{\text{far}} = \min(y_0, y_1)$, $x_{\text{far}}$ is the matching $x$.
- Filter: Require $y_{\text{near}} \ge 20$ (lower half of frame) and $|\Delta y| \ge 4\,\text{px}$ (reject horizontal noise).
- Classify:
  - **Left Candidate:** $x_{\text{near}} < 39.0$. Find the vector with $\max(x_{\text{near}})$ (closest to center from the left).
  - **Right Candidate:** $x_{\text{near}} > 39.0$. Find the vector with $\min(x_{\text{near}})$ (closest to center from the right).

**Centerline & Error Calculation:**
```c
if (left_found && right_found) {
    center_x = (left_x + right_x) / 2.0f;
    heading_deg = (left_angle + right_angle) / 2.0f;
} else if (left_found) {
    center_x = left_x + TRACK_HALF_WIDTH_PX;
    heading_deg = left_angle;
} else if (right_found) {
    center_x = right_x - TRACK_HALF_WIDTH_PX;
    heading_deg = right_angle;
} else {
    // Retain previous or center at 39
}

cte = center_x - 39.0f;
```

---

### 2.2 Front Camera: Direct Horizon Road Angle & 30° Curvature Trigger
The front camera looks $1.0\dots 1.5\,\text{m}$ down the track.

For each vector $j$ in the forward view:
- $\Delta x = x_{\text{far}} - x_{\text{near}}$
- $\Delta y = y_{\text{near}} - y_{\text{far}} > 0$
- Angle: $\theta_j = \operatorname{atan2}(\Delta x, \Delta y) \times \frac{180}{\pi}$
- Length: $L_j = \sqrt{\Delta x^2 + \Delta y^2}$

Length-weighted average horizon road angle:
$$\theta_{\text{front}} = \frac{\sum L_j \theta_j}{\sum L_j}$$

**Threshold Evaluation:**
$$\text{is\_sharp\_corner} = (|\theta_{\text{front}}| > 30.0^\circ)$$

---

## 3. Control Actions Based on Curvature Trigger

| Condition | Target Speed | Differential Braking | Steering Command |
| :--- | :--- | :--- | :--- |
| **Straight / Mild** ($|\theta_{\text{front}}| \le 30^\circ$) | **$100\%$ Full Throttle** | Zero ($v_L = v_R = 100\%$) | $S = K_{\text{cte}} \times \text{cte} + K_{\text{head}} \times H$ (Top Camera) |
| **Sharp Corner** ($|\theta_{\text{front}}| > 30^\circ$) | **$48\%\dots 52\%$ Corner Speed** | Inside wheel drops to $30\%\dots 35\%$ | $S = S_{\text{top}} + K_{\text{front}} \times (\theta_{\text{front}} - \text{sign} \times 30^\circ)$ |

- If transitioning from $100\%$ straight into a $>30^\circ$ sharp corner while pointing straight, execute a brief $40\,\text{ms}$ active brake pulse (`HbridgeBrake()`) to rapidly drop vehicle momentum before the apex.

---

## 4. Software Modularization

1. **`source/vectors.c` & `source/vectors.h`:**
   - Reduced from 939 lines to under 150 lines.
   - Only 2 clear functions:
     - `compute_top_camera_lane(const uint16_t *vectors, size_t n, steering_result_t *out)`
     - `compute_front_camera_horizon(const uint16_t *vectors, size_t n, horizon_result_t *out)`
2. **`source/main.c`:**
   - Stripped of redundant state machines, deadbands, and complex polynomial scalers.
   - Clean, direct 50Hz tick:
     1. Poll Top Camera $\to$ Lane CTE & Heading.
     2. Poll Front Camera $\to$ Horizon Angle $\theta_{\text{front}}$ and $>30^\circ$ flag.
     3. Apply Speed & Differential Governor.
     4. Actuate Servo & H-Bridge.
     5. Telemetry.
