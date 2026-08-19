# Design Spec: Dynamic Cornering & Robust Lookahead Vision Pipeline

**Date:** 2026-08-19  
**Target Platform:** NXP MCXN947 (ARM Cortex-M33), Pixy2 Camera, Dual DC Motors (H-Bridge)  
**Branch:** `main_vectors`  

---

## 1. Overview & Objectives

This design establishes a high-performance autonomous driving pipeline for the NXP Cup race car based on real-world PixyMon video footage (`WhatsApp Video 2026-08-19 at 10.54.27 AM.mp4`).

### Key Performance Goals:
1. **High-Speed Straight Stability:** Suppress servo oscillations and tank-slapping by dynamically scheduling steering gain down to 55% at top speed ($80\%$).
2. **Smooth Cornering & Trail-Braking:** Continuously decelerate from top speed ($80\%$) down to cornering speed floor ($38\%$) ahead of curves using heading slope preview, maintaining full ($100\%$) steering authority in the turn.
3. **90° Intersection Immunity:** Reject horizontal/shallow crossing vectors ($|\text{slope}| > 0.75$) and coast straight through the intersection box using a 20-frame ($\approx 250\text{ms}$) dead-reckoning window.
4. **Resilient Vector Parsing:** Handle single-line apex drift with persistent boundary memory and merge fragmented piecewise line segments ($< 16\text{px}$).

---

## 2. Architecture & Subsystems

```
+-----------------------------------------------------------------------------------+
|                                Pixy2 Camera (I2C)                                 |
+-----------------------------------------+-----------------------------------------+
                                          | Raw Vectors [x0, y0, x1, y1, idx, flags]
                                          v
+-----------------------------------------------------------------------------------+
|                        1. Vision Pipeline (source/vectors.c)                      |
|  - Endpoint Normalization (y0 near, y1 far)                                       |
|  - Crossing Line Rejection (|slope| > 0.75 or dy < 1px)                            |
|  - Lookahead Projection at Y = 22                                                 |
|  - Piecewise Segment Merging (|dx| < 16px)                                        |
|  - Dual Boundary Pairing (W in [28, 52]px) + Dynamic EMA Half-Lane Tracking       |
|  - Persistent Spatial Memory Fallback (s_last_left_x, s_last_right_x)             |
+-----------------------------------------+-----------------------------------------+
                                          | CTE & Heading Slope
                                          v
+-----------------------------------------------------------------------------------+
|                     2. Steering Controller (source/control_theory.c)              |
|  - EMA Filtered CTE (alpha = 0.80)                                                |
|  - PID on CTE (Kp = 1.40, Kd = 0.35, alpha = 0.20, dt measured via DWT)           |
|  - Heading Feedforward (K_heading = 10.0 * slope)                                 |
|  - Curvature-Preserving Gain Scheduling (55% on straights -> 100% in turns)       |
|  - Slew-Rate Limiter (max 450% duty / sec)                                        |
+-----------------------------------------+-----------------------------------------+
                                          | Steer Command & Heading Error
                                          v
+-----------------------------------------------------------------------------------+
|                   3. Motor Control & Powertrain (source/hbridge.c)                |
|  - Trail-Braking Curvature Interpolation (80% max -> 38% base corner floor)       |
|  - Slew-Rate Accel (+200%/s) / Brake (-500%/s) Limiter                            |
|  - Low-Pass Filtered Ackermann Differential Drive Split (alpha = 0.20)            |
|  - Normalized Left/Right Motor Polarities (DIR=0 left, DIR=1 right)               |
|  - Intersection Dead-Reckoning Coasting (20-frame window)                          |
+-----------------------------------------------------------------------------------+
```

---

## 3. Detailed Component Specifications

### 3.1 Vision Pipeline (`source/vectors.c`, `source/vectors.h`)

* **Camera Frame Dimensions:** $79 \times 52$ pixels. Centerline: $X = 39.0$. Lookahead row: $Y = 22$.
* **Vector Normalization:**
  $$\text{If } y_0 < y_1 \implies \text{swap } (x_0, y_0) \leftrightarrow (x_1, y_1)$$
  Near point $y_0 \in [0, 51]$ (bottom), Far point $y_1 \in [0, 51]$ (top).
* **Intersection Filter:**
  $$\Delta y = y_0 - y_1; \quad \text{slope} = \frac{x_1 - x_0}{\Delta y}$$
  If $\Delta y < 1.0\text{px}$ or $|\text{slope}| > 0.75$, discard vector.
* **Lookahead Projection:**
  $$x_{\text{look}} = x_0 + \text{slope} \times (y_0 - 22)$$
  Clamped within $[x_{\min}, x_{\max}]$ extent of the vector.
* **Piecewise Segment Merging:**
  Sort candidates by $x_{\text{look}}$. If $x_{i} - x_{i-1} < 16.0\text{px}$, merge into single boundary candidate by averaging $x_{\text{look}}$ and $\text{slope}$.
* **Dual Line Pairing:**
  Find pair $(i, j)$ with width $W = x_j - x_i \in [28.0, 52.0]\text{px}$ minimizing $|W - 2 \cdot s_{\text{half\_lane}}|$.
  Update tracked width:
  $$s_{\text{half\_lane}} = 0.85 \cdot s_{\text{half\_lane}} + 0.15 \cdot (0.5 \cdot W)$$
  $$s_{\text{last\_left\_x}} = x_i, \quad s_{\text{last\_right\_x}} = x_j$$
  $$\text{CTE} = \frac{x_i + x_j}{2} - 39.0, \quad \text{Heading} = \frac{\text{slope}_i + \text{slope}_j}{2}$$
* **Single Line Memory Fallback:**
  If only 1 candidate $c$ is found:
  $$\Delta_{\text{left}} = |c.x - s_{\text{last\_left\_x}}|, \quad \Delta_{\text{right}} = |c.x - s_{\text{last\_right\_x}}|$$
  If $\Delta_{\text{left}} < \Delta_{\text{right}}$:
  $$\text{Center} = c.x + s_{\text{half\_lane}}, \quad s_{\text{last\_left\_x}} = c.x, \quad s_{\text{last\_right\_x}} = c.x + 2 s_{\text{half\_lane}}$$
  Else:
  $$\text{Center} = c.x - s_{\text{half\_lane}}, \quad s_{\text{last\_right\_x}} = c.x, \quad s_{\text{last\_left\_x}} = c.x - 2 s_{\text{half\_lane}}$$
  $$\text{CTE} = \text{Center} - 39.0, \quad \text{Heading} = c.\text{slope}$$

---

### 3.2 Steering Control & Gain Scheduling (`source/control_theory.c`, `source/main.c`)

* **Loop Timing:** Real hardware loop period $\Delta t$ measured dynamically via Cortex-M33 DWT cycle counter (`DWT->CYCCNT`).
* **CTE Filtering:**
  $$\text{CTE}_{\text{filt}} = 0.80 \cdot \text{CTE} + 0.20 \cdot \text{CTE}_{\text{filt\_prev}}$$
* **Total Steer Demand:**
  $$\text{PID}_{\text{out}} = -\text{pid\_update}(\text{PID}, 0.0, K_{\text{cte}} \cdot \text{CTE}_{\text{filt}}, \Delta t)$$
  $$\text{Steer}_{\text{raw}} = \text{PID}_{\text{out}} + K_{\text{heading}} \cdot \text{Heading}$$
  $$\text{Steer}_{\text{clamped}} = \text{clamp}(\text{Steer}_{\text{raw}}, -100.0, +100.0)$$
* **Adaptive Gain Scheduling (`compute_speed_steer_gain_adaptive`):**
  $$\text{ratio}_{\text{spd}} = \text{clamp}\left(\frac{v - v_{\text{base}}}{v_{\text{max}} - v_{\text{base}}}, 0.0, 1.0\right)$$
  $$\text{Atten}_{\text{straight}} = 1.0 - (1.0 - 0.55) \cdot \text{ratio}_{\text{spd}}$$
  $$\text{Urgency}_{\text{corner}} = \max\left(\frac{|\text{CTE}_{\text{filt}}|}{16.0}, \frac{|\text{Heading}|}{0.35}\right) \in [0.0, 1.0]$$
  $$\text{Steer}_{\text{gain}} = \text{Atten}_{\text{straight}} + (1.0 - \text{Atten}_{\text{straight}}) \cdot \text{Urgency}_{\text{corner}}$$
  $$\text{Steer}_{\text{scaled}} = \text{Steer}_{\text{clamped}} \cdot \text{Steer}_{\text{gain}}$$
* **Slew-Rate Limiter:**
  $$\Delta_{\text{max}} = 450.0 \cdot \Delta t$$
  $$\text{Steer}_{\text{final}} = \text{clamp}(\text{Steer}_{\text{scaled}}, \text{Steer}_{\text{prev}} - \Delta_{\text{max}}, \text{Steer}_{\text{prev}} + \Delta_{\text{max}})$$

---

### 3.3 Motor Control & Powertrain (`source/hbridge.c`, `source/wifi_config.c`)

* **H-Bridge Polarity:**
  * **Motor 1 (Right):** Forward `DIR=1`, `duty = 100 - speed`. Reverse `DIR=0`, `duty = -speed`.
  * **Motor 2 (Left):** Forward `DIR=0`, `duty = speed`. Reverse `DIR=1`, `duty = 100 - (-speed)`.
  * **Function:** `HbridgeSpeed(Hbridge *h, int16_t speed_right, int16_t speed_left)`
* **Lookahead Trail-Braking:**
  $$I_{\text{turn}} = \max\left(\frac{|\text{Steer}_{\text{final}}|}{100.0}, 2.0 \cdot |\text{Heading}|\right) \in [0.0, 1.0]$$
  $$v_{\text{target}} = v_{\text{max}}(80\%) - I_{\text{turn}} \cdot (v_{\text{max}} - v_{\text{base}}(38\%))$$
  * Acceleration rate limited to $+200\%/\text{s}$; deceleration rate limited to $-500\%/\text{s}$.
* **Ackermann Differential Drive:**
  $$\delta_{\text{rad}} = \left(\frac{\text{Steer}_{\text{final}}}{100.0}\right) \cdot \theta_{\text{max\_steer}}$$
  $$R = \frac{L_{\text{wheelbase}}}{\tan|\delta_{\text{rad}}|}$$
  $$\text{diff}_{\text{raw}} = \text{clamp}\left(\frac{W_{\text{track}} / 2}{R}, 0.0, 0.25\right)$$
  $$\text{diff}_{\text{filt}} = 0.80 \cdot \text{diff}_{\text{filt\_prev}} + 0.20 \cdot \text{diff}_{\text{raw}}$$
  $$\text{speed}_{\text{inside}} = v_{\text{current}} \cdot (1.0 - \text{diff}_{\text{filt}}), \quad \text{speed}_{\text{outside}} = v_{\text{current}} \cdot (1.0 + \text{diff}_{\text{filt}})$$
* **Coasting Window:**
  If `lines_found == 0` and `lost_counter <= 20`: Maintain `Steer(Steer_prev)` and current motor drive.
  If `lost_counter > 20`: `HbridgeBrake()`.

---

## 4. Test & Verification Plan

1. **Unit Tests (`scratch/`):**
   * `test_lane_vectors.c`: Verify dual-boundary pairing, straight centered tracking, corner entry with both lines shifted, single-line apex fallback, and piecewise segment merging.
   * `test_intersection.c`: Verify rejection of crossing vectors ($|\text{slope}| > 0.75$) and preservation of road vectors.
   * `test_steer_scheduling.c`: Verify 55% attenuation on straights at $80\%$ speed, and 100% authority in corners.
   * `test_motor_speeds.c`: Verify trail-braking deceleration triggered by heading slope preview, differential split, and motor speed get/reset functions.
2. **Build Verification:**
   * Compile with GCC against Cortex-M33 headers to ensure zero compiler warnings, no missing symbols, and clean linker layout.

---

## 5. Calibrated Default Parameter Set

| Parameter | Symbol | Value | Unit / Description |
|---|---|---|---|
| `kp` | $K_p$ | `1.40` | PID proportional gain on CTE |
| `ki` | $K_i$ | `0.00` | Integral gain |
| `kd` | $K_d$ | `0.35` | Derivative gain (ratio $K_d/K_p = 0.25$) |
| `alpha` | $\alpha$ | `0.20` | PID derivative low-pass filter |
| `k_cte` | $K_{\text{cte}}$ | `1.00` | Lateral cross-track error weight |
| `k_heading` | $K_{\text{heading}}$ | `10.00` | Lookahead heading feedforward gain |
| `base_speed_left` | $v_{\text{base\_l}}$ | `38.0` | Cornering speed floor (%) |
| `base_speed_right` | $v_{\text{base\_r}}$ | `38.0` | Cornering speed floor (%) |
| `max_speed_left` | $v_{\text{max\_l}}$ | `80.0` | Straightaway top speed (%) |
| `max_speed_right` | $v_{\text{max\_r}}$ | `80.0` | Straightaway top speed (%) |
| `steering_offset` | $\delta_0$ | `0.0` | Servo trim offset |
