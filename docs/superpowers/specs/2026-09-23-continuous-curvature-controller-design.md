# Continuous Curvature Controller & Three-Macro-State Architecture Spec

**Date:** 2026-09-23  
**Status:** Approved Design  
**Target Platform:** NXP MCX-N947 (Dual ARM Cortex-M33, 150 MHz)  
**Module:** High-Level Vehicle Control Pipeline (`source/main.c`)

---

## 1. Overview & Motivation

The previous control pipeline attempted to classify continuous physical driving into discrete binary modes (`DRIVE_FORWARD` vs `DRIVE_TURNING`). Because real race tracks feature smooth, continuous transitions, this binary classification created brittle boundary traps, requiring complex hysteresis, timeout counters (`s_corner_abort_ticks`), apex latches (`s_corner_apex_reached`), and urgency floors. This resulted in the car getting locked in cornering mode (62% speed) down straightaways.

This design completely eliminates the binary corner state machine. In its place, it implements a **Three-Macro-State Architecture** (`NORMAL`, `INTERSECTION`, `LINE_LOST`), where all cornering, pre-braking, and straight-line sprinting inside `NORMAL` are governed by a **Continuous Curvature Function ($\kappa$)** with deadbands for sensor noise immunity.

---

## 2. Macro State Machine Architecture

The vehicle operates in exactly one of three high-level states:

```
                  ┌────────────────────────────────────────┐
                  │                 NORMAL                 │
                  │  • Continuous Curvature Governor (κ)   │
                  │  • Global Rate-Limited Speed Slew      │
                  │  • Decoupled Steering (PD + Feedfwd)   │
                  │  • Dynamic Rear Differential Drag      │
                  └───────┬────────────────────────▲───────┘
                          │                        │
         Crossing lines   │                        │ Fixed timeout
         detected         │                        │ (18 ticks = 360ms)
                          ▼                        │
                  ┌────────────────────────────────┴───────┐
                  │              INTERSECTION              │
                  │  • Steering held straight (0° + trim)  │
                  │  • Target speed held/capped (70%)      │
                  │  • Lane geometry parsing bypassed      │
                  └────────────────────────────────────────┘
                          │                        ▲
            No lines seen │                        │ Valid lane lines seen
            for >= 3 ticks│                        │ for 3 consecutive ticks
                          ▼                        │
                  ┌────────────────────────────────┴───────┐
                  │               LINE_LOST                │
                  │  • Coast straight (ticks 1–18, 360ms)  │
                  │  • Active brake drag (ticks 19–50)     │
                  │  • Full emergency stop (> 50 ticks, 1s)│
                  └────────────────────────────────────────┘
```

### 2.1 State Definitions & Transition Rules

#### `NORMAL`
* **Entry**: System start or exiting `INTERSECTION` (after timeout) or `LINE_LOST` (after 3 valid frames).
* **Behavior**: Runs the Continuous Curvature Governor. Speed, steering, and differential drag are calculated every 20ms tick.
* **Transition to `INTERSECTION`**:
  * Trigger: Perpendicular crossing vector detected ($raw\_dx \ge 18\text{px}, raw\_dy \le 7\text{px}$) while lane width expands ($sep > 68\text{px}$) or disappears.
* **Transition to `LINE_LOST`**:
  * Trigger: Top camera reports `num_vectors == 0` for $\ge 3$ consecutive ticks (debouncing 1-frame optical flickers).

#### `INTERSECTION`
* **Purpose**: Cross track intersections safely without reacting to intersecting transverse tape lines.
* **Behavior**:
  * Steering angle is held dead-straight (`steer_cmd = 0.0f` + physical `steering_offset`).
  * Target speed is held at a steady crossing speed ($70\%$).
  * Speed slew limiter continues to execute to prevent motor torque jolts.
* **Exit Condition**:
  * Strict tick timeout: `intersection_ticks >= 18` ($360\text{ms} \approx 0.6\text{m}$ of travel).
  * Guaranteed never to hang or latch.
* **Transition to `NORMAL`**:
  * Trigger: Timeout expires.
  * Action on Exit: Reset PID integrator and derivative EMA filter (`pid_reset`, `filter_init = 0`) to prevent re-entry steering kick.

#### `LINE_LOST`
* **Purpose**: Safe dead-reckoning coasting through momentary track gaps, with staged emergency braking if the track is completely lost.
* **Behavior**:
  * **Phase 1: Coasting (Ticks 1–18, 0 to 360ms)**: Hold steering straight, maintain last applied speed with no braking.
  * **Phase 2: Active Braking (Ticks 19–50, 360ms to 1.0s)**: Steer straight, command active motor braking (`HbridgeBrake`).
  * **Phase 3: Emergency Shutdown (> 50 ticks, > 1.0s)**: Full stop, transition robot state to `STATE_IDLE`.
* **Transition to `NORMAL`**:
  * Trigger: Top camera confirms valid lane boundary vectors (`num_vectors >= 1`) for **3 consecutive ticks**.
  * Action on Exit: Reset PID integrator and derivative EMA filter; resume `NORMAL` tracking immediately.

---

## 3. Continuous Curvature Governor (Inside `NORMAL`)

Inside `NORMAL`, there are zero sub-states. Every 20ms tick executes four smooth mathematical equations:

### 3.1 Preview Curvature ($\kappa$) with Noise Deadband

To prevent camera vector jitter and chassis vibrations from reducing speed on straightaways, deadbands map normal straight-line noise to exactly $\kappa = 0.0$:

$$\kappa_{\text{base}} = \begin{cases} 
0.0 & \text{if } |\text{Heading}| \le 2.5^\circ \\
\dfrac{|\text{Heading}| - 2.5^\circ}{25.0^\circ - 2.5^\circ} & \text{if } |\text{Heading}| > 2.5^\circ
\end{cases}$$

$$\kappa_{\text{front}} = \begin{cases} 
0.0 & \text{if } |mid\_dx| \le 4.0\text{px} \\
\dfrac{|mid\_dx| - 4.0\text{px}}{14.0\text{px} - 4.0\text{px}} & \text{if } |mid\_dx| > 4.0\text{px}
\end{cases}$$

*If a transverse 90° corner wall is detected ahead ($raw\_dx \ge 18\text{px}, raw\_dy \le 7\text{px}$ in forward camera), $\kappa_{\text{front}} = 1.0$ immediately.*

$$\kappa = \text{clamp}\Big(\max(\kappa_{\text{base}},\, \kappa_{\text{front}}),\, 0.0,\, 1.0\Big)$$

* **Straightaway** ($|H| \le 2.5^\circ, |mid\_dx| \le 4.0\text{px}$): $\kappa = \mathbf{0.00}$.
* **Approaching 90° Corner** ($mid\_dx > 4.0\text{px}$ or wall detected): $\kappa$ ramps continuously to $\mathbf{1.00}$ (1.5m ahead).
* **Corner Apex**: $\kappa \approx \mathbf{0.60 \dots 1.00}$.

### 3.2 Continuous Target Speed & Global Slew Limiter

$$\text{Speed}_{\text{target}} = \text{Speed}_{\text{straight}} - \kappa \times (\text{Speed}_{\text{straight}} - \text{Speed}_{\text{corner}})$$

With $\text{Speed}_{\text{straight}} = 100\%$ and $\text{Speed}_{\text{corner}} = 58\%$:
* $\kappa = 0.0 \implies \text{Speed}_{\text{target}} = \mathbf{100\%}$.
* $\kappa = 1.0 \implies \text{Speed}_{\text{target}} = \mathbf{58\%}$.

**Global Slew Limiting (Runs across all states)**:
$$\Delta v = \text{Speed}_{\text{target}} - \text{Speed}_{\text{applied}}$$
$$\text{Speed}_{\text{applied}} += \text{clamp}(\Delta v,\, -\text{decel\_step},\, +\text{accel\_step})$$

### 3.3 Decoupled Steering Authority

$$\text{Steer}_{\text{cmd}} = -K_{CTE} \cdot \text{PID}(CTE) + K_{\text{heading}} \cdot \text{EffectiveHeading}$$

Where:
$$\text{EffectiveHeading} = \begin{cases} 
0.0 & \text{if } |\text{Heading}| \le 2.0^\circ \\
\text{Heading} - 2.0^\circ \cdot \text{sgn}(\text{Heading}) & \text{if } |\text{Heading}| > 2.0^\circ
\end{cases}$$

* Steering is clamped to physical mechanical travel $[-77.5^\circ, +52.5^\circ]$ and rate-limited to $650^\circ/\text{s}$ ($13.0^\circ/\text{tick}$).
* Front camera never injects steering angle ($FF_{lookahead} = 0$).

### 3.4 Dynamic Differential Pivot Drag

$$\text{Steer}_{\text{norm}} = \text{clamp}\left(\frac{|\text{Steer}_{\text{cmd}}| - 3.5^\circ}{30.0^\circ - 3.5^\circ},\, 0.0,\, 1.0\right)$$
$$\text{Drag} = \text{Steer}_{\text{norm}} \times (\text{Speed}_{\text{applied}} - \text{Speed}_{\text{inside\_min}})$$

$$\text{Outside Wheel} = \text{Speed}_{\text{applied}}$$
$$\text{Inside Wheel} = \text{Speed}_{\text{applied}} - \text{Drag}$$

* On straights ($|\text{Steer}| \le 3.5^\circ$): Outside = Inside = 100%.
* In hard turns ($|\text{Steer}| \ge 30^\circ$): Inside wheel dragged down to 35%, generating pivot torque.

---

## 4. Key Invariant Guarantees

1. **Global Slew Continuity**: The speed slew limiter is global and state-agnostic. Motor PWM never steps discontinuously when changing states.
2. **Anti-Kick Filter Resets**: Exiting `INTERSECTION` or `LINE_LOST` resets `pid_struct` and EMA filter state to prevent servo snap.
3. **No Corner States**: Corners are purely continuous high-$\kappa$ events inside `NORMAL`.

---

## 5. Telemetry Protocol Compatibility

The telemetry string format is preserved for logging and analysis:
`L:<top>/<front> R:<raw_cte> F:<filtered_cte> H:<heading> TJ:<traj_angle> S:<steer> LA:<kappa*100> ST:<N|I|L> VL:<left> VR:<right>`
* `ST:N` = `NORMAL`
* `ST:I` = `INTERSECTION`
* `ST:L` = `LINE_LOST`
