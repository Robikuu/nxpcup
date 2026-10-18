# NXP Cup Trackside Tuning Reference & Cheat Sheet

This document is the complete tuning reference for every active parameter in [`wifi_config.c`](../source/wifi_config.c) and [`Config.h`](../include/Config.h).

> [!IMPORTANT]
> **Command Syntax**: Every command sent over Wi-Fi/Serial must end with a semicolon `;`.  
> *Example*: `k_head 1.20;` or `status;` or `scale 0.70;`

---

## 1. Steering & Lane-Centering Parameters (`cam_pid`)

The primary steering formula computed in [`source/main.c`](../source/main.c) is:

$$\text{steer\_top} = k_{\text{cte}} \cdot \text{CTE} + k_{\text{head}} \cdot \text{heading}$$

| Command | Default | What It Does | How to Tune It |
| :--- | :---: | :--- | :--- |
| `k_cte` | `0.55` | **Position Gain** ($\text{deg/pixel}$). Pulls the car back toward the center of the lane ($X = 39.5$) when it drifts left or right. | • **Too low** ($< 0.40$): Car lazily drifts near the inside/outside tape without centering.<br>• **Too high** ($> 0.75$): Car over-corrects across center and weaves in wide S-patterns on straights. |
| `k_head` | `1.30` | **Angle / Damping Gain** ($\text{deg/deg}$). Aligns wheels parallel to the tape angle. Natural damping against $k_{\text{cte}}$ overshoot. | • **Increase** ($1.40 - 1.60$): If $k_{\text{cte}}$ causes centerline overshoot, raising $k_{\text{head}}$ straightens the nose earlier.<br>• **Decrease** ($1.00 - 1.15$): If the car chatters/twitches rapidly left-right on straights or reacts too aggressively to small curves. |
| `offset` | `0.00` | **Mechanical Servo Trim** ($\text{deg}$). Added to all steering commands so $0^\circ$ means dead-straight wheels. | Place the car on a long straight at low speed (`scale 0.4;`). If it drifts right, send `offset -2.0;`. If it drifts left, send `offset 2.0;`. |
| `k_cte_c`<br>`k_head_c` | `0.55`<br>`1.30` | **Corner-Specific Gains** (Only active in `blend 1`). Interpolates from straight gains to corner gains based on blend level. | • Ignored in `blend 0`.<br>• In `blend 1`, increase `k_head_c 1.50;` for sharper corner turn-in without making straights twitchy. |

---

## 2. Speed, Braking & Differential Pivot (H-Bridge)

| Command | Default | What It Does | How to Tune It |
| :--- | :---: | :--- | :--- |
| `spd_s` | `100.00` | **Straightaway Target Speed** ($0 - 100\%$). Speed when no corner is detected ahead. | Lower to $85 - 90\%$ if the track has very short straight chutes before hairpins where the tires cannot brake in time. |
| `spd_out` | `55.00` | **Corner Entry / Outside Wheel Speed** ($0 - 100\%$). Target speed in corners, and drive power of the outer pushing wheel. | • **Too high** ($> 65$): Car understeers and pushes wide off the outside of the corner.<br>• **Too low** ($< 45$): Car bogs down and loses lap time mid-corner. |
| `spd_in` | `32.00` | **Inside Wheel Minimum Speed** ($0 - 100\%$). As steering locks toward $40^\circ$, inside rear wheel drops to `spd_in` to pivot the chassis around the apex. | • **Lower** ($20 - 25$): Stronger torque vectoring; helps rotate the car in tight hairpins.<br>• **Raise** ($38 - 42$): Less pivot drag; use if the car fishtails or scrubs too much speed in open corners. |
| `ramp_d` | `30.00` | **Braking / Decel Rate** ($\%/tick$ at 50 Hz). Maximum motor PWM drop per 20ms tick when braking or applying differential drag. | At $30.0$, dropping from $100\%$ to $32\%$ takes 2.3 ticks (~45 ms).<br>• **Keep between 25–35**: Below 20, the car won't brake in time before corner entry. |
| `ramp_a` | `6.00` | **Acceleration Rate** ($\%/tick$ at 50 Hz). How fast motor PWM climbs when exiting a corner back to `spd_s`. | At $6.0$, ramping from $55\%$ to $100\%$ takes 7.5 ticks (~150 ms).<br>• **Lower** ($4.0$): Smoother corner exit if rear tires spin or break traction.<br>• **Raise** ($8.0 - 10.0$): Harder punch out of corners once steering is stable. |
| `scale` | `1.00` | **Global Speed Multiplier** ($0.10 - 1.00$). Scales final left/right wheel speeds right before output, preserving differential ratios. | Ideal for safe initial track validation (`scale 0.70;`) before unlocking full speed (`scale 1.0;`). |

---

## 3. Mode & Continuous Blend Parameters (`blend 0` vs `blend 1`)

| Command | Default | What It Does | How to Tune It |
| :--- | :---: | :--- | :--- |
| `blend` | `0` | **Corner Mode Selector** (`0` = Discrete + Severity Speed, `1` = Leaky Integrator). | • **`blend 0` (Recommended)**: Uses `corner_severity` (`BT:` in telemetry) to scale speed smoothly between `spd_s` (on straights) and `spd_out` (in 90° corners), while keeping discrete 3-tick corner gating for feedforward and differential.<br>• **`blend 1`**: Filters `corner_severity` through an asymmetric attack/release leaky integrator (`BL:` in telemetry). |
| `fa_th` | `25.00` | Front camera horizon angle ($^\circ$) where corner speed scaling begins ($0\%$ at `fa_th`, $100\%$ at $50^\circ$ or on a transverse corner wall). | If the car brakes slightly on straightaways, raise `fa_th 28;`. |
| `h_lo`<br>`h_hi` | `18.00`<br>`35.00` | Top camera heading range ($^\circ$) where apex speed hold ramps from $0\%$ (`h_lo`) to $100\%$ (`h_hi`). | Keeps speed held down through the apex after the front camera already sees the straight exit. Raise `h_lo 20; h_hi 38;` if minor straightaway corrections cause slight speed dips. |
| `att` | `0.35` | *(blend 1 only)* **Attack Rate**: How fast `BL:` rises toward `BT:` when entering a corner ($0.05 - 1.00$). | Higher ($0.50$) = faster pre-braking onset in `blend 1`. |
| `rel` | `0.06` | *(blend 1 only)* **Release Rate**: How fast `BL:` decays back to $0$ on corner exit ($0.01 - 0.50$). | At $0.06$, returning to 100% takes ~1 second. If using `blend 1`, increase to `rel 0.20;` or `rel 0.25;`. |

---

## 4. Track-Side Troubleshooting Cheat Sheet

| What You See on Track | What to Send Over Wi-Fi | Why It Works |
| :--- | :--- | :--- |
| **Fast twitching / rapid left-right chatter on straights** | `k_head 1.10;`<br>*(down from 1.30)* | Reduces sensitivity to minor line angle changes and camera noise. |
| **Slow, wide S-curve weaving across the lane on straights** | `k_cte 0.45;` *(down from 0.55)*<br>or `k_head 1.45;` | Less aggressive lateral position pull (`k_cte`) or higher heading damping (`k_head`). |
| **Car enters corners too fast and slides wide off the outside** | `spd_out 48;`<br>*(down from 55)* | Lowers corner entry speed so front tires retain steering authority. |
| **Car slows down in time, but does not rotate tightly enough at apex** | `spd_in 24;`<br>*(down from 32)* | Slows the inside rear wheel more, generating stronger differential pivot torque. |
| **Car pivots too aggressively in corners (rear end kicks out / fishtails)** | `spd_in 38;`<br>*(up from 32)* | Decreases speed difference between outer ($55\%$) and inner ($38\%$) rear wheels. |
| **Car accelerates too slowly out of corners onto the straight** | `ramp_a 8.0;`<br>*(up from 6.0)* | Ramps motor PWM from $55\%$ to $100\%$ faster on corner exit. |
| **Want to test safely at 70% speed without ruining corner balance** | `scale 0.70;` | Scales both $100\%$ straights $\to 70\%$ and $55\% / 32\%$ corners $\to 38.5\% / 22.4\%$ proportionally. |

---

## 5. Quick Command Reference

```text
start;               # Starts the vehicle run (RUNNING state)
stop;                # Stops the vehicle and brakes motors (IDLE state)
status;              # Prints current values of all parameters
drive 0; / drive 1;  # Disables/enables drive motors (push calibration mode)
dump;                # Dumps one frame of raw Pixy2 vectors over Wi-Fi
dump 1; / dump 0;    # Enables/disables continuous vector telemetry
```