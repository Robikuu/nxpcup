# NXP Cup Autonomous Car: Track-Side Tuning & Parameter Field Guide

This guide is the complete reference for every tunable runtime parameter, compile-time threshold, and telemetry field on the NXP Cup race car (NXP MCXN947 ARM Cortex-M33, dual Pixy2 cameras, DRV8871 H-bridge).

---

## 1. Quick Diagnostic Lookup Table (Symptoms & Fixes)

If the car behaves unexpectedly on the track, locate the symptom in this table first:

| What the Car Does on the Track | Root Cause | Primary Variable to Adjust | Direction | Recommended Step |
| :--- | :--- | :--- | :--- | :--- |
| **Weaves back and forth on straightaways** (oscillates left-right) | Heading feedback under-damped or CTE gain too aggressive | `k_head`<br>`k_cte` | **Raise** `k_head`<br>**Lower** `k_cte` | `k_head 1.10 -> 1.25`<br>`k_cte 0.55 -> 0.50` |
| **Drifts slowly off-center to one side on straights** | Physical servo horn or steering linkage asymmetry | `offset` | **Offset towards opposite side** | E.g., drifts right: `offset -1.5` |
| **Dives/snaps into corners violently (oversteer / spin out)** | Inside wheel drag too severe; excessive differential yaw torque | `spd_in` | **Raise** (softens inside drag) | `spd_in 36 -> 42` |
| **Pushes wide / washes out of corners (understeer / hits outer wall)** | Apex outside wheel too fast, or inside pivot drag too weak | `spd_out`<br>`spd_in` | **Lower** `spd_out`<br>**Lower** `spd_in` | `spd_out 55 -> 48`<br>`spd_in 40 -> 34` |
| **Darts into corner too early** (before reaching the actual corner entry) | Front camera triggering corner mode from too far away | `fa_th` | **Raise** (requires steeper angle) | `fa_th 15 -> 20` |
| **Slow / sluggish out of corners onto the straightaway** | Motor acceleration ramp too soft, or blend release lingering | `ramp_a`<br>`blend` | **Raise** `ramp_a`<br>Set `blend 0` | `ramp_a 35 -> 45`<br>`blend 0` |
| **Rear wheels spin / break traction on corner exit** | Motor acceleration ramp too aggressive for tire friction | `ramp_a` | **Lower** | `ramp_a 40 -> 30` |
| **Brakes too late into corners** (skids straight past the braking zone) | Deceleration ramp too soft | `ramp_d` | **Raise** | `ramp_d 45 -> 60` |
| **Fishtails / snaps back when straightening out on corner exit** | Heading damping too loose, or differential releasing asymmetrically | `k_head`<br>`spd_in` | **Raise** `k_head`<br>**Raise** `spd_in` | `k_head 1.10 -> 1.25`<br>`spd_in 36 -> 40` |
| **Car stops or hesitates in the middle of an intersection** | Optical dropout duration exceeding transit window | Checked via telemetry `IX:1` | Verify track gap width | Window scales automatically with speed |

---

## 2. Live Runtime Serial Commands Reference

All parameters below can be queried using `status;` and tuned live over the COM serial terminal (500000 baud) by sending `<command> <value>;`.

### 2.1 Vehicle Speed & Differential Pivot

```text
                                     +--------------------+
                                     |  Straightaway      |
                                     |  Left:  spd_s      |
                                     |  Right: spd_s      |
                                     +---------+----------+
                                               |
                                     [ Corner Detected ]
                                               |
                                               v
                                     +--------------------+
                           Left Turn |  spd_out = Right   | Right Turn
                       +-------------+  spd_in  = Left    +-------------+
                       |             +--------------------+             |
                       v                                                v
              Left:  spd_in  (inside pivot)                    Left:  spd_out (outside drive)
              Right: spd_out (outside drive)                    Right: spd_in  (inside pivot)
```

#### `spd_s` (Speed Straight)
* **Aliases**: `speed_straight`, `speed_s`
* **Default**: `100.00%` | **Safe Range**: `50.0 - 100.0%`
* **What it does**: Full throttle motor duty cycle on clean straightaways when neither corner nor intersection mode is active.
* **When to change**:
  * **Lower** (e.g. `80.0`): For initial track mapping, new layouts, low battery calibration, or slippery track surfaces.
  * **Raise** (up to `100.0`): For competition race pace once steering stability is confirmed.

#### `spd_out` (Speed Outside Corner)
* **Aliases**: `speed_outside`, `speed_out`
* **Default**: `55.00%` (calibrated) | **Safe Range**: `40.0 - 75.0%`
* **What it does**: Target speed of the **outside wheel** in corner mode. This dictates the actual vehicle momentum and radius maintained through the apex.
* **When to change**:
  * **Lower** (e.g. `48.0`): If the car understeers (slides wide towards the outer wall) or enters the apex carrying too much speed.
  * **Raise** (e.g. `60.0 - 65.0`): If the car slows down too much in corners and easily hugs the inner rail without sliding.

#### `spd_in` (Speed Inside Min / Pivot Anchor)
* **Aliases**: `speed_inside`, `speed_in`, `speed_corner`
* **Default**: `40.00%` (calibrated) | **Safe Range**: `25.0 - 50.0%`
* **What it does**: Floor speed of the **inside wheel** when steering exceeds the differential deadband ($8.0^\circ$). The difference between `spd_out` and `spd_in` creates mechanical **yaw pivot torque** that forces the chassis to rotate.
* **When to change**:
  * **Raise** (e.g. `42.0 - 45.0`): If the car **snaps / oversteers** inward violently, spins out, or hops its rear axle upon turn-in.
  * **Lower** (e.g. `32.0 - 36.0`): If the car struggles to turn into tight $90^\circ$ bends and needs sharper mechanical rotation.

#### `ramp_a` (Acceleration Ramp Step)
* **Aliases**: `accel_ramp`
* **Default**: `35.00%/tick` | **Safe Range**: `15.0 - 60.0%/tick`
* **What it does**: Slew-rate limit on motor throttle increase per 20 ms tick ($50\text{ Hz}$). At `35%/tick`, transitioning from $55\%$ to $100\%$ takes $\approx 25\text{ ms}$ (~1–2 ticks).
* **When to change**:
  * **Raise** (e.g. `45.0 - 50.0`): For instant, punchy exit acceleration out of corners onto the straightaway.
  * **Lower** (e.g. `20.0 - 28.0`): If the car spins its rear tires or fishtails under hard acceleration when exiting apexes on slick tiles.

#### `ramp_d` (Deceleration Ramp Step)
* **Aliases**: `decel_ramp`
* **Default**: `45.00%/tick` | **Safe Range**: `20.0 - 80.0%/tick`
* **What it does**: Slew-rate limit on motor throttle decrease per 20 ms tick.
* **When to change**:
  * **Raise** (e.g. `60.0`): For aggressive pre-braking right before corner entry.
  * **Lower** (e.g. `25.0 - 30.0`): If hard deceleration unweights the rear axle and causes straightaway instability.

---

### 2.2 Steering Control (Stanley State Feedback)

Steering angle is computed every 20 ms using Stanley cross-track and heading error terms:
$$\text{steer\_cmd} = k_{\text{cte}} \times \text{filtered\_cte} + k_{\text{head}} \times \text{filtered\_heading} + \text{feedforward}$$

#### `k_cte` (Cross-Track Error Gain)
* **Default**: `0.55` | **Safe Range**: `0.30 - 0.85`
* **What it does**: Proportional centering pull towards the lane centerline. It converts lateral offset (in pixels) into steering angle.
* **When to change**:
  * **Raise** (e.g. `0.65 - 0.75`): If the car drifts lazily away from the center and cuts too close to boundary lines.
  * **Lower** (e.g. `0.45 - 0.50`): If the car hunts or weaves back and forth across the centerline at high speed.

#### `k_head` (Heading Damping Gain)
* **Aliases**: `k_heading`
* **Default**: `1.10` (calibrated) | **Safe Range**: `0.70 - 1.60`
* **What it does**: Directly damps vehicle yaw angle relative to the track tangent. Acts as lateral rate damping to eliminate straightaway oscillations.
* **Auto-tracking**: When tuned live (`k_head 1.10;`), it **automatically updates `k_head_c`** to match unless corner heading gain has been explicitly overridden.
* **When to change**:
  * **Raise** (e.g. `1.20 - 1.35`): If the car weaves or fishtails on straightaways.
  * **Lower** (e.g. `0.95 - 1.05`): If steering feels stiff and sluggish when turning into curves.

#### `k_cte_c` & `k_head_c` (Decoupled Corner Gains)
* **Aliases**: `k_cte_corner`, `k_heading_corner`, `k_head_c`
* **Default**: Equal to `k_cte` (`0.55`) and `k_head` (`1.10`)
* **What it does**: Dedicated steering gains for cornering. In `blend 1`, gains continuously interpolate between straight and corner values. In `blend 0`, straight gains govern throughout.
* **When to change**: Only adjust if you specifically want sharper tracking inside corners without changing straightaway gains.

#### `offset` (Mechanical Steering Trim)
* **Aliases**: `steering_offset`
* **Default**: `0.00 deg` | **Safe Range**: `-8.0 to +8.0 deg`
* **What it does**: Static center trim added directly to the physical servo output.
* **When to change**:
  * If the car pulls consistently to the **left** on straights: increase `offset` positively (`+1.0`, `+2.0`).
  * If the car pulls consistently to the **right** on straights: decrease `offset` negatively (`-1.0`, `-2.0`).

---

### 2.3 Corner Architecture & Continuous Blend Governor

#### `blend` (Continuous Blend Enable Switch)
* **Default**: `0` (Baseline Discrete Mode) | **Options**: `0` or `1`
* **What it does**:
  * **`blend 0` (Recommended)**: Discrete binary mode. Front camera triggers sharp corner mode instantly; unlatches instantly at apex for maximum corner-exit acceleration punch.
  * **`blend 1`**: Continuous blend mode. Ramps into corner using front preview (`att`), holds corner using top heading (`h_lo`, `h_hi`), and decays smoothly (`rel`) on exit.
* **When to change**:
  * Keep at `0` for crisp, punchy racing.
  * Switch to `1` if corner exits snap or fishtail uncontrollably despite `spd_in` and `ramp_a` tuning.

#### `fa_th` (Front Angle Threshold)
* **Aliases**: `fa_thresh`
* **Default**: `15.00 deg` | **Safe Range**: `10.0 - 25.0 deg`
* **What it does**: The front camera horizon angle at which corner pre-steer and speed governance begin ramping in.
* **When to change**:
  * **Raise** (e.g. `18.0 - 22.0`): If the car reacts too early to distant corners and cuts across the inside line before the turn begins.
  * **Lower** (e.g. `10.0 - 12.0`): If the car reacts too late and doesn't begin turning until it is already at the track wall.

#### `h_lo` & `h_hi` (Top Heading Floor & Ceiling)
* **Defaults**: `h_lo = 12.00 deg`, `h_hi = 25.00 deg`
* **What it does** (Active in `blend 1`):
  * `h_lo`: Heading deadband below which top heading cannot hold the corner blend (prevents straightaway drag).
  * `h_hi`: Heading angle representing full 100% corner hold.
* **When to change**:
  * Raise `h_lo` (e.g. `14.0`) if corner blend lingers too long after returning to a straight.
  * Lower `h_lo` (e.g. `10.0`) if the car washes out mid-corner when the front camera loses sight of the outer tape.

#### `att` (Blend Attack Factor)
* **Aliases**: `blend_att`
* **Default**: `0.35` (reaches ~88% in 5 ticks / 100 ms) | **Safe Range**: `0.10 - 0.60`
* **What it does**: Rate at which corner mode engages once front camera angle exceeds `fa_th`.
* **When to change**:
  * **Lower** (e.g. `0.20 - 0.25`): Softens corner entry and eliminates abrupt turn-in jerks.
  * **Raise** (e.g. `0.45`): Makes entry decisive for high-speed corners.

#### `rel` (Blend Release Factor)
* **Aliases**: `blend_rel`
* **Default**: `0.06` (~25–30 ticks decay / 500 ms) | **Safe Range**: `0.02 - 0.15`
* **What it does**: Exponential decay factor when transitioning from corner mode back to straight mode.
* **When to change**:
  * **Raise** (e.g. `0.10 - 0.12`): Faster throttle recovery on exit (less sluggishness).
  * **Lower** (e.g. `0.04`): Longer, smoother throttle recovery if corner exit fishtails persist.

---

### 2.4 System & Calibration Commands

* `start;` : Transitions state machine to `RUNNING` (activates motors and steering).
* `stop;` / `brake;` : Immediate stop, brakes motors, centers steering, sets state to `IDLE`.
* `status;` : Prints all live configuration values.
* `help;` : Lists all available serial commands.
* `drive 0;` / `drive 1;` : Disables or enables motor outputs (useful for push-testing steering while walking the car on track).
* `steer <deg>;` : Tests physical steering deflection directly while in `IDLE` (e.g. `steer 20;`, `steer -20;`, `steer 0;`).
* `motor_left <speed>;` / `motor_right <speed>;` : Pulses specified motor at speed for 4 seconds in `IDLE` (useful for wheel direction and bench tests).
* `dump 1;` / `dump 0;` : Enables or disables raw vector logging from both Pixy2 cameras.

---

## 3. Important Compile-Time Constants (`Config.h`)

These constants are hardcoded in [`include/Config.h`](file:///d:/nxpcup/include/Config.h) and require recompilation if modified:

| Constant | Value | Description & Function |
| :--- | :--- | :--- |
| `DIFFERENTIAL_DEADZONE_DEG` | `8.0f` | Steering deadband below which zero differential drag applies. Eliminates torque-steer weave on straights while wheels are centered. |
| `STEER_FULL_BRAKE_DEG` | `40.0f` | Steering deflection angle at which inside wheel differential drag reaches 100% of its available span. |
| `FRONT_CORNER_ANGLE_THRESH_DEG`| `30.0f` | Horizon road angle threshold where front camera marks `is_sharp_corner = 1`. |
| `FRONT_FEEDFORWARD_GAIN` | `0.45f` | Feedforward gain applied to `filtered_front_angle` to steer into corner before apex. |
| `STEERING_LIMIT_RIGHT` | `+65.0f` | Physical servo mechanical right travel limit. |
| `STEERING_LIMIT_LEFT` | `-65.0f` | Physical servo mechanical left travel limit. |
| `DEFAULT_DT` | `0.02f` | Control loop period ($20\text{ ms} = 50\text{ Hz}$). |
| `OFF_TRACK_FAILSAFE_TICKS` | `125U` | 2.5 seconds without seeing any lines triggers automatic emergency stop. |
| `TOP_CAM_EXCLUDE_Y_MIN` | `38U` | Y-coordinate cutoff to ignore front bumper, tie rods, and wheels from lane boundary detection. |

---

## 4. Telemetry Field Guide

Every 4th tick ($12.5\text{ Hz} = 80\text{ ms}$), the car outputs a telemetry row over serial and WiFi:

```text
L:2/3 R:5 F:2 H:-11 TS:-15 FS:24 S:-15 BW:100 BL:0 FT:0 HT:0 DOM:T FV:1 IX:0 ST:S VL:100 VR:100
```

| Token | Meaning | Scaling | Expected Normal Range | Red Flags / What to Watch |
| :--- | :--- | :--- | :--- | :--- |
| `L:x/y` | Lines seen by Top Cam (`x` used / `y` raw) | Integer count | `2/2` or `2/3` | `L:0` (blind/dropout), `L:1` (single line tracking) |
| `R` | Raw Cross-Track Error (CTE) | $\times 10$ pixels | $-15 \dots +15$ | $|R| > 50$ (car is near the edge of track) |
| `F` | Filtered Cross-Track Error | $\times 10$ pixels | $-10 \dots +10$ | Large values with sign flipping indicates weave |
| `H` | Filtered Heading Angle | $\times 10$ degrees | $-8 \dots +8$ on straights | $|H| > 150$ on straights means severe weave |
| `TS` | Top Camera Steering Component | $\times 10$ degrees | Proportional to $F$ and $H$ | Sudden spikes indicate bad vector selection |
| `FS` | Filtered Front Horizon Angle | $\times 10$ degrees | $-5 \dots +5$ on straights | $> +200$ or $< -200$ indicates incoming corner |
| `S` | Final Steer Command to Servo | $\times 10$ degrees | $-650 \dots +650$ | Pinned at $\pm 650$ for $>1\text{s}$ means car cannot make radius |
| `BW` | Baseline Corner Flag Metric | $0$ or $100$ | $100$ (straight), $0$ (corner) | Matches legacy logging format |
| `BL` | Continuous Corner Blend Level | $0 \dots 100\%$ | $0\%$ straight, $100\%$ corner | In `blend 0`, snaps $0 \leftrightarrow 100$. In `blend 1`, smooth ramp |
| `FT` | Front Camera Target Intensity | $0 \dots 100\%$ | Attacks corner on entry | If $FT > 0$ on clean straight, check lighting / glare |
| `HT` | Top Heading Floor Intensity | $0 \dots 100\%$ | Holds corner through apex | Prevents washout when $FT$ drops to $0$ mid-corner |
| `DOM` | Dominant Sensor View | `T` (Top) or `F` (Front) | `T` straight, `F` in corner | Stays `T` on clean straights |
| `FV` | Front Camera Valid Lines | $0$ or $1$ | $1$ (lines detected) | Sustained $0$ means front camera tilted or obscured |
| `IX` | Intersection Active Flag | $0$ or $1$ | $0$ (track), $1$ (crossing) | Should only be $1$ when crossing perpendicular tape |
| `ST` | Macro State Indicator | `S`, `C`, or `I` | `S` (Straight), `C` (Corner), `I` (Intersection) | `C` inside straight means false corner trigger |
| `VL` | Applied Left Wheel Speed | $0 \dots 100\%$ | Matches right on straights | Drops to `spd_in` during right turns |
| `VR` | Applied Right Wheel Speed | $0 \dots 100\%$ | Matches left on straights | Drops to `spd_in` during left turns |

---

## 5. Recommended 4-Step Tuning Sequence on Track

When arriving at a new track or testing after tire/battery changes, always follow this sequence:

```
+-------------------------------------------------------------+
| Step 1: Straightaway Stability & Trim                      |
| Goal: Car tracks dead-straight at 100% with zero weave     |
| Adjust: offset, k_head, k_cte                              |
+-------------------------------------------------------------+
                              |
                              v
+-------------------------------------------------------------+
| Step 2: Corner Apex Bite & Rotation                        |
| Goal: Decisive turn-in without outer-wall push or spin-out |
| Adjust: spd_out, spd_in, fa_th                             |
+-------------------------------------------------------------+
                              |
                              v
+-------------------------------------------------------------+
| Step 3: Corner Exit Punch & Traction                       |
| Goal: Immediate acceleration out of apex without fishtail  |
| Adjust: ramp_a, spd_in, blend (0 vs 1)                     |
+-------------------------------------------------------------+
                              |
                              v
+-------------------------------------------------------------+
| Step 4: Race Pace & Final Lap Time Optimization            |
| Goal: Maximum carry speed without understeering            |
| Adjust: spd_out (+2% increments), ramp_a (+5% increments)  |
+-------------------------------------------------------------+
```
