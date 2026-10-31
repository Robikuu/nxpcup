# NXP Cup Autonomous Car: Track-Side Parameter & Tuning Walkthrough

> **Platform**: NXP MCXN947 (Dual ARM Cortex-M33 @ 150 MHz)  
> **Perception**: Dual Pixy2 CMUCam5 (Top Downward Homography + Forward Horizon Lookahead)  
> **Drivetrain**: Dual DC Motors (DRV8871 H-Bridge with Active Dynamic Braking & Differential Torque Vectoring)  
> **Steering**: High-Torque RC Servo with Stanley State Feedback (CTIMER2 50 Hz PWM)  
> **Telemetry & Remote Shell**: ESP32 ESP-NOW <-> UART2 Transparent Bridge (115,200 Baud)  

---

## 1. System Dynamics & Tuning Philosophy

To extract maximum performance and achieve competitive lap times on the track, the vehicle balances two coupled physical domains: **Lateral Steering Stability** (front servo) and **Longitudinal Torque Vectoring** (rear dual motors).

```
                                  [ Forward Camera ]
                             Surveys Horizon 1-2m Ahead
                                          |
                        +-----------------+-----------------+
                        |                                   |
                  (Corner Detected)                 (Intersection Detected)
                        |                                   |
                        v                                   v
             [ Pre-Steer & Speed Governance ]     [ Straight Latch & Blind Gap ]
                        |                                   |
                        +-----------------+-----------------+
                                          |
                                          v
                                  [ Top Camera ]
                          Measures Immediate Lane Tangent
                            (Cross-Track Error & Heading)
                                          |
                        +-----------------+-----------------+
                        |                                   |
                        v                                   v
               [ Stanley Steering ]               [ Differential Torque ]
             Front Servo Deflection               Rear Wheel Speed Delta
           (Centering + Heading Damping)       (Outside Drive - Inside Drag)
```

### 1.1 How the Car Navigates Corners
1. **Preview & Entry**: As the car sprints down a straightaway at `spd_s` (100%), the forward camera detects an upcoming curve when the horizon road angle exceeds `fa_th`.
2. **Pre-Braking & Apex Setup**: Speed transitions down to `spd_out`. As the servo turns beyond the differential deadzone ($8.0^\circ$), the inside rear wheel slows toward `spd_in`. The speed difference ($\text{spd\_out} - \text{spd\_in}$) generates a mechanical **yaw torque** that actively rotates the chassis into the turn, preventing front-tire understeer.
3. **Apex Hold**: In the corner, the top camera maintains precise lane centering via $k_{\text{cte}}$ and heading damping via $k_{\text{head}}$.
4. **Exit Punch**: Once the road straightens, the differential drag releases and the motors accelerate back to `spd_s` at the rate controlled by `ramp_a` (preventing rear wheel slip).

### 1.2 Two Corner Architectures: Discrete (`blend 0`) vs Continuous (`blend 1`)
The car firmware supports two distinct corner governance strategies selected via the runtime command `blend <0|1>;`:

* **`blend 0` (Discrete Binary Mode — Recommended Baseline)**:
  * When a corner is detected (`fa_th` exceeded, sharp corner flag, or heading $\ge 20^\circ$), corner mode latches instantly with a 3-tick hold filter.
  * When exiting the apex, the latch releases immediately, unleashing full acceleration punch onto the straightaway.
  * *Best for*: High-grip tracks, sharp transitions, and aggressive racing lines where maximum acceleration out of corners is critical.

* **`blend 1` (Continuous Asymmetric Blend Governor)**:
  * Uses a diode/ratchet leaky integrator. Corner intensity ($0.0 \dots 1.0$) ramps up smoothly at rate `att` and decays gradually on exit at rate `rel`.
  * Top camera heading holds the blend active through the apex via the window $[h_{\text{lo}}, h_{\text{hi}}]$ even if the forward camera loses the horizon tape.
  * *Best for*: Low-grip surfaces, smooth sweeping corners, or track layouts where sudden throttle transitions cause the rear axle to break loose.

---

## 2. Quick Diagnostic Lookup Table (Symptoms & Fixes)

Locate what the car is physically doing on the track to find the exact variable, direction, and recommended step size to correct it:

| Observed Track Behavior | Root Cause | Primary Parameter | Direction | Recommended Adjustment |
| :--- | :--- | :--- | :--- | :--- |
| **Weaves left-and-right on straightaways** (hunting/oscillating) | Heading damping under-damped or CTE gain too stiff | `k_head`<br>`k_cte` | **Raise** `k_head`<br>**Lower** `k_cte` | `k_head 1.10 -> 1.25`<br>`k_cte 0.50 -> 0.45` |
| **Drifts steadily off-center to one side on straights** | Physical servo horn or linkage offset asymmetry | `offset` | **Trim opposite** | If drifts right: `offset -1.5`<br>If drifts left: `offset +1.5` |
| **Dives/snaps into corners violently (spins out / oversteers)** | Inside wheel drag too severe; excessive differential yaw torque | `spd_in` | **Raise** (softens pivot) | `spd_in 32 -> 38` or `42` |
| **Pushes wide / washes out of corners (understeers into outer wall)** | Outside wheel carrying too much speed, or inside pivot drag too weak | `spd_out`<br>`spd_in` | **Lower** `spd_out`<br>**Lower** `spd_in` | `spd_out 55 -> 48`<br>`spd_in 32 -> 26` |
| **Turns into corner prematurely** (clips inside line before reaching curve) | Forward camera triggering corner pre-steer from too far away | `fa_th` | **Raise** (requires steeper angle) | `fa_th 15 -> 22` |
| **Turns into corner too late** (reaches outer corner wall before steering turns) | Forward camera threshold too conservative or feedforward gated | `fa_th` | **Lower** (earlier detection) | `fa_th 20 -> 14` |
| **Sluggish acceleration exiting corners onto straightaways** | Acceleration slew rate too gentle, or blend release lingering | `ramp_a`<br>`blend` | **Raise** `ramp_a`<br>Set `blend 0` | `ramp_a 6 -> 15` or `25`<br>`blend 0` |
| **Rear wheels spin / break traction on corner exit** | Acceleration slew rate exceeds tire friction coefficient | `ramp_a` | **Lower** | `ramp_a 35 -> 15` (or `6.0`) |
| **Brakes too late into corners** (skids straight past the braking zone) | Deceleration slew rate too soft | `ramp_d` | **Raise** | `ramp_d 30 -> 50` |
| **Rear axle becomes unstable / snaps during hard braking** | Weight transfer too violent unweighting rear tires | `ramp_d` | **Lower** | `ramp_d 45 -> 25` |
| **Fishtails / snaps back when straightening out exiting a corner** | Heading damping too loose, or inside differential releasing unevenly | `k_head`<br>`spd_in` | **Raise** `k_head`<br>**Raise** `spd_in` | `k_head 1.10 -> 1.30`<br>`spd_in 32 -> 38` |
| **Stops or spins out in the middle of a 90° intersection** | Optical dropout duration exceeding transit window or false corner | Check `IX:1` | Verify track gap width | Window scales automatically with speed |
| **Car feels slow and lacks overall punch across whole track** | Global drive scale or battery voltage low | `scale` | **Raise** | `scale 0.70 -> 1.00` |

---

## 3. Master Parameter Catalog

Every parameter listed below is organized by functional domain. For runtime parameters, the exact serial/ESP command alias is provided.

```
Master Parameter Domain Breakdown:
├── 3.1 Drivetrain Speeds & Differential Torque Vectoring
├── 3.2 Lateral Steering & Stanley State Feedback
├── 3.3 Corner Preview & Transition Governance
├── 3.4 Motor Slew-Rate Limiters (Traction Guards)
├── 3.5 Global Governors & System Test Commands
└── 3.6 Critical Compile-Time Constants (Config.h & vectors.c)
```

---

### 3.1 Drivetrain Speeds & Differential Torque Vectoring

The car drives both rear wheels independently via CTIMER0 match channels. Longitudinal speed is governed by three primary targets:

```text
Straightaway Mode:
Left Wheel  = spd_s  * scale
Right Wheel = spd_s  * scale

Corner Mode (Turning Right, Steer > 8°):
Left Wheel  (Outside Drive) = spd_out * scale
Right Wheel (Inside Pivot)  = (spd_out - diff_drag) * scale  [Floored at spd_in]
```

#### `spd_s` (Straightaway Sprint Speed)
* **Command Aliases**: `spd_s <val>;`, `speed_straight <val>;`, `speed_s <val>;`
* **Default**: `100.0%` | **Safe Range**: `40.0 - 100.0%` | **Units**: PWM duty cycle percentage
* **What it does**: Full-throttle forward power applied equally to both rear wheels when no corner or intersection is active.
* **Track Dynamics**:
  * *Increasing (+)*: Decreases straightaway transit time; increases maximum speed entering corner braking zones.
  * *Decreasing (-)*: Lowers overall vehicle speed. Essential when testing a new track or on low-traction, dusty surfaces.
* **When to change**:
  * Set to `50.0 - 70.0%` during initial calibration to safely inspect steering stability.
  * Increase in steps of `+5.0%` up to `100.0%` once the car tracks clean and straight.

#### `spd_out` (Outside Wheel Apex Speed)
* **Command Aliases**: `spd_out <val>;`, `speed_outside <val>;`, `speed_out <val>;`
* **Default**: `55.0%` | **Safe Range**: `35.0 - 75.0%` | **Units**: PWM duty cycle percentage
* **What it does**: Controls the target speed of the **outside wheel** in a turn. The outside wheel provides the actual forward driving thrust that pushes the chassis through the curve.
* **Track Dynamics**:
  * *Increasing (+)*: Car carries more momentum and higher speed through the corner apex. If too high, centrifugal force overcomes front tire grip, causing understeer (washing wide into the outer wall).
  * *Decreasing (-)*: Car slows down deeper in corners, tightening the turning radius. If too low, car bogs down and loses significant lap time.
* **When to change**:
  * **Lower** (by `3 - 5%`) if the car enters the apex too hot and pushes wide towards the outer white tape line.
  * **Raise** (by `2 - 4%`) if the car easily hugs the inside rail and could carry substantially more speed.

#### `spd_in` (Inside Wheel Minimum Speed / Pivot Anchor)
* **Command Aliases**: `spd_in <val>;`, `speed_inside <val>;`, `speed_in <val>;`, `speed_corner <val>;`
* **Default**: `32.0%` | **Safe Range**: `20.0 - 50.0%` | **Units**: PWM duty cycle percentage
* **What it does**: Sets the minimum speed floor for the **inside wheel** during a turn. As the steering servo angles into a corner, differential drag reduces the inside wheel from `spd_out` down toward `spd_in`. The speed delta ($\text{spd\_out} - \text{spd\_in}$) creates a mechanical yaw moment.
* **Track Dynamics**:
  * *Increasing (+)*: Reduces the differential torque delta. The inside wheel continues rolling faster, reducing yaw rotation. Softens turn-in aggression.
  * *Decreasing (-)*: Increases the differential torque delta. The inside wheel acts as a heavy pivot anchor, violently rotating the car around the inside tire.
* **When to change**:
  * **Raise** (e.g. `32.0 -> 38.0%`): If the car snaps inwards violently, over-rotates, or spins out its rear axle on corner entry.
  * **Lower** (e.g. `32.0 -> 26.0%`): If the car struggles to turn into tight $90^\circ$ corners and front steering alone cannot make the turn.

---

### 3.2 Lateral Steering & Stanley State Feedback

Steering deflection is computed every 20 ms ($50\text{ Hz}$) using filtered cross-track error (distance from lane center) and filtered heading angle (vehicle orientation relative to lane tangent):

$$\text{steer\_cmd} = k_{\text{cte}} \times \text{filtered\_cte} + k_{\text{head}} \times \text{filtered\_heading} + \text{feedforward}$$

```text
             Track Centerline
                    |
      +-------------|-------------+
      |      <- CTE ->            |
      |             \  Heading    |
      |              \  Angle     |
      |               \           |
      |             [Car]         |
```

#### `k_cte` (Cross-Track Error Gain)
* **Command Aliases**: `k_cte <val>;`
* **Default**: `0.50` | **Safe Range**: `0.25 - 0.85` | **Units**: Steering degrees per pixel
* **What it does**: Proportional centering stiffness. It converts lateral offset from track center (measured by the downward top camera in pixels, where 1 px $\approx 6-7\text{ mm}$) into steering deflection.
* **Track Dynamics**:
  * *Increasing (+)*: Car pulls aggressively back toward the center of the lane. If too high, it induces rapid side-to-side lateral oscillations (hunting).
  * *Decreasing (-)*: Smoother, gentler steering. If too low, car drifts wide in gentle curves and rides close to the boundary tape.
* **When to change**:
  * **Raise** (`+0.05` steps) if the car wanders lazily off-center or cuts too close to boundary lines on straights.
  * **Lower** (`-0.05` steps) if the car oscillates or weaves side-to-side across the centerline.

#### `k_head` (Heading Angle Damping Gain)
* **Command Aliases**: `k_head <val>;`, `k_heading <val>;`
* **Default**: `1.10` | **Safe Range**: `0.70 - 1.80` | **Units**: Steering degrees per vehicle yaw degree
* **What it does**: Direct vehicle yaw rate damping. Measures the angle of the road vectors relative to the car's centerline. When the car begins to yaw, $k_{\text{head}}$ applies immediate counter-steer to align the chassis parallel with the track.
* **Track Dynamics**:
  * *Increasing (+)*: Heavily damps vehicle yaw. Eliminates straightaway fishtailing and stabilizes high-speed runs. If too high, steering feels locked up and resists turning into curves.
  * *Decreasing (-)*: Allows the chassis to rotate freely. If too low, high-speed straights will induce divergent sinusoidal weaving until the car spins off the track.
* **Special Behavior**: In `wifi_config.c`, setting `k_head` **automatically updates `k_head_c`** to match, ensuring consistent damping across straight and corner modes unless corner gain was explicitly overridden.
* **When to change**:
  * **Raise** (`1.10 -> 1.25 -> 1.40`): If the car weaves or fishtails on straightaways at high speed.
  * **Lower** (`1.10 -> 0.95`): If the car feels overly rigid and hesitates to turn into corners.

#### `k_cte_c` & `k_head_c` (Decoupled Corner Steering Gains)
* **Command Aliases**: `k_cte_c <val>;`, `k_cte_corner <val>;`, `k_head_c <val>;`, `k_heading_c <val>;`
* **Default**: Linked to `k_cte` (`0.50`) and `k_head` (`1.10`)
* **What it does**: Dedicated lateral gains applied specifically when corner mode is active. In `blend 1`, gains continuously interpolate between straight and corner values based on corner intensity. In `blend 0`, straight gains govern unless decoupled.
* **When to change**: Adjust only if you need aggressive lane-hugging centering inside sharp corners (`k_cte_c 0.65`) while maintaining soft, low-drag gains on straightaways (`k_cte 0.45`).

#### `offset` (Mechanical Steering Servo Trim)
* **Command Aliases**: `offset <val>;`, `steering_offset <val>;`
* **Default**: `0.00°` | **Safe Range**: `-8.0° to +8.0°` | **Units**: Physical degrees
* **What it does**: Direct zero-point calibration added to the servo PWM pulse. Compensates for mechanical spline misalignment, steering tie-rod length differences, or asymmetric chassis weight distribution.
* **Track Dynamics**:
  * Shifting positive (`+`) trims the front wheels toward the **right**.
  * Shifting negative (`-`) trims the front wheels toward the **left**.
* **When to change**:
  * Place the car on a clean straight line with `drive 0;` (motors off) and push it by hand, or run at low speed (`spd_s 40;`).
  * If the car pulls slowly to the **left**: increase `offset` positively (`+0.5°, +1.0°, +1.5°`).
  * If the car pulls slowly to the **right**: decrease `offset` negatively (`-0.5°, -1.0°, -1.5°`).

---

### 3.3 Corner Preview & Transition Governance

These parameters control when and how the car transitions from straightaway sprint mode to corner mode using forward-camera horizon lookahead and top-camera heading tracking.

```text
Straightaway ───────────► [ Horizon Angle > fa_th ] ───────────► Corner Apex
  Speed: spd_s                Attack Ramp: att                     Speed: spd_out
  Servo: Centered             Decel Ramp: ramp_d                  Servo: Full Deflection
                              Diff Pivot Engages                  Diff Pivot: spd_in
```

#### `fa_th` (Forward Lookahead Horizon Angle Threshold)
* **Command Aliases**: `fa_th <val>;`, `fa_thresh <val>;`
* **Default**: `20.0°` (calibrated) | **Safe Range**: `10.0° - 35.0°` | **Units**: Degrees
* **What it does**: The angle of upcoming track vectors (detected 1–2 meters ahead by the forward-facing camera) required to trigger corner pre-steer and longitudinal speed governance.
* **Track Dynamics**:
  * *Increasing (+)*: Makes the car less sensitive to distant bends. Delays corner entry pre-steer until the car is closer to the corner apex.
  * *Decreasing (-)*: Makes the car detect corners much earlier down the straight. Begins pre-braking and pre-steering earlier.
* **When to change**:
  * **Raise** (`20.0 -> 25.0°`): If the car spots a corner from too far away on a straightaway and begins turning in early, clipping the inside boundary tape before the turn actually starts.
  * **Lower** (`20.0 -> 14.0°`): If the car approaches corners carrying too much speed and reaches the outer wall before the steering begins turning.

#### `blend` (Continuous Corner Blend Enable Switch)
* **Command Aliases**: `blend <0|1>;`
* **Default**: `0` (Discrete Binary Mode) | **Options**: `0` (Disabled / Baseline) or `1` (Enabled / Continuous)
* **What it does**: Selects between the two core corner state machines:
  * `blend 0`: Instant binary state switching with 3-tick hold. Once a corner is confirmed, full corner speed and differential authority activate immediately; on exit, full straightaway throttle returns instantly.
  * `blend 1`: Smooth, continuous blending using an asymmetric leaky integrator (`att`, `rel`, `h_lo`, `h_hi`). Speed, gains, and differential drag transition continuously.
* **When to change**:
  * Keep at `0` for competitive racing on standard tracks where maximum exit punch is required.
  * Switch to `1` if track grip is very low and discrete transitions cause the rear tires to break traction or snap on corner exit.

#### `att` (Blend Attack Rate) & `rel` (Blend Release Rate)
* **Command Aliases**: `att <val>;` (or `blend_att <val>;`), `rel <val>;` (or `blend_rel <val>;`)
* **Defaults**: `att = 0.35`, `rel = 0.06` | **Safe Ranges**: `att: 0.10 - 0.80`, `rel: 0.02 - 0.20`
* **Active in**: `blend 1` mode only.
* **What it does**:
  * `att`: Controls how fast corner blend ramps from 0.0 to 1.0 upon detecting a corner. At `0.35`, reaches ~88% corner engagement in 5 ticks (100 ms).
  * `rel`: Controls how slowly corner blend decays back from 1.0 to 0.0 when exiting. At `0.06`, takes ~25–30 ticks (500–600 ms) for full straightaway recovery.
* **When to change**:
  * **Lower `att`** (`0.35 -> 0.20`): Softens abrupt turn-in jerks at corner entry.
  * **Raise `rel`** (`0.06 -> 0.12`): Provides faster throttle recovery exiting corners onto the straightaway.

#### `h_lo` & `h_hi` (Top Camera Heading Floor & Ceiling)
* **Command Aliases**: `h_lo <val>;`, `h_hi <val>;`
* **Defaults**: `h_lo = 12.0°`, `h_hi = 25.0°` | **Safe Ranges**: `h_lo: 8.0 - 18.0°`, `h_hi: 20.0 - 45.0°`
* **Active in**: `blend 1` mode only.
* **What it does**: Acts as a diode hold for corner mode. In long corners, the forward camera can lose sight of the outer tape; `h_lo` and `h_hi` look at the downward top camera's heading angle to keep corner mode held through the apex. Below `h_lo`, heading hold is zero (preventing false drag on straights); above `h_hi`, heading hold is 100%.
* **When to change**:
  * **Raise `h_lo`** (`12.0 -> 15.0°`): If the car straightens out onto a straightaway but corner speed reduction lingers too long.
  * **Lower `h_lo`** (`12.0 -> 9.0°`): If the car washes wide mid-corner because corner mode releases prematurely while still turning.

---

### 3.4 Motor Slew-Rate Limiters (Traction Guards)

The DRV8871 H-bridge receives target speeds from the control law, but applied motor PWM is rate-limited every 20 ms tick ($50\text{ Hz}$) by `ramp_a` and `ramp_d`:

$$\text{applied\_speed}[k] = \text{clamp}(\text{target\_speed}, \text{applied}[k-1] - \text{ramp\_d}, \text{applied}[k-1] + \text{ramp\_a})$$

```text
Throttle Command Step (55% -> 100%):
  If ramp_a = 6.0%/tick  ──► Takes 8 ticks  (160 ms)  [Soft, zero wheel spin]
  If ramp_a = 25.0%/tick ──► Takes 2 ticks  (40 ms)   [Aggressive race punch]
  If ramp_a = 45.0%/tick ──► Takes 1 tick   (20 ms)   [Instant step response]
```

#### `ramp_a` (Acceleration Ramp Slew Rate)
* **Command Aliases**: `ramp_a <val>;`, `accel_ramp <val>;`
* **Default**: `6.0%/tick` in `Config.h` (or `35.0%/tick` in fast profiles) | **Safe Range**: `4.0 - 50.0%/tick`
* **What it does**: Limits how fast motor throttle can increase per 20 ms tick. Functions as software traction control on corner exits.
* **Track Dynamics**:
  * *Increasing (+)*: Delivers punchier, instant acceleration out of corners onto straightaways. Improves lap time significantly if rear tires have sufficient grip.
  * *Decreasing (-)*: Smoothes throttle application. Prevents rear wheel spin and power-oversteer fishtailing on slick or dusty tiles.
* **When to change**:
  * **Raise** (`6.0 -> 15.0 -> 30.0`): If the car exits corners smoothly and straight with zero tire slip, but takes too long to get up to top speed on the straight.
  * **Lower** (`35.0 -> 10.0`): If the car's rear tires spin or the rear axle snaps sideways under hard acceleration when unwinding steering.

#### `ramp_d` (Deceleration / Pre-Braking Ramp Slew Rate)
* **Command Aliases**: `ramp_d <val>;`, `decel_ramp <val>;`
* **Default**: `30.0%/tick` | **Safe Range**: `15.0 - 70.0%/tick`
* **What it does**: Limits how fast motor throttle can decrease per 20 ms tick. Governs braking deceleration entering corners.
* **Track Dynamics**:
  * *Increasing (+)*: Allows rapid, late pre-braking right before corner entry.
  * *Decreasing (-)*: Softens braking. Prevents aggressive weight transfer that unweights the rear tires and causes the rear axle to break loose entering turns.
* **When to change**:
  * **Raise** (`30.0 -> 50.0`): If the car carries too much speed into corners and skids past the turn-in point.
  * **Lower** (`45.0 -> 25.0`): If hard deceleration makes the car twitch or yaw violently on straightaways right before corner entry.

---

### 3.5 Global Governors & System Test Commands

These utility commands can be executed live over the serial terminal / ESP-NOW:

#### `scale` (Global Output Plant Gain Multiplier)
* **Command Aliases**: `scale <val>;`, `drive_scale <val>;`
* **Default**: `1.00` | **Safe Range**: `0.10 - 1.00`
* **What it does**: Multiplies all motor outputs by a scalar factor before sending them to the DRV8871 driver:
  $$\text{motor\_pwm} = \text{applied\_speed} \times \text{drive\_scale}$$
* **When to change**:
  * Set to `0.50 - 0.70` when mapping an unknown track, validating camera detection, or demonstrating at safe speeds.
  * Set to `1.00` for full race pace.

#### `drive` (Motor Drive Enable)
* **Command Aliases**: `drive 0;`, `drive 1;`, `motor_enable <0|1>;`
* **What it does**:
  * `drive 0;`: Instantly disables motor PWM while keeping cameras, state machine, and servo steering fully active.
  * `drive 1;`: Re-enables motor drive.
* **Track Use**: Crucial for **hand-push testing**. Set `drive 0;`, start the car (`start;`), and walk it by hand along the track to visually inspect if the front wheels steer accurately and center properly without motor noise.

#### `steer` (Manual Servo Actuation Test)
* **Command Aliases**: `steer <angle_deg>;`
* **Active in**: `STATE_IDLE` only (must send `stop;` first).
* **What it does**: Commands the front servo directly to a specified angle (e.g. `steer 20;`, `steer -20;`, `steer 0;`).
* **Track Use**: Verifies physical steering linkage freedom, checks for mechanical binding against the chassis, and verifies that `steer 0;` points wheels dead-straight.

#### `motor_left` & `motor_right` (Bench Motor Spin Tests)
* **Command Aliases**: `motor_left <speed>;`, `motor_right <speed>;`
* **Active in**: `STATE_IDLE` only.
* **What it does**: Spins the selected motor for exactly 4 seconds (200 ticks) at the specified duty cycle (e.g. `motor_left 40;`).
* **Track Use**: Verifies motor wiring polarity, wheel rotation direction, and gear mesh before placing the car on the track.

#### `dump` (Raw Vector Stream Toggle)
* **Command Aliases**: `dump 0;`, `dump 1;`, `dump 2;`, `dump_vectors <val>;`
* **What it does**:
  * `dump 0;`: Disables raw vector output.
  * `dump 1;`: Streams all raw vector endpoints `(x0, y0, x1, y1)` from both cameras over serial at 12.5 Hz.
  * `dump 2;`: One-shot vector dump (prints one frame then automatically reverts to 0).
* **Track Use**: Used to diagnose camera glare, missing lane lines, or reflections from track lighting.

#### `status;`
* **What it does**: Dumps a complete snapshot of all 18 live parameter values, blend state, drive enable, and current robot state over the terminal.

---

### 3.6 Critical Compile-Time Parameters (`Config.h`, `vectors.c`, `main.c`)

These constants are compiled into the firmware binary. Modifying them requires editing the source and flashing the MCXN947 via LinkServer/MCUXpresso:

| Parameter / Constant | File Location | Value | Engineering Role & Track Effect |
| :--- | :--- | :--- | :--- |
| `DIFFERENTIAL_DEADZONE_DEG` | [`Config.h`](file:///home/robert/facultate/Practica/workspace/nxpcup/include/Config.h#L32) | `8.0f` | Steering deflection angle below which differential drag is exactly zero. Prevents torque-steer weaving on straightaways while front wheels make minor centering corrections. |
| `STEER_FULL_BRAKE_DEG` | [`Config.h`](file:///home/robert/facultate/Practica/workspace/nxpcup/include/Config.h#L33) | `40.0f` | Steering angle at which inside wheel differential drag reaches 100% of its available span (`spd_out - spd_in`). |
| `FRONT_CORNER_ANGLE_THRESH_DEG` | [`Config.h`](file:///home/robert/facultate/Practica/workspace/nxpcup/include/Config.h#L36) | `30.0f` | Raw horizon angle threshold in `vectors.c` where the forward camera sets `is_sharp_corner = 1`. |
| `FRONT_FEEDFORWARD_GAIN` | [`Config.h`](file:///home/robert/facultate/Practica/workspace/nxpcup/include/Config.h#L38) | `0.45f` | Proportional multiplier applied to `filtered_front_angle` to steer into corner before apex. Gated by top camera turn entry. |
| `STEERING_LIMIT_RIGHT / LEFT` | [`Config.h`](file:///home/robert/facultate/Practica/workspace/nxpcup/include/Config.h#L17-L18) | `±65.0f` | Mechanical travel limits protecting servo gears and linkages from binding against suspension arms. |
| `OFF_TRACK_FAILSAFE_TICKS` | [`Config.h`](file:///home/robert/facultate/Practica/workspace/nxpcup/include/Config.h#L51) | `125U` (2.5s) | Control loop ticks without detecting any lines before triggering automatic emergency stop and dynamic braking. |
| `TOP_CAM_EXCLUDE_Y_MIN` | [`Config.h`](file:///home/robert/facultate/Practica/workspace/nxpcup/include/Config.h#L12) | `38U` | Frame Y coordinate below which vectors are rejected unless extending into the upper view. Eliminates bumper/tie-rod reflections. |
| `TOP_CAM_EXCLUDE_X_MIN / MAX` | [`Config.h`](file:///home/robert/facultate/Practica/workspace/nxpcup/include/Config.h#L13-L14) | `18U / 62U` | Horizontal frame bounds for the chassis exclusion zone. |
| `TRACK_HALF_WIDTH_PX` | [`Config.h`](file:///home/robert/facultate/Practica/workspace/nxpcup/include/Config.h#L37) | `25.0f` | Virtual track half-width (pixels) used to reconstruct track center when only a single boundary line is visible. |
| `HEADING_DEADBAND_DEG` | [`Config.h`](file:///home/robert/facultate/Practica/workspace/nxpcup/include/Config.h#L47) | `2.0f` | Straightaway heading deadband in `main.c` that zeroes small heading jitter to prevent high-speed twitch. |
| `TOP_CAM_LOOKAHEAD_Y_REF` | [`vectors.c`](file:///home/robert/facultate/Practica/workspace/nxpcup/source/vectors.c#L12) | `44.0f` | Reference evaluation row for linear slope projection, ensuring consistent CTE measurement regardless of vector endpoint height. |
| `TOP_SINGLE_LINE_PERSP_DEG` | [`vectors.c`](file:///home/robert/facultate/Practica/workspace/nxpcup/source/vectors.c#L15) | `14.0f` | Geometric perspective compensation subtracted when only one line is visible, preventing false inward steer bias. |
| `FRONT_SINGLE_LINE_PERSP_DEG`| [`vectors.c`](file:///home/robert/facultate/Practica/workspace/nxpcup/source/vectors.c#L16) | `18.0f` | Perspective vanishing-point compensation for single lines in the forward camera, preventing false corner triggers on straights. |
| `Intersection Persistence Timer`| [`main.c`](file:///home/robert/facultate/Practica/workspace/nxpcup/source/main.c#L226) | `15 ticks` (300ms) | Duration to latch intersection mode across the blind gap with 3-tick sharp corner cancel hysteresis. |
| `Re-acquisition Slew Limiter`| [`main.c`](file:///home/robert/facultate/Practica/workspace/nxpcup/source/main.c#L380-L388) | `4.0°/tick` (3 ticks) | Limits steering delta to $\le 4.0^\circ/\text{tick}$ upon re-detecting lane lines exiting an intersection gap. |

---

## 4. Step-by-Step 4-Stage Track-Side Tuning Protocol

Follow this structured protocol whenever testing on a new track or after replacing tires, steering linkages, or battery packs:

```text
+--------------------------------------------------------------------------+
| Stage 1: Mechanical Zeroing, Trim & Straight Stability                   |
| Target: Car tracks dead-straight at 100% throttle with zero weave        |
| Key Parameters: offset, k_head, k_cte, spd_s                             |
+--------------------------------------------------------------------------+
                                     |
                                     v
+--------------------------------------------------------------------------+
| Stage 2: Corner Turn-In & Apex Rotation                                  |
| Target: Crisp, decisive turn-in hugging inside apex without spinning     |
| Key Parameters: spd_out, spd_in, fa_th                                   |
+--------------------------------------------------------------------------+
                                     |
                                     v
+--------------------------------------------------------------------------+
| Stage 3: Corner Exit Punch & Traction Management                         |
| Target: Maximum acceleration out of apex with zero fishtailing           |
| Key Parameters: ramp_a, blend (0 vs 1), rel                              |
+--------------------------------------------------------------------------+
                                     |
                                     v
+--------------------------------------------------------------------------+
| Stage 4: High-Speed Race Pace Optimization                               |
| Target: Shaving tenths off lap time while maintaining 100% stability     |
| Key Parameters: spd_out (+2% steps), ramp_a (+5% steps), ramp_d          |
+--------------------------------------------------------------------------+
```

### Stage 1: Mechanical Zeroing, Trim & Straightaway Stability
1. **Elevate the wheels** and verify zero steering angle with `steer 0;`. Ensure the front wheels point dead-ahead physically.
2. Set `drive 0;` and `start;`. Push the car by hand down a clean straight track section. Observe telemetry token `S` (servo command). It should hover within $\pm 10$ ($\pm 1.0^\circ$).
3. Lower motor speed for baseline testing: `spd_s 60;`, `drive 1;`.
4. Run the car down the straightaway:
   * If it pulls steadily to the left, increase `offset` positively (`offset 1.0;`).
   * If it pulls steadily to the right, decrease `offset` negatively (`offset -1.0;`).
5. Increase straightaway speed to full: `spd_s 100;`.
   * If the car begins **weaving / hunting** side-to-side, raise heading damping in steps of `0.10`: `k_head 1.20;` -> `k_head 1.30;`. If hunting persists, slightly lower centering gain: `k_cte 0.45;`.

### Stage 2: Corner Turn-In & Apex Rotation
1. Observe the car approaching a standard $90^\circ$ bend:
   * If it begins turning **too early** on the straightaway, raise the forward camera threshold: `fa_th 22;` or `fa_th 25;`.
   * If it reacts **too late** and reaches the outer wall before turning, lower: `fa_th 16;`.
2. Observe apex momentum and rotation:
   * If the car **washes wide (understeers)**, lower outside speed in steps of `3%`: `spd_out 52;` -> `spd_out 48;`. If it still pushes wide, drop the inside pivot speed: `spd_in 28;`.
   * If the car **spins out (oversteers)** upon entering the corner, the inside wheel drag is too aggressive: raise `spd_in 36;` or `spd_in 40;`.

### Stage 3: Corner Exit Punch & Traction Management
1. Observe the transition as the car unwinds steering onto the straightaway:
   * If the car straightens out cleanly but feels **sluggish** gaining speed, raise the acceleration ramp: `ramp_a 15;` -> `ramp_a 25;` -> `ramp_a 35;`. Ensure `blend 0;` is active.
   * If the rear wheels spin, squeak, or fishtail sideways as throttle is applied, lower `ramp_a` to `10;` or `6;` to protect tire traction.
2. If track tiles are extremely dusty or slick and discrete mode (`blend 0`) snaps violently, switch to continuous blend: `blend 1;`, and tune `rel 0.08;` for a smooth throttle ramp-up.

### Stage 4: High-Speed Race Pace Optimization
1. Once 3 consecutive clean laps are completed without touching boundary lines, incrementally push apex carry speed:
   * Raise `spd_out` in `+2%` increments (`55 -> 57 -> 59`).
   * Raise `ramp_a` in `+5%` increments for maximum corner exit acceleration.
2. If the car enters braking zones too deep at elevated straight speeds, raise deceleration ramp: `ramp_d 50;`.

---

## 5. Live ESP-NOW / Serial Shell Cheat Sheet

All commands are ASCII strings terminated with a semicolon (`;`). Commands can be sent via an ESP32 bridge (over ESP-NOW) or directly via USB serial console.

### Command Syntax & Rules
* Format: `<command> <value>;` or `<command>;`
* Numbers can be integer or floating point (e.g. `spd_s 95.5;`).
* Every valid command returns an acknowledgement (e.g. `OK speed_straight=95.50\r\n`).

### Quick Command Table
```text
Command                  Example             Description
----------------------------------------------------------------------------------------
start;                   start;              Transitions state to RUNNING (motors active)
stop; / brake;           stop;               Immediate stop, dynamic motor brake, IDLE
status;                  status;             Prints snapshot of all 18 active variables
help;                    help;               Prints list of supported commands
version;                 version;            Prints build timestamp and firmware version

drive <0|1>;             drive 0;            Disables (0) or enables (1) motor outputs
scale <val>;             scale 0.85;         Sets global plant drive scale (0.10 to 1.00)
steer <deg>;             steer 15;           Tests physical servo angle in IDLE
motor_left <spd>;        motor_left 40;      Spins left motor for 4 seconds in IDLE
motor_right <spd>;       motor_right 40;     Spins right motor for 4 seconds in IDLE
dump <0|1|2>;            dump 1;             Enables (1), disables (0), or one-shots (2) vector dump

spd_s <val>;             spd_s 100;          Sets straightaway sprint speed (%)
spd_out <val>;           spd_out 52;         Sets outside wheel corner speed (%)
spd_in <val>;            spd_in 34;          Sets inside wheel pivot speed floor (%)
offset <deg>;            offset -1.5;        Sets mechanical steering center trim (degrees)

k_cte <val>;             k_cte 0.50;         Sets cross-track error centering gain
k_head <val>;            k_head 1.25;        Sets heading angle damping gain
k_cte_c <val>;           k_cte_c 0.60;       Sets decoupled corner CTE gain
k_head_c <val>;          k_head_c 1.30;      Sets decoupled corner heading gain

fa_th <deg>;             fa_th 18;           Sets forward horizon corner threshold (degrees)
blend <0|1>;             blend 0;            Sets discrete binary (0) or continuous blend (1)
att <val>;               att 0.35;           Sets blend attack rate (0.05 to 1.00)
rel <val>;               rel 0.08;           Sets blend release rate (0.01 to 0.50)
h_lo <deg>;              h_lo 12;            Sets top camera heading hold floor (degrees)
h_hi <deg>;              h_hi 25;            Sets top camera heading hold ceiling (degrees)
ramp_a <val>;            ramp_a 25;          Sets acceleration slew rate (% per tick)
ramp_d <val>;            ramp_d 45;          Sets deceleration slew rate (% per tick)
```

---

## 6. Live Telemetry Decoder & Diagnostic Signals

Every 4th control loop tick ($12.5\text{ Hz} = 80\text{ ms}$), the car transmits a single line of telemetry over serial and ESP-NOW:

```text
L:2/3 R:5 F:2 H:-11 TS:-15 FS:24 S:-15 BL:0 BT:0 DOM:T FV:1 IX:0 ST:S VL:100 VR:100
```

### Token Reference Table
| Token | Meaning | Scaling / Units | Healthy Range | Red Flag / What to Watch |
| :--- | :--- | :--- | :--- | :--- |
| `L:x/y` | Lines detected (`x`=top cam valid, `y`=front cam valid) | Counts (0 to 10) | `L:2/2` or `L:2/3` | `L:0` (blind/dropout), `L:1` (single line tracking active) |
| `R` | Raw Cross-Track Error (CTE) | $\times 10$ pixels | $-15 \dots +15$ | $|R| > 50$ (car is riding on the edge of track tape) |
| `F` | Filtered Cross-Track Error | $\times 10$ pixels | $-10 \dots +10$ | Large values flipping sign rapidly indicates lateral hunting |
| `H` | Filtered Heading Angle | $\times 10$ degrees | $-8 \dots +8$ on straights | $|H| > 150$ on straights means severe yaw oscillation |
| `TS` | Top Camera Steering Component | $\times 10$ degrees | Varies with CTE & H | Sudden isolated spikes indicate bad vector selection |
| `FS` | Filtered Forward Horizon Angle | $\times 10$ degrees | $-5 \dots +5$ on straights | $> +200$ or $< -200$ indicates incoming corner detected |
| `S` | Final Steer Command to Servo | $\times 10$ degrees | $-650 \dots +650$ | Pinned at $\pm 650$ for $>1\text{s}$ means car cannot make radius |
| `BL` | Applied Corner Blend Level | $0 \dots 100\%$ | $0\%$ straight, $100\%$ corner | In `blend 0`, snaps $0 \leftrightarrow 100$. In `blend 1`, ramps smoothly |
| `BT` | Raw Blend Target Metric | $0 \dots 100\%$ | Matches `BL` hold | Sustained $BT > 0$ on clean straight indicates glare/false detection |
| `DOM` | Dominant View Sensor | `T` (Top) or `F` (Front) | `T` straight, `F` in corner | Stays `T` on clean straights |
| `FV` | Forward Horizon Valid Flag | `0` or `1` | `1` (lines visible) | Sustained `0` means front camera tilted or obscured |
| `IX` | Intersection Active Flag | `0` or `1` | `0` (normal track) | Should only read `1` when crossing perpendicular intersection |
| `ST` | Macro Tactical State | `S`, `C`, or `I` | S=Straight, C=Corner, I=Intersection | `ST:C` on a straightaway indicates false corner trigger |
| `VL` | Left Wheel Applied Duty Cycle | $0 \dots 100\%$ | Matches `VR` on straight | Drops to `spd_in` during right turns |
| `VR` | Right Wheel Applied Duty Cycle | $0 \dots 100\%$ | Matches `VL` on straight | Drops to `spd_in` during left turns |

---

## 7. Environmental & Electrical Variables

When tuning, always keep these three physical real-world variables in mind:

1. **Battery Voltage Sag**:
   * As the 2S/3S LiPo battery discharges from $8.4\text{V}$ down to $7.4\text{V}$, DC motor output torque decreases proportionally.
   * *Symptom*: A car tuned at full charge begins understeering or failing to make corners late in a battery cycle because differential drag produces less mechanical yaw.
   * *Fix*: Always calibrate critical corner parameters (`spd_out`, `spd_in`) at nominal battery voltage (~$7.6 - 7.8\text{V}$), or compensate by slightly increasing `scale` (`0.90 -> 1.00`).

2. **Ambient Lighting & Camera Glare**:
   * Pixy2 sensors rely on high-contrast edge detection between white track tape and dark floor surfaces.
   * Direct overhead spotlights can create specular reflections that Pixy2 interprets as diagonal vectors.
   * *Diagnostic*: Enable `dump 1;` while parked under questionable lighting. If raw vectors appear where no track tape exists, adjust camera tilt angle or lower Pixy2 exposure.

3. **Track Surface Friction Coefficient**:
   * Dusty sport-hall PVC tiles have significantly lower friction than clean wooden parquet or rubberized track mats.
   * On low-friction surfaces, lower `ramp_a` (`6 - 15%`) to prevent wheel spin, and raise `spd_in` (`36 - 40%`) to prevent snap-oversteer.
