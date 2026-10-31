# NXP Cup Autonomous Race Car: System & Codebase Architecture Guide

## 1. Executive Summary

This document serves as the comprehensive technical guide and architectural reference for the NXP Cup autonomous racing car firmware (`nxpcup-ciorba_curata`).

The vehicle employs a **Two-Degree-of-Freedom (2-DOF) Autonomous Racing System** running on an **NXP MCXN947 (Dual ARM Cortex-M33)** microcontroller. The architecture combines:
1. **High-bandwidth local tracking & stabilization** via a top-mounted homography camera.
2. **Predictive lookahead trajectory planning & apex cutting** via a forward-facing camera.
3. **Active torque-vectoring differential braking** on the dual rear motors to induce crisp chassis yaw.
4. **Bilateral anti-spin traction guarding** with dynamic chicane reversal overrides.

---

## 2. System Hardware & Electrical Architecture

```
                                      +---------------------------------------------+
                                      |          NXP MCXN947 (Dual Cortex-M33)      |
                                      +---------------------------------------------+
                                         |                   |                   |
                     [Flexcomm 3 / I2C]  |  [Flexcomm 2 / I2C|  [CTIMER2 / PWM]  |  [CTIMER0 / PWM]
                         (400 kHz eDMA)  |    (400 kHz eDMA) |          (50 Hz)  |          (10 kHz)
                                         v                   v                   v                   v
                                +-----------------+ +-----------------+ +-----------------+ +-----------------+
                                |  Top Camera     | |  Front Camera   | | Steering Servo  | | Dual H-Bridge   |
                                |  (Flexcomm 3)   | |  (Flexcomm 2)   | | (Front Wheels)  | | (Rear Wheels)   |
                                |  Homography/PID | |  Trajectory/Apex| | [-70°, +87°]    | | Torque Vectoring|
                                +-----------------+ +-----------------+ +-----------------+ +-----------------+
```

### 2.1 Microcontroller Unit (MCU)
- **Processor**: NXP MCXN947 (Cortex-M33 Core 0 @ 150 MHz).
- **FPU**: Hardware single-precision floating-point unit (`-mfpu=fpv5-sp-d16 -mfloat-abi=hard`).
- **Memory**: 1 MB Dual-Bank Flash, 384 KB System SRAM + 96 KB SRAMX.

### 2.2 Dual Vision Subsystems (Pixy2 CMUCam5)
Both cameras operate on independent I2C buses to eliminate bus arbitration contention and maximize throughput:
- **Top / PID Camera (`cam_pid`)**:
  - **Mounting**: High mount pointing directly down in front of the front bumper (homography projection).
  - **Peripheral**: `LP_FLEXCOMM3` (I2C Master, Header J5 / mikroBUS Right socket).
  - **Pins**: `P1_0` (SDA), `P1_1` (SCL).
  - **Baud Rate**: 400 kHz.
  - **DMA Allocation**: eDMA Channel 2 (RX), eDMA Channel 3 (TX).
  - **Loop Rate**: 50 Hz (polled every 20ms tick).
  - **Status LED**: Green (`pixy_set_led(&cam_pid, 0, 255, 0)`).
- **Front / Trajectory Camera (`cam_front`)**:
  - **Mounting**: Forward-tilted mount surveying 1–2+ meters down the track.
  - **Peripheral**: `LP_FLEXCOMM2` (I2C Master, Header J8).
  - **Pins**: `P4_0` (SDA), `P4_1` (SCL).
  - **Baud Rate**: 400 kHz.
  - **DMA Allocation**: eDMA Channel 0 (RX), eDMA Channel 1 (TX).
  - **Loop Rate**: 25 Hz (polled alternating ticks).
  - **Status LED**: Blue (`pixy_set_led(&cam_front, 0, 0, 255)`).

### 2.3 Actuators & Motor Control
- **Steering Actuator**:
  - High-torque RC steering servo connected via `CTIMER2`.
  - PWM Frequency: 50 Hz ($T = 20.0\,\text{ms}$).
  - Hardware Mechanical Travel Limits: `[-70.0°, +87.0°]`.
  - Neutral Hardware Trim Offset: `+14.0°`.
- **Rear Propulsion & Differential Drive**:
  - Dual DC motors driven via a high-current H-bridge module.
  - PWM Generation: `CTIMER0` (Match channels 1 & 2).
  - Direction Control: `GPIO0_24` (Motor 1 / Right), `GPIO0_27` (Motor 2 / Left).
  - Independent PWM duty cycle modulation allows active differential drag torque vectoring.

### 2.4 Telemetry & Remote Configuration
- **Transceiver**: ESP32 Wi-Fi bridge connected to `LP_FLEXCOMM9` (LPUART @ 115,200 baud).
- **Protocol**: ASCII telemetry stream throttled to 12.5 Hz (every 80ms) to prevent buffer overflows while providing real-time telemetry and parameter tuning.

---

## 3. Real-Time Deterministic Scheduling & Timing Profile

The car executes a strictly deterministic control loop driven by the Cortex-M33 `SysTick` timer configured for **50 Hz ($T_s = 20.0\,\text{ms}$)**.

```
0 ms                       2.5 ms                      2.7 ms             8.8 ms (every 4th tick)     20 ms
|----------------------------|---------------------------|-------------------|-----------------------------|
   I2C DMA Vector Transfer      PID & Trajectory Fusion     Motor & Servo PWM   UART Telemetry (12.5 Hz)      Idle /
   (Top / Front alternating)    Math (Cortex-M33 FPU)       Update              LPUART_WriteBlocking          Safety
```

### Execution Budget
| Task / Phase | Execution Time | Type | Description |
|---|---|---|---|
| **Front Cam Query (25 Hz)** | $\approx 2.5\,\text{ms}$ | eDMA Non-blocking | Asynchronous vector packet retrieval from `cam_front` |
| **Top Cam Query (50 Hz)** | $\approx 2.5\,\text{ms}$ | eDMA Non-blocking | Asynchronous vector packet retrieval from `cam_pid` |
| **Vector Filtering** | $< 0.08\,\text{ms}$ | CPU / FPU | Angle rejection, pair combinations & continuity tracking |
| **Trajectory & Apex Planner** | $< 0.05\,\text{ms}$ | CPU / FPU | Midpoint horizon angle, offset, and apex shift calculation |
| **PID & Gain Scheduler** | $< 0.02\,\text{ms}$ | CPU / FPU | Dynamic gain interpolation & filtered derivative PID |
| **Torque Vectoring Logic** | $< 0.04\,\text{ms}$ | CPU / FPU | Speed calculations, bilateral traction guard & reversal logic |
| **PWM & GPIO Actuation** | $< 0.02\,\text{ms}$ | Hardware MMIO | CTIMER Match Register updates for servo & motors |
| **Telemetry (Throttled)** | $\approx 6.1\,\text{ms}$ (1 of 4 ticks) | LPUART Blocking | Throttled to 12.5 Hz; takes $0.0\,\text{ms}$ on other 3 ticks |
| **Total Worst-Case Time** | **$11.3\,\text{ms}$** | — | **$> 43\%$ CPU margin remaining within the 20ms frame** |

---

## 4. Module-by-Module Codebase Architecture

```
nxpcup-ciorba_curata/
├── include/
│   ├── Config.h              # Master constants, gains, speed profiles, and vehicle parameters
│   ├── hbridge.h             # Dual motor H-bridge driver definitions
│   ├── pixy.h                # Asynchronous eDMA Pixy2 camera driver definitions
│   └── servo.h               # Steering servo interface and calibration limits
└── source/
    ├── main.c                # Control loop, 2-DOF fusion, torque vectoring, state machine
    ├── vectors.c / .h        # Geometric computer vision, pair filtering, trajectory planner
    ├── control_theory.c / .h # Filtered-derivative PID controller implementation
    ├── hbridge.c             # Motor PWM & direction GPIO control
    ├── servo.c               # Servo pulse-width generation and physical clamping
    ├── pixy.c                # Dual-instance eDMA I2C communication engine
    └── wifi_config.c / .h    # Dynamic ASCII command shell & throttled telemetry engine
```

### 4.1 [`include/Config.h`](file:///home/robert/facultate/Practica/workspace/nxpcup/include/Config.h) — Central Calibration Repository & Presets
All vehicle parameters are consolidated into centralized definitions, now supporting **Dual Runtime Presets** and hold-button switching:
- **Geometry**: `PIXY_FRAME_WIDTH (79.0)`, `PIXY_FRAME_HEIGHT (52.0)`, `PIXY_CENTER_X (39.5)`, `PIXY_TRACK_HALF_WIDTH (25.0)`.
- **Physical Steering Limits**: `STEERING_LIMIT_RIGHT (+77.0)`, `STEERING_LIMIT_LEFT (-67.0)`, `STEERING_OFFSET (+10.0)`.
- **Dual Car Parameter Presets**:
  - **Preset 1 (Calibrated Race Setup)**:
    - Straight speed: `PRESET1_SPEED_STRAIGHT (100.0%)`
    - Outside corner speed: `PRESET1_SPEED_OUTSIDE_CORNER (65.0%)`
    - Inside pivot speed: `PRESET1_SPEED_INSIDE_MIN (34.0%)`
    - Drive plant scale: `PRESET1_DRIVE_SCALE (1.00)`
    - Accel / Decel ramps: `PRESET1_ACCEL_RAMP (6.5%)`, `PRESET1_DECEL_RAMP (30.0%)`
    - LED Status: Green on Top Camera
  - **Preset 2 (Conservative / Technical Track Setup)**:
    - Straight speed: `PRESET2_SPEED_STRAIGHT (85.0%)`
    - Outside corner speed: `PRESET2_SPEED_OUTSIDE_CORNER (55.0%)`
    - Inside pivot speed: `PRESET2_SPEED_INSIDE_MIN (30.0%)`
    - Drive plant scale: `PRESET2_DRIVE_SCALE (1.00)`
    - Accel / Decel ramps: `PRESET2_ACCEL_RAMP (5.0%)`, `PRESET2_DECEL_RAMP (30.0%)`
    - LED Status: Magenta on Top Camera
- **Hardware Button Interaction**:
  - `BUTTON_HOLD_TICKS (50 ticks = 1.0s @ 50 Hz)`:
    - **In `STATE_IDLE`**: Short tap (< 1.0s) & release starts the vehicle (`STATE_RUNNING`). Long hold (>= 1.0s) toggles between Preset 1 and Preset 2.
    - **In `STATE_RUNNING`**: Any press immediately halts the vehicle into `STATE_IDLE` with motor brake (fail-safe emergency stop).
- **Ramp Rates & Traction Guards**:
  - `DECEL_RAMP_STEP (30.0% / tick)`: Rapid pre-braking ramp (~2 ticks from 100% to 40%).
  - `ACCEL_RAMP_STEP (6.5% / tick)`: Smooth traction-preserving corner exit acceleration (~160ms).
- **Deadzones & Gates**:
  - `DIFFERENTIAL_DEADZONE_DEG (8.0°)`: Steering deadband below which zero differential drag applies.
  - `STEER_FULL_BRAKE_DEG (40.0°)`: Steering angle for 100% pivot drag.
  - `FRONT_ANGLE_THRESH (28.0°)`: Front camera corner angle threshold.
- **Dynamic PID Gain Matrix**:
  - $K_p \in [0.70, 2.30]$
  - $K_d \in [0.160, 0.130]$
  - $K_i = 0.00$ (eliminates windup)
  - $\alpha = 0.30$ (derivative low-pass filter coefficient)

---

### 4.2 [`source/main.c`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/main.c) — Master Controller
`main.c` executes the 50 Hz control pipeline:
1. **Camera Polling**: Queries the lookahead camera on alternating ticks (25 Hz) and the PID camera on every tick (50 Hz).
2. **State Machine Transitions**:
   - Evaluates line loss: If `sr.found == 0` for $> 5$ ticks (`LINE_LOST_LIMIT`), transitions to `DRIVE_BRAKING`.
   - Evaluates corner entry: If $|heading| \ge 14.0^\circ$, transitions to `DRIVE_TURNING` with immediate 1-tick responsiveness.
   - Evaluates corner exit: Requires 5 consecutive ticks (`STATE_EXIT_STREAK`) of low heading to return to `DRIVE_FORWARD`, preventing state chatter.
3. **Sensor Fusion (2-DOF Steering)**:
   - Evaluates feedback steering from top camera: $\delta_{\text{feedback}} = -\text{PID}(e)$.
   - Evaluates feedforward trajectory from front camera:
     $$\delta_{\text{trajectory}} = \text{traj\_blend} \cdot (0.45 \cdot \theta_{\text{traj}} + 0.20 \cdot \Delta x_{\text{traj}})$$
     where $\text{traj\_blend} = 0.35$ on straights and $0.70$ in corners.
   - Total Command: $\delta_{\text{cmd}} = \delta_{\text{feedback}} + \delta_{\text{trajectory}}$.
4. **Predictive PID Gain Pre-Arming**:
   $$\text{raw\_corner\_urgency} = \max(u_{\text{heading}}, u_{\text{cte}}, \text{lookahead\_turn\_factor})$$
5. **Asymmetric Differential Torque Vectoring**: Modulates left and right wheel speeds independently based on steering direction, longitudinal pre-braking, and bilateral acceleration limiters.

---

### 4.3 [`source/vectors.c`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/vectors.c) — Vision Processing & Trajectory Planning

#### Top Camera Pipeline
- **`vector_filtering()`**:
  - Pre-pass: Rejects horizontal vectors ($|dx| > 0.90 \cdot |dy|$).
  - Combinatorial evaluation: Evaluates all pairs for lane width ($[40, 68]\,\text{px}$), heading divergence ($[-10^\circ, +62^\circ]$), and connection proximity ($15\,\text{px}$).
  - Selects the optimal pair based on bumper proximity and target width fit.
  - Fallback to single boundary: Uses temporal boundary memory (`s_last_left_near_x`, `s_last_right_near_x`) and penalizes deviations $> 18^\circ$ from `s_last_heading` to prevent peeling off at intersections.
- **`compute_steering_error()`**:
  - Reconstructs immediate Cross-Track Error ($CTE = x_{\text{mid}} - 39.5$).
  - Reconstructs heading angle ($\psi = \arctan(dx / dy)$).

#### Front Camera Pipeline (`compute_lookahead_trajectory()`)
- Extracts upcoming path heading $\theta_{\text{traj}}$ and lateral offset $\Delta x_{\text{traj}}$.
- **Apex-Cut Racing Line Bias**:
  When path curves ($|\Delta x| > 1.5\,\text{px}$), shifts the far midpoint inside by $20\%$ of the half lane-width:
  $$\text{mid\_far\_x} \leftarrow \text{mid\_far\_x} + \text{sgn}(\Delta x) \cdot \text{lane\_half} \cdot 0.20$$
- **Deadbanded Lookahead Urgency**:
  Applies a $5.0^\circ$ deadband on trajectory angle and a $14.0\,\text{px}$ deadband on offset to guarantee zero pre-braking on straights while reacting decisively to upcoming turns.

---

### 4.4 [`source/control_theory.c`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/control_theory.c) — Industrial Filtered PID
- **Derivative Calculation**: Calculated strictly on measurement ($-\frac{\Delta y}{\Delta t}$) rather than error ($-\frac{\Delta e}{\Delta t}$), eliminating derivative kicks on setpoint shifts.
- **Low-Pass Derivative Filtering**:
  $$D_{\text{filt}}[k] = 0.30 \cdot D_{\text{raw}}[k] + 0.70 \cdot D_{\text{filt}}[k-1]$$
- **Anti-Windup Protection**: Integrator updates are conditionally gated when output is saturated.

---

### 4.5 [`source/pixy.c`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/pixy.c) — Asynchronous eDMA Driver
- Interfaces Pixy2 cameras over I2C using `LPI2C_MasterTransferEDMA`.
- Encapsulates per-instance state: `instance`, `address`, `transferDone`, `transferStatus`, `edmaHandle`.
- Features hardware watchdog timeout (`PIXY_TIMEOUT_CYCLES = 5,000,000`) that triggers `LPI2C_MasterTransferAbortEDMA()` if hardware lines stall, guaranteeing real-time safety.

---

### 4.6 [`source/wifi_config.c`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/wifi_config.c) — Dynamic Tuning Shell
- Provides real-time parameter tuning over LPUART without recompiling:
  - `kp_straight <val>;`, `kp_corner <val>;`
  - `kd_straight <val>;`, `kd_corner <val>;`
  - `speed_straight <val>;`, `speed_out <val>;`, `speed_in <val>;`
  - `accel_ramp <val>;`, `decel_ramp <val>;`
  - `status;` (prints full parameter state)
- Telemetry output throttled to 12.5 Hz to prevent ESP32 Wi-Fi buffer congestion.

---

## 5. Control Dynamics & Mathematical Formulations

```
   [ Top Camera ]                                                  [ Front Camera ]
         |                                                                |
         v                                                                v
   CTE & Heading                                                 Apex-Biased Trajectory
         |                                                                |
         +------------------------------+---------------------------------+
                                        |
                                        v
                               Steering Command
                                        |
                +-----------------------+-----------------------+
                |                                               |
                v                                               v
        Servo Steer Angle                              Differential Drag Delta
                |                                               |
                v                                               v
       Ackermann Geometry                              Rear Wheel Torque Bias
```

### 5.1 Error Formulation
$$\text{combined\_error}(t) = 0.60 \cdot \text{CTE}_{\text{filt}}(t) + 1.00 \cdot \psi_{\text{filt}}(t)$$

### 5.2 Dynamic Gain Scheduling
$$u = \max\left(\frac{|\psi_{\text{filt}}|}{22.0^\circ}, \; \frac{|\text{CTE}_{\text{filt}}| - 14.0}{10.0}, \; \text{lookahead\_turn\_factor}\right)$$
$$K_p(u) = 0.70 + u \cdot (2.30 - 0.70)$$
$$K_d(u) = 0.160 + u \cdot (0.130 - 0.160)$$

### 5.3 Asymmetric Differential Drag & Torque Vectoring
$$V_{\text{base}} = 95.0\% - u_{\text{effective}} \cdot (95.0\% - 75.0\%)$$
$$\delta_{\text{norm}} = \text{clamp}\left(\frac{|\delta_{\text{cmd}}| - 6.0^\circ}{30.0^\circ - 6.0^\circ}, \; 0.0, \; 1.0\right)$$
$$\Delta V_{\text{diff}} = \delta_{\text{norm}} \cdot (V_{\text{base}} - 48.0\%)$$

- **Right Turn ($\delta_{\text{cmd}} > 0$)**:
  $$V_{\text{left}} = V_{\text{base}} \quad (\text{outside pusher})$$
  $$V_{\text{right}} = V_{\text{base}} - \Delta V_{\text{diff}} \quad (\text{inside pivot})$$
- **Left Turn ($\delta_{\text{cmd}} < 0$)**:
  $$V_{\text{left}} = V_{\text{base}} - \Delta V_{\text{diff}} \quad (\text{inside pivot})$$
  $$V_{\text{right}} = V_{\text{base}} \quad (\text{outside pusher})$$

---

## 6. Telemetry Dictionary & Diagnostics Reference

Telemetry format broadcast at 12.5 Hz over UART/Wi-Fi:
```
L:%u/%u R:%d F:%d H:%d TJ:%d S:%d LA:%d ST:%s VL:%d VR:%d
```

| Field | Description | Nominal Straight | Nominal Corner | Diagnostic Warning Condition |
|---|---|---|---|---|
| `L` | Line counts (`PID/Front`) | `2/1` or `2/2` | `2/2` or `1/1` | `0/x` (Top cam blinded), `x/0` (Front cam blinded) |
| `R` | Raw CTE ($\times 10$ px) | $\approx 0 \pm 10$ | $\pm 10 \dots \pm 150$ | Large sustained deviation on straight |
| `F` | Filtered CTE ($\times 10$ px) | $\approx 0 \pm 8$ | $\pm 10 \dots \pm 120$ | Disagreeing with `R` for $> 10$ ticks |
| `H` | Filtered Heading ($\times 10^\circ$) | $\approx 0 \pm 15$ | $\pm 140 \dots \pm 250$ | Oscillating wildly on straights (weave) |
| `TJ` | Trajectory Angle ($\times 10^\circ$) | $\approx 0 \pm 20$ | $\pm 150 \dots \pm 350$ | Persistently $> 100$ on straights (cam tilted) |
| `S` | Steer Command ($\times 10^\circ$) | $\approx 0 \pm 25$ | $\pm 200 \dots \pm 650$ | Pegged at limit without turn (servo binding) |
| `LA` | Lookahead Factor ($0 \dots 100\%$) | $0\%$ | $50\% \dots 100\%$ | Non-zero on straight (causes unwanted pre-braking) |
| `ST` | State Machine Mode | `ST:F` | `ST:T` | `ST:B` (Line lost / Emergency braking) |
| `VL` | Left Motor Speed (%) | $95\%$ | $48\% \dots 75\%$ | Asymmetric duty on straights |
| `VR` | Right Motor Speed (%) | $95\%$ | $48\% \dots 75\%$ | Asymmetric duty on straights |

---

## 7. Build and Verification Procedures

To compile the complete firmware under Windows PowerShell:
```powershell
$env:PATH = "C:\Users\rober\.mcuxpressotools\arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi\bin;" + $env:PATH
make -C Debug all
```

### Memory Footprint
- **PROGRAM_FLASH0**: 83,308 Bytes / 1,048,576 Bytes (**7.94%**)
- **SRAM**: 11,648 Bytes / 393,216 Bytes (**2.96%**)
- **Compilation Diagnostics**: `0 errors, 0 warnings`.
