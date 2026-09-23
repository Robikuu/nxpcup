# Decoupled Dual-Camera & Lookahead Speed Governor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Decouple front lookahead camera from steering authority, upgrade camera polling to 50 Hz, implement foreground root anchor vector filtering with a 3-tick dropout hold filter and top-camera sanity guard, rationalize PID gains, and establish a comprehensive 7-scenario host test suite.

**Architecture:** The base camera retains 100% exclusive authority over the steering servo via PD and heading deadband feedforward. The front camera operates strictly as a lookahead speed governor at 50 Hz, selecting foreground root boundaries ($Y \ge 30$, excluding center dashes $X \in [35, 44]$) to compute a lookahead point at horizon $Y = 20$. A $\pm 5.0\text{px}$ deadband with hysteresis dictates pre-braking and state transitions, while a 3-tick hold-last-valid filter and base camera straightaway sanity veto ensure zero false braking on optical flickers.

**Tech Stack:** C99, ARM Cortex-M33 (NXP MCX-N947), MCUXpresso SDK, Clang/LLVM native Windows test simulator.

---

## File Structure

- **Modify**: [`include/Config.h`](file:///d:/nxpcup/include/Config.h)
  - Add root anchor, lookahead horizon, and deadband macros.
  - Rationalize PID gains ($K_p: 0.75 \to 1.60$, $K_d: 0.180$, $k_{heading}: 1.10$, $k_{lookahead}: 0.0$).
- **Modify**: [`source/vectors.h`](file:///d:/nxpcup/source/vectors.h)
  - Update `trajectory_result_t` with `lookahead_x`, `left_root_found`, `right_root_found`.
- **Modify**: [`source/vectors.c`](file:///d:/nxpcup/source/vectors.c)
  - Refactor `compute_lookahead_trajectory` to implement Foreground Root Anchor and horizon projection at $Y = 20$.
- **Modify**: [`source/main.c`](file:///d:/nxpcup/source/main.c)
  - Poll front camera at 50 Hz every tick.
  - Implement 3-tick hold-last-valid filter and top camera sanity veto.
  - Remove lookahead steering feedforward ($FF_{LA} = 0$).
  - Clean up speed profiling and hysteresis state transitions.
- **Modify**: [`scratch/test_comprehensive_suite.c`](file:///d:/nxpcup/scratch/test_comprehensive_suite.c)
  - Implement the 7 comprehensive simulation test scenarios.

---

### Task 1: Update Configuration & Geometry Constants in `include/Config.h`

**Files:**
- Modify: `d:/nxpcup/include/Config.h`

- [ ] **Step 1: Edit `include/Config.h` with new lookahead geometry and rationalized gains**

Replace existing lookahead, urgency, and PID macros in [`include/Config.h`](file:///d:/nxpcup/include/Config.h) with:
```c
// ---- Forward Camera Lookahead Horizon & Root Anchor Geometry ----
#define FORWARD_CAM_HORIZON_Y_MIN    13U       // Discards upper 25% of forward camera view (walls/lights)
#define FORWARD_CAM_ROOT_ANCHOR_Y_MIN 30U      // Vector base must anchor in foreground bumper zone (Y >= 30)
#define FORWARD_CAM_SAMPLE_HORIZON_Y 20.0f     // Lookahead horizon sample line (~1.5m ahead)
#define FORWARD_CAM_LEFT_MAX_X       34.0f     // Left root vector base must lie in X <= 34.0
#define FORWARD_CAM_RIGHT_MIN_X      45.0f     // Right root vector base must lie in X >= 45.0
#define LOOKAHEAD_DEADBAND_PX        5.0f      // Neutral straightaway margin from center (+/- 5.0px)
#define LOOKAHEAD_HYSTERESIS_ENTER_PX 6.0f     // Lookahead offset threshold to trigger DRIVE_TURNING
#define LOOKAHEAD_HYSTERESIS_EXIT_PX  4.0f     // Lookahead offset threshold to return to DRIVE_FORWARD
#define LOOKAHEAD_DROPOUT_HOLD_TICKS 3U        // 3 ticks (60ms) hold-last-valid window for optical flicker

// ---- Dynamic PID Gain Scheduling (Rationalized Decoupled Architecture) ----
#define KP_STRAIGHT                  0.75f     // Damped straight-line centering (no twitch)
#define KP_CORNER                    1.60f     // Crisp cornering authority (no oversteer snap)
#define KD_STRAIGHT                  0.180f    // Lateral velocity damping
#define KD_CORNER                    0.180f    // Lateral velocity damping
#define DEFAULT_KI                   0.0f
#define DEFAULT_ALPHA                0.25f     // Low-pass filter coefficient on derivative term
#define MAX_STEER_RATE_PER_SEC       650.0f    // Max steering slew rate (deg/s) -> 13.0 deg/tick

// ---- Error Weights & Decoupled Feedforward ----
#define DEFAULT_K_CTE                0.60f     // Calibrated CTE weight
#define DEFAULT_K_HEADING            1.10f     // Direct base camera heading feedforward into curves
#define DEFAULT_K_LOOKAHEAD          0.00f     // STRICTLY 0: Front camera never steers the servo
#define HEADING_DEADBAND_DEG         4.0f      // Heading deadband below which base feedforward is 0.0

// ---- State Machine Thresholds ----
#define CORNER_HEADING_THRESHOLD_DEG 8.0f      // Base camera heading threshold for corner confirmation
#define STATE_ENTER_STREAK           2         // 2 ticks (40ms) sustained confirmation
#define STATE_EXIT_STREAK            2         // 2 ticks (40ms) before returning to DRIVE_FORWARD
#define LINE_LOST_LIMIT              18U       // ~360ms coast window for traversing intersection gaps
#define LINE_LOST_BRAKE_TICKS        3         // Debounce frames before cutting power
```

- [ ] **Step 2: Commit Config changes**

```bash
git add include/Config.h
git commit -m "feat(config): add root anchor lookahead constants and rationalized gains"
```

---

### Task 2: Implement Foreground Root Vector Filtering & Lookahead Sampling in `source/vectors.*`

**Files:**
- Modify: `d:/nxpcup/source/vectors.h`
- Modify: `d:/nxpcup/source/vectors.c`

- [ ] **Step 1: Update `trajectory_result_t` struct in `source/vectors.h`**

In [`source/vectors.h`](file:///d:/nxpcup/source/vectors.h):
```c
typedef struct {
    float   trajectory_angle_deg;  /* Ahead path angle relative to vehicle axis (-35 to +35 deg) */
    float   trajectory_offset_px;  /* Ahead path lateral displacement from center line (Y=20) */
    float   lookahead_x;           /* Projected centerline X coordinate at horizon Y=20 */
    float   curvature;             /* Path curvature indicator */
    float   lookahead_urgency;     /* 0.0 to 1.0 urgency for pre-braking */
    uint8_t left_root_found;       /* 1 if left foreground root vector detected */
    uint8_t right_root_found;      /* 1 if right foreground root vector detected */
    uint8_t found;                 /* Total root boundary vectors found (0, 1, or 2) */
} trajectory_result_t;
```

- [ ] **Step 2: Refactor `compute_lookahead_trajectory` in `source/vectors.c`**

Replace the implementation of `compute_lookahead_trajectory` with the Foreground Root Anchor algorithm:
1. Orient all candidate vectors with $y_n \ge y_f$.
2. Discard if $y_n < 13$ and $y_f < 13$ (horizon ceiling).
3. Discard if $y_n - y_f < 6$ (horizontal cross lines).
4. Clip far tip to $y=13$ if it enters the ceiling.
5. Identify **Left Root**: $x_n \le 34.0$ and $y_n \ge 30$, select maximum $y_n$ and maximum length.
6. Identify **Right Root**: $x_n \ge 45.0$ and $y_n \ge 30$, select maximum $y_n$ and maximum length.
7. If both found: project to $Y = 20.0$, compute $X_{lookahead} = (X_{L,20} + X_{R,20}) / 2.0f$, offset $= |X_{lookahead} - 39.5|$. Apply $\pm 5.0\text{px}$ deadband to compute urgency.
8. If only one found: set `left_root_found` / `right_root_found` accordingly; project the single line to $Y=20$ and compute synthesized offset.

- [ ] **Step 3: Update `reset_steering_tracking` to initialize lookahead memory cleanly**

Ensure `s_last_lookahead_left_x` and `s_last_lookahead_right_x` are initialized to $39.5 - 25.0$ and $39.5 + 25.0$.

- [ ] **Step 4: Commit vectors changes**

```bash
git add source/vectors.h source/vectors.c
git commit -m "feat(vision): implement foreground root anchor and lookahead horizon sampling"
```

---

### Task 3: Refactor 50 Hz Control Loop, Dropout Hold Filter, Sanity Veto, and Steering Decoupling in `source/main.c`

**Files:**
- Modify: `d:/nxpcup/source/main.c`

- [ ] **Step 1: Remove the alternating divider and poll front camera every tick (50 Hz)**

In [`source/main.c`](file:///d:/nxpcup/source/main.c):
Change:
```c
if ((front_cam_phase++ & 1) == 0) { ... }
```
To direct 50 Hz polling:
```c
/* Trajectory Camera polling (LP_FLEXCOMM2 - 50 Hz every tick) */
size_t num_vectors_front = 0;
status_t s_front = pixy_get_vectors(&cam_front, vectors_front, MAX_VECTORS, &num_vectors_front);
if (s_front == kStatus_Success && num_vectors_front > 0) {
    g_traj = compute_lookahead_trajectory(vectors_front, num_vectors_front);
} else {
    memset(&g_traj, 0, sizeof(g_traj));
}
```

- [ ] **Step 2: Add Hold-Last-Valid Dropout Filter & Top-Camera Sanity Guard**

Maintain static variables in `main.c`:
* `static float s_last_valid_lookahead_x = PIXY_CENTER_X;`
* `static uint32_t s_front_dropout_ticks = 0;`

Logic:
* If `g_traj.found == 2`:
  * `s_last_valid_lookahead_x = g_traj.lookahead_x;`
  * `s_front_dropout_ticks = 0;`
  * `lookahead_turn_factor = g_traj.lookahead_urgency;`
* If `g_traj.found < 2`:
  * `s_front_dropout_ticks++;`
  * If `s_front_dropout_ticks <= LOOKAHEAD_DROPOUT_HOLD_TICKS`:
    * Coast on `s_last_valid_lookahead_x`. Do not change urgency.
  * If `s_front_dropout_ticks > LOOKAHEAD_DROPOUT_HOLD_TICKS`:
    * Top Cam Sanity Check:
      `if (fabsf(filtered_heading) <= 3.0f && fabsf(filtered_cte) <= 4.0f)` $\implies$ Veto dropout, `lookahead_turn_factor = 0.0f`.
      `else` $\implies$ Confirm corner, `lookahead_turn_factor = 0.60f`.

- [ ] **Step 3: Strictly Decouple Steering Output**

In [`source/main.c`](file:///d:/nxpcup/source/main.c):
```c
/* Decoupled Steering Authority:
 * Base camera CTE is actively damped by PID.
 * Base camera heading provides direct deadband feedforward.
 * Lookahead feedforward is STRICTLY ZERO (decoupled). */
float raw_steer = pid_cte + heading_feedforward;
```
Remove any addition of `lookahead_feedforward`.

- [ ] **Step 4: Update State Machine with Lookahead Hysteresis**

Corner trigger condition:
```c
float la_offset = fabsf(s_last_valid_lookahead_x - (float)PIXY_CENTER_X);
bool corner_trigger = (fabsf(filtered_heading) >= (float)CORNER_HEADING_THRESHOLD_DEG) ||
                      (la_offset >= (float)LOOKAHEAD_HYSTERESIS_ENTER_PX && s_front_dropout_ticks == 0) ||
                      (s_front_dropout_ticks > LOOKAHEAD_DROPOUT_HOLD_TICKS && !(fabsf(filtered_heading) <= 3.0f && fabsf(filtered_cte) <= 4.0f));

if (drive_state == DRIVE_TURNING) {
    bool straight_confirmed = (fabsf(filtered_heading) <= 4.0f && fabsf(filtered_cte) <= 6.0f && la_offset <= (float)LOOKAHEAD_HYSTERESIS_EXIT_PX);
    desired_state = straight_confirmed ? DRIVE_FORWARD : DRIVE_TURNING;
} else {
    desired_state = corner_trigger ? DRIVE_TURNING : DRIVE_FORWARD;
}
```

- [ ] **Step 5: Commit main.c changes**

```bash
git add source/main.c
git commit -m "feat(control): decouple front cam steering, add 50Hz polling and dropout sanity filter"
```

---

### Task 4: Rewrite Comprehensive Host Simulation Test Suite in `scratch/test_comprehensive_suite.c`

**Files:**
- Modify: `d:/nxpcup/scratch/test_comprehensive_suite.c`

- [ ] **Step 1: Write all 7 test cases according to spec §6**

Include:
* **Test 1**: Camera Decoupling & Straight Sprint Retention (lookahead sees curve, steer == 0.0°, speed == 100%).
* **Test 2**: 1–3 Frame Optical Flicker Immunity (flicker dropout leaves speed == 100%, state == FORWARD).
* **Test 3**: Top-Camera Sanity Veto on Extended Dropouts (6 frames dropout with straight base camera vetoes braking).
* **Test 4**: False-Pair & Diverging Branch Rejection (user's sketch: 4 vectors including center dash & upper branch).
* **Test 5**: Real Corner Pre-Braking & 50 Hz Fast Transition ($\le 2$ ticks / 40ms to pre-brake).
* **Test 6**: Chicane Reversal & Slew Limiter Anti-Windup ($\le 13.0^\circ/\text{tick}$).
* **Test 7**: Numerical Fuzzing Immunity (NaN, Inf, extreme inputs clamped).

- [ ] **Step 2: Compile and run test suite with Clang**

```powershell
& "C:\Program Files (x86)\clang+llvm-22.1.6-x86_64-pc-windows-msvc\bin\clang.exe" -O2 -I include -I source scratch/test_comprehensive_suite.c source/control_theory.c source/vectors.c -o scratch/test_suite.exe; & "d:\nxpcup\scratch\test_suite.exe"
```
Expected: All 7 test suites pass.

- [ ] **Step 3: Commit test suite changes**

```bash
git add scratch/test_comprehensive_suite.c
git commit -m "test(control): implement 7-suite benchmark for decoupled lookahead and dropout filter"
```

---

### Task 5: Build ARM Target ELF & Synchronize Workspaces

**Files:**
- Build: `Debug/nxpcup.axf`
- Sync to: `D:/robotica/workspace/nxpcup-ciorba_curata`

- [ ] **Step 1: Compile ARM Cortex-M33 target binary with GNU Toolchain**

```powershell
$env:PATH = "C:\Users\rober\.mcuxpressotools\arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi\bin;C:\ProgramData\chocolatey\bin;" + $env:PATH
make -C Debug clean
make -C Debug all
```
Expected: Exit code 0, `Debug/nxpcup.axf` generated without errors.

- [ ] **Step 2: Sync modified files to `D:/robotica/workspace/nxpcup-ciorba_curata`**

Copy updated `include/Config.h`, `source/vectors.h`, `source/vectors.c`, `source/main.c`, `scratch/test_comprehensive_suite.c`, and `docs/superpowers/plans/2026-09-23-decoupled-front-camera-lookahead.md` to the MCUXpresso IDE workspace.

- [ ] **Step 3: Final Commit and Verification**

```bash
git status
git commit -a -m "feat(release): deploy decoupled 50Hz dual-camera lookahead architecture"
```
