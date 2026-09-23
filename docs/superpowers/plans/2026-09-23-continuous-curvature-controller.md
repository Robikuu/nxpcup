# Continuous Curvature Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the brittle binary corner state machine in `source/main.c` with a clean, robust Three-Macro-State Architecture (`NORMAL`, `INTERSECTION`, `LINE_LOST`) featuring a Continuous Curvature Governor ($\kappa$) with noise deadbands.

**Architecture:** 
* `NORMAL` state runs a continuous mathematical speed governor based on preview curvature $\kappa$, rate-limited by a global speed slew limiter, with decoupled PD steering and dynamic differential drag. Corners are handled continuously without discrete sub-states.
* `INTERSECTION` state dead-reckons straight across crossing lines with a guaranteed 360ms timeout.
* `LINE_LOST` state executes staged recovery (coast -> dynamic brake -> emergency stop).
* Slew limiter runs globally across all state transitions to prevent motor torque jolts.

**Tech Stack:** C99, ARM Cortex-M33 (NXP MCX-N947), MCUXpresso SDK, Clang (host testing).

---

### Task 1: Update Configuration Constants

**Files:**
- Modify: `include/Config.h:40-95`
- Test: Host compiler check via `clang -Iinclude -c source/control_theory.c`

- [ ] **Step 1: Define macro state enums and continuous governor parameters in `include/Config.h`**

Define the macro states (`MACRO_STATE_NORMAL`, `MACRO_STATE_INTERSECTION`, `MACRO_STATE_LINE_LOST`), deadband thresholds, and timeout constants:

```c
// ---- Macro States ----
#define MACRO_STATE_NORMAL           0
#define MACRO_STATE_INTERSECTION     1
#define MACRO_STATE_LINE_LOST        2

// ---- Continuous Curvature Governor (Noise Deadbands & Saturation) ----
#define GOVERNOR_HEADING_DEADBAND_DEG 2.5f     // Noise floor below which base heading curvature is 0.0
#define GOVERNOR_HEADING_MAX_DEG      25.0f    // Heading angle corresponding to 100% corner intensity
#define GOVERNOR_CURVATURE_DEADBAND_PX 4.0f    // Noise floor below which front lookahead mid_dx is 0.0
#define GOVERNOR_CURVATURE_MAX_PX     14.0f    // Horizon lateral curvature displacement for 100% urgency
#define INTERSECTION_TIMEOUT_TICKS    18U      // 18 ticks (360ms) fixed crossing timeout
#define LINE_LOST_DEBOUNCE_TICKS      3U       // 3 ticks (60ms) optical dropout debounce before entering LINE_LOST
#define LINE_LOST_BRAKE_TICKS         18U      // 18 ticks (360ms) coast window before active braking
#define LINE_LOST_STOP_TICKS          50U      // 50 ticks (1000ms) before full emergency stop
#define LINE_REACQUIRE_TICKS          3U       // 3 consecutive valid frames required to return to NORMAL
```

- [ ] **Step 2: Run compiler syntax check**

Run: `clang -Iinclude -Isource -c source/control_theory.c -o scratch/test_syntax.o`  
Expected: PASS (exited with code 0).

- [ ] **Step 3: Commit**

```bash
git add include/Config.h
git commit -m "config: define macro states and continuous curvature governor constants"
```

---

### Task 2: Host Test Suite for Continuous Governor

**Files:**
- Create: `scratch/test_continuous_governor.c`
- Compile & Run: Clang host test runner

- [ ] **Step 1: Write comprehensive host test runner testing all 5 critical behaviors**

Create `scratch/test_continuous_governor.c` verifying:
1. Straightaway Deadband: $H \le 2.5^\circ$ and $|mid\_dx| \le 4.0\text{px}$ produces $\kappa = 0.0 \implies \text{Speed} = 100\%$.
2. Continuous Scaling: Sharp turn ($H = 20^\circ$ or wall) smoothly commands speed down to $58\%$.
3. Global Slew Limiting across state transitions.
4. Intersection Timeout: Enters intersection, holds straight, exits to NORMAL after exactly 18 ticks without hanging.
5. Anti-Kick Filter Reset: PID state reset on transition from `LINE_LOST` / `INTERSECTION` to `NORMAL`.

- [ ] **Step 2: Compile and run test suite**

Run: `clang -O2 -Iinclude -Isource scratch/test_continuous_governor.c source/vectors.c source/control_theory.c -o scratch/test_continuous_governor.exe && ./scratch/test_continuous_governor.exe`  
Expected: PASS with all tests passing.

- [ ] **Step 3: Commit**

```bash
git add scratch/test_continuous_governor.c
git commit -m "test: add automated test suite for continuous curvature governor"
```

---

### Task 3: Rewrite `source/main.c`

**Files:**
- Modify: `source/main.c:1-550`
- Test: Replay on `docs/debug_telemetry.md` runs and compile with Clang / ARM GCC

- [ ] **Step 1: Implement clean Three-Macro-State architecture in `source/main.c`**

Replace the 560-line binary state machine in `source/main.c` with the streamlined ~150-line macro state controller:
1. **Initialize**: Macro state = `MACRO_STATE_NORMAL`.
2. **Every 20ms Tick**:
   * Poll sensors (Base Pixy2, Front Pixy2, command buffer, push buttons).
   * Evaluate Macro State Transitions:
     * Check for intersection crossing.
     * Check for line loss debounce ($\ge 3$ ticks).
     * Check for line re-acquisition ($3$ consecutive valid frames).
   * In `MACRO_STATE_NORMAL`:
     * Compute base heading curvature $\kappa_{\text{base}}$ with $2.5^\circ$ deadband.
     * Compute front lookahead curvature $\kappa_{\text{front}}$ with $4.0\text{px}$ deadband (or $1.0$ if transverse wall).
     * $\kappa = \text{clamp}(\max(\kappa_{\text{base}}, \kappa_{\text{front}}), 0.0, 1.0)$.
     * $\text{Speed}_{\text{target}} = 100.0f - \kappa \times (100.0f - 58.0f)$.
     * Update decoupled PID steering and heading feedforward.
     * Calculate dynamic differential drag on inside wheel.
   * In `MACRO_STATE_INTERSECTION`:
     * Hold steering straight (`steer_cmd = 0.0f`).
     * Set $\text{Speed}_{\text{target}} = 70.0f$.
     * Increment `intersection_ticks`. If $\ge 18$, reset PID filters and transition to `NORMAL`.
   * In `MACRO_STATE_LINE_LOST`:
     * If ticks $\le 18$: coast straight.
     * If ticks $19 \dots 50$: brake.
     * If ticks $> 50$: emergency stop (`STATE_IDLE`).
     * If valid lines seen for 3 ticks: reset PID filters and transition to `NORMAL`.
   * **Global Slew Rate Limiter**:
     * Execute on `applied_speed_left` and `applied_speed_right` in every state.
     * Command actuators (`Steer`, `HbridgeSpeed` / `HbridgeBrake`).

- [ ] **Step 2: Verify against host comprehensive test suite**

Run: `clang -O2 -Iinclude -Isource scratch/test_continuous_governor.c source/vectors.c source/control_theory.c -o scratch/test_continuous_governor.exe && ./scratch/test_continuous_governor.exe`  
Expected: PASS with 0 failures.

- [ ] **Step 3: Commit**

```bash
git add source/main.c
git commit -m "refactor(control): rewrite main.c with three-macro-state continuous governor"
```

---

### Task 4: Target Firmware Build & MCUXpresso Verification

**Files:**
- Compile: `Debug/nxpcup.axf`
- Verify memory: Flash < 10%, SRAM < 5%
- Sync: `D:\robotica\workspace\nxpcup-ciorba_curata`

- [ ] **Step 1: Compile ARM Cortex-M33 target binary with MCUXpresso toolchain**

Run:
```powershell
$env:PATH = "C:\nxp\MCUXpressoIDE_25.6.136\ide\plugins\com.nxp.mcuxpresso.tools.win32_25.6.0.202501151204\tools\bin;" + $env:PATH
make -C Debug clean
make -C Debug all
```
Expected: `nxpcup.axf` built successfully with 0 errors.

- [ ] **Step 2: Sync updated sources and binary to MCUXpresso workspace**

Run:
```powershell
Copy-Item "include\Config.h" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\include\Config.h" -Force
Copy-Item "source\main.c" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\source\main.c" -Force
Copy-Item "Debug\nxpcup.axf" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\Debug\nxpcup.axf" -Force
```

- [ ] **Step 3: Commit**

```bash
git add Debug/ include/ source/
git commit -m "build: compile continuous curvature controller target binary"
```
