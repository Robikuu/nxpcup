# Dual-Camera Blended Steering & Shadow Mode (Phase 1A & 1B) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement continuous dual-camera steering fusion with self-healing front plausibility, collision-free telemetry, log-only intersection detection, and zero-risk Shadow Mode validation, followed by active blend activation.

**Architecture:** 
- Phase 1A: Implement front-camera steering synthesis ($\delta_{\text{front}}$), self-healing temporal plausibility, collision-free telemetry (`TS:`, `FS:`, `BW:`, `DOM:`, `FV:`, `IX:`), and log-only intersection detection in Shadow Mode (`shadow_mode_enable = 1`), verified via a push-by-hand track run.
- Phase 1B: Enable active asymmetric blending ($\alpha$), full Top PID reseed on recovery, output servo slew limiting ($\le 12^\circ/\text{tick}$), confidence speed caps, and dual-blind `STATE_LOST_LINE`.

**Tech Stack:** Embedded C (ARM Cortex-M33 / NXP FRDM-MCXN947), MCUXpresso IDE / GCC toolchain, Python simulation and telemetry analysis harness.

---

### Task 1: Telemetry Stream Expansion & Config Parameters (Phase 1A)

**Files:**
- Modify: `include/Config.h`
- Modify: `source/wifi_config.h`
- Modify: `source/wifi_config.c`
- Test: `scratch/test_telemetry_keys.py`

- [ ] **Step 1: Write test for collision-free telemetry parser**

Create `scratch/test_telemetry_keys.py` to verify the new format:
```
L:2/3 R:-10 F:-11 H:6 TS:-15 FS:-12 S:-14 BW:85 DOM:T FV:1 IX:0 ST:N VL:88 VR:88
```
Verify that `TS:` (top steer) and `ST:` (vehicle state) parse without key collision, and all new metrics (`FS:`, `BW:`, `DOM:`, `FV:`, `IX:`) are parsed accurately.

- [ ] **Step 2: Run test to verify failure**

```powershell
python scratch/test_telemetry_keys.py
```
Expected: FAIL (keys missing or colliding with old format).

- [ ] **Step 3: Update `include/Config.h`, `source/wifi_config.h`, and `source/wifi_config.c`**

1. Add `shadow_mode_enable` to `control_params_t` in `include/Config.h`:
   ```c
   uint8_t shadow_mode_enable; /* 1 = shadow mode (steer on top, log front), 0 = active blend */
   ```
   Set default: `g_params.shadow_mode_enable = 1U;` in initialization.
2. In `source/wifi_config.c`, format the telemetry string:
   ```c
   snprintf(status_buf, sizeof(status_buf),
       "L:%u/%u R:%d F:%d H:%d TS:%d FS:%d S:%d BW:%d DOM:%c FV:%u IX:%u ST:%c VL:%d VR:%d\r\n",
       (unsigned)top_v, (unsigned)front_v, (int)r_cte, (int)f_cte, (int)h,
       (int)ts, (int)fs, (int)s, (int)bw, dom, (unsigned)fv, (unsigned)ix,
       st, (int)vl, (int)vr);
   ```
3. Add `shadow` serial command parser (`shadow 1`, `shadow 0`) in `source/wifi_config.c`.

- [ ] **Step 4: Run test to verify it passes**

```powershell
python scratch/test_telemetry_keys.py
```
Expected: PASS.

- [ ] **Step 5: Commit Task 1**

```bash
git add include/Config.h source/wifi_config.h source/wifi_config.c scratch/test_telemetry_keys.py
git commit -m "feat(telemetry): add collision-free keys TS/FS/BW/DOM/FV/IX and shadow_mode parameter"
```

---

### Task 2: Self-Healing Front Camera Plausibility & Steering Synthesis (Phase 1A)

**Files:**
- Modify: `source/main.c`
- Test: `scratch/test_front_plausibility.c`

- [ ] **Step 1: Write unit test for self-healing front plausibility**

Create `scratch/test_front_plausibility.c`:
1. Normal smooth cornering ($\Delta \theta = 5^\circ, 10^\circ, 20^\circ$) -> `FV = 1`.
2. Glitch jump ($\Delta \theta = 45^\circ$) -> Rejected (`FV = 0`).
3. Sustained sharp turn: 3 consecutive rejections -> Auto-reseed occurs and accepts frame (`FV = 1`).
4. Camera recovery from 0 vectors -> Immediate reseed (`FV = 1`) and `s_front_dropout_ticks` reset to 0.

- [ ] **Step 2: Run test to verify failure**

```powershell
gcc scratch/test_front_plausibility.c -o scratch/test_front_plausibility.exe
./scratch/test_front_plausibility.exe
```
Expected: FAIL / undefined symbols.

- [ ] **Step 3: Implement front plausibility and $\delta_{\text{front}}$ in `source/main.c`**

1. Implement `s_front_dropout_ticks` tracking and 3-frame auto-reseed logic.
2. Low-pass filter trajectory angle: $\theta_{\text{filtered}} = 0.40 \cdot \theta_{\text{raw}} + 0.60 \cdot \theta_{k-1}$.
3. Synthesize $\delta_{\text{front}} = -(1.00 \cdot \theta_{\text{filtered}} + 0.35 \cdot \Delta x_{\text{traj}})$.
4. Implement invalid front decay: hold for 3 ticks (60 ms), then decay $0.95^k$.
5. Implement log-only intersection detector:
   ```c
   bool is_roughly_straight = (fabsf(filtered_heading) <= 15.0f) && (fabsf(steer_output) <= 18.0f);
   bool has_transverse_tape = (g_traj.has_transverse_tape || top_has_transverse_tape);
   if (!g_intersection_active && has_transverse_tape && is_roughly_straight) {
       g_intersection_active = true;
       s_intersection_ticks = 0;
   }
   ```
6. Enforce Shadow Mode: if `g_params.shadow_mode_enable == 1`, set $\delta_{\text{raw}} = \delta_{\text{top}}$, `DOM = 'T'`, `BW = 100`.

- [ ] **Step 4: Run test to verify it passes**

```powershell
gcc scratch/test_front_plausibility.c -o scratch/test_front_plausibility.exe
./scratch/test_front_plausibility.exe
```
Expected: `ALL FRONT PLAUSIBILITY TESTS PASSED!`

- [ ] **Step 5: Commit Task 2**

```bash
git add source/main.c scratch/test_front_plausibility.c
git commit -m "feat(control): implement self-healing front plausibility filter, delta_front synthesis, and shadow mode"
```

---

### Task 3: Dual Workspace Sync & Hardware Push-by-Hand Calibration (Phase 1A Verification)

**Files:**
- Modify: `D:\robotica\workspace\nxpcup-ciorba_curata\source\*`
- Modify: `D:\robotica\workspace\nxpcup-ciorba_curata\include\*`
- Script: `scratch/analyze_shadow_calibration.py`

- [ ] **Step 1: Synchronize all modified files to MCUXpresso workspace**

```powershell
Copy-Item "d:\nxpcup\source\main.c" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\source\main.c" -Force
Copy-Item "d:\nxpcup\include\Config.h" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\include\Config.h" -Force
Copy-Item "d:\nxpcup\source\wifi_config.c" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\source\wifi_config.c" -Force
Copy-Item "d:\nxpcup\source\wifi_config.h" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\source\wifi_config.h" -Force
```

- [ ] **Step 2: Clean, build, and flash in MCUXpresso IDE**

Verify 0 errors and 0 warnings. Flash to FRDM-MCXN947 board.

- [ ] **Step 3: User executes push-by-hand track run**

User pushes vehicle around the entire track (including the inner loop and intersection) with `start;` active to capture side-by-side `TS:` and `FS:` calibration data.

- [ ] **Step 4: Run calibration comparison script**

Create and run `scratch/analyze_shadow_calibration.py` to evaluate correlation between `TS:` and `FS:` during valid dual-camera frames. Verify $K_{p,\text{front}}$ and $K_{\text{offset,front}}$ calibration before enabling active blend.

- [ ] **Step 5: Commit Task 3**

```bash
git add scratch/analyze_shadow_calibration.py
git commit -m "chore(sync): synchronize Phase 1A shadow mode to MCUXpresso workspace"
```

---

### Task 4: Active Blending, Slew Limiter, and Dual-Blind Safety (Phase 1B)

**Files:**
- Modify: `source/main.c`
- Modify: `include/Config.h`
- Test: `scratch/test_active_blend_sim.py`

- [ ] **Step 1: Write simulation test for asymmetric blend and dropout recovery**

Create `scratch/test_active_blend_sim.py`:
1. Simulate Top camera losing lines for 15 frames: verify $\alpha$ drops to 0.0 in 2 ticks (40 ms) and $\delta_{\text{cmd}}$ smoothly tracks $\delta_{\text{front}}$.
2. Simulate Top camera re-acquiring lines: verify $\alpha$ ramps up smoothly at 0.08/tick without steering step.
3. Verify output slew limiter limits rate of change to $\le 12^\circ/\text{tick}$.
4. Verify PID reseed clears derivative spike on frame of recovery.

- [ ] **Step 5: Implement active blend, PID reseed, and output slew limiter in `source/main.c`**

1. Implement asymmetric slew-limited blend weight ($\Delta_{\text{fall}} = 0.50, \Delta_{\text{rise}} = 0.08$).
2. Implement Top PID state freeze when $C_{\text{top}} == 0$ and full reseed (`prev_error = current_cte`, `cte_derivative = 0.0f`).
3. Implement servo output slew rate limiter:
   $$\Delta \delta = \text{clamp}(\delta_{\text{raw}} - \delta_{\text{cmd}, k-1}, -12.0^\circ, +12.0^\circ)$$
4. Implement confidence-based speed caps:
   - When $\alpha < 0.30$, cap target speed to $\le 52\%$.
   - When `g_intersection_active == true`, cap speed to $\le 55\%$.
5. Implement dual-blind `STATE_LOST_LINE`:
   ```c
   bool line_lost = (c_top == 0.0f) && (!front_plausible || c_front == 0.0f);
   ```
6. Set `g_params.shadow_mode_enable = 0U;` to enable live active blending.

- [ ] **Step 3: Run simulation test to verify it passes**

```powershell
python scratch/test_active_blend_sim.py
```
Expected: `ACTIVE BLEND SIMULATION TEST PASSED!`

- [ ] **Step 4: Synchronize to MCUXpresso workspace and commit**

```powershell
Copy-Item "d:\nxpcup\source\main.c" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\source\main.c" -Force
git add source/main.c include/Config.h scratch/test_active_blend_sim.py
git commit -m "feat(control): enable Phase 1B active dual-camera blending, PID reseed, and servo slew limiting"
```
