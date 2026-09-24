# Dual-Camera Lookahead Pre-Braking & Speed Governor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enable the 1-meter forward camera to detect upcoming curves across its full preview window ($Y \in [13, 45]$) without bumper-anchor restrictions, feeding a predictive curvature metric ($\kappa$) into the continuous speed governor so the car decelerates from 100% to 58% on the straightaway before tires enter the corner.

**Architecture:** Option B: Strict Separation of Concerns. Top camera (~25cm) retains 100% ownership of lateral steering control (PID CTE + heading feedforward). Front camera (~1m) retains 100% ownership of longitudinal predictive speed control via continuous curvature governor ($\kappa \to v_{\text{target}}$).

**Tech Stack:** Embedded C (ARM Cortex-M33 / NXP FRDM-MCXN947), MCUXpresso IDE / GCC toolchain, Python verification test harness.

---

### Task 1: Relax Horizon Filter & Implement Extended Preview Curvature

**Files:**
- Modify: `include/Config.h:53-66`
- Modify: `source/vectors.h:45-75`
- Modify: `source/vectors.c:388-570`
- Test: `scratch/test_preview_curvature.c`

- [ ] **Step 1: Write unit test for extended preview curvature**

Create `scratch/test_preview_curvature.c` to test the new curvature estimator on:
1. Far curve at 1.0m ($Y \in [18, 32]$): must return high curvature ($\kappa \ge 0.70$) without requiring bumper-anchor points ($Y \ge 30$).
2. Straightaway vectors: must return neutral curvature ($\kappa \le 0.15$).
3. Transverse crossing tape (90° corner wall ahead): must return full curvature ($\kappa = 1.0$).

```c
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#define MAX_FILTER_VECTORS 16
#define PIXY_CENTER_X 39.5f
#define FORWARD_CAM_HORIZON_Y_MIN 13U
#define FORWARD_CAM_SAMPLE_HORIZON_Y 22.0f
#define LOOKAHEAD_DEADBAND_PX 3.5f
#define LOOKAHEAD_FULL_URGENCY_PX 12.0f

typedef struct {
    float trajectory_angle_deg;
    float trajectory_offset_px;
    float lookahead_x;
    float curvature;
    float lookahead_urgency;
    uint8_t left_root_found;
    uint8_t right_root_found;
    uint8_t found;
} trajectory_result_t;

// Prototype
trajectory_result_t compute_lookahead_curvature(const uint16_t *vectors, size_t num_vectors);

int main(void) {
    printf("Running preview curvature tests...\n");

    // Test 1: Curve ahead in mid-far horizon (Y between 15 and 28, NO vectors at Y >= 30)
    // Left boundary: (15, 28) -> (10, 16)
    // Right boundary: (45, 28) -> (28, 16) -- strongly curving LEFT
    uint16_t curve_left[8] = {15, 28, 10, 16,  45, 28, 28, 16};
    trajectory_result_t r1 = compute_lookahead_curvature(curve_left, 2);
    printf("Test 1 (Far Curve Left): urgency=%.2f, angle=%.1f deg, found=%d\n",
           r1.lookahead_urgency, r1.trajectory_angle_deg, r1.found);
    assert(r1.found >= 1);
    assert(r1.lookahead_urgency > 0.60f);
    assert(r1.trajectory_angle_deg < -10.0f);

    // Test 2: Straightaway ahead (centered lines at Y=15..35)
    uint16_t straight[8] = {25, 35, 28, 15,  55, 35, 52, 15};
    trajectory_result_t r2 = compute_lookahead_curvature(straight, 2);
    printf("Test 2 (Straight): urgency=%.2f, angle=%.1f deg, found=%d\n",
           r2.lookahead_urgency, r2.trajectory_angle_deg, r2.found);
    assert(r2.lookahead_urgency < 0.20f);

    // Test 3: Transverse tape wall ahead (90 deg turn wall: dx=30, dy=4 at Y=24)
    uint16_t wall[4] = {20, 24, 52, 22};
    trajectory_result_t r3 = compute_lookahead_curvature(wall, 1);
    printf("Test 3 (Transverse Wall): urgency=%.2f, found=%d\n",
           r3.lookahead_urgency, r3.found);
    assert(r3.lookahead_urgency >= 0.95f);

    printf("ALL PREVIEW CURVATURE TESTS PASSED!\n");
    return 0;
}
```

- [ ] **Step 2: Compile and run test to verify failure**

Compile using gcc or cl:
```powershell
python -c "import subprocess; print(subprocess.run(['gcc', 'scratch/test_preview_curvature.c', '-o', 'scratch/test_preview_curvature.exe'], capture_output=True).stderr.decode())"
```
Expected: Linking error: `undefined reference to compute_lookahead_curvature`.

- [ ] **Step 3: Implement `compute_lookahead_curvature` in `source/vectors.c` and declare in `source/vectors.h`**

In `source/vectors.h`, declare:
```c
trajectory_result_t compute_lookahead_curvature(const uint16_t *vectors, size_t num_vectors);
```

In `source/vectors.c`, implement:
- Relax horizon filter to accept vectors in $Y \in [13, 45]$.
- Filter transverse wall segments ($\Delta X \ge 16\text{px}, \Delta Y \le 10\text{px}$) to trigger $\kappa = 1.0$.
- Pair left and right candidate vectors without requiring bumper contact ($Y \ge 30$).
- Calculate horizon midpoint at $Y = 22.0\text{f}$ and evaluate lateral offset curvature.

- [ ] **Step 4: Run test to verify it passes**

```powershell
gcc scratch/test_preview_curvature.c source/vectors.c -I include -I source -o scratch/test_preview_curvature.exe
./scratch/test_preview_curvature.exe
```
Expected: `ALL PREVIEW CURVATURE TESTS PASSED!`

- [ ] **Step 5: Commit Task 1**

```bash
git add include/Config.h source/vectors.h source/vectors.c scratch/test_preview_curvature.c
git commit -m "feat(vision): implement extended lookahead curvature preview without bumper anchor constraint"
```

---

### Task 2: Integrate Curvature Preview with Continuous Speed Governor

**Files:**
- Modify: `source/main.c:160-185, 280-310`
- Test: `scratch/test_speed_governor_sim.py`

- [ ] **Step 1: Write simulation test for pre-braking timing**

Create `scratch/test_speed_governor_sim.py` using telemetry log data from the user's run:
Simulate the vehicle approaching Corner 1 with the new extended preview estimator. Verify that target speed drops from 100% to 58% at least 6 ticks (120ms) before base camera heading swings into the turn.

- [ ] **Step 2: Run simulation test to verify failure on current main.c logic**

```powershell
python scratch/test_speed_governor_sim.py
```
Expected: Output showing current logic only begins deceleration at Frame 14 (too late).

- [ ] **Step 3: Update `source/main.c` control loop**

1. Replace `compute_lookahead_trajectory` call with `compute_lookahead_curvature`:
   ```c
   if (s_front == kStatus_Success && num_vectors_front > 0) {
       g_traj = compute_lookahead_curvature(vectors_front, num_vectors_front);
       s_front_dropout_ticks = 0;
       filtered_traj_angle = 0.50f * g_traj.trajectory_angle_deg + 0.50f * filtered_traj_angle;
       if (g_traj.lookahead_urgency >= lookahead_turn_factor) {
           lookahead_turn_factor = 0.70f * g_traj.lookahead_urgency + 0.30f * lookahead_turn_factor;
       } else {
           lookahead_turn_factor = 0.15f * g_traj.lookahead_urgency + 0.85f * lookahead_turn_factor;
       }
   }
   ```
2. Feed `lookahead_turn_factor` directly into `k_front` and `kappa`:
   ```c
   float k_front = lookahead_turn_factor;
   kappa = (k_base > k_front) ? k_base : k_front;
   target_speed = g_params.speed_straight - kappa * (g_params.speed_straight - g_params.speed_outside_corner);
   ```
3. Keep steering servo strictly on top camera (PID CTE + heading feedforward) with gain scheduling.

- [ ] **Step 4: Run simulation test to verify it passes**

```powershell
python scratch/test_speed_governor_sim.py
```
Expected: `PRE-BRAKING TIMING TEST PASSED (speed reaches 58% 300ms prior to apex entry)`.

- [ ] **Step 5: Commit Task 2**

```bash
git add source/main.c scratch/test_speed_governor_sim.py
git commit -m "feat(governor): integrate extended preview curvature into continuous speed governor"
```

---

### Task 3: Dual Workspace Sync, Clean Build & Bench Verification

**Files:**
- Modify: `D:\robotica\workspace\nxpcup-ciorba_curata\source\*`
- Modify: `D:\robotica\workspace\nxpcup-ciorba_curata\include\*`

- [ ] **Step 1: Synchronize all modified files to the MCUXpresso IDE workspace**

```powershell
Copy-Item "d:\nxpcup\source\vectors.c" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\source\vectors.c" -Force
Copy-Item "d:\nxpcup\source\vectors.h" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\source\vectors.h" -Force
Copy-Item "d:\nxpcup\source\main.c" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\source\main.c" -Force
Copy-Item "d:\nxpcup\include\Config.h" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\include\Config.h" -Force
Copy-Item "d:\nxpcup\source\wifi_config.c" -Destination "D:\robotica\workspace\nxpcup-ciorba_curata\source\wifi_config.c" -Force
```

- [ ] **Step 2: Verify git status and compile in MCUXpresso**

Confirm 0 compiler errors or warnings in MCUXpresso IDE.

- [ ] **Step 3: Commit Task 3**

```bash
git commit -m "chore(sync): synchronize lookahead pre-braking build to MCUXpresso workspace"
```
