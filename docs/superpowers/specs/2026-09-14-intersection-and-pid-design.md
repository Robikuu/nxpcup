# Intersection Detection & High-Speed PID Refinement Design

**Date:** 2026-09-14  
**Project:** NXP Cup MCXN947 Autonomous Racer  
**Target:** 12.2V Battery Pack Supply (~11.5V sprint throttle)

---

## 1. Problem Statement

1. **Intersection Branch Trap**:
   - Approaching a 4-way/T-junction crossing immediately following a curved section, the right lane boundary splits $90^\circ$ to the East.
   - The camera detects this closer branch vector (`near_y` lower in the image). The current vector pairing and fallback logic in `vectors.c` picks this branch instead of the distant through-line going straight North into the loop, causing the vehicle to turn right and skip the loop.
2. **Straightaway Oscillation**:
   - At $94\%$ throttle (~$11.5\text{V}$), $K_{p,\text{straight}} = 0.25$ is too sensitive to camera pixel jitter, causing high-speed hunting/weaving.
3. **Corner Understeer**:
   - Corners preceding the intersection require earlier and firmer steering engagement to prevent running wide before crossing.

---

## 2. Architecture & Components

### A. Raw Vector Intersection Detection (`vectors.c`)
Detection runs directly on raw vectors **before** `vector_filtering()` collapses them into 1–2 candidates:
- **Criterion 1 (Geometric Segment Crossing)**:
  - Any pair of raw line segments that strictly intersect in 2D space:
    $$d_1 = \text{cross2}(B_0, B_1, A_0), \quad d_2 = \text{cross2}(B_0, B_1, A_1)$$
    $$d_3 = \text{cross2}(A_0, A_1, B_0), \quad d_4 = \text{cross2}(A_0, A_1, B_1)$$
    $$\text{crosses} = ((d_1 > 0 \land d_2 < 0) \lor (d_1 < 0 \land d_2 > 0)) \land ((d_3 > 0 \land d_4 < 0) \lor (d_3 < 0 \land d_4 > 0))$$
  - A clean track never has intersecting boundaries. Crossings or T-junctions do.
- **Criterion 2 (Wide Aperture Near-$X$ Spread)**:
  - The horizontal spread between the leftmost and rightmost near-vertical vector near-$X$ points:
    $$\Delta X_{\text{near}} = \max(near\_x) - \min(near\_x) > \text{INTERSECTION\_WIDTH\_PX} \ (70.0\text{ px})$$
  - Normal track width is $\sim 50\text{ px}$. An opening branch extends the aperture to $70 - 80\text{ px}$.
- **Criterion 3 (Vector Count Gating)**:
  - Clean tracks have 1–2 vectors. Center dashed lines or fragmented curves have 3 vectors.
  - To prevent false triggers from center dashes, `raw_count >= 3` must be combined with Criterion 1 or 2, OR require `raw_count >= 4`.

```c
typedef struct {
    uint8_t  is_intersection;
    uint8_t  reason; // Bitmask: 0x01=COUNT, 0x02=CROSSING, 0x04=WIDTH
    size_t   raw_count;
} intersection_result_t;
```

### B. State Machine Integration (`main.c`)
- Add `DRIVE_INTERSECTION` to `drive_state_t`.
- When `detect_intersection()` flags an intersection and `drive_state != DRIVE_INTERSECTION`:
  - Transition to `DRIVE_INTERSECTION`.
  - Set `intersection_hold_ticks = INTERSECTION_HOLD_TICKS` (10 ticks = $200\text{ ms}$ @ $50\text{Hz}$).
- While in `DRIVE_INTERSECTION`:
  - **Drive Straight Ahead**: Steer directly forward using center trim:
    `Steer(g_params.steering_offset);`
    *(Crucial: do not freeze positive curve yaw, which would turn the car into the branch).*
  - Maintain through-lane motor duty:
    `HbridgeSpeed(&g_hbridge, (int16_t)applied_speed_left, (int16_t)applied_speed_right);`
  - Decrement `intersection_hold_ticks`.
  - When timer expires: return to `DRIVE_FORWARD`, call `reset_pid_state(&steer_pid)` to clear integrator and prevent derivative kick upon re-acquiring loop lines.
  - `continue;` skips normal vector pairing and PID update during crossing.

### C. PID & Lookahead Calibrations (`Config.h`)
- **Straightaway Calm**:
  - Soften $K_{p,\text{straight}}$ to `0.15f` (from 0.25f).
  - Derivative damping $K_{d,\text{straight}} = 0.035f$.
  - Straightaway deadband: ignore CTE errors under $\pm 1.5\text{ px}$.
- **Corner Turn Authority**:
  - $K_{p,\text{corner}} = 3.20f$ (from 2.90f).
  - $K_{d,\text{corner}} = 0.075f$.
  - Lower `STEER_FULL_BRAKE_DEG` from $25.0^\circ$ to $18.0^\circ$ so inside-wheel pivot drag and corner authority engage earlier on entry.
  - $K_{\text{heading}} = 0.48f$ (prevents pre-turning on straights).

---

## 3. Interfaces & Files Changed

1. [`include/Config.h`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/include/Config.h):
   - Add `INTERSECTION_MIN_VECTORS` (3), `INTERSECTION_WIDTH_PX` (70.0f), `INTERSECTION_HOLD_TICKS` (10).
   - Update `KP_STRAIGHT` (0.15f), `KD_STRAIGHT` (0.035f), `KP_CORNER` (3.20f), `STEER_FULL_BRAKE_DEG` (18.0f).
2. [`source/vectors.h`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/vectors.h):
   - Declare `intersection_result_t` and `detect_intersection()`.
3. [`source/vectors.c`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/vectors.c):
   - Implement `detect_intersection()` with `segments_intersect()` and width spread checks.
4. [`source/control_theory.h`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/control_theory.h):
   - Add `reset_pid_state(pid_struct *pid)` helper function.
5. [`source/main.c`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/main.c):
   - Add `DRIVE_INTERSECTION` state and hold handling.

---

## 4. Verification Plan

1. **Unit Tests**:
   - `test_intersection_detection.c`:
     - Test clean 2-vector straight $\implies$ `is_intersection == 0`.
     - Test 3-vector straight with center dash (separation 25px) $\implies$ `is_intersection == 0`.
     - Test perpendicular crossing lines $\implies$ `is_intersection == 1` (`REASON_CROSSING`).
     - Test branching line with 75px spread $\implies$ `is_intersection == 1` (`REASON_WIDTH`).
2. **Control Dynamics Test**:
   - Verify straightaway calm (1px error commands $< 0.20^\circ$).
   - Verify corner bite (20-unit error commands $> 55^\circ$).
3. **Firmware Build**:
   - Clean compilation and linking with `arm-none-eabi-gcc 14.2` (`0 errors, 0 warnings`).
