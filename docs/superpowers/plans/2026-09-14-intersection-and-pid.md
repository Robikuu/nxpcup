# Intersection Detection & High-Speed PID Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement robust multi-modal raw-vector intersection detection and hold-straight state machine, while recalibrating straightaway PID ($K_p=0.15$) and corner authority ($K_p=3.20$) to prevent right-branch diversion and high-speed hunting.

**Architecture:** Detect crossings, wide branch apertures ($> 70\text{ px}$), and multi-line forks directly on raw vectors before `vector_filtering()` collapses them. Add `DRIVE_INTERSECTION` state that drives straight ahead for $200\text{ ms}$ (10 ticks) across the crossing. Soften straightaway gain to eliminate 11.5V weave while boosting corner gain and lowering differential braking threshold.

**Tech Stack:** Embedded C (C99), MCUXpresso SDK, NXP MCXN947 (ARM Cortex-M33 @ 150MHz), ARM GNU Toolchain 14.2.

---

### Task 1: Update Configuration Constants (`include/Config.h`)

**Files:**
- Modify: [`include/Config.h:20-60`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/include/Config.h#L20-L60)

- [ ] **Step 1: Edit `Config.h`**
Add intersection detection constants and adjust straight/corner PID gains:
```c
// ---- Intersection Detection ----
#define INTERSECTION_MIN_VECTORS     3        // Raw vector count threshold for multi-branch check
#define INTERSECTION_WIDTH_PX        70.0f    // Near-X spread indicating wide fork / branch (px)
#define INTERSECTION_HOLD_TICKS      10       // 200ms dead-reckoning window @ 50Hz (ticks)

// ---- Corner Urgency & Predictive Lookahead Gating ----
#define CTE_URGENCY_PX               16.0f     // |filtered_cte| (px) at which corner gains fully engage
#define CORNER_URGENCY_STREAK        2         // Consecutive ticks rising urgency signal must persist
#define STEER_FULL_BRAKE_DEG         18.0f     // Steer angle (deg) for 100% corner differential drag engagement (lowered from 25 for fast bite)
#define HEADING_FULL_BRAKE_DEG       20.0f     // Heading angle (deg) for full corner intensity pre-braking

// ---- Dynamic PID Gain Scheduling ----
#define KP_CORNER                    3.20f     // High-authority turn gain for apex carving
#define KP_STRAIGHT                  0.15f     // Softened straight tracking (suppresses 11.5V sprint weave)
#define KD_CORNER                    0.075f    // Solid apex damping
#define KD_STRAIGHT                  0.035f    // Firm straightaway damping
#define STRAIGHT_DEADBAND_PX         1.5f      // Deadband below which zero straightaway steering is commanded
```

- [ ] **Step 2: Verify compile dependencies**
Run unit test harness build to verify macros are valid C syntax.

---

### Task 2: Implement Intersection Detection in `vectors.h` and `vectors.c`

**Files:**
- Modify: [`source/vectors.h:1-25`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/vectors.h#L1-L25)
- Modify: [`source/vectors.c:1-60`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/vectors.c#L1-L60)
- Create Test: `scratch/test_intersection.c`

- [ ] **Step 1: Declare `intersection_result_t` and `detect_intersection` in `vectors.h`**
```c
#define INTERSECTION_REASON_COUNT    0x01u
#define INTERSECTION_REASON_CROSSING 0x02u
#define INTERSECTION_REASON_WIDTH    0x04u

typedef struct {
    uint8_t  is_intersection;
    uint8_t  reason;      // bitmask: 1=count, 2=crossing, 4=width
    size_t   raw_count;
} intersection_result_t;

intersection_result_t detect_intersection(const uint16_t *raw_vectors, size_t raw_num_vectors);
```

- [ ] **Step 2: Implement `segments_intersect` and `detect_intersection` in `vectors.c`**
Implement 2D cross-product segment intersection and wide aperture detection with center-dash safety gating:
```c
static float cross2(float ox, float oy, float ax, float ay, float bx, float by)
{
    return (ax - ox) * (by - oy) - (ay - oy) * (bx - ox);
}

static bool segments_intersect(float ax0, float ay0, float ax1, float ay1,
                               float bx0, float by0, float bx1, float by1)
{
    float d1 = cross2(bx0, by0, bx1, by1, ax0, ay0);
    float d2 = cross2(bx0, by0, bx1, by1, ax1, ay1);
    float d3 = cross2(ax0, ay0, ax1, ay1, bx0, by0);
    float d4 = cross2(ax0, ay0, ax1, ay1, bx1, by1);

    return (((d1 > 0.0f && d2 < 0.0f) || (d1 < 0.0f && d2 > 0.0f)) &&
            ((d3 > 0.0f && d4 < 0.0f) || (d3 < 0.0f && d4 > 0.0f)));
}

intersection_result_t detect_intersection(const uint16_t *raw_vectors, size_t raw_num_vectors)
{
    intersection_result_t result;
    result.is_intersection = 0;
    result.reason          = 0;
    result.raw_count       = raw_num_vectors;

    if (raw_num_vectors == 0) {
        return result;
    }

    if (raw_num_vectors >= (size_t)INTERSECTION_MIN_VECTORS) {
        result.reason |= INTERSECTION_REASON_COUNT;
    }

    /* Check for geometric segment crossing */
    for (size_t i = 0; i < raw_num_vectors && !(result.reason & INTERSECTION_REASON_CROSSING); i++) {
        float ax0 = (float)raw_vectors[4*i + 0], ay0 = (float)raw_vectors[4*i + 1];
        float ax1 = (float)raw_vectors[4*i + 2], ay1 = (float)raw_vectors[4*i + 3];

        for (size_t j = i + 1; j < raw_num_vectors; j++) {
            float bx0 = (float)raw_vectors[4*j + 0], by0 = (float)raw_vectors[4*j + 1];
            float bx1 = (float)raw_vectors[4*j + 2], by1 = (float)raw_vectors[4*j + 3];

            if (segments_intersect(ax0, ay0, ax1, ay1, bx0, by0, bx1, by1)) {
                result.reason |= INTERSECTION_REASON_CROSSING;
                break;
            }
        }
    }

    /* Check for wide aperture near-x spread */
    float min_near_x =  1e9f;
    float max_near_x = -1e9f;
    for (size_t i = 0; i < raw_num_vectors; i++) {
        int dx = (int)raw_vectors[4*i + 2] - (int)raw_vectors[4*i + 0];
        int dy = (int)raw_vectors[4*i + 3] - (int)raw_vectors[4*i + 1];
        if (dx < 0) dx = -dx;
        if (dy < 0) dy = -dy;
        if (dy == 0 || dx > 2 * dy) {
            continue;
        }

        uint16_t y0 = raw_vectors[4*i + 1];
        uint16_t y1 = raw_vectors[4*i + 3];
        float near_x = (float)((y0 >= y1) ? raw_vectors[4*i + 0] : raw_vectors[4*i + 2]);

        if (near_x < min_near_x) min_near_x = near_x;
        if (near_x > max_near_x) max_near_x = near_x;
    }

    if (max_near_x > min_near_x && (max_near_x - min_near_x) > (float)INTERSECTION_WIDTH_PX) {
        result.reason |= INTERSECTION_REASON_WIDTH;
    }

    /* Safe gating: crossing or wide aperture triggers intersection; count alone requires >= 4 to avoid center-dash false positive */
    if ((result.reason & (INTERSECTION_REASON_CROSSING | INTERSECTION_REASON_WIDTH)) || (raw_num_vectors >= 4)) {
        result.is_intersection = 1;
    }

    return result;
}
```

- [ ] **Step 3: Write and run unit test `scratch/test_intersection.c`**
Verify that 2-vector normal track, 3-vector center dash track, 2-vector crossing track, and 3-vector wide fork track are classified accurately.
Run: `clang -Iinclude -Isource scratch/test_intersection.c source/vectors.c -o scratch/test_intersection.exe; & scratch/test_intersection.exe`
Expected: `[PASS] All intersection test cases passed!`

---

### Task 3: Add `reset_pid_state` Helper (`source/control_theory.h`)

**Files:**
- Modify: [`source/control_theory.h:35-43`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/control_theory.h#L35-L43)

- [ ] **Step 1: Add inline helper function `reset_pid_state`**
```c
static inline void reset_pid_state(pid_struct *pid) {
    pid->integral = 0.0f;
    pid->d_filtered = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->prev_output = 0.0f;
    pid->initialized = 0;
}
```

---

### Task 4: Integrate `DRIVE_INTERSECTION` State Machine into `source/main.c`

**Files:**
- Modify: [`source/main.c:20-50, 130-170, 220-250`](file:///d:/robotica/workspace/nxpcup-ciorba_curata/source/main.c)

- [ ] **Step 1: Add `DRIVE_INTERSECTION` to `drive_state_t`**
```c
typedef enum {
    DRIVE_FORWARD,
    DRIVE_TURNING,
    DRIVE_BRAKING,
    DRIVE_INTERSECTION
} drive_state_t;
```

- [ ] **Step 2: Add state variables and reset in `main.c`**
Declare `static uint32_t s_intersection_hold_ticks = 0;`. Reset in idle loop.

- [ ] **Step 3: Insert raw intersection detection before `vector_filtering`**
```c
/* Raw intersection detection before vector pruning */
intersection_result_t ir = detect_intersection(vectors, num_vectors);

if (ir.is_intersection && drive_state != DRIVE_INTERSECTION) {
    drive_state = DRIVE_INTERSECTION;
    s_intersection_hold_ticks = (uint32_t)INTERSECTION_HOLD_TICKS;
    esp_printf("INTERSECTION enter reason=%u count=%u\r\n", (unsigned)ir.reason, (unsigned)ir.raw_count);
}

if (drive_state == DRIVE_INTERSECTION) {
    /* Drive straight ahead across crossing to enter upper loop instead of turning into right branch */
    Steer(g_params.steering_offset);
    HbridgeSpeed(&g_hbridge, (int16_t)applied_speed_left, (int16_t)applied_speed_right);

    if (s_intersection_hold_ticks > 0) {
        s_intersection_hold_ticks--;
    } else {
        drive_state = DRIVE_FORWARD;
        reset_pid_state(&steer_pid);
    }
    continue;
}
```

- [ ] **Step 4: Add deadband for straight tracking**
```c
float effective_cte = filtered_cte;
if (drive_state == DRIVE_FORWARD && fabsf(filtered_cte) < (float)STRAIGHT_DEADBAND_PX) {
    effective_cte = 0.0f;
}
float combined = g_params.k_cte * effective_cte + g_params.k_heading * sr.heading_deg;
```

---

### Task 5: End-to-End Build and Verification

**Files:**
- Modify: `scratch/verify_all.c`
- Binary: `Debug/nxpcup.axf`

- [ ] **Step 1: Update assertions in `scratch/verify_all.c`**
Verify `KP_STRAIGHT == 0.15f`, `KP_CORNER == 3.20f`, `INTERSECTION_HOLD_TICKS == 10`.
Run with `clang` and verify all assertions pass.

- [ ] **Step 2: Compile target firmware binary**
Run: `$env:PATH = "C:\Users\rober\.mcuxpressotools\arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi\bin;" + $env:PATH; make -C Debug all`
Verify: `nxpcup.axf` links with 0 errors and 0 warnings.
