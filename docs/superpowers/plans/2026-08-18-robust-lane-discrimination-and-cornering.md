# Robust Lane Discrimination & Cornering Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate cornering underturn/overturn and line-loss failures by replacing static centerline vector classification with relative pair discrimination, tracking dynamic lane width, and preventing steering attenuation during corner entry.

**Architecture:**
1. **Vision Discrimination (`source/vectors.c`):** Replace static `x < 39` classification with relative spatial sorting (leftmost candidate is always left boundary, rightmost is right boundary) and dynamic EMA lane-width tracking. Implement history-aware fallback when only a single line is visible.
2. **Curvature-Preserving Gain Scheduling (`source/control_theory.c`, `source/main.c`):** Ensure steering attenuation only applies when tracking straight near the center ($|\text{CTE}| \approx 0$). Scale authority back up to 100% when high lateral error or heading curvature is detected, eliminating corner-entry understeer.
3. **Feedforward Heading Integration (`source/wifi_config.c`, `source/main.c`):** Set a calibrated default `k_heading` gain and pass raw unattenuated curvature demand to `compute_motor_speeds()` for responsive trail-braking.

**Tech Stack:** C99, ARM Cortex-M33 (NXP MCXN947), CMSIS, GCC unit testing.

---

### Task 1: Relative Spatial Pair Discrimination & Dynamic Lane Width in `vectors.c`

**Files:**
- Modify: `source/vectors.c`
- Modify: `source/vectors.h`
- Test: `scratch/test_lane_vectors.c`

- [ ] **Step 1: Write the failing unit tests for cornering vector discrimination**

Create `scratch/test_lane_vectors.c`:
```c
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "vectors.h"

// Mock esp_printf if referenced
void esp_printf(const char *fmt, ...) { (void)fmt; }

int main(void) {
    reset_lane_tracker();

    // Test 1: Straight lane centered (left x=20, right x=58 at lookahead y=30)
    // Vectors format: [x0, y0, x1, y1, idx, flags] where (x0,y0) is near (y=50), (x1,y1) is far (y=10)
    uint16_t straight_vecs[2 * 6] = {
        20, 50, 20, 10, 0, 0, // Left line at x=20
        58, 50, 58, 10, 1, 0  // Right line at x=58
    };
    lane_result_t res = compute_lane_center(straight_vecs, 2);
    assert(res.lines_found == 2);
    assert(fabsf(res.cte - (39.0f - 39.0f)) < 1.0f); // Center at 39, CTE = 0
    printf("Test 1 (Straight) PASSED\n");

    // Test 2: Right corner entry - BOTH lines shifted LEFT of image center (x=39)
    // Left line at x=8, Right line at x=34 (both < 39!)
    uint16_t right_turn_vecs[2 * 6] = {
        8,  50, 18, 10, 0, 0,  // Left line: x(y=30) = 8 + (10/40)*20 = 13.0
        29, 50, 39, 10, 1, 0   // Right line: x(y=30) = 29 + (10/40)*20 = 34.0
    };
    // Expected center: (13.0 + 34.0) / 2 = 23.5
    // Expected CTE: 23.5 - 39.0 = -15.5 (negative = steer right)
    res = compute_lane_center(right_turn_vecs, 2);
    assert(res.lines_found == 2);
    assert(fabsf(res.cte - (-15.5f)) < 1.0f);
    assert(res.cte < 0.0f); // Must NOT be positive!
    printf("Test 2 (Right turn - both lines < 39) PASSED\n");

    // Test 3: Left corner entry - BOTH lines shifted RIGHT of image center (x=39)
    // Left line at x=44, Right line at x=70 (both > 39!)
    uint16_t left_turn_vecs[2 * 6] = {
        44, 50, 44, 10, 0, 0,
        70, 50, 70, 10, 1, 0
    };
    // Expected center: (44 + 70) / 2 = 57.0
    // Expected CTE: 57.0 - 39.0 = +18.0 (positive = steer left)
    res = compute_lane_center(left_turn_vecs, 2);
    assert(res.lines_found == 2);
    assert(fabsf(res.cte - 18.0f) < 1.0f);
    assert(res.cte > 0.0f); // Must NOT be negative!
    printf("Test 3 (Left turn - both lines > 39) PASSED\n");

    // Test 4: Single inside line visible in right turn (left line at x=10, right line off-screen)
    // Should use tracked lane half-width (approx 18-20px) to project center to the right
    uint16_t single_left_vec[1 * 6] = {
        10, 50, 20, 10, 0, 0 // x(y=30) = 15.0, slope = +0.25
    };
    res = compute_lane_center(single_left_vec, 1);
    assert(res.lines_found == 1);
    assert(res.cte < -4.0f); // Center projected to ~33 -> CTE ~ -6.0
    printf("Test 4 (Single line in turn) PASSED\n");

    printf("All Lane Vector Discrimination Tests PASSED!\n");
    return 0;
}
```

- [ ] **Step 2: Run test to verify it fails against current code**

Run: `gcc -Wall -Wextra -Iinclude -Isource scratch/test_lane_vectors.c source/vectors.c -lm -o scratch/test_lane_vectors && ./scratch/test_lane_vectors`
Expected: Assertion failure on Test 2 (current code misidentifies right line as left line and produces positive CTE).

- [ ] **Step 3: Implement robust pair discrimination and dynamic lane width in `vectors.c`**

In `source/vectors.h`:
```c
void reset_lane_tracker(void);
```

In `source/vectors.c`:
```c
#define MIN_VALID_LANE_WIDTH_PX 16.0f
#define MAX_VALID_LANE_WIDTH_PX 65.0f
#define DEFAULT_HALF_LANE_PX    18.0f

static float s_tracked_half_lane = DEFAULT_HALF_LANE_PX;
static float s_last_lane_center  = PIXY_CENTER_X;

void reset_lane_tracker(void)
{
    s_tracked_half_lane = DEFAULT_HALF_LANE_PX;
    s_last_lane_center  = PIXY_CENTER_X;
}

lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors)
{
    lane_result_t result = {0.0f, 0.0f, 0};
    if (num_vectors == 0) return result;

    typedef struct {
        float x_at_look;
        float slope;
    } cand_t;

    cand_t cands[MAX_VECTORS];
    size_t cand_count = 0;

    for (size_t i = 0; i < num_vectors && cand_count < MAX_VECTORS; i++) {
        float x0 = (float)vectors[6*i + 0];
        float y0 = (float)vectors[6*i + 1];
        float x1 = (float)vectors[6*i + 2];
        float y1 = (float)vectors[6*i + 3];

        if (y0 < y1) {
            float tx = x0; float ty = y0;
            x0 = x1; y0 = y1;
            x1 = tx; y1 = ty;
        }

        float dy = y0 - y1;
        if (dy < 1.0f) continue;

        float slope = (x1 - x0) / dy;
        float eval_y = (float)LOOKAHEAD_Y;
        if (eval_y < y1) eval_y = y1;
        if (eval_y > y0) eval_y = y0;
        float x_look = x0 + slope * (y0 - eval_y);

        cands[cand_count].x_at_look = x_look;
        cands[cand_count].slope     = slope;
        cand_count++;
    }

    if (cand_count == 0) return result;

    // Sort candidates left-to-right by x_at_look
    for (size_t i = 0; i < cand_count - 1; i++) {
        for (size_t j = i + 1; j < cand_count; j++) {
            if (cands[j].x_at_look < cands[i].x_at_look) {
                cand_t tmp = cands[i];
                cands[i] = cands[j];
                cands[j] = tmp;
            }
        }
    }

    if (cand_count >= 2) {
        // Find best pair with plausible lane width
        float best_diff = 1e6f;
        int best_i = -1, best_j = -1;

        for (size_t i = 0; i < cand_count - 1; i++) {
            for (size_t j = i + 1; j < cand_count; j++) {
                float w = cands[j].x_at_look - cands[i].x_at_look;
                if (w >= MIN_VALID_LANE_WIDTH_PX && w <= MAX_VALID_LANE_WIDTH_PX) {
                    float target_w = s_tracked_half_lane * 2.0f;
                    float score = fabsf(w - target_w);
                    if (score < best_diff) {
                        best_diff = score;
                        best_i = (int)i;
                        best_j = (int)j;
                    }
                }
            }
        }

        if (best_i >= 0 && best_j >= 0) {
            float left_x    = cands[best_i].x_at_look;
            float right_x   = cands[best_j].x_at_look;
            float center_x  = 0.5f * (left_x + right_x);
            float width     = right_x - left_x;

            s_tracked_half_lane = 0.85f * s_tracked_half_lane + 0.15f * (0.5f * width);
            s_last_lane_center  = center_x;

            result.cte         = center_x - PIXY_CENTER_X;
            result.heading     = 0.5f * (cands[best_i].slope + cands[best_j].slope);
            result.lines_found = 2;
            return result;
        }
    }

    // Single line fallback
    float line_x     = cands[0].x_at_look;
    float line_slope = cands[0].slope;
    float center_x;

    // Decide if single line is left or right boundary using history + slope
    if (line_x < s_last_lane_center) {
        // Left boundary: center is to the right
        center_x = line_x + s_tracked_half_lane;
    } else {
        // Right boundary: center is to the left
        center_x = line_x - s_tracked_half_lane;
    }

    s_last_lane_center  = center_x;
    result.cte         = center_x - PIXY_CENTER_X;
    result.heading     = line_slope;
    result.lines_found = 1;
    return result;
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `gcc -Wall -Wextra -Iinclude -Isource scratch/test_lane_vectors.c source/vectors.c -lm -o scratch/test_lane_vectors && ./scratch/test_lane_vectors`
Expected: `All Lane Vector Discrimination Tests PASSED!`

---

### Task 2: Curvature-Preserving Gain Scheduling & Control Integration

**Files:**
- Modify: `source/control_theory.h`
- Modify: `source/control_theory.c`
- Modify: `source/wifi_config.c`
- Modify: `source/main.c`
- Test: `scratch/test_steer_scheduling.c`

- [ ] **Step 1: Write failing unit test for curvature-preserving gain scheduling**

Create `scratch/test_steer_scheduling.c`:
```c
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "control_theory.h"

int main(void) {
    float base_spd = 20.0f;
    float max_spd  = 80.0f;
    float min_scale = 0.55f;

    // 1. High speed on straight (CTE = 0, Heading = 0) -> attenuated to 0.55f
    float g_straight = compute_speed_steer_gain_adaptive(80.0f, base_spd, max_spd, min_scale, 0.0f, 0.0f);
    assert(fabsf(g_straight - 0.55f) < 1e-3f);

    // 2. High speed approaching corner (CTE = 18.0f or Heading = 0.4f) -> full authority 1.0f
    float g_corner = compute_speed_steer_gain_adaptive(80.0f, base_spd, max_spd, min_scale, 18.0f, 0.3f);
    assert(fabsf(g_corner - 1.0f) < 0.05f);

    // 3. Cornering speed (20.0f) -> always 1.0f
    float g_slow = compute_speed_steer_gain_adaptive(20.0f, base_spd, max_spd, min_scale, 0.0f, 0.0f);
    assert(fabsf(g_slow - 1.0f) < 1e-3f);

    printf("Curvature-Preserving Gain Scheduling PASSED!\n");
    return 0;
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `gcc -Wall -Wextra -Iinclude -Isource scratch/test_steer_scheduling.c source/control_theory.c -lm -o scratch/test_steer_scheduling`
Expected: Failure (undefined reference to `compute_speed_steer_gain_adaptive`).

- [ ] **Step 3: Implement `compute_speed_steer_gain_adaptive` and update control loop**

In `source/control_theory.c`:
```c
float compute_speed_steer_gain_adaptive(float current_speed, float base_speed, float max_speed,
                                       float min_scale, float abs_cte, float abs_heading)
{
    if (max_speed <= base_speed) return 1.0f;

    float speed_ratio = (current_speed - base_speed) / (max_speed - base_speed);
    if (speed_ratio < 0.0f) speed_ratio = 0.0f;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;

    float straight_attenuation = 1.0f - ((1.0f - min_scale) * speed_ratio);

    // Curvature demand [0.0 = straight line, 1.0 = sharp turn]
    float cte_urgency     = abs_cte / 16.0f;
    float heading_urgency = abs_heading / 0.35f;
    float urgency         = (cte_urgency > heading_urgency) ? cte_urgency : heading_urgency;
    if (urgency > 1.0f) urgency = 1.0f;

    // Blend: when cornering urgency is high, restore full 100% steering authority
    return straight_attenuation + (1.0f - straight_attenuation) * urgency;
}
```

In `source/wifi_config.c`:
Set active default for `k_heading` feedforward:
```c
g_params.k_heading = 15.0f;
```

In `source/main.c`:
Update the gain scheduling call:
```c
float steer_scale = compute_speed_steer_gain_adaptive(current_spd, base_avg, max_avg, 0.55f,
                                                      fabsf(filtered_cte), fabsf(e_heading));
steer_cmd *= steer_scale;
```

- [ ] **Step 4: Run test to verify it passes**

Run: `gcc -Wall -Wextra -Iinclude -Isource scratch/test_steer_scheduling.c source/control_theory.c -lm -o scratch/test_steer_scheduling && ./scratch/test_steer_scheduling`
Expected: `Curvature-Preserving Gain Scheduling PASSED!`

---

### Task 3: Full Workspace Build & Regression Verification

**Files:**
- Verify: Full compilation of project source files

- [ ] **Step 1: Compile all source files to verify ARM Cortex-M33 / GCC compatibility**
- [ ] **Step 2: Clean up scratch test binaries**
- [ ] **Step 3: Commit all changes**
