# Dynamic Cornering & Robust Lookahead Vision Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a robust vision parsing and dynamic trail-braking control pipeline for the NXP Cup race car based on real-world PixyMon footage, eliminating cornering underturn/overturn, intersection phantom steering, and high-speed straightaway oscillations.

**Architecture:**
1. **Lookahead Vision Pipeline (`source/vectors.c`, `source/vectors.h`):** Evaluates vector intersections at row $Y=22$ in native pixel space, unconditionally discards crossing lines ($|\text{slope}| > 0.75$), merges piecewise fragmented line chords ($<16\text{px}$), performs relative spatial pairing ($W \in [28, 52]\text{px}$), and uses persistent boundary memory (`s_last_left_x`, `s_last_right_x`) for single-line apex fallback.
2. **Anti-Wobble & Curvature Steering (`source/control_theory.c`, `source/main.c`):** PID on EMA-filtered CTE + heading feedforward, with dynamic gain scheduling (55% authority on straightaways, restoring 100% in turns) and steering slew-rate limiting ($\le 450\%/\text{s}$).
3. **Trail-Braking & Differential Drive (`source/hbridge.c`, `source/wifi_config.c`):** Smooth deceleration from $80\%$ down to $38\%$ base cornering floor based on heading preview, low-pass filtered Ackermann differential drive ($\alpha=0.20$), normalized Left/Right motor polarities, and a 20-frame ($\approx 250\text{ms}$) coasting dead-reckoning window through 90° intersections.

**Tech Stack:** C99, ARM Cortex-M33 (NXP MCXN947), CMSIS, GCC unit testing.

---

### Task 1: Vision Normalization, Crossing Filter, Piecewise Merging & Memory Fallback in `vectors.c`

**Files:**
- Modify: `source/vectors.h`
- Modify: `source/vectors.c`
- Create: `scratch/test_intersection.c`
- Create: `scratch/test_lane_vectors.c`

- [ ] **Step 1: Write failing unit tests for intersection filtering and lane vector discrimination**

Create `scratch/test_intersection.c`:
```c
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include "vectors.h"

int main(void) {
    // Vector format: [x0, y0, x1, y1, idx, flags]
    // Vector 1: Forward line (x0=20, y0=50, x1=20, y1=10) -> dy=40, dx=0, slope=0.0 (KEEP)
    // Vector 2: Crossing tape (x0=5, y0=35, x1=35, y1=0) -> dy=35, dx=30, slope=0.857 > 0.75 (DISCARD)
    // Vector 3: Horizontal tape (x0=10, y0=20, x1=60, y1=20) -> dy=0 (DISCARD)
    uint16_t vecs[3 * 6] = {
        20, 50, 20, 10, 0, 0,
        5,  35, 35, 0,  1, 0,
        10, 20, 60, 20, 2, 0
    };

    size_t kept = filter_intersection_vectors(vecs, 3);
    assert(kept == 1);
    assert(vecs[0] == 20 && vecs[1] == 50 && vecs[2] == 20 && vecs[3] == 10);
    printf("Intersection filtering test PASSED!\n");
    return 0;
}
```

Create `scratch/test_lane_vectors.c`:
```c
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include "vectors.h"

void esp_printf(const char *fmt, ...) { (void)fmt; }

int main(void) {
    reset_lane_tracker();

    // Test 1: Straight lane centered (left x=20, right x=58 at lookahead y=22)
    uint16_t straight_vecs[2 * 6] = {
        20, 50, 20, 10, 0, 0,
        58, 50, 58, 10, 1, 0
    };
    lane_result_t res = compute_lane_center(straight_vecs, 2);
    assert(res.lines_found == 2);
    assert(fabsf(res.cte - 0.0f) < 1.0f);
    printf("Test 1 (Straight) PASSED\n");

    // Test 2: Piecewise segment merging on right curve (left at x=10, right split into x=46 and x=48)
    uint16_t piecewise_vecs[3 * 6] = {
        10, 50, 10, 10, 0, 0,
        46, 50, 46, 30, 1, 0,
        48, 30, 48, 10, 2, 0
    };
    res = compute_lane_center(piecewise_vecs, 3);
    assert(res.lines_found == 2);
    // Center: (10 + 47) / 2 = 28.5 -> CTE: 28.5 - 39.0 = -10.5
    assert(fabsf(res.cte - (-10.5f)) < 1.0f);
    printf("Test 2 (Piecewise Merge) PASSED\n");

    // Test 3: Single line apex drift (left line dropped out, right line at x=50)
    // Should use tracked half lane (approx 19px) to project center to 50 - 19 = 31 -> CTE = 31 - 39 = -8.0
    uint16_t single_right_vec[1 * 6] = {
        50, 50, 50, 10, 1, 0
    };
    res = compute_lane_center(single_right_vec, 1);
    assert(res.lines_found == 1);
    assert(fabsf(res.cte - (-8.0f)) < 2.0f);
    printf("Test 3 (Single Line Apex) PASSED\n");

    printf("All Vision Tests PASSED!\n");
    return 0;
}
```

- [ ] **Step 2: Run tests to verify failure**

Run: `gcc -Wall -Wextra -Iinclude -Isource scratch/test_intersection.c source/vectors.c -lm -o scratch/test_intersection`
Expected: Undefined reference or assertion failure.

- [ ] **Step 3: Implement `vectors.h` and `vectors.c`**

In `source/vectors.h`:
```c
#ifndef VECTORS_H
#define VECTORS_H

#include <stddef.h>
#include <stdint.h>

#define PIXY_CENTER_X           39.0f
#define LOOKAHEAD_Y             22
#define INTERSECTION_MAX_SLOPE  0.75f

typedef struct {
    float   cte;          /* lateral offset from center in pixels (+ = right of center) */
    float   heading;      /* slope: lateral px / forward px (+ = tilting right)          */
    uint8_t lines_found;  /* 0 = lost, 1 = single line tracked, 2 = dual lane boundaries */
} lane_result_t;

void          reset_lane_tracker(void);
lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors);
size_t        filter_intersection_vectors(uint16_t *vectors, size_t num_vectors);
void          print_vector_details(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, size_t index);

#endif /* VECTORS_H */
```

In `source/vectors.c`:
```c
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include "wifi_config.h"
#include "vectors.h"

#define MIN_VALID_LANE_WIDTH_PX 28.0f
#define MAX_VALID_LANE_WIDTH_PX 52.0f
#define DEFAULT_HALF_LANE_PX    19.0f

static float s_tracked_half_lane = DEFAULT_HALF_LANE_PX;
static float s_last_left_x       = PIXY_CENTER_X - DEFAULT_HALF_LANE_PX;
static float s_last_right_x      = PIXY_CENTER_X + DEFAULT_HALF_LANE_PX;

void reset_lane_tracker(void)
{
    s_tracked_half_lane = DEFAULT_HALF_LANE_PX;
    s_last_left_x       = PIXY_CENTER_X - DEFAULT_HALF_LANE_PX;
    s_last_right_x      = PIXY_CENTER_X + DEFAULT_HALF_LANE_PX;
}

lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors)
{
    lane_result_t result = {0.0f, 0.0f, 0};
    if (num_vectors == 0) return result;

    typedef struct {
        float x_at_look;
        float slope;
    } cand_t;

    #define MAX_LOCAL_CANDS 16
    cand_t cands[MAX_LOCAL_CANDS];
    size_t cand_count = 0;

    for (size_t i = 0; i < num_vectors && cand_count < MAX_LOCAL_CANDS; i++) {
        float x0 = (float)vectors[6*i + 0];
        float y0 = (float)vectors[6*i + 1];
        float x1 = (float)vectors[6*i + 2];
        float y1 = (float)vectors[6*i + 3];

        /* Ensure (x0, y0) is near (larger Y) and (x1, y1) is far (smaller Y) */
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
        float x_at_look = x0 + slope * (y0 - eval_y);

        cands[cand_count].x_at_look = x_at_look;
        cands[cand_count].slope     = slope;
        cand_count++;
    }

    if (cand_count == 0) return result;

    /* Sort candidates left-to-right */
    for (size_t i = 0; i < cand_count - 1; i++) {
        for (size_t j = i + 1; j < cand_count; j++) {
            if (cands[j].x_at_look < cands[i].x_at_look) {
                cand_t tmp = cands[i];
                cands[i] = cands[j];
                cands[j] = tmp;
            }
        }
    }

    /* Merge adjacent piecewise segments on the same boundary (< 16px gap) */
    cand_t merged[MAX_LOCAL_CANDS];
    size_t merged_count = 0;
    for (size_t i = 0; i < cand_count; i++) {
        if (merged_count > 0 && (cands[i].x_at_look - merged[merged_count - 1].x_at_look) < 16.0f) {
            merged[merged_count - 1].x_at_look = 0.5f * (merged[merged_count - 1].x_at_look + cands[i].x_at_look);
            merged[merged_count - 1].slope     = 0.5f * (merged[merged_count - 1].slope + cands[i].slope);
        } else {
            merged[merged_count++] = cands[i];
        }
    }

    if (merged_count >= 2) {
        float best_diff = 1e6f;
        int best_i = -1, best_j = -1;
        float target_w = s_tracked_half_lane * 2.0f;

        for (size_t i = 0; i < merged_count - 1; i++) {
            for (size_t j = i + 1; j < merged_count; j++) {
                float w = merged[j].x_at_look - merged[i].x_at_look;
                if (w >= MIN_VALID_LANE_WIDTH_PX && w <= MAX_VALID_LANE_WIDTH_PX) {
                    float score = fabsf(w - target_w);
                    if (score <= 14.0f && score < best_diff) {
                        best_diff = score;
                        best_i = (int)i;
                        best_j = (int)j;
                    }
                }
            }
        }

        if (best_i >= 0 && best_j >= 0) {
            float left_x   = merged[best_i].x_at_look;
            float right_x  = merged[best_j].x_at_look;
            float center_x = 0.5f * (left_x + right_x);
            float width    = right_x - left_x;

            s_tracked_half_lane = 0.85f * s_tracked_half_lane + 0.15f * (0.5f * width);
            s_last_left_x       = left_x;
            s_last_right_x      = right_x;

            result.cte         = center_x - PIXY_CENTER_X;
            result.heading     = 0.5f * (merged[best_i].slope + merged[best_j].slope);
            result.lines_found = 2;
            return result;
        }
    }

    /* Single line fallback: select candidate closest to tracked boundary memory */
    cand_t *c = &merged[0];
    float min_dist = 1e6f;
    for (size_t i = 0; i < merged_count; i++) {
        float dl = fabsf(merged[i].x_at_look - s_last_left_x);
        float dr = fabsf(merged[i].x_at_look - s_last_right_x);
        float d  = (dl < dr) ? dl : dr;
        if (d < min_dist) {
            min_dist = d;
            c = &merged[i];
        }
    }

    float d_to_left  = fabsf(c->x_at_look - s_last_left_x);
    float d_to_right = fabsf(c->x_at_look - s_last_right_x);
    float center_x;

    if (d_to_left < d_to_right) {
        center_x       = c->x_at_look + s_tracked_half_lane;
        s_last_left_x  = c->x_at_look;
        s_last_right_x = c->x_at_look + (2.0f * s_tracked_half_lane);
    } else {
        center_x       = c->x_at_look - s_tracked_half_lane;
        s_last_right_x = c->x_at_look;
        s_last_left_x  = c->x_at_look - (2.0f * s_tracked_half_lane);
    }

    result.cte         = center_x - PIXY_CENTER_X;
    result.heading     = c->slope;
    result.lines_found = 1;
    return result;
}

size_t filter_intersection_vectors(uint16_t *vectors, size_t num_vectors)
{
    if (num_vectors == 0) return 0;
    size_t kept = 0;

    for (size_t i = 0; i < num_vectors; i++) {
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

        float abs_slope = fabsf((x1 - x0) / dy);
        if (abs_slope <= INTERSECTION_MAX_SLOPE) {
            if (kept != i) {
                for (int j = 0; j < 6; j++) {
                    vectors[6*kept + j] = vectors[6*i + j];
                }
            }
            kept++;
        }
    }
    return kept;
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `gcc -Wall -Wextra -Iinclude -Isource scratch/test_intersection.c source/vectors.c -lm -o scratch/test_intersection && ./scratch/test_intersection`
Run: `gcc -Wall -Wextra -Iinclude -Isource scratch/test_lane_vectors.c source/vectors.c -lm -o scratch/test_lane_vectors && ./scratch/test_lane_vectors`
Expected: `PASS` for all suites.

- [ ] **Step 5: Commit**

```bash
git add source/vectors.h source/vectors.c scratch/test_intersection.c scratch/test_lane_vectors.c
git commit -m "feat(vision): implement lookahead evaluation, crossing filter, piecewise merging, and persistent memory"
```

---

### Task 2: Adaptive Speed-Steer Gain Scheduling in `control_theory.c`

**Files:**
- Modify: `source/control_theory.h`
- Modify: `source/control_theory.c`
- Create: `scratch/test_steer_scheduling.c`

- [ ] **Step 1: Write failing unit test for adaptive steer gain scheduling**

Create `scratch/test_steer_scheduling.c`:
```c
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "control_theory.h"

int main(void) {
    float base_spd = 38.0f;
    float max_spd  = 80.0f;
    float min_scale = 0.55f;

    // 1. High speed on straight (CTE = 0, Heading = 0) -> attenuated to 0.55f
    float g_straight = compute_speed_steer_gain_adaptive(80.0f, base_spd, max_spd, min_scale, 0.0f, 0.0f);
    assert(fabsf(g_straight - 0.55f) < 1e-3f);

    // 2. High speed approaching corner (CTE = 16.0f or Heading = 0.35f) -> full authority 1.0f
    float g_corner = compute_speed_steer_gain_adaptive(80.0f, base_spd, max_spd, min_scale, 16.0f, 0.35f);
    assert(fabsf(g_corner - 1.0f) < 0.05f);

    // 3. Base cornering speed (38.0f) -> always 1.0f
    float g_slow = compute_speed_steer_gain_adaptive(38.0f, base_spd, max_spd, min_scale, 0.0f, 0.0f);
    assert(fabsf(g_slow - 1.0f) < 1e-3f);

    printf("Steer scheduling test PASSED!\n");
    return 0;
}
```

- [ ] **Step 2: Run test to verify failure**

Run: `gcc -Wall -Wextra -Iinclude -Isource scratch/test_steer_scheduling.c source/control_theory.c -lm -o scratch/test_steer_scheduling`
Expected: Linker error (undefined reference to `compute_speed_steer_gain_adaptive`).

- [ ] **Step 3: Implement adaptive gain function in `control_theory.h` & `control_theory.c`**

In `source/control_theory.h`:
```c
float compute_speed_steer_gain_adaptive(float current_speed, float base_speed, float max_speed,
                                       float min_scale, float abs_cte, float abs_heading);
```

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

    float cte_urgency     = abs_cte / 16.0f;
    float heading_urgency = abs_heading / 0.35f;
    float urgency         = (cte_urgency > heading_urgency) ? cte_urgency : heading_urgency;
    if (urgency > 1.0f) urgency = 1.0f;

    return straight_attenuation + (1.0f - straight_attenuation) * urgency;
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `gcc -Wall -Wextra -Iinclude -Isource scratch/test_steer_scheduling.c source/control_theory.c -lm -o scratch/test_steer_scheduling && ./scratch/test_steer_scheduling`
Expected: `Steer scheduling test PASSED!`

- [ ] **Step 5: Commit**

```bash
git add source/control_theory.h source/control_theory.c scratch/test_steer_scheduling.c
git commit -m "feat(control): add curvature-preserving adaptive steering gain scheduling"
```

---

### Task 3: Motor Trail-Braking, Polarities & Differential Drive in `hbridge.c` & `wifi_config.c`

**Files:**
- Modify: `include/hbridge.h`
- Modify: `source/hbridge.c`
- Modify: `source/wifi_config.h`
- Modify: `source/wifi_config.c`
- Create: `scratch/test_motor_speeds.c`

- [ ] **Step 1: Write failing unit test for trail-braking and differential drive**

Create `scratch/test_motor_speeds.c`:
```c
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "hbridge.h"
#include "wifi_config.h"

int main(void) {
    params_init();
    motor_speeds_reset();

    float sl, sr;
    // Test 1: Straightaway (steer=0, heading=0) -> accelerates toward max_speed (80%)
    compute_motor_speeds(0.0f, 0.0f, 2, 0.05f, &sl, &sr);
    assert(sl > 0.0f && sr > 0.0f);
    assert(fabsf(sl - sr) < 1e-3f); // Equal wheel speeds

    // Accelerate to top speed over multiple cycles
    for (int i = 0; i < 50; i++) {
        compute_motor_speeds(0.0f, 0.0f, 2, 0.02f, &sl, &sr);
    }
    assert(motor_speeds_get_current() > 75.0f);
    printf("Test 1 (Straightaway Acceleration) PASSED\n");

    // Test 2: Preview sharp corner heading (slope = 0.40) -> decelerates toward base_speed (38%)
    for (int i = 0; i < 20; i++) {
        compute_motor_speeds(0.0f, 0.40f, 2, 0.02f, &sl, &sr);
    }
    assert(motor_speeds_get_current() < 50.0f);
    printf("Test 2 (Trail-Braking Deceleration) PASSED\n");

    // Test 3: Ackermann differential drive split in sharp right turn (steer = +60.0f)
    compute_motor_speeds(60.0f, 0.30f, 2, 0.02f, &sl, &sr);
    assert(sl > sr); // Left wheel outside (faster), right wheel inside (slower)
    printf("Test 3 (Ackermann Differential Split) PASSED\n");

    printf("All Motor Control Tests PASSED!\n");
    return 0;
}
```

- [ ] **Step 2: Run test to verify failure**

Run: `gcc -Wall -Wextra -Iinclude -Isource scratch/test_motor_speeds.c source/hbridge.c source/wifi_config.c -lm -o scratch/test_motor_speeds`
Expected: Compilation/signature failure.

- [ ] **Step 3: Implement `include/hbridge.h`, `source/hbridge.c`, and default parameters in `source/wifi_config.c`**

In `include/hbridge.h`:
```c
#ifndef HBRIDGE_H_
#define HBRIDGE_H_

#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#include <stdint.h>

typedef struct {
    CTIMER_Type   *pwmPeripheral;
    ctimer_match_t periodChannel;
    ctimer_match_t pwm1Channel;
    ctimer_match_t pwm2Channel;
    GPIO_Type     *motor1DirPort;
    uint32_t       motor1DirPin;
    GPIO_Type     *motor2DirPort;
    uint32_t       motor2DirPin;
} Hbridge;

extern Hbridge g_hbridge;

void HbridgeInit(Hbridge *h,
                 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1DirPort, uint32_t m1DirPin,
                 GPIO_Type *m2DirPort, uint32_t m2DirPin);

void  HbridgeSpeed(Hbridge *h, int16_t speed_right, int16_t speed_left);
void  HbridgeBrake(Hbridge *h);
void  motor_speeds_reset(void);
float motor_speeds_get_current(void);
void  compute_motor_speeds(float steer_cmd, float heading_err, uint8_t lines_found, float dt, float *speed_left, float *speed_right);

#endif /* HBRIDGE_H_ */
```

In `source/hbridge.c`:
```c
#include "hbridge.h"
#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "Config.h"
#include "wifi_config.h"
#include <math.h>

Hbridge g_hbridge;

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

void HbridgeInit(Hbridge *h,
                 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1DirPort, uint32_t m1DirPin,
                 GPIO_Type *m2DirPort, uint32_t m2DirPin)
{
    h->periodChannel    = periodCh;
    h->pwm1Channel      = pwm1Ch;
    h->pwm2Channel      = pwm2Ch;
    h->motor1DirPort    = m1DirPort;
    h->motor1DirPin     = m1DirPin;
    h->motor2DirPort    = m2DirPort;
    h->motor2DirPin     = m2DirPin;
    h->pwmPeripheral    = pwmPeriph;
    g_hbridge = *h;
}

void HbridgeSpeed(Hbridge *h, int16_t speed_right, int16_t speed_left)
{
    if (speed_right >  100) speed_right =  100;
    if (speed_right < -100) speed_right = -100;
    if (speed_left  >  100) speed_left  =  100;
    if (speed_left  < -100) speed_left  = -100;

    uint8_t duty_right, duty_left;

    /* Motor 1 (Right): Forward DIR=1, duty=100-speed */
    if (speed_right >= 0) {
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 1U);
        duty_right = (uint8_t)(100 - speed_right);
    } else {
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
        duty_right = (uint8_t)(-speed_right);
    }

    /* Motor 2 (Left): Forward DIR=0, duty=speed */
    if (speed_left >= 0) {
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);
        duty_left = (uint8_t)speed_left;
    } else {
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 1U);
        duty_left = (uint8_t)(100 - (-speed_left));
    }

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, duty_right);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, duty_left);
}

void HbridgeBrake(Hbridge *h)
{
    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, 0U);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, 0U);
}

static float s_current_speed = 0.0f;
static float s_filtered_diff  = 0.0f;

void motor_speeds_reset(void)
{
    s_current_speed = 0.0f;
    s_filtered_diff = 0.0f;
}

float motor_speeds_get_current(void)
{
    return s_current_speed;
}

void compute_motor_speeds(float steer_cmd, float heading_err, uint8_t lines_found, float dt, float *speed_left, float *speed_right)
{
    const float HEADING_CURVATURE_GAIN = 2.0f;
    float steer_factor   = fabsf(steer_cmd) / 100.0f;
    float heading_factor = fabsf(heading_err) * HEADING_CURVATURE_GAIN;
    float turn_intensity = (steer_factor > heading_factor) ? steer_factor : heading_factor;
    if (turn_intensity > 1.0f) turn_intensity = 1.0f;

    float target_left  = g_params.max_speed_left  - turn_intensity * (g_params.max_speed_left  - g_params.base_speed_left);
    float target_right = g_params.max_speed_right - turn_intensity * (g_params.max_speed_right - g_params.base_speed_right);
    float target_base  = 0.5f * (target_left + target_right);

    if (lines_found < 2) {
        float safe_cap = (g_params.base_speed_left + g_params.base_speed_right) * 0.5f;
        if (target_base > safe_cap) {
            target_base  = safe_cap;
            target_left  = g_params.base_speed_left;
            target_right = g_params.base_speed_right;
        }
    }

    const float MAX_ACCEL_PER_SEC = 200.0f;
    const float MAX_BRAKE_PER_SEC = 500.0f;

    float max_pos_step = MAX_ACCEL_PER_SEC * dt;
    float max_neg_step = MAX_BRAKE_PER_SEC * dt;

    if (target_base > s_current_speed) {
        float diff = target_base - s_current_speed;
        s_current_speed += (diff > max_pos_step) ? max_pos_step : diff;
    } else {
        float diff = s_current_speed - target_base;
        s_current_speed -= (diff > max_neg_step) ? max_neg_step : diff;
    }

    float steer_rad = (steer_cmd / 100.0f)
                    * (MAX_SERVO_ANGLE_DEG * (float)M_PI / 180.0f);

    float radius = (fabsf(steer_rad) > 0.001f)
                 ? (WHEELBASE_MM / tanf(fabsf(steer_rad)))
                 : 1e6f;

    float half_track    = TRACK_WIDTH_MM * 0.5f;
    float raw_diff      = (radius > 1.0f) ? (half_track / radius) : 0.0f;
    if (raw_diff > 0.25f) raw_diff = 0.25f;

    s_filtered_diff = 0.80f * s_filtered_diff + 0.20f * raw_diff;

    float inside_scale  = 1.0f - s_filtered_diff;
    float outside_scale = 1.0f + s_filtered_diff;
    if (inside_scale < 0.0f) inside_scale = 0.0f;

    float base_l = (target_base > 0.0f) ? (s_current_speed * (target_left / target_base)) : s_current_speed;
    float base_r = (target_base > 0.0f) ? (s_current_speed * (target_right / target_base)) : s_current_speed;

    if (steer_cmd > 0.0f) {
        *speed_right = base_r * inside_scale;
        *speed_left  = base_l * outside_scale;
    } else if (steer_cmd < 0.0f) {
        *speed_left  = base_l * inside_scale;
        *speed_right = base_r * outside_scale;
    } else {
        *speed_left  = base_l;
        *speed_right = base_r;
    }

    if (*speed_left  >  g_params.max_speed_left)  *speed_left  =  g_params.max_speed_left;
    if (*speed_left  < -g_params.max_speed_left)  *speed_left  = -g_params.max_speed_left;
    if (*speed_right >  g_params.max_speed_right) *speed_right =  g_params.max_speed_right;
    if (*speed_right < -g_params.max_speed_right) *speed_right = -g_params.max_speed_right;
}
```

In `source/wifi_config.c`:
```c
void params_init(void)
{
    g_params.kp        = 1.40f;
    g_params.ki        = 0.00f;
    g_params.kd        = 0.35f;
    g_params.k_cte     = 1.00f;
    g_params.k_heading = 10.00f;

    g_params.dt    = 0.02f;
    g_params.alpha = 0.20f;

    g_params.base_speed_left  = 38.0f;
    g_params.base_speed_right = 38.0f;
    g_params.max_speed_left   = 80.0f;
    g_params.max_speed_right  = 80.0f;

    g_params.steering_offset = 0.0f;
    g_params.state           = STATE_IDLE;
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `gcc -Wall -Wextra -Iinclude -Isource scratch/test_motor_speeds.c source/hbridge.c source/wifi_config.c -lm -o scratch/test_motor_speeds && ./scratch/test_motor_speeds`
Expected: `All Motor Control Tests PASSED!`

- [ ] **Step 5: Commit**

```bash
git add include/hbridge.h source/hbridge.c source/wifi_config.c scratch/test_motor_speeds.c
git commit -m "feat(motors): implement lookahead trail-braking, filtered Ackermann diff, and normalized polarities"
```

---

### Task 4: Main Loop Integration & Workspace Build Verification

**Files:**
- Modify: `source/main.c`

- [ ] **Step 1: Update `source/main.c` control loop**

Integrate DWT hardware cycle timing, intersection filtering, lookahead centering, adaptive gain scheduling, steering slew-rate limiter, 20-frame coasting window, and telemetric formatting in `source/main.c`:

```c
#define LINE_LOST_LIMIT 20
#define CTE_FILTER_NEW  0.80f
#define CTE_FILTER_OLD  0.20f
```

- [ ] **Step 2: Run all unit tests**

Run:
```bash
./scratch/test_intersection && ./scratch/test_lane_vectors && ./scratch/test_steer_scheduling && ./scratch/test_motor_speeds
```
Expected: All 4 test suites output `PASSED`.

- [ ] **Step 3: Verify full ARM compilation compatibility**

Compile all source modules to verify zero missing symbols or compiler warnings.

- [ ] **Step 4: Clean up scratch test binaries and commit**

```bash
rm -f scratch/test_intersection scratch/test_lane_vectors scratch/test_steer_scheduling scratch/test_motor_speeds
git add source/main.c
git commit -m "feat(main): integrate real DWT dt, adaptive gain scheduling, slew rate limiting, and intersection coasting"
```
