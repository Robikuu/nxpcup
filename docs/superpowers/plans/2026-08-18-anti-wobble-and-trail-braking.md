# Anti-Wobble Gain Scheduling and Lookahead Trail-Braking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement speed-dependent steering gain scheduling to prevent high-speed weaving/tank-slapping on straights, and enhance lookahead curvature sensitivity for early trail-braking before entering corners.

**Architecture:** 
1. Expose current tracked speed from `hbridge.c` to feed a gain-scheduling function in `control_theory.c`, dynamically attenuating steering authority (down to 55% at top speed while retaining 100% at base cornering speed).
2. Enhance `hbridge.c` curvature intensity calculation by scaling lookahead heading slope sensitivity (`heading_factor`) so that previewed curves trigger anticipatory deceleration before the vehicle reaches the apex.

**Tech Stack:** C99, ARM Cortex-M33 (NXP MCXN947), CMSIS, GCC.

---

### Task 1: Speed-Dependent Steering Gain Scheduling (Anti-Wobble)

**Files:**
- Modify: `include/control_theory.h`
- Modify: `source/control_theory.c`
- Modify: `include/hbridge.h`
- Modify: `source/hbridge.c`
- Modify: `source/main.c`
- Test: `scratch/test_steer_gain.c`

- [ ] **Step 1: Write the failing unit test for `compute_speed_steer_gain`**

```c
// scratch/test_steer_gain.c
#include <stdio.h>
#include <assert.h>
#include <math.h>

float compute_speed_steer_gain(float current_speed, float base_speed, float max_speed, float min_scale);

int main(void) {
    // At or below base speed -> 100% steering gain (1.0f)
    assert(fabsf(compute_speed_steer_gain(20.0f, 20.0f, 80.0f, 0.55f) - 1.0f) < 1e-4f);
    assert(fabsf(compute_speed_steer_gain(10.0f, 20.0f, 80.0f, 0.55f) - 1.0f) < 1e-4f);

    // At max speed -> min_scale (0.55f)
    assert(fabsf(compute_speed_steer_gain(80.0f, 20.0f, 80.0f, 0.55f) - 0.55f) < 1e-4f);
    assert(fabsf(compute_speed_steer_gain(90.0f, 20.0f, 80.0f, 0.55f) - 0.55f) < 1e-4f);

    // Midpoint (50.0f) -> (1.0 + 0.55) / 2 = 0.775f
    assert(fabsf(compute_speed_steer_gain(50.0f, 20.0f, 80.0f, 0.55f) - 0.775f) < 1e-4f);

    printf("Steer gain scheduling test PASSED!\n");
    return 0;
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `gcc -Wall -Wextra scratch/test_steer_gain.c -o scratch/test_steer_gain`
Expected: Linker/Compiler failure (undefined reference to `compute_speed_steer_gain`).

- [ ] **Step 3: Implement `compute_speed_steer_gain` and `motor_speeds_get_current`**

In `include/control_theory.h`:
```c
float compute_speed_steer_gain(float current_speed, float base_speed, float max_speed, float min_scale);
```

In `source/control_theory.c`:
```c
float compute_speed_steer_gain(float current_speed, float base_speed, float max_speed, float min_scale)
{
    if (max_speed <= base_speed) return 1.0f;
    float speed_ratio = (current_speed - base_speed) / (max_speed - base_speed);
    if (speed_ratio < 0.0f) speed_ratio = 0.0f;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;
    return 1.0f - ((1.0f - min_scale) * speed_ratio);
}
```

In `include/hbridge.h`:
```c
float motor_speeds_get_current(void);
```

In `source/hbridge.c`:
```c
float motor_speeds_get_current(void)
{
    return s_current_speed;
}
```

In `source/main.c`:
```c
/* Scale steering authority based on current speed to prevent high-speed oscillation */
float current_speed = motor_speeds_get_current();
float base_avg      = 0.5f * (g_params.base_speed_left + g_params.base_speed_right);
float max_avg       = 0.5f * (g_params.max_speed_left + g_params.max_speed_right);
float steer_gain    = compute_speed_steer_gain(current_speed, base_avg, max_avg, 0.55f);
steer_cmd *= steer_gain;
```

- [ ] **Step 4: Run test to verify it passes**

Run: `gcc -Wall -Wextra scratch/test_steer_gain.c source/control_theory.c -o scratch/test_steer_gain && ./scratch/test_steer_gain`
Expected: `Steer gain scheduling test PASSED!`

---

### Task 2: Lookahead Trail-Braking & Enhanced Curvature Anticipation

**Files:**
- Modify: `source/hbridge.c`
- Test: `scratch/test_motor_speeds.c`

- [ ] **Step 1: Write the failing unit test for heading lookahead sensitivity**

In `scratch/test_motor_speeds.c`, add an assertion that a heading slope of 0.4 triggers deceleration down toward cornering base speed even when `steer_cmd` is 0 (i.e. straight approach).

- [ ] **Step 2: Run test to verify it fails**

Run: `gcc -Wall -Wextra -Iinclude scratch/test_motor_speeds.c source/hbridge.c -lm -o scratch/test_motor_speeds && ./scratch/test_motor_speeds`

- [ ] **Step 3: Update `turn_intensity` calculation in `source/hbridge.c`**

```c
    /* 1. Combined Curvature Demand [0.0 to 1.0] from steering + heading preview
     * HEADING_CURVATURE_GAIN = 2.0f makes a 0.5 slope trigger 100% corner derating early. */
    const float HEADING_CURVATURE_GAIN = 2.0f;
    float steer_factor   = fabsf(steer_cmd) / 100.0f;
    float heading_factor = fabsf(heading_err) * HEADING_CURVATURE_GAIN;
    float turn_intensity = (steer_factor > heading_factor) ? steer_factor : heading_factor;
    if (turn_intensity > 1.0f) turn_intensity = 1.0f;
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./scratch/test_motor_speeds`
Expected: PASS.
