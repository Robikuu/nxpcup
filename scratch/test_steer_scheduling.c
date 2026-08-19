#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "control_theory.h"

int main(void) {
    float base_spd = 38.0f;
    float max_spd  = 75.0f;
    float min_scale = 0.55f;

    // 1. Straightaway (CTE = 0, Heading = 0, speed = 75) -> 0.55f
    float g_straight = compute_speed_steer_gain_adaptive(75.0f, base_spd, max_spd, min_scale, 0.0f, 0.0f);
    assert(fabsf(g_straight - 0.55f) < 1e-3f);

    // 2. Corner entry (CTE = 8.0px or Heading = 0.15, speed = 75) -> 1.0f
    float g_corner_cte = compute_speed_steer_gain_adaptive(75.0f, base_spd, max_spd, min_scale, 8.0f, 0.0f);
    assert(fabsf(g_corner_cte - 1.0f) < 1e-3f);

    float g_corner_heading = compute_speed_steer_gain_adaptive(75.0f, base_spd, max_spd, min_scale, 0.0f, 0.15f);
    assert(fabsf(g_corner_heading - 1.0f) < 1e-3f);

    float g_corner_both = compute_speed_steer_gain_adaptive(75.0f, base_spd, max_spd, min_scale, 8.0f, 0.15f);
    assert(fabsf(g_corner_both - 1.0f) < 1e-3f);

    // 3. Base cornering speed (speed = 38) -> 1.0f
    float g_slow = compute_speed_steer_gain_adaptive(38.0f, base_spd, max_spd, min_scale, 0.0f, 0.0f);
    assert(fabsf(g_slow - 1.0f) < 1e-3f);

    printf("Steer scheduling test PASSED!\n");
    return 0;
}

