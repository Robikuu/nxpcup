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
