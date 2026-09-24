#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include "vectors.h"

int main(void) {
    printf("Testing linked compute_lookahead_curvature from source/vectors.c...\n");

    // Test 1: Curve ahead in mid-far horizon (Y between 16 and 28, NO vectors at Y >= 30)
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

    // Test 3: Transverse tape wall ahead (90 deg turn wall: dx=32, dy=2 at Y=24)
    uint16_t wall[4] = {20, 24, 52, 22};
    trajectory_result_t r3 = compute_lookahead_curvature(wall, 1);
    printf("Test 3 (Transverse Wall): urgency=%.2f, found=%d\n",
           r3.lookahead_urgency, r3.found);
    assert(r3.lookahead_urgency >= 0.95f);

    printf("ALL LINKED PREVIEW CURVATURE TESTS PASSED!\n");
    return 0;
}
