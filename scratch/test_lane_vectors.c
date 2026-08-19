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
