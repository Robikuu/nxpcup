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
    assert(fabsf(res.cte - 0.0f) < 0.1f);
    assert(fabsf(res.heading - 0.0f) < 0.01f);
    printf("Test 1 (Straight Lane Centered) PASSED\n");

    // Test 2: Piecewise segment merging on right curve (left at x=10, right split into x=46 and x=48)
    uint16_t piecewise_vecs[3 * 6] = {
        10, 50, 10, 10, 0, 0,
        46, 50, 46, 30, 1, 0,
        48, 30, 48, 10, 2, 0
    };
    res = compute_lane_center(piecewise_vecs, 3);
    assert(res.lines_found == 2);
    // Center: (10 + 47) / 2 = 28.5 -> CTE: 28.5 - 39.0 = -10.5
    assert(fabsf(res.cte - (-10.5f)) < 0.5f);
    printf("Test 2 (Piecewise Segment Merging) PASSED\n");

    // Test 3: Single line apex drift (left line dropped out, right line at x=50)
    // Should use tracked half lane (approx 19px) to project center to 50 - 19 = 31 -> CTE = 31 - 39 = -8.0
    uint16_t single_right_vec[1 * 6] = {
        50, 50, 50, 10, 1, 0
    };
    res = compute_lane_center(single_right_vec, 1);
    assert(res.lines_found == 1);
    assert(fabsf(res.cte - (-8.0f)) < 1.0f);
    printf("Test 3 (Single Line Apex Memory Fallback) PASSED\n");

    // Test 4: Short vertical span rejection (dy < 4.0f)
    reset_lane_tracker();
    uint16_t short_dy_vecs[3 * 6] = {
        20, 50, 20, 10, 0, 0, // valid left line (dy=40)
        58, 23, 58, 20, 1, 0, // dy = 3 < 4.0f (node junction noise)
        30, 20, 50, 20, 2, 0  // dy = 0 < 4.0f (horizontal tape)
    };
    res = compute_lane_center(short_dy_vecs, 3);
    assert(res.lines_found == 1); // Only left line kept
    assert(fabsf(res.cte - 0.0f) < 0.1f); // 20 + 19 - 39 = 0.0
    printf("Test 4 (Short Vertical dy Rejection) PASSED\n");

    // Test 5: Short total length rejection (total length < 8.0f px)
    reset_lane_tracker();
    uint16_t short_len_vecs[2 * 6] = {
        20, 50, 20, 10, 0, 0, // valid left line (dy=40, len=40)
        58, 25, 61, 21, 1, 0  // dy=4, dx=3 -> length = 5.0 < 8.0 px
    };
    res = compute_lane_center(short_len_vecs, 2);
    assert(res.lines_found == 1); // Short vector rejected
    printf("Test 5 (Short Length < 8px Rejection) PASSED\n");

    // Test 6: Crossing line rejection (|slope| > 0.75f)
    reset_lane_tracker();
    uint16_t crossing_vecs[2 * 6] = {
        20, 50, 20, 10, 0, 0, // valid left line
        40, 40, 70, 10, 1, 0  // dy=30, dx=30 -> slope = 1.0 > 0.75 (crossing tape)
    };
    res = compute_lane_center(crossing_vecs, 2);
    assert(res.lines_found == 1);
    printf("Test 6 (Crossing Line Slope > 0.75 Rejection) PASSED\n");

    // Test 7: Lookahead row Y = 22 evaluation with slanted vectors
    reset_lane_tracker();
    // Left: (28, 50) -> (12, 10): dy=40, dx=-16, slope = -0.40. At Y=22: x = 28 + (-0.4)*(50-22) = 16.8
    // Right: (58, 50) -> (58, 10): dy=40, dx=0, slope = 0.0. At Y=22: x = 58.0
    uint16_t slanted_vecs[2 * 6] = {
        28, 50, 12, 10, 0, 0,
        58, 50, 58, 10, 1, 0
    };
    res = compute_lane_center(slanted_vecs, 2);
    assert(res.lines_found == 2);
    // Center = (16.8 + 58.0)/2 = 37.4 -> CTE = 37.4 - 39.0 = -1.6
    assert(fabsf(res.cte - (-1.6f)) < 0.2f);
    assert(fabsf(res.heading - (-0.2f)) < 0.05f);
    printf("Test 7 (Lookahead Row Y=22 Slanted Projection) PASSED\n");

    // Test 8: Valid lane width bounds [28, 52] and EMA half-width update (alpha = 0.15)
    reset_lane_tracker(); // s_tracked_half_lane = 19.0
    // Narrow lane width = 20.0 px (29 to 49) -> Rejected as pair
    uint16_t narrow_vecs[2 * 6] = {
        29, 50, 29, 10, 0, 0,
        49, 50, 49, 10, 1, 0
    };
    res = compute_lane_center(narrow_vecs, 2);
    assert(res.lines_found == 1);

    // Wide lane width = 56.0 px (10 to 66) -> Rejected as pair
    uint16_t wide_vecs[2 * 6] = {
        10, 50, 10, 10, 0, 0,
        66, 50, 66, 10, 1, 0
    };
    res = compute_lane_center(wide_vecs, 2);
    assert(res.lines_found == 1);

    // Valid lane width = 42.0 px (18 to 60) -> half_width = 21.0
    // EMA update: 0.85 * 19.0 + 0.15 * 21.0 = 19.30
    uint16_t valid_width_vecs[2 * 6] = {
        18, 50, 18, 10, 0, 0,
        60, 50, 60, 10, 1, 0
    };
    res = compute_lane_center(valid_width_vecs, 2);
    assert(res.lines_found == 2);
    assert(fabsf(res.cte - 0.0f) < 0.1f);

    // Now single line at left = 20: center = 20 + 19.30 = 39.30 -> CTE = 0.30
    uint16_t single_left[1 * 6] = {
        20, 50, 20, 10, 0, 0
    };
    res = compute_lane_center(single_left, 1);
    assert(res.lines_found == 1);
    assert(fabsf(res.cte - 0.30f) < 0.05f);
    printf("Test 8 (Lane Width Bounds & EMA Half-Width Update) PASSED\n");

    // Test 9: Persistent boundary memory fallback (left vs right discrimination)
    reset_lane_tracker(); // s_last_left_x = 20, s_last_right_x = 58
    // Line at x=22 is closer to left memory (20) than right (58)
    uint16_t near_left_vec[1 * 6] = {
        22, 50, 22, 10, 0, 0
    };
    res = compute_lane_center(near_left_vec, 1);
    assert(res.lines_found == 1);
    // Center = 22 + 19 = 41 -> CTE = 41 - 39 = 2.0
    assert(fabsf(res.cte - 2.0f) < 0.1f);

    // Next frame, line at x=59 is closer to updated right memory (22 + 38 = 60)
    uint16_t near_right_vec[1 * 6] = {
        59, 50, 59, 10, 0, 0
    };
    res = compute_lane_center(near_right_vec, 1);
    assert(res.lines_found == 1);
    // Center = 59 - 19 = 40 -> CTE = 40 - 39 = 1.0
    assert(fabsf(res.cte - 1.0f) < 0.1f);
    printf("Test 9 (Persistent Boundary Memory Discrimination) PASSED\n");

    // Test 10: Empty vector handling
    res = compute_lane_center(NULL, 0);
    assert(res.lines_found == 0);
    assert(res.cte == 0.0f && res.heading == 0.0f);
    printf("Test 10 (Empty Vector Set) PASSED\n");

    printf("All Vision Tests PASSED!\n");
    return 0;
}
