#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include "vectors.h"
#include "Config.h"

int main(void) {
    printf("=== Testing Intersection Detection Logic ===\n");

    /* Case 1: Normal 2-vector straight lane (left at x=14, right at x=64) */
    uint16_t lane_straight[8] = {
        14, 50, 14, 10,   // Left boundary (near_x=14, near_y=50)
        64, 50, 64, 10    // Right boundary (near_x=64, near_y=50)
    };
    intersection_result_t r1 = detect_intersection(lane_straight, 2);
    printf("  Case 1 (Clean 2-vector lane): is_intersection=%u (reason=0x%02x)\n", r1.is_intersection, r1.reason);
    assert(r1.is_intersection == 0);

    /* Case 2: 3-vector straight with center dash (left=14, dash=39, right=64) */
    uint16_t lane_center_dash[12] = {
        14, 50, 14, 10,   // Left boundary (near_x=14)
        39, 45, 39, 25,   // Center dash (near_x=39)
        64, 50, 64, 10    // Right boundary (near_x=64)
    };
    intersection_result_t r2 = detect_intersection(lane_center_dash, 3);
    printf("  Case 2 (3 vectors with center dash): is_intersection=%u (reason=0x%02x)\n", r2.is_intersection, r2.reason);
    // Gated check: count alone (3) must NOT trigger intersection on a normal 50px lane!
    assert(r2.is_intersection == 0);

    /* Case 3: Geometric segment crossing (X-intersection) */
    // Segment A goes from (20, 10) to (60, 50)
    // Segment B goes from (60, 10) to (20, 50)
    uint16_t lane_crossing[8] = {
        20, 10, 60, 50,
        60, 10, 20, 50
    };
    intersection_result_t r3 = detect_intersection(lane_crossing, 2);
    printf("  Case 3 (Geometric crossing segments): is_intersection=%u (reason=0x%02x)\n", r3.is_intersection, r3.reason);
    assert(r3.is_intersection == 1);
    assert(r3.reason & INTERSECTION_REASON_CROSSING);

    /* Case 4: Wide aperture branch (near_x spread > 70px) */
    // Left boundary at x=5, right branch opening to x=78 (spread = 73px > 70px)
    uint16_t lane_fork[12] = {
        5, 50, 8, 10,     // Left boundary (near_x=5)
        40, 50, 42, 10,   // Through boundary (near_x=40)
        78, 48, 65, 10    // Branch peeling East (near_x=78)
    };
    intersection_result_t r4 = detect_intersection(lane_fork, 3);
    printf("  Case 4 (Wide aperture branch): is_intersection=%u (reason=0x%02x)\n", r4.is_intersection, r4.reason);
    assert(r4.is_intersection == 1);
    assert(r4.reason & INTERSECTION_REASON_WIDTH);

    /* Case 5: 4+ vectors in intersection zone */
    uint16_t lane_complex[16] = {
        10, 50, 10, 10,
        30, 50, 30, 10,
        50, 50, 50, 10,
        70, 50, 70, 10
    };
    intersection_result_t r5 = detect_intersection(lane_complex, 4);
    printf("  Case 5 (4+ vectors): is_intersection=%u (reason=0x%02x)\n", r5.is_intersection, r5.reason);
    assert(r5.is_intersection == 1);

    /* Case 6: Perpendicular horizontal crossing line (T-junction / + cross) */
    // Left boundary (14, 50)-(14, 10), Right boundary (64, 50)-(64, 10), Horizontal bar (15, 25)-(65, 25)
    uint16_t lane_crossbar[12] = {
        14, 50, 14, 10,
        64, 50, 64, 10,
        15, 25, 65, 25
    };
    intersection_result_t r6 = detect_intersection(lane_crossbar, 3);
    printf("  Case 6 (Perpendicular crossbar): is_intersection=%u (reason=0x%02x)\n", r6.is_intersection, r6.reason);
    assert(r6.is_intersection == 1);
    assert(r6.reason & INTERSECTION_REASON_HORIZONTAL);

    /* Case 7: Normal sharp corner (2 curved vectors, no crossing, spread ~48px) */
    // Left boundary (18, 50)-(35, 15), Right boundary (66, 50)-(75, 15)
    uint16_t lane_corner[8] = {
        18, 50, 35, 15,
        66, 50, 75, 15
    };
    intersection_result_t r7 = detect_intersection(lane_corner, 2);
    printf("  Case 7 (Normal sharp corner): is_intersection=%u (reason=0x%02x)\n", r7.is_intersection, r7.reason);
    assert(r7.is_intersection == 0);

    printf("  [PASS] All intersection test cases passed!\n");
    return 0;
}
