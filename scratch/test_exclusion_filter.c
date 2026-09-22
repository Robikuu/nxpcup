#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>

#include "Config.h"
#include "vectors.h"

int main(void)
{
    printf("=== Testing Top Camera Chassis Exclusion Filter ===\n");

    /* Test 1: Normal 2-boundary track (left at x=15, right at x=62).
     * Both vectors reach near the bottom (y=50), but their X coordinates
     * (x=15 and x=62) are strictly outside [24, 56].
     * Expected: Both vectors retained, num_vectors = 2. */
    uint16_t v_normal[8] = {
        15, 50, 16, 10,   // Left lane line
        62, 50, 60, 10    // Right lane line
    };
    size_t n_normal = 2;
    vector_filtering(v_normal, &n_normal);
    printf("  Test 1 (Normal 2 boundaries): num_vectors=%zu\n", n_normal);
    assert(n_normal == 2);
    steering_result_t sr1 = compute_steering_error(v_normal, n_normal);
    printf("    Steering result: found=%u, cte=%.2f, heading=%.2f\n", sr1.found, sr1.cte, sr1.heading_deg);
    assert(sr1.found == 2);

    /* Test 2: Chassis/Bumper artifact alone (x in [30, 40], y in [47, 50]).
     * Expected: Completely filtered out, num_vectors = 0, sr.found = 0. */
    uint16_t v_chassis_only[4] = {
        32, 49, 45, 50
    };
    size_t n_chassis_only = 1;
    vector_filtering(v_chassis_only, &n_chassis_only);
    printf("  Test 2 (Chassis artifact alone): num_vectors=%zu\n", n_chassis_only);
    assert(n_chassis_only == 0);
    steering_result_t sr2 = compute_steering_error(v_chassis_only, n_chassis_only);
    printf("    Steering result: found=%u\n", sr2.found);
    assert(sr2.found == 0);

    /* Test 3: One true left lane boundary + one chassis/wheel artifact.
     * Expected: Chassis artifact eliminated, 1 true boundary retained. */
    uint16_t v_mixed[8] = {
        16, 50, 18, 12,   // Valid left lane
        28, 48, 30, 51    // Left front tire detection
    };
    size_t n_mixed = 2;
    vector_filtering(v_mixed, &n_mixed);
    printf("  Test 3 (True lane + wheel artifact): num_vectors=%zu\n", n_mixed);
    assert(n_mixed == 1);
    assert(v_mixed[0] == 16 && v_mixed[1] == 50); // Valid left lane kept
    steering_result_t sr3 = compute_steering_error(v_mixed, n_mixed);
    printf("    Steering result: found=%u, cte=%.2f\n", sr3.found, sr3.cte);
    assert(sr3.found == 1);

    /* Test 4: Right lane boundary + chassis crossbar artifact.
     * Expected: Crossbar dropped, right boundary retained. */
    uint16_t v_mixed_right[8] = {
        35, 48, 52, 49,   // Chassis crossbar
        61, 50, 59, 15    // Valid right lane
    };
    size_t n_mixed_right = 2;
    vector_filtering(v_mixed_right, &n_mixed_right);
    printf("  Test 4 (Chassis crossbar + right lane): num_vectors=%zu\n", n_mixed_right);
    assert(n_mixed_right == 1);
    assert(v_mixed_right[0] == 61 && v_mixed_right[1] == 50); // Valid right lane kept
    steering_result_t sr4 = compute_steering_error(v_mixed_right, n_mixed_right);
    printf("    Steering result: found=%u, cte=%.2f\n", sr4.found, sr4.cte);
    assert(sr4.found == 1);

    /* Test 5: Boundary edges near bottom but right next to exclusion box.
     * Vector A: x=23 (just outside x_min=24) paired with x=65 (sep=42 >= 40) -> KEPT.
     * Vector B: x=57 (just outside x_max=56) paired with x=15 (sep=42 >= 40) -> KEPT. */
    uint16_t v_margin_left[8] = {
        23, 48, 23, 10,
        65, 48, 65, 10
    };
    size_t n_margin_left = 2;
    vector_filtering(v_margin_left, &n_margin_left);
    printf("  Test 5a (Margin left x=23 with x=65): num_vectors=%zu\n", n_margin_left);
    assert(n_margin_left == 2);

    uint16_t v_margin_right[8] = {
        15, 48, 15, 10,
        57, 48, 57, 10
    };
    size_t n_margin_right = 2;
    vector_filtering(v_margin_right, &n_margin_right);
    printf("  Test 5b (Margin right x=57 with x=15): num_vectors=%zu\n", n_margin_right);
    assert(n_margin_right == 2);

    /* Test 6: In-zone edge cases (x=24, y=46) and (x=56, y=46) -> MUST BE REJECTED. */
    uint16_t v_edges[8] = {
        24, 46, 24, 50,   // Exact min edge
        56, 46, 56, 50    // Exact max edge
    };
    size_t n_edges = 2;
    vector_filtering(v_edges, &n_edges);
    printf("  Test 6 (Exact box boundaries x=24/56, y=46): num_vectors=%zu\n", n_edges);
    assert(n_edges == 0);

    printf("\n>>> ALL EXCLUSION FILTER TESTS PASSED SUCCESSFULLY! <<<\n");
    return 0;
}
