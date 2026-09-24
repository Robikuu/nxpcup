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

    /* Test 1: Normal 2-boundary track (left at x=14, right at x=65).
     * Both vectors reach near the bottom (y=50), but their X coordinates
     * (x=14 and x=65) are strictly outside [18, 62].
     * Expected: Both vectors retained, num_vectors = 2. */
    uint16_t v_normal[8] = {
        14, 50, 16, 10,   // Left lane line
        65, 50, 63, 10    // Right lane line
    };
    size_t n_normal = 2;
    vector_filtering(v_normal, &n_normal);
    printf("  Test 1 (Normal 2 boundaries): num_vectors=%zu\n", n_normal);
    assert(n_normal == 2);
    steering_result_t sr1 = compute_steering_error(v_normal, n_normal);
    printf("    Steering result: found=%u, cte=%.2f, heading=%.2f\n", sr1.found, sr1.cte, sr1.heading_deg);
    assert(sr1.found == 2);

    /* Test 2: Chassis/Bumper artifact alone (x in [30, 45], y in [40, 44]).
     * Expected: Completely filtered out, num_vectors = 0, sr.found = 0. */
    uint16_t v_chassis_only[4] = {
        32, 43, 45, 42
    };
    size_t n_chassis_only = 1;
    vector_filtering(v_chassis_only, &n_chassis_only);
    printf("  Test 2 (Chassis artifact alone): num_vectors=%zu\n", n_chassis_only);
    assert(n_chassis_only == 0);
    steering_result_t sr2 = compute_steering_error(v_chassis_only, n_chassis_only);
    printf("    Steering result: found=%u\n", sr2.found);
    assert(sr2.found == 0);

    /* Test 3: One true left lane boundary + one wheel artifact (x=28, y=45).
     * Expected: Wheel artifact eliminated, 1 true boundary retained. */
    uint16_t v_mixed[8] = {
        14, 50, 16, 12,   // Valid left lane
        28, 48, 30, 42    // Left front tire detection
    };
    size_t n_mixed = 2;
    vector_filtering(v_mixed, &n_mixed);
    printf("  Test 3 (True lane + wheel artifact): num_vectors=%zu\n", n_mixed);
    assert(n_mixed == 1);
    assert(v_mixed[0] == 14 && v_mixed[1] == 50); // Valid left lane kept
    steering_result_t sr3 = compute_steering_error(v_mixed, n_mixed);
    printf("    Steering result: found=%u, cte=%.2f\n", sr3.found, sr3.cte);
    assert(sr3.found == 1);

    /* Test 4: Right lane boundary + chassis tie-rod artifact.
     * Expected: Tie-rod dropped, right boundary retained. */
    uint16_t v_mixed_right[8] = {
        35, 41, 52, 42,   // Chassis tie-rod
        65, 50, 63, 15    // Valid right lane
    };
    size_t n_mixed_right = 2;
    vector_filtering(v_mixed_right, &n_mixed_right);
    printf("  Test 4 (Chassis tie-rod + right lane): num_vectors=%zu\n", n_mixed_right);
    assert(n_mixed_right == 1);
    assert(v_mixed_right[0] == 65 && v_mixed_right[1] == 50); // Valid right lane kept
    steering_result_t sr4 = compute_steering_error(v_mixed_right, n_mixed_right);
    printf("    Steering result: found=%u, cte=%.2f\n", sr4.found, sr4.cte);
    assert(sr4.found == 1);

    /* Test 5: Boundary dipping into exclusion zone (x=25, y=50 to x=25, y=10)
     * Must be trimmed to y=37 (TOP_CAM_EXCLUDE_Y_MIN - 1) and retained! */
    uint16_t v_dipping[8] = {
        25, 50, 25, 10,
        65, 50, 65, 10
    };
    size_t n_dipping = 2;
    vector_filtering(v_dipping, &n_dipping);
    printf("  Test 5 (Lane line dipping into bumper): num_vectors=%zu, y0=%u\n", n_dipping, v_dipping[1]);
    assert(n_dipping == 2);
    assert(v_dipping[1] == (TOP_CAM_EXCLUDE_Y_MIN - 1));

    /* Test 6: In-zone edge cases (x=18, y=38) and (x=62, y=38) -> MUST BE REJECTED. */
    uint16_t v_edges[8] = {
        18, 38, 18, 45,   // Exact min edge
        62, 38, 62, 45    // Exact max edge
    };
    size_t n_edges = 2;
    vector_filtering(v_edges, &n_edges);
    printf("  Test 6 (Exact box boundaries x=18/62, y=38): num_vectors=%zu\n", n_edges);
    assert(n_edges == 0);

    printf("\n>>> ALL EXCLUSION FILTER TESTS PASSED SUCCESSFULLY! <<<\n");
    return 0;
}
