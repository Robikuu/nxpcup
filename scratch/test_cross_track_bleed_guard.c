#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include "Config.h"
#include "vectors.h"

int main(void) {
    printf("=================================================================\n");
    printf("  TESTING CROSS-TRACK BLEED GUARD & ROOT ANCHOR DISCRIMINATION  \n");
    printf("=================================================================\n\n");

    /* Scenario 1: Adjacent track line detected only in far distance (ny=22 < 30, fy=15) */
    /* Vector: x0=55, y0=22, x1=58, y1=15. (This is in parallel lane 1.5m away) */
    uint16_t adjacent_lane_vectors[4] = { 55, 22, 58, 15 };
    trajectory_result_t traj_adj = compute_lookahead_curvature(adjacent_lane_vectors, 1);

    printf(">>> TEST 1: Far-field adjacent lane vector rejection for steering\n");
    printf("  Adjacent vector: ny=22, fy=15\n");
    printf("  Result: found=%u, left_root=%u, right_root=%u\n",
           traj_adj.found, traj_adj.left_root_found, traj_adj.right_root_found);
    printf("  Lookahead Urgency (speed governor): %.2f\n", traj_adj.lookahead_urgency);

    /* Urgency can be non-zero for pre-braking, but found MUST be 0 for steering root */
    assert(traj_adj.left_root_found == 0);
    assert(traj_adj.right_root_found == 0);
    assert(traj_adj.found == 0);
    printf("  [PASS] Far-field adjacent lane vector correctly rejected as steering root!\n\n");

    /* Scenario 2: Real foreground lane vector (ny=48 >= 30, fy=20) */
    uint16_t real_lane_vectors[8] = {
        18, 48, 28, 20,
        62, 48, 52, 20
    };
    trajectory_result_t traj_real = compute_lookahead_curvature(real_lane_vectors, 2);

    printf(">>> TEST 2: True foreground lane vectors accepted\n");
    printf("  Real vectors: ny=48\n");
    printf("  Result: found=%u, left_root=%u, right_root=%u\n",
           traj_real.found, traj_real.left_root_found, traj_real.right_root_found);
    assert(traj_real.found == 2);
    assert(traj_real.left_root_found == 1);
    assert(traj_real.right_root_found == 1);
    printf("  [PASS] True foreground lane vectors correctly accepted!\n\n");

    printf("=================================================================\n");
    printf("   >>> ALL CROSS-TRACK BLEED TESTS PASSED! <<<                   \n");
    printf("=================================================================\n");
    return 0;
}
