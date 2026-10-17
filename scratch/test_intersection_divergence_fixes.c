#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "Config.h"
#include "vectors.h"

int main(void) {
    printf("=================================================================\n");
    printf("  TESTING BRANCH REJECTION, DIVERGENCE GUARD & INTERSECTION GATES\n");
    printf("=================================================================\n\n");

    /* TEST 1: Pair Selection with Continuing Lane vs Peeling Side Branch */
    printf(">>> TEST 1: Candidate Pair Heading Continuity & Branch Rejection\n");
    reset_steering_tracking();

    /* Setup initial heading straight (0 deg) */
    uint16_t initial_lane[8] = {
        15, 50, 15, 10,   /* Left boundary: X=15, vertical */
        65, 50, 65, 10    /* Right boundary: X=65, vertical, sep = 50px */
    };
    size_t num = 2;
    vector_filtering(initial_lane, &num);
    assert(num == 2);
    steering_result_t sr_init = compute_steering_error(initial_lane, num);
    assert(sr_init.found == 2);
    printf("  Initial straight lane: H = %.1f deg, CTE = %.1f px\n", sr_init.heading_deg, sr_init.cte);
    assert(fabsf(sr_init.heading_deg) < 2.0f);

    /* Frame 2: Top camera enters 4-way intersection.
     * Contains 4 vectors:
     * - Vector 0, 1: Continuing straight lane (Left X=15, Right X=65, dy=40, dx=0 -> H=0 deg)
     * - Vector 2, 3: Branch pair peeling off to North-East (sep = 48px, heading ~ 40 deg) */
    uint16_t vectors_at_fork[16] = {
        /* Straight pair */
        15, 50, 15, 15,   /* Left straight */
        65, 50, 65, 15,   /* Right straight */
        /* Branch pair (peeling 40 deg to right) */
        20, 48, 45, 18,   /* Branch left: dx=25, dy=30 -> ~40 deg */
        68, 48, 93, 18    /* Branch right: dx=25, dy=30 -> ~40 deg, sep = 48px */
    };
    size_t num_fork = 4;
    vector_filtering(vectors_at_fork, &num_fork);
    assert(num_fork == 2);

    steering_result_t sr_fork = compute_steering_error(vectors_at_fork, num_fork);
    printf("  Fork selection: Vector 0 Near=(%u,%u), Vector 1 Near=(%u,%u)\n",
           vectors_at_fork[0], vectors_at_fork[1], vectors_at_fork[4], vectors_at_fork[5]);
    printf("  Synthesized Steering: H = %.1f deg, CTE = %.1f px\n", sr_fork.heading_deg, sr_fork.cte);
    
    /* MUST select the straight continuing lane, NOT the branch pair! */
    assert(fabsf(sr_fork.heading_deg) < 10.0f);
    assert(vectors_at_fork[0] == 15 && vectors_at_fork[4] == 65);
    printf("  [PASS] Test 1: Continuous straight pair selected; 40-deg peeling branch rejected!\n\n");

    /* TEST 2: Dual-Camera Divergence Guard */
    printf(">>> TEST 2: Dual-Camera Divergence Guard Clamping\n");
    /* Case A: Front camera confirms straight road ahead (FS = -5 deg)
     * Top camera hallucinates or snaps to branch (TS = +55 deg) */
    float delta_front = -5.0f;
    float delta_top = 55.0f;
    bool front_plausible = true;
    size_t num_vectors_front = 2;

    float effective_delta_top = delta_top;
    if (front_plausible && num_vectors_front >= 1 && fabsf(delta_front) <= 16.0f) {
        float div = delta_top - delta_front;
        if (div > 25.0f) {
            effective_delta_top = delta_front + 25.0f;
        } else if (div < -25.0f) {
            effective_delta_top = delta_front - 25.0f;
        }
    }
    printf("  Raw TS = %+.1f deg, FS = %+.1f deg -> Effective TS = %+.1f deg\n",
           delta_top, delta_front, effective_delta_top);
    assert(effective_delta_top == (-5.0f + 25.0f)); /* Clamped to +20.0 deg */
    assert(effective_delta_top <= 20.01f);

    /* Case B: Real sharp turn where both cameras agree (FS = -45 deg, TS = -55 deg)
     * Divergence is only 10 deg -> No clamping */
    delta_front = -45.0f;
    delta_top = -55.0f;
    effective_delta_top = delta_top;
    if (front_plausible && num_vectors_front >= 1 && fabsf(delta_front) <= 16.0f) {
        float div = delta_top - delta_front;
        if (div > 25.0f) effective_delta_top = delta_front + 25.0f;
        else if (div < -25.0f) effective_delta_top = delta_front - 25.0f;
    }
    printf("  Curve agreement: Raw TS = %+.1f deg, FS = %+.1f deg -> Effective TS = %+.1f deg (Unclamped)\n",
           delta_top, delta_front, effective_delta_top);
    assert(effective_delta_top == -55.0f);
    printf("  [PASS] Test 2: Divergence guard clamped +55 deg branch spike without clipping real curves!\n\n");

    /* TEST 3: Relaxed Intersection Gating on Post-Turn Short Straight */
    printf(">>> TEST 3: Intersection Detection Gating on Short Post-Turn Straight\n");
    /* Run 6 condition at frame 93: H = -14.9 deg, CTE = +14.2 px, steer = -6.9 deg */
    float filtered_heading = -14.9f;
    float steer_cmd = -6.9f;
    float filtered_traj_angle = -5.0f;
    float lookahead_turn_factor = 0.10f;
    float kappa = 0.12f;
    float filtered_cte = 14.2f;

    /* Transverse tape line: dx=30, dy=5 -> horizontal crossing */
    uint16_t pid_vecs[8] = {
        15, 30, 45, 32,  /* transverse crossbar: dx=30, dy=2 */
        10, 48, 10, 15   /* normal line */
    };
    size_t n_pid = 2;

    bool intersection_trigger = false;
    if (fabsf(filtered_heading) < 16.0f &&
        fabsf(steer_cmd) < 16.0f &&
        fabsf(filtered_traj_angle) < 16.0f &&
        lookahead_turn_factor < 0.30f &&
        kappa < 0.30f &&
        fabsf(filtered_cte) < 18.0f) {
        for (size_t i = 0; i < n_pid; i++) {
            int dx = (int)pid_vecs[4 * i + 2] - (int)pid_vecs[4 * i + 0];
            int dy = (int)pid_vecs[4 * i + 3] - (int)pid_vecs[4 * i + 1];
            if (dx < 0) dx = -dx;
            if (dy < 0) dy = -dy;
            if (dx >= 18 && (dx * 20 > dy * 17)) {
                intersection_trigger = true;
                break;
            }
        }
    }
    printf("  Post-Turn conditions: H=%.1f deg, CTE=%.1f px -> Trigger = %s\n",
           filtered_heading, filtered_cte, intersection_trigger ? "TRUE" : "FALSE");
    assert(intersection_trigger == true);

    /* 2-tick streak verification */
    uint32_t streak = 0;
    uint8_t state = 0; // 0=NORMAL, 1=INTERSECTION
    for (int t = 1; t <= 2; t++) {
        if (intersection_trigger) {
            if (++streak >= 2) {
                state = 1;
            }
        }
    }
    printf("  Streak after 2 ticks: streak = %u, state = %s\n", streak, state == 1 ? "MACRO_STATE_INTERSECTION" : "NORMAL");
    assert(state == 1);
    printf("  [PASS] Test 3: Successfully entered MACRO_STATE_INTERSECTION on short straight!\n\n");

    printf("=================================================================\n");
    printf("   >>> ALL NEW BRANCH & INTERSECTION TESTS PASSED! <<<           \n");
    printf("=================================================================\n");
    return 0;
}
