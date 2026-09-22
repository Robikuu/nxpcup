#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>

#include "Config.h"
#include "vectors.h"

int main(void)
{
    printf("=== Testing Dual-Camera Architecture & PID Gains ===\n");

    /* Test 1: Forward Camera Horizon & Ground Filter
     * Case A: Vector floating in upper quarter (y0=5, y1=10 < 13) -> MUST BE REJECTED */
    uint16_t v_ceiling[4] = { 20, 5, 30, 10 };
    trajectory_result_t t_ceil = compute_lookahead_trajectory(v_ceiling, 1);
    printf("  Test 1A (Upper quarter ceiling noise): found=%u, urgency=%.2f\n", t_ceil.found, t_ceil.lookahead_urgency);
    assert(t_ceil.found == 0);
    assert(t_ceil.lookahead_urgency == 0.0f);

    /* Case B: Vector ungrounded in middle of frame (y0=15, y1=22 < 24) -> MUST BE REJECTED */
    uint16_t v_floating[4] = { 25, 15, 35, 22 };
    trajectory_result_t t_float = compute_lookahead_trajectory(v_floating, 1);
    printf("  Test 1B (Floating room clutter Y<24): found=%u, urgency=%.2f\n", t_float.found, t_float.lookahead_urgency);
    assert(t_float.found == 0);
    assert(t_float.lookahead_urgency == 0.0f);

    /* Case C: Valid track pair on ground (ynear=45, sep=46px) -> ACCEPTED */
    uint16_t v_fwd_straight[8] = {
        16, 45, 20, 15,   // Left boundary
        62, 45, 58, 15    // Right boundary
    };
    trajectory_result_t t_straight = compute_lookahead_trajectory(v_fwd_straight, 2);
    printf("  Test 1C (Clean straight track): found=%u, angle=%.2f deg, urgency=%.2f\n",
           t_straight.found, t_straight.trajectory_angle_deg, t_straight.lookahead_urgency);
    assert(t_straight.found == 2);
    assert(t_straight.lookahead_urgency <= 0.15f); // Straight track produces near-zero urgency!

    /* Case D: Upcoming sharp curve ahead (ynear=35, slanted at ~25 deg) -> HIGH URGENCY */
    uint16_t v_fwd_curve[8] = {
        16, 35, 30, 14,   // Left curving line
        62, 35, 75, 14    // Right curving line
    };
    trajectory_result_t t_curve = compute_lookahead_trajectory(v_fwd_curve, 2);
    printf("  Test 1D (Upcoming curve ahead): found=%u, angle=%.2f deg, urgency=%.2f\n",
           t_curve.found, t_curve.trajectory_angle_deg, t_curve.lookahead_urgency);
    assert(t_curve.found == 2);
    assert(t_curve.lookahead_urgency >= 0.50f); // Triggers pre-braking on straight!

    /* Test 2: Top Camera Steering & New PID Dynamics
     * Verify gains and error weights:
     * KP_STRAIGHT = 0.50f, KP_CORNER = 1.45f
     * K_CTE = 0.50f, K_HEADING = 0.60f */
    printf("\n  Test 2 (PID Calibrations):\n");
    printf("    KP_STRAIGHT: %.2f (expected 0.50)\n", (double)KP_STRAIGHT);
    printf("    KD_STRAIGHT: %.3f (expected 0.120)\n", (double)KD_STRAIGHT);
    printf("    KP_CORNER:   %.2f (expected 1.45)\n", (double)KP_CORNER);
    printf("    KD_CORNER:   %.3f (expected 0.080)\n", (double)KD_CORNER);
    printf("    K_CTE:       %.2f (expected 0.50)\n", (double)DEFAULT_K_CTE);
    printf("    K_HEADING:   %.2f (expected 0.60)\n", (double)DEFAULT_K_HEADING);
    assert(fabsf(KP_STRAIGHT - 0.50f) < 0.01f);
    assert(fabsf(KP_CORNER - 1.45f) < 0.01f);
    assert(fabsf(KD_STRAIGHT - 0.120f) < 0.001f);
    assert(fabsf(KD_CORNER - 0.080f) < 0.001f);
    assert(fabsf(DEFAULT_K_CTE - 0.50f) < 0.01f);
    assert(fabsf(DEFAULT_K_HEADING - 0.60f) < 0.01f);

    /* Straightaway response on 5px error:
     * combined = 0.50 * 5.0 = 2.50
     * steer = KP_STRAIGHT * combined = 0.50 * 2.50 = 1.25 deg */
    float steer_straight = KP_STRAIGHT * (DEFAULT_K_CTE * 5.0f);
    printf("    Straight response on 5px drift: %.2f deg (firm, tight centering)\n", steer_straight);
    assert(steer_straight >= 1.0f && steer_straight <= 2.0f);

    /* Corner response on 30 deg heading:
     * combined = 0.60 * 30.0 = 18.00
     * steer = KP_CORNER * combined = 1.45 * 18.00 = 26.10 deg (firm lock, no tire scrub) */
    float steer_corner = KP_CORNER * (DEFAULT_K_HEADING * 30.0f);
    printf("    Corner response on 30 deg heading: %.2f deg (firm lock, within tire adhesion)\n", steer_corner);
    assert(steer_corner >= 24.0f && steer_corner <= 35.0f);

    printf("\n>>> ALL DUAL-CAMERA & PID ARCHITECTURE TESTS PASSED! <<<\n");
    return 0;
}
