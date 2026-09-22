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
    printf("  Test 1C (Clean straight track dual): found=%u, angle=%.2f deg, urgency=%.2f\n",
           t_straight.found, t_straight.trajectory_angle_deg, t_straight.lookahead_urgency);
    assert(t_straight.found == 2);
    assert(t_straight.lookahead_urgency <= 0.15f); // Straight track produces near-zero urgency!

    /* Case D: SINGLE straight line converging in perspective (x0=64, y0=50, x1=46, y1=15)
     * In old code: computed 27.2 deg and 100% urgency!
     * With perspective compensation: MUST be near 0 deg and 0% urgency! */
    uint16_t v_single_straight[4] = { 64, 50, 46, 15 };
    trajectory_result_t t_single_straight = compute_lookahead_trajectory(v_single_straight, 1);
    printf("  Test 1D (Single straight line in perspective): found=%u, angle=%.2f deg, urgency=%.2f\n",
           t_single_straight.found, t_single_straight.trajectory_angle_deg, t_single_straight.lookahead_urgency);
    assert(t_single_straight.found == 1);
    assert(fabsf(t_single_straight.trajectory_angle_deg) < 3.0f);
    assert(t_single_straight.lookahead_urgency <= 0.05f);

    /* Case E: SINGLE curving line (x0=64, y0=50, x1=58, y1=15 -> bent 12px right of expected 46.8px)
     * Genuine upcoming curve detected from single line! */
    uint16_t v_single_curve[4] = { 64, 50, 58, 15 };
    trajectory_result_t t_single_curve = compute_lookahead_trajectory(v_single_curve, 1);
    printf("  Test 1E (Single curving line ahead): found=%u, angle=%.2f deg, urgency=%.2f\n",
           t_single_curve.found, t_single_curve.trajectory_angle_deg, t_single_curve.lookahead_urgency);
    assert(t_single_curve.found == 1);
    assert(t_single_curve.trajectory_angle_deg > 10.0f);
    assert(t_single_curve.lookahead_urgency >= 0.25f);

    /* Case F: Upcoming sharp curve ahead dual (ynear=35, slanted at ~25 deg) -> HIGH URGENCY */
    uint16_t v_fwd_curve[8] = {
        16, 35, 30, 14,   // Left curving line
        62, 35, 75, 14    // Right curving line
    };
    trajectory_result_t t_curve = compute_lookahead_trajectory(v_fwd_curve, 2);
    printf("  Test 1F (Upcoming curve ahead dual): found=%u, angle=%.2f deg, urgency=%.2f\n",
           t_curve.found, t_curve.trajectory_angle_deg, t_curve.lookahead_urgency);
    assert(t_curve.found == 2);
    assert(t_curve.lookahead_urgency >= 0.50f); // Triggers pre-braking on straight!

    /* Test 2: Top Camera Steering & New PID Dynamics
     * Verify gains and error weights:
     * KP_STRAIGHT = 0.50f, KP_CORNER = 1.75f
     * K_CTE = 0.50f, K_HEADING = 0.85f */
    printf("\n  Test 2 (PID Calibrations):\n");
    printf("    KP_STRAIGHT: %.2f (expected 0.50)\n", (double)KP_STRAIGHT);
    printf("    KD_STRAIGHT: %.3f (expected 0.120)\n", (double)KD_STRAIGHT);
    printf("    KP_CORNER:   %.2f (expected 1.75)\n", (double)KP_CORNER);
    printf("    KD_CORNER:   %.3f (expected 0.050)\n", (double)KD_CORNER);
    printf("    K_CTE:       %.2f (expected 0.50)\n", (double)DEFAULT_K_CTE);
    printf("    K_HEADING:   %.2f (expected 0.85)\n", (double)DEFAULT_K_HEADING);
    assert(fabsf(KP_STRAIGHT - 0.50f) < 0.01f);
    assert(fabsf(KP_CORNER - 1.75f) < 0.01f);
    assert(fabsf(KD_STRAIGHT - 0.120f) < 0.001f);
    assert(fabsf(KD_CORNER - 0.050f) < 0.001f);
    assert(fabsf(DEFAULT_K_CTE - 0.50f) < 0.01f);
    assert(fabsf(DEFAULT_K_HEADING - 0.85f) < 0.01f);
    assert(fabsf(DIFFERENTIAL_DEADZONE_DEG - 3.5f) < 0.01f);
    assert(fabsf(CORNER_HEADING_THRESHOLD_DEG - 12.0f) < 0.01f);

    /* Straightaway response on 5px error:
     * combined = 0.50 * 5.0 = 2.50
     * steer = KP_STRAIGHT * combined = 0.50 * 2.50 = 1.25 deg */
    float steer_straight = KP_STRAIGHT * (DEFAULT_K_CTE * 5.0f);
    printf("    Straight response on 5px drift: %.2f deg (firm, tight centering)\n", steer_straight);
    assert(steer_straight >= 1.0f && steer_straight <= 2.0f);

    /* Corner response on 25 deg heading (with apex turn-in):
     * combined = 0.85 * 25.0 = 21.25
     * steer = KP_CORNER * combined = 1.75 * 21.25 = 37.19 deg (decisive Ackermann authority!) */
    float steer_corner = KP_CORNER * (DEFAULT_K_HEADING * 25.0f);
    printf("    Corner response on 25 deg heading: %.2f deg (decisive authority within tire grip)\n", steer_corner);
    assert(steer_corner >= 35.0f && steer_corner <= 45.0f);

    printf("\n>>> ALL DUAL-CAMERA & PID ARCHITECTURE TESTS PASSED! <<<\n");
    return 0;
}
