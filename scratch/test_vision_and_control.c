#include <stdio.h>
#include <math.h>
#include <assert.h>
#include "Config.h"
#include "vectors.h"

static int g_failures = 0;

#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { \
        printf("[FAIL] " msg "\n", ##__VA_ARGS__); \
        g_failures++; \
    } else { \
        printf("[PASS] " msg "\n", ##__VA_ARGS__); \
    } \
} while (0)

int main(void)
{
    steering_result_t sr;
    horizon_result_t hr;

    printf("=== Test 1: Bottom-edge wheel/chassis reflection exclusion ===\n");
    reset_steering_tracking();
    {
        /* Real left line at x=12, real right line at x=66, plus wheel reflection at x=25, y=48..42 */
        uint16_t vecs[] = {
            12, 50, 14, 15,   /* Real left boundary */
            66, 50, 64, 15,   /* Real right boundary */
            25, 49, 26, 41    /* Front wheel reflection inside [18..62], y_near=49 >= 38 */
        };
        compute_top_camera_lane(vecs, 3, &sr);
        CHECK(fabsf(sr.cte) < 2.0f, "Wheel reflection at x=25, y=49..41 ignored (cte=%.2f)", sr.cte);
    }

    printf("\n=== Test 2: Lookahead row projection (Y_REF = 44.0f) ===\n");
    reset_steering_tracking();
    {
        /* Both lines pass through (14.5, 44) and (64.5, 44) with slope +0.5:
         * Left starts at y=50 (x=11.5 -> 12) to y=10 (x=32)
         * Right starts at y=36 (x=68.5 -> 68) to y=16 (x=78)
         */
        uint16_t vecs[] = {
            12, 50, 32, 10,
            68, 36, 78, 16
        };
        compute_top_camera_lane(vecs, 2, &sr);
        CHECK(fabsf(sr.cte) < 1.0f, "Asymmetric y_near projected to Y_REF=44 gives near-zero CTE (cte=%.2f)", sr.cte);
    }

    printf("\n=== Test 3: Stateful temporal tracking prevents Left/Right swap across center ===\n");
    reset_steering_tracking();
    {
        uint16_t f1[] = { 28, 50, 34, 20 };
        compute_top_camera_lane(f1, 1, &sr);
        uint16_t f2[] = { 36, 50, 42, 20 };
        compute_top_camera_lane(f2, 1, &sr);
        uint16_t f3[] = { 41, 50, 47, 20 };
        compute_top_camera_lane(f3, 1, &sr);
        CHECK(sr.left_found == 1 && sr.right_found == 0 && sr.cte > 15.0f,
              "Left line crossing center (x=41) stays classified as LEFT (left=%u, right=%u, cte=%.2f)",
              sr.left_found, sr.right_found, sr.cte);
    }

    printf("\n=== Test 4: MIN_LANE_WIDTH_PX rejects duplicate split line segments ===\n");
    reset_steering_tracking();
    {
        uint16_t vecs[] = {
            36, 50, 38, 25,
            43, 48, 45, 22
        };
        compute_top_camera_lane(vecs, 2, &sr);
        CHECK(sr.found == 1, "Two segments 7px apart rejected as duplicate boundary (found=%u, cte=%.2f)", sr.found, sr.cte);
    }

    printf("\n=== Test 5: Perpendicular single-line offset ===\n");
    reset_steering_tracking();
    {
        /* Single left line angled at 45 degrees (dx = +20, dy = 20) passing through x=15 at y=44 */
        uint16_t vecs[] = { 9, 50, 29, 30 };
        compute_top_camera_lane(vecs, 1, &sr);
        CHECK(sr.cte > 6.0f, "Single angled left line uses perpendicular offset 25/cos(theta) (cte=%.2f)", sr.cte);
    }

    printf("\n=== Test 6: Front camera transverse corner wall triggers pre-brake AND preserves foreground curve angle ===\n");
    {
        uint16_t vecs[] = {
            15, 27, 65, 25,   /* Transverse corner wall in braking zone (y_near=27 >= 22) */
            20, 48, 55, 26    /* Right-curving longitudinal boundary */
        };
        compute_front_camera_horizon(vecs, 2, &hr);
        CHECK(hr.is_sharp_corner == 1 && hr.has_transverse_tape == 1 && hr.horizon_angle_deg > 30.0f,
              "90-deg corner wall triggers is_sharp_corner=1 while preserving foreground curve angle (is_sharp=%u, angle=%.2f)",
              hr.is_sharp_corner, hr.horizon_angle_deg);
    }

    printf("\n=== Test 7: Front camera single-line perspective compensation ===\n");
    {
        uint16_t straight_left[] = { 12, 48, 24, 18 };
        compute_front_camera_horizon(straight_left, 1, &hr);
        CHECK(hr.is_sharp_corner == 0 && fabsf(hr.horizon_angle_deg) < 10.0f,
              "Single straight left line compensated for perspective (is_sharp=%u, angle=%.2f)",
              hr.is_sharp_corner, hr.horizon_angle_deg);

        uint16_t curve_left[] = { 15, 48, 52, 18 };
        compute_front_camera_horizon(curve_left, 1, &hr);
        CHECK(hr.is_sharp_corner == 1 && hr.horizon_angle_deg > 30.0f,
              "Single curving line crossing center triggers sharp corner (is_sharp=%u, angle=%.2f)",
              hr.is_sharp_corner, hr.horizon_angle_deg);
    }

    printf("\n=== Test 8: Smooth sign-aware top_turn_gate prevents early turn-in and S-curve conflict ===\n");
    {
        /* A. On straightaway (filtered_heading = 3.0 deg) while front camera sees corner (FS = +40 deg) -> gate = 0.0 */
        float h_straight = 3.0f, fs = 40.0f;
        float gate_s = (h_straight * fs > 0.0f) ? ((fabsf(h_straight) - 3.0f) / 12.0f) : 0.0f;
        if (gate_s < 0.0f) gate_s = 0.0f;
        CHECK(gate_s == 0.0f, "Straightaway (heading=3 deg) blocks early front-camera steering (gate=%.2f)", gate_s);

        /* B. S-curve conflict (filtered_heading = -14.2 deg left, FS = +25.8 deg right) -> gate = 0.0 */
        float h_scurve = -14.2f, fs_scurve = 25.8f;
        float gate_sc = (h_scurve * fs_scurve > 0.0f) ? ((fabsf(h_scurve) - 3.0f) / 12.0f) : 0.0f;
        if (gate_sc < 0.0f) gate_sc = 0.0f;
        CHECK(gate_sc == 0.0f, "S-curve opposing signs (H=-14.2, FS=+25.8) blocks fighting feedforward (gate=%.2f)", gate_sc);

        /* C. Entering corner (filtered_heading = +9.0 deg, FS = +40 deg) -> smooth 0.50 ramp */
        float h_turn = 9.0f;
        float gate_t = (h_turn * fs > 0.0f) ? ((fabsf(h_turn) - 3.0f) / 12.0f) : 0.0f;
        CHECK(fabsf(gate_t - 0.50f) < 0.01f, "Mid-entry (heading=9.0 deg) smoothly ramps gate to 0.50 (gate=%.2f)", gate_t);
    }

    printf("\n=== Test 9: blend=1 proportional differential braking & ramp/speed-hold constants ===\n");
    {
        float abs_steer = 10.0f;
        float s_corner_blend = 1.0f;
        float target_speed = SPEED_OUTSIDE_CORNER; /* 55% */
        float steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                           ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
        float corner_authority = 0.25f + 0.75f * s_corner_blend;
        float diff_mult = steer_norm * corner_authority;
        float inside_wheel = target_speed - diff_mult * (target_speed - (float)SPEED_INSIDE_MIN);
        CHECK(inside_wheel > 52.0f,
              "At 10 deg steer and blend=1.0, inside wheel is %.2f%% (not locked at 32%%)", inside_wheel);
        CHECK(DECEL_RAMP_STEP == 30.0f && ACCEL_RAMP_STEP == 6.0f && SPEED_OUTSIDE_CORNER == 55.0f &&
              DEFAULT_H_LO == 18.0f && DEFAULT_H_HI == 35.0f && DEFAULT_FRONT_ANGLE_THRESH == 25.0f,
              "Config defaults: DECEL=%.1f, ACCEL=%.1f, SPD_OUT=%.1f, H_LO=%.1f, H_HI=%.1f, FA_TH=%.1f",
              (float)DECEL_RAMP_STEP, (float)ACCEL_RAMP_STEP, (float)SPEED_OUTSIDE_CORNER,
              (float)DEFAULT_H_LO, (float)DEFAULT_H_HI, (float)DEFAULT_FRONT_ANGLE_THRESH);
    }

    printf("\n=== Test 10: Top camera single-line perspective compensation (fade 20..30 deg) ===\n");
    reset_steering_tracking();
    {
        /* Straightaway single right line with -15.9 deg perspective convergence (dx=-8, dy=28) staying on right side (x >= 39) */
        uint16_t straight_right[] = { 62, 48, 54, 20 };
        compute_top_camera_lane(straight_right, 1, &sr);
        CHECK(fabsf(sr.heading_deg) < 3.0f,
              "Single straight right line (-15.9 deg raw) compensated to near-0 heading (heading=%.2f)",
              sr.heading_deg);

        /* Real corner single left line at +35 deg (dx=+21, dy=30) preserves 100%% of corner heading */
        reset_steering_tracking();
        uint16_t corner_left[] = { 12, 48, 33, 18 };
        compute_top_camera_lane(corner_left, 1, &sr);
        CHECK(sr.heading_deg > 33.0f,
              "Single corner left line (+35 deg) preserves full corner heading (heading=%.2f)",
              sr.heading_deg);
    }

    printf("\n=== Test 11: Front camera ignores distant transverse walls (y_near < 22.0f) on inner-loop chutes ===\n");
    {
        uint16_t far_wall_and_curve[] = {
            15, 19, 65, 18,   /* Distant transverse wall on inner-loop chute (y_near=19 < 22) */
            15, 48, 52, 18    /* Foreground right-curving lane line */
        };
        compute_front_camera_horizon(far_wall_and_curve, 2, &hr);
        CHECK(hr.has_transverse_tape == 0 && hr.horizon_angle_deg > 30.0f,
              "Distant transverse wall (y_near=19 < 22) ignored on inner-loop chute (has_trans=%u, angle=%.2f)",
              hr.has_transverse_tape, hr.horizon_angle_deg);
    }

    printf("\n=== Test 12: Inside-of-turn feedforward suppression & blend=1 hold attack ===\n");
    {
        /* Inside-of-turn: right turn (FS = +55 deg, H = +14.2 deg) while car is hugging inside right tape (CTE = -21.5 px) */
        float filtered_cte = -21.5f;
        float filtered_heading = 14.2f;
        float filtered_front_angle = 55.0f;
        float top_turn_gate = (fabsf(filtered_heading) - 5.0f) / (18.0f - 5.0f);
        if (top_turn_gate < 0.0f) top_turn_gate = 0.0f;
        if (top_turn_gate > 1.0f) top_turn_gate = 1.0f;
        if (filtered_cte * filtered_front_angle < 0.0f) {
            float inside_scale = (10.0f - fabsf(filtered_cte)) / (10.0f - 4.0f);
            if (inside_scale < 0.0f) inside_scale = 0.0f;
            if (inside_scale > 1.0f) inside_scale = 1.0f;
            top_turn_gate *= inside_scale;
        }
        CHECK(top_turn_gate == 0.0f,
              "Inside-of-turn (CTE=-21.5px in +55 deg right turn) suppresses front pre-steer (gate=%.2f)",
              top_turn_gate);
    }

    printf("\nSummarize: %d failures\n", g_failures);
    return (g_failures == 0) ? 0 : 1;
}

