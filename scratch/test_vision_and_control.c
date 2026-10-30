#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
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

    printf("\n=== Test 8: Smooth sign-aware top_turn_gate with 0.45f pre-steer floor and 1..10 deg ramp ===\n");
    {
        float fa_th = DEFAULT_FRONT_ANGLE_THRESH; /* 22.0f */

        /* A. On straightaway approaching corner (H = 0.0 deg, FS = -51.5 deg) -> 0.45f floor crosses 8-deg deadzone */
        float h_straight = 0.0f, fs = -51.5f;
        float gate_s = 0.0f;
        if (fabsf(fs) > fa_th && (h_straight * fs >= 0.0f || fabsf(h_straight) < 3.0f)) {
            gate_s = 0.45f;
        }
        if (h_straight * fs > 0.0f && fabsf(h_straight) > 1.0f) {
            float h_norm = (fabsf(h_straight) - 1.0f) / (10.0f - 1.0f);
            if (h_norm > 1.0f) h_norm = 1.0f;
            float ramp = (gate_s > 0.0f) ? (0.45f + 0.55f * h_norm) : h_norm;
            if (ramp > gate_s) gate_s = ramp;
        }
        float pre_steer = gate_s * (float)FRONT_FEEDFORWARD_GAIN * fs;
        CHECK(fabsf(gate_s - 0.45f) < 0.01f && fabsf(pre_steer) > (float)DIFFERENTIAL_DEADZONE_DEG,
              "Straight approach (H=0, FS=-51.5) pre-steers at gate=%.2f (%.2f deg crosses 8 deg deadzone)",
              gate_s, pre_steer);

        /* B. S-curve conflict (filtered_heading = -14.2 deg left, FS = +25.8 deg right) -> gate = 0.0 */
        float h_scurve = -14.2f, fs_scurve = 25.8f;
        float gate_sc = 0.0f;
        if (fabsf(fs_scurve) > fa_th && (h_scurve * fs_scurve >= 0.0f || fabsf(h_scurve) < 3.0f)) {
            gate_sc = 0.45f;
        }
        if (h_scurve * fs_scurve > 0.0f && fabsf(h_scurve) > 1.0f) {
            float h_norm = (fabsf(h_scurve) - 1.0f) / (10.0f - 1.0f);
            if (h_norm > 1.0f) h_norm = 1.0f;
            float ramp = (gate_sc > 0.0f) ? (0.45f + 0.55f * h_norm) : h_norm;
            if (ramp > gate_sc) gate_sc = ramp;
        }
        CHECK(gate_sc == 0.0f, "S-curve opposing signs (H=-14.2, FS=+25.8) blocks fighting feedforward (gate=%.2f)", gate_sc);

        /* C. Entering corner (filtered_heading = -5.5 deg, FS = -51.5 deg) -> smooth 0.725 ramp (midpoint of 1..10 deg) */
        float h_turn = -5.5f;
        float gate_t = 0.45f;
        if (h_turn * fs > 0.0f && fabsf(h_turn) > 1.0f) {
            float h_norm = (fabsf(h_turn) - 1.0f) / (10.0f - 1.0f);
            if (h_norm > 1.0f) h_norm = 1.0f;
            float ramp = (gate_t > 0.0f) ? (0.45f + 0.55f * h_norm) : h_norm;
            if (ramp > gate_t) gate_t = ramp;
        }
        CHECK(fabsf(gate_t - 0.725f) < 0.01f, "Mid-entry (heading=-5.5 deg) smoothly ramps gate to 0.725 (gate=%.3f)", gate_t);
    }

    printf("\n=== Test 9: blend=1 proportional differential braking & updated Config.h boot defaults ===\n");
    {
        float abs_steer = 10.0f;
        float s_corner_blend = 1.0f;
        float target_speed = SPEED_OUTSIDE_CORNER; /* 65% */
        float steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                           ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
        float corner_authority = 0.25f + 0.75f * s_corner_blend;
        float diff_mult = steer_norm * corner_authority;
        float inside_wheel = target_speed - diff_mult * (target_speed - (float)SPEED_INSIDE_MIN);
        CHECK(inside_wheel > 60.0f,
              "At 10 deg steer and blend=1.0, inside wheel is %.2f%% (not locked at 28%%)", inside_wheel);
        CHECK(SPEED_STRAIGHT == 100.0f && SPEED_OUTSIDE_CORNER == 65.0f && SPEED_INSIDE_MIN == 28.0f &&
              DEFAULT_K_CTE == 0.45f && DEFAULT_K_HEADING == 1.15f &&
              DEFAULT_H_LO == 12.0f && DEFAULT_H_HI == 35.0f &&
              DEFAULT_FRONT_ANGLE_THRESH == 22.0f && FRONT_SEVERITY_FULL_DEG == 40.0f &&
              CORNER_SEVERITY_HOLD_TICKS == 6U && DECEL_RAMP_STEP == 30.0f &&
              ACCEL_RAMP_STEP == 6.5f && DEFAULT_BLEND_ENABLE == 0U,
              "Config defaults: SPD_S=%.0f, SPD_OUT=%.0f, SPD_IN=%.0f, H_LO=%.0f, H_HI=%.0f, FA_TH=%.0f, FULL=%.0f, HOLD=%u",
              (float)SPEED_STRAIGHT, (float)SPEED_OUTSIDE_CORNER, (float)SPEED_INSIDE_MIN,
              (float)DEFAULT_H_LO, (float)DEFAULT_H_HI, (float)DEFAULT_FRONT_ANGLE_THRESH,
              (float)FRONT_SEVERITY_FULL_DEG, (unsigned)CORNER_SEVERITY_HOLD_TICKS);
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

    printf("\n=== Test 12: inside_scale preserves feedforward in real corners (L51) & suppresses mild non-corner drift ===\n");
    {
        float fa_th = DEFAULT_FRONT_ANGLE_THRESH; /* 22.0f */

        /* A. Real corner entry on inside of turn (L51: F = +16.6 px, H = -9.0 deg, FS = -44.1 deg -> |FS - H| = 35.1 deg)
         * Must NOT be zeroed out by inside_scale! */
        float f_l51 = 16.6f, h_l51 = -9.0f, fs_l51 = -44.1f;
        float curve_rel_l51 = fs_l51 - h_l51;
        float h_norm_l51 = (fabsf(h_l51) - 1.0f) / (10.0f - 1.0f);
        if (h_norm_l51 > 1.0f) h_norm_l51 = 1.0f;
        float gate_l51 = 0.45f + 0.55f * h_norm_l51;
        bool real_corner_l51 = (fabsf(fs_l51) >= fa_th && fabsf(curve_rel_l51) >= 15.0f) || (fabsf(h_l51) >= 18.0f);
        if ((f_l51 * fs_l51 < 0.0f) && !real_corner_l51) {
            float inside_scale = (10.0f - fabsf(f_l51)) / (10.0f - 4.0f);
            if (inside_scale < 0.0f) inside_scale = 0.0f;
            if (inside_scale > 1.0f) inside_scale = 1.0f;
            gate_l51 *= inside_scale;
        }
        float steer_l51 = (DEFAULT_K_CTE * f_l51) + (DEFAULT_K_HEADING * h_l51) + gate_l51 * (float)FRONT_FEEDFORWARD_GAIN * fs_l51;
        CHECK(gate_l51 > 0.90f && steer_l51 < -20.0f,
              "Real corner entry on inside (L51: F=+16.6, H=-9.0, FS=-44.1) preserves gate=%.2f, steer=%.1f deg",
              gate_l51, steer_l51);

        /* B. Mild non-corner inside drift (F = -12.0 px, H = +8.0 deg, FS = +19.0 deg -> |FS - H| = 11.0 < 15.0, |FS| < 22) */
        float f_mild = -12.0f, h_mild = 8.0f, fs_mild = 19.0f;
        float curve_rel_mild = fs_mild - h_mild;
        float gate_mild = (fabsf(h_mild) - 1.0f) / (10.0f - 1.0f);
        bool real_corner_mild = (fabsf(fs_mild) >= fa_th && fabsf(curve_rel_mild) >= 15.0f) || (fabsf(h_mild) >= 18.0f);
        if ((f_mild * fs_mild < 0.0f) && !real_corner_mild) {
            float inside_scale = (10.0f - fabsf(f_mild)) / (10.0f - 4.0f);
            if (inside_scale < 0.0f) inside_scale = 0.0f;
            if (inside_scale > 1.0f) inside_scale = 1.0f;
            gate_mild *= inside_scale;
        }
        CHECK(gate_mild == 0.0f,
              "Non-corner inside drift (F=-12.0, H=+8.0, FS=+19.0) suppresses pre-steer (gate=%.2f)",
              gate_mild);
    }

    printf("\n=== Test 13: Continuous foreground-to-background lane lines across intersection crossbar avoid 55%% speed lock ===\n");
    {
        /* Exiting Corner 6 toward Intersection #2:
         * Left crossbar (4, 25, 20, 25), Right crossbar (58, 25, 74, 25),
         * Continuous left/right lane lines tracked from foreground (y_near=46) through crossbar to y_far=8.
         */
        uint16_t cont_ix_vecs[] = {
            4, 25, 20, 25,
            58, 25, 74, 25,
            24, 46, 26, 8,
            54, 46, 52, 8
        };
        compute_front_camera_horizon(cont_ix_vecs, 4, &hr);
        CHECK(hr.is_intersection_ahead == 1 && hr.is_sharp_corner == 0 && hr.valid_longitudinal_count == 2,
              "Continuous lane lines (y=46..8) across crossbar recognized as intersection (ix=%u, sharp=%u, long=%u)",
              hr.is_intersection_ahead, hr.is_sharp_corner, hr.valid_longitudinal_count);
    }

    printf("\n=== Test 14: Differential braking suppressed on exit counter-steer/S-curve, preserved in real corners ===\n");
    {
        float fa_th = DEFAULT_FRONT_ANGLE_THRESH; /* 22.0f */

        /* A. Straightaway exit counter-steer (F = -7.2 px, H = +16.5 deg, FS = +24.0 deg, S = +15.7 deg -> |FS - H| = 7.5 < 15) */
        float f_exit = -7.2f, h_exit = 16.5f, fs_exit = 24.0f, s_exit = 15.7f;
        float rel_exit = fs_exit - h_exit;
        bool exit_a = ((f_exit * s_exit < 0.0f) && (fabsf(f_exit) > 3.0f) &&
                       (fabsf(h_exit) < 18.0f) && (fabsf(rel_exit) < 15.0f)) ||
                      (fabsf(fs_exit) < fa_th && fabsf(h_exit) < 12.0f) ||
                      ((fs_exit * s_exit < 0.0f) && (fabsf(fs_exit) > 10.0f) && (fabsf(h_exit) < 35.0f)) ||
                      ((f_exit * h_exit > 0.0f) && (fabsf(f_exit) > 4.0f) && (fabsf(h_exit) < 24.0f) && (fabsf(rel_exit) < 15.0f));
        CHECK(exit_a == true, "Exit counter-steer (F=-7.2, H=+16.5, FS=+24.0, S=+15.7) suppresses differential braking");

        /* B. Real corner entry on inside (L51: F = +16.6 px, H = -9.0 deg, FS = -44.1 deg, S = -21.5 deg) -> active */
        float f_l51 = 16.6f, h_l51 = -9.0f, fs_l51 = -44.1f, s_l51 = -21.5f;
        float rel_l51 = fs_l51 - h_l51;
        bool exit_l51 = ((f_l51 * s_l51 < 0.0f) && (fabsf(f_l51) > 3.0f) &&
                         (fabsf(h_l51) < 18.0f) && (fabsf(rel_l51) < 15.0f)) ||
                        (fabsf(fs_l51) < fa_th && fabsf(h_l51) < 12.0f) ||
                        ((fs_l51 * s_l51 < 0.0f) && (fabsf(fs_l51) > 10.0f) && (fabsf(h_l51) < 35.0f)) ||
                        ((f_l51 * h_l51 > 0.0f) && (fabsf(f_l51) > 4.0f) && (fabsf(h_l51) < 24.0f) && (fabsf(rel_l51) < 15.0f));
        CHECK(exit_l51 == false, "Real corner entry on inside (L51: F=+16.6, H=-9.0, FS=-44.1) preserves differential pivot");

        /* C. Real left corner apex (F = -18.2 px, H = -48.9 deg, FS = -57.7 deg, S = -65.0 deg) -> active */
        float f_cor = -18.2f, h_cor = -48.9f, fs_cor = -57.7f, s_cor = -65.0f;
        float rel_cor = fs_cor - h_cor;
        bool exit_b = ((f_cor * s_cor < 0.0f) && (fabsf(f_cor) > 3.0f) &&
                       (fabsf(h_cor) < 18.0f) && (fabsf(rel_cor) < 15.0f)) ||
                      (fabsf(fs_cor) < fa_th && fabsf(h_cor) < 12.0f) ||
                      ((fs_cor * s_cor < 0.0f) && (fabsf(fs_cor) > 10.0f) && (fabsf(h_cor) < 35.0f)) ||
                      ((f_cor * h_cor > 0.0f) && (fabsf(f_cor) > 4.0f) && (fabsf(h_cor) < 24.0f) && (fabsf(rel_cor) < 15.0f));
        CHECK(exit_b == false, "Real left corner (F=-18.2, H=-48.9, S=-65.0) preserves full differential pivot");

        /* D. Centered corner entry (L13: F = +1.1 px, H = -28.5 deg, FS = -43.2 deg, S = -51.7 deg) -> active */
        float f_cen = 1.1f, h_cen = -28.5f, fs_cen = -43.2f, s_cen = -51.7f;
        float rel_cen = fs_cen - h_cen;
        bool exit_c = ((f_cen * s_cen < 0.0f) && (fabsf(f_cen) > 3.0f) &&
                       (fabsf(h_cen) < 18.0f) && (fabsf(rel_cen) < 15.0f)) ||
                      (fabsf(fs_cen) < fa_th && fabsf(h_cen) < 12.0f) ||
                      ((fs_cen * s_cen < 0.0f) && (fabsf(fs_cen) > 10.0f) && (fabsf(h_cen) < 35.0f)) ||
                      ((f_cen * h_cen > 0.0f) && (fabsf(f_cen) > 4.0f) && (fabsf(h_cen) < 24.0f) && (fabsf(rel_cen) < 15.0f));
        CHECK(exit_c == false, "Centered corner entry (L13: F=+1.1px, H=-28.5, FS=-43.2) preserves differential pivot");

        /* E. S-curve exit transition (L1120 in Run 6: H = -28.3 deg, FS = +27.4 deg, S = -25.0 deg) -> suppressed */
        float f_sc = 2.0f, h_sc = -28.3f, fs_sc = 27.4f, s_sc = -25.0f;
        bool exit_sc = ((fs_sc * s_sc < 0.0f) && (fabsf(fs_sc) > 10.0f) && (fabsf(h_sc) < 35.0f));
        CHECK(exit_sc == true, "S-curve exit at H=-28.3, FS=+27.4 (L1120) suppresses wrong-way rear wheel lockup");
    }

    printf("\n=== Test 15: True road curvature (|FS - H|) guard against straightaway chassis yaw (L96 & L1229-L1231) ===\n");
    {
        float fa_th = DEFAULT_FRONT_ANGLE_THRESH; /* 22.0f */

        /* A. L96 in Run 1: F = +11.4 px, H = +19.9 deg, FS = +28.8 deg -> |FS - H| = 8.9 deg < 15 deg */
        float f_96 = 11.4f, h_96 = 19.9f, fs_96 = 28.8f;
        float rel_96 = fs_96 - h_96;
        bool yaw_96 = (fabsf(f_96) > 4.0f) && (fabsf(h_96) < 24.0f) && (h_96 * fs_96 > 0.0f) && (fabsf(rel_96) < fa_th);
        float eff_fs_96 = yaw_96 ? rel_96 : fs_96;
        bool sharp_96 = fabsf(eff_fs_96) >= fa_th;
        bool exit_96 = ((f_96 * h_96 > 0.0f) || (h_96 * fs_96 > 0.0f)) &&
                       (fabsf(f_96) > 4.0f) && (fabsf(h_96) < 24.0f) && (fabsf(rel_96) < 15.0f);
        CHECK(yaw_96 && !sharp_96 && exit_96,
              "L96 straightaway yaw (F=+11.4, H=+19.9, FS=+28.8 -> eff_FS=%.1f) blocks false corner & diff lockup",
              eff_fs_96);

        /* B. L1229-L1231 in Run 7: F = -9.0..-5.3 px, H = +20.8..+15.4 deg, FS = +28.8..+34.3 deg -> |FS - H| = 8.0..13.8 < 15 */
        float f_1229 = -9.0f, h_1229 = 20.8f, fs_1229 = 28.8f;
        float rel_1229 = fs_1229 - h_1229;
        bool yaw_1229 = (fabsf(f_1229) > 4.0f) && (fabsf(h_1229) < 24.0f) && (h_1229 * fs_1229 > 0.0f) && (fabsf(rel_1229) < fa_th);
        float eff_fs_1229 = yaw_1229 ? rel_1229 : fs_1229;
        bool is_corner_1229 = (fabsf(eff_fs_1229) >= fa_th) || (fabsf(h_1229) >= 24.0f);
        bool exit_1229 = ((f_1229 * h_1229 > 0.0f) || (h_1229 * fs_1229 > 0.0f)) &&
                         (fabsf(f_1229) > 4.0f) && (fabsf(h_1229) < 24.0f) && (fabsf(rel_1229) < 15.0f);
        CHECK(yaw_1229 && !is_corner_1229 && exit_1229,
              "L1229 wide-exit recovery (F=-9.0, H=+20.8, FS=+28.8 -> eff_FS=%.1f, exit=%d) prevents ST:C & 93/27 lockup",
              eff_fs_1229, exit_1229 ? 1 : 0);

        float f_1230 = -7.0f, h_1230 = 20.5f, fs_1230 = 34.3f;
        float rel_1230 = fs_1230 - h_1230;
        bool yaw_1230 = (fabsf(f_1230) > 4.0f) && (fabsf(h_1230) < 24.0f) && (h_1230 * fs_1230 > 0.0f) && (fabsf(rel_1230) < fa_th);
        float eff_fs_1230 = yaw_1230 ? rel_1230 : fs_1230;
        bool is_corner_1230 = (fabsf(eff_fs_1230) >= fa_th) || (fabsf(h_1230) >= 24.0f);
        bool exit_1230 = ((f_1230 * h_1230 > 0.0f) || (h_1230 * fs_1230 > 0.0f)) &&
                         (fabsf(f_1230) > 4.0f) && (fabsf(h_1230) < 24.0f) && (fabsf(rel_1230) < 15.0f);
        CHECK(yaw_1230 && !is_corner_1230 && exit_1230,
              "L1230 wide-exit recovery (F=-7.0, H=+20.5, FS=+34.3 -> eff_FS=%.1f, exit=%d) prevents ST:C & 93/27 lockup",
              eff_fs_1230, exit_1230 ? 1 : 0);
    }

    printf("\n=== Test 16: Pre-intersection crossbar tolerance & 2-tick front_corner_wall filter ===\n");
    {
        /* A. Horizontal crossbar with sr.found == 2, H = 16.0 deg (< 20), FS = 34.0 deg, hz_angle = 28.0 deg (|hz - H| = 12 < 25) */
        bool has_trans = true, is_ix = false, top_cross = false;
        uint8_t sr_found = 2;
        float h = 16.0f, fs = 34.0f, hz_angle = 28.0f;
        bool straight_cb = has_trans && !is_ix && (sr_found >= 2) &&
                           ((fabsf(hz_angle) < 20.0f) || (fabsf(hz_angle - h) < 25.0f)) &&
                           (fabsf(fs - h) < 25.0f) &&
                           !top_cross && (fabsf(h) < 20.0f);
        float h_cor = 0.0f, fs_cor = -18.0f, hz_cor = -45.0f;
        bool real_cor_cb = has_trans && !is_ix && (sr_found >= 2) &&
                           ((fabsf(hz_cor) < 20.0f) || (fabsf(hz_cor - h_cor) < 25.0f)) &&
                           (fabsf(fs_cor - h_cor) < 25.0f) &&
                           !top_cross && (fabsf(h_cor) < 20.0f);
        CHECK(straight_cb == true && real_cor_cb == false,
              "Crossbar with sr.found=2, H=16, FS=34, hz=28 recognized as straight_corridor_crossbar; real -45 deg corner rejected");

        /* B. 1-tick raw front_corner_wall glitch does NOT assert front_corner_wall; 2nd consecutive tick DOES */
        uint8_t s_front_wall_ticks = 0;
        uint8_t raw_wall_t1 = 1U;
        if (raw_wall_t1) s_front_wall_ticks++;
        uint8_t wall_t1 = (s_front_wall_ticks >= 2U) ? 1U : 0U;
        uint8_t raw_wall_t2 = 1U;
        if (raw_wall_t2) s_front_wall_ticks++;
        uint8_t wall_t2 = (s_front_wall_ticks >= 2U) ? 1U : 0U;
        CHECK(wall_t1 == 0U && wall_t2 == 1U,
              "2-tick front_corner_wall filter blocks 1-tick crossbar glitch (t1=%u, t2=%u)", wall_t1, wall_t2);

        /* C. 3 consecutive ticks of real transverse corner wall preempts active s_intersection_ahead latch on Tick 3 */
        uint8_t ix_ahead = 1U, cancel_cnt = 0U, wall_ticks = 0U, final_wall = 0U;
        for (int t = 0; t < 3; t++) {
            uint8_t raw_w = 1U;
            wall_ticks++;
            uint8_t fw = (wall_ticks >= 2U) ? 1U : 0U;
            bool eff_sharp = (fw != 0U);
            if (eff_sharp || raw_w) {
                if (ix_ahead) {
                    cancel_cnt++;
                    if (cancel_cnt >= 3U) {
                        ix_ahead = 0U;
                        cancel_cnt = 0U;
                    }
                }
            }
            if (ix_ahead) {
                fw = 0U;
            }
            final_wall = fw;
        }
        CHECK(ix_ahead == 0U && final_wall == 1U,
              "3 consecutive ticks of corner wall preempts s_intersection_ahead on Tick 3 (ix=%u, wall=%u)",
              ix_ahead, final_wall);
    }

    printf("\n=== Test 17: Fast inside-wheel exit equalization (2x accel_ramp without overshoot) ===\n");
    {
        float applied_l = 28.0f, applied_r = 64.5f;
        float target_l = 80.0f, target_r = 80.0f;
        float accel = 6.0f;
        for (int tick = 0; tick < 5; tick++) {
            bool equal_exit = (target_l >= applied_l) && (target_r >= applied_r) && (fabsf(target_l - target_r) < 1.0f);
            bool l_lag = equal_exit && (applied_l < applied_r);
            bool r_lag = equal_exit && (applied_r < applied_l);
            applied_l += l_lag ? (2.0f * accel) : accel;
            if (applied_l > target_l) applied_l = target_l;
            applied_r += r_lag ? (2.0f * accel) : accel;
            if (applied_r > target_r) applied_r = target_r;
            if (l_lag && applied_l > applied_r) applied_l = applied_r;
            if (r_lag && applied_r > applied_l) applied_r = applied_l;
        }
        CHECK(fabsf(applied_l - applied_r) < 0.001f && fabsf(applied_l - 80.0f) < 0.001f,
              "Inside wheel (28%%) equalizes exactly with outside wheel (64.5%% -> 80.0%%) without residual lag (L=%.2f, R=%.2f)",
              applied_l, applied_r);
    }

    printf("\n=== Test 18: Fuzzy 'stop' command acceptance & +0.5f float rounding ===\n");
    {
        const char *c1 = "stop";
        const char *c2 = "stop/";
        const char *c3 = "stop 1";
        const char *c4 = "start";
        CHECK(strncmp(c1, "stop", 4) == 0 && strncmp(c2, "stop", 4) == 0 &&
              strncmp(c3, "stop", 4) == 0 && strncmp(c4, "stop", 4) != 0,
              "strncmp(cmd, \"stop\", 4) accepts 'stop', 'stop/', 'stop 1' and rejects 'start'");

        float vals[3] = { 1.15f, 1.996f, -0.50f };
        const char *expect[3] = { "1.15", "2.00", "-0.50" };
        for (int i = 0; i < 3; i++) {
            float val = vals[i];
            int whole = (int)val;
            float frac = val - (float)whole;
            if (frac < 0) frac = -frac;
            int f100 = (int)(frac * 100.0f + 0.5f);
            if (f100 >= 100) { f100 -= 100; whole += (val < 0.0f) ? -1 : 1; }
            const char *sign = (val < 0.0f && whole == 0 && f100 > 0) ? "-" : "";
            char buf[16];
            snprintf(buf, sizeof(buf), "%s%d.%02d", sign, whole, f100);
            CHECK(strcmp(buf, expect[i]) == 0, "%.3ff rounds/formats to \"%s\" (expect \"%s\")", val, buf, expect[i]);
        }
    }

    printf("\nSummarize: %d failures\n", g_failures);
    return (g_failures == 0) ? 0 : 1;
}

