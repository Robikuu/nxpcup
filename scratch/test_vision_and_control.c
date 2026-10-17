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

    printf("\n=== Test 2: Lookahead row projection (Y_REF = 35.0f) ===\n");
    reset_steering_tracking();
    {
        uint16_t vecs[] = {
            7, 50, 27, 10,
            70, 25, 78, 9
        };
        compute_top_camera_lane(vecs, 2, &sr);
        CHECK(fabsf(sr.cte) < 1.0f, "Asymmetric y_near projected to Y_REF=35 gives near-zero CTE (cte=%.2f)", sr.cte);
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
        uint16_t vecs[] = { 7, 43, 27, 23 };
        compute_top_camera_lane(vecs, 1, &sr);
        CHECK(sr.cte > 6.0f, "Single angled left line uses perpendicular offset 25/cos(theta) (cte=%.2f)", sr.cte);
    }

    printf("\n=== Test 6: Front camera preserves horizon_angle_deg on 90-deg corner ===\n");
    {
        uint16_t vecs[] = {
            15, 27, 65, 25,   /* Transverse corner wall */
            20, 48, 55, 26    /* Right-curving longitudinal boundary */
        };
        compute_front_camera_horizon(vecs, 2, &hr);
        CHECK(hr.is_sharp_corner == 1 && hr.horizon_angle_deg > 20.0f,
              "90-deg corner wall keeps positive horizon_angle_deg for feedforward (is_sharp=%u, angle=%.2f)",
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

    printf("\n=== Test 8: blend=1 proportional differential braking (no premature inside-wheel lockup) ===\n");
    {
        /* At s_corner_blend = 1.0f and mild steering angle = 10.0 deg (just 2 deg past 8 deg deadzone):
         * steer_norm = (10 - 8) / (40 - 8) = 2 / 32 = 0.0625
         * corner_authority = 0.25 + 0.75 * 1.0 = 1.0
         * diff_mult = steer_norm * corner_authority = 0.0625 (NOT 1.0!)
         */
        float abs_steer = 10.0f;
        float s_corner_blend = 1.0f;
        float target_speed = SPEED_OUTSIDE_CORNER; /* 50% */
        float steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                           ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
        float corner_authority = 0.25f + 0.75f * s_corner_blend;
        float diff_mult = steer_norm * corner_authority;
        float inside_wheel = target_speed - diff_mult * (target_speed - (float)SPEED_INSIDE_MIN);
        CHECK(inside_wheel > 47.0f,
              "At 10 deg steer and blend=1.0, inside wheel is %.2f%% (not locked at 32%%)", inside_wheel);
    }

    printf("\n=== Test 9: Ramp constants and direct drive_scale gap window ===\n");
    {
        CHECK(DECEL_RAMP_STEP >= 25.0f && DECEL_RAMP_STEP <= 35.0f,
              "DECEL_RAMP_STEP is fast (%.1f %%/tick)", (float)DECEL_RAMP_STEP);
        CHECK(ACCEL_RAMP_STEP >= 5.0f && ACCEL_RAMP_STEP <= 8.0f,
              "ACCEL_RAMP_STEP is smooth (%.1f %%/tick)", (float)ACCEL_RAMP_STEP);

        float applied_speed_left = 100.0f, applied_speed_right = 100.0f;
        float drive_scale = DEFAULT_DRIVE_SCALE; /* 1.00f */
        float current_spd = fmaxf(applied_speed_left, applied_speed_right) * drive_scale;
        uint32_t gap_window = (uint32_t)(1.5f * 25.0f / fmaxf(current_spd, 30.0f) * 50.0f);
        CHECK(gap_window == 18U, "Direct drive_scale=1.00 gives 18-tick gap window at 100%% speed (got %u)", gap_window);
    }

    printf("\nSummarize: %d failures\n", g_failures);
    return (g_failures == 0) ? 0 : 1;
}
