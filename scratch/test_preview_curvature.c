#define _USE_MATH_DEFINES
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define MAX_FILTER_VECTORS 16
#define PIXY_CENTER_X 39.5f
#define FORWARD_CAM_HORIZON_Y_MIN 13U
#define FORWARD_CAM_SAMPLE_HORIZON_Y 22.0f
#define LOOKAHEAD_DEADBAND_PX 3.5f
#define LOOKAHEAD_FULL_URGENCY_PX 12.0f

typedef struct {
    float trajectory_angle_deg;
    float trajectory_offset_px;
    float lookahead_x;
    float curvature;
    float lookahead_urgency;
    uint8_t left_root_found;
    uint8_t right_root_found;
    uint8_t found;
} trajectory_result_t;

// Prototype to be implemented in source/vectors.c
trajectory_result_t compute_lookahead_curvature(const uint16_t *vectors, size_t num_vectors)
{
    trajectory_result_t result;
    result.trajectory_angle_deg = 0.0f;
    result.trajectory_offset_px = 0.0f;
    result.lookahead_x          = (float)PIXY_CENTER_X;
    result.curvature            = 0.0f;
    result.lookahead_urgency    = 0.0f;
    result.left_root_found      = 0;
    result.right_root_found     = 0;
    result.found                = 0;

    if (num_vectors == 0) return result;

    size_t max_in = (num_vectors > (size_t)MAX_FILTER_VECTORS) ? (size_t)MAX_FILTER_VECTORS : num_vectors;
    size_t count = 0;
    float nx[MAX_FILTER_VECTORS], ny[MAX_FILTER_VECTORS], fx[MAX_FILTER_VECTORS], fy[MAX_FILTER_VECTORS];
    float len[MAX_FILTER_VECTORS];
    bool  transverse_wall_detected = false;
    float transverse_wall_len = 0.0f;
    float transverse_wall_mid_x = (float)PIXY_CENTER_X;

    for (size_t i = 0; i < max_in; i++) {
        float x0 = (float)vectors[4 * i + 0];
        float y0 = (float)vectors[4 * i + 1];
        float x1 = (float)vectors[4 * i + 2];
        float y1 = (float)vectors[4 * i + 3];

        float n_x, n_y, f_x, f_y;
        if (y0 >= y1) {
            n_x = x0;  n_y = y0;  f_x = x1;  f_y = y1;
        } else {
            n_x = x1;  n_y = y1;  f_x = x0;  f_y = y0;
        }

        /* 1. Upper ceiling discard: if both endpoints are in the upper quarter (y < 13), clutter */
        if (n_y < (float)FORWARD_CAM_HORIZON_Y_MIN && f_y < (float)FORWARD_CAM_HORIZON_Y_MIN) {
            continue;
        }

        /* 2. Check for transverse horizontal wall across track (90 deg corner boundary ahead) */
        float raw_dx = fabsf(x1 - x0);
        float raw_dy = fabsf(y1 - y0);
        if (raw_dx >= 16.0f && raw_dy <= 10.0f && n_y >= (float)FORWARD_CAM_HORIZON_Y_MIN && n_y <= 48.0f) {
            transverse_wall_detected = true;
            if (raw_dx > transverse_wall_len) {
                transverse_wall_len   = raw_dx;
                transverse_wall_mid_x = (x0 + x1) * 0.5f;
            }
        }

        /* 3. Clip far endpoint if extending into ceiling (y < 13) */
        if (f_y < (float)FORWARD_CAM_HORIZON_Y_MIN) {
            float total_dy = n_y - f_y;
            if (total_dy > 0.5f) {
                float t = (n_y - (float)FORWARD_CAM_HORIZON_Y_MIN) / total_dy;
                f_x = n_x + t * (f_x - n_x);
            }
            f_y = (float)FORWARD_CAM_HORIZON_Y_MIN;
        }

        float dx = f_x - n_x;
        float dy = n_y - f_y;

        /* Reject backward or flat vectors with near-zero dy */
        if (dy < 4.0f) {
            continue;
        }

        nx[count] = n_x;
        ny[count] = n_y;
        fx[count] = f_x;
        fy[count] = f_y;
        len[count] = sqrtf(dx * dx + dy * dy);
        count++;
    }

    if (transverse_wall_detected) {
        result.found = 2;
        result.lookahead_urgency = 1.0f;
        result.curvature = 1.0f;
        if (transverse_wall_mid_x > (float)PIXY_CENTER_X + 2.0f) {
            result.lookahead_x = 70.0f;
            result.trajectory_angle_deg = 45.0f;
        } else {
            result.lookahead_x = 10.0f;
            result.trajectory_angle_deg = -45.0f;
        }
        result.trajectory_offset_px = result.lookahead_x - (float)PIXY_CENTER_X;
        return result;
    }

    if (count == 0) {
        return result;
    }

    const float sample_y = (float)FORWARD_CAM_SAMPLE_HORIZON_Y; // 22.0f

    int best_left_idx = -1;
    float best_left_score = -99999.0f;
    int best_right_idx = -1;
    float best_right_score = -99999.0f;

    for (size_t i = 0; i < count; i++) {
        float dy = ny[i] - fy[i];
        float x_sample = nx[i];
        if (dy > 0.5f) {
            float t = (ny[i] - sample_y) / dy;
            x_sample = nx[i] + t * (fx[i] - nx[i]);
        }

        float score = len[i] + ny[i];

        if (x_sample < (float)PIXY_CENTER_X) {
            if (score > best_left_score) {
                best_left_score = score;
                best_left_idx = (int)i;
            }
        } else {
            if (score > best_right_score) {
                best_right_score = score;
                best_right_idx = (int)i;
            }
        }
    }

    if (best_left_idx >= 0 && best_right_idx >= 0) {
        result.left_root_found  = 1;
        result.right_root_found = 1;
        result.found            = 2;

        float l_dy = ny[best_left_idx] - fy[best_left_idx];
        float xl_sample = nx[best_left_idx];
        if (l_dy > 0.5f) {
            float t_l = (ny[best_left_idx] - sample_y) / l_dy;
            xl_sample = nx[best_left_idx] + t_l * (fx[best_left_idx] - nx[best_left_idx]);
        }

        float r_dy = ny[best_right_idx] - fy[best_right_idx];
        float xr_sample = nx[best_right_idx];
        if (r_dy > 0.5f) {
            float t_r = (ny[best_right_idx] - sample_y) / r_dy;
            xr_sample = nx[best_right_idx] + t_r * (fx[best_right_idx] - nx[best_right_idx]);
        }

        result.lookahead_x = (xl_sample + xr_sample) * 0.5f;
        result.trajectory_offset_px = result.lookahead_x - (float)PIXY_CENTER_X;

        float near_mid_x = (nx[best_left_idx] + nx[best_right_idx]) * 0.5f;
        float near_mid_y = (ny[best_left_idx] + ny[best_right_idx]) * 0.5f;
        float mid_dx = result.lookahead_x - near_mid_x;
        float mid_dy = near_mid_y - sample_y;
        if (mid_dy > 0.5f) {
            result.trajectory_angle_deg = atan2f(mid_dx, mid_dy) * (180.0f / (float)M_PI);
        }

        float abs_curvature = fabsf(mid_dx);
        if (abs_curvature > (float)LOOKAHEAD_DEADBAND_PX) {
            result.lookahead_urgency = (abs_curvature - (float)LOOKAHEAD_DEADBAND_PX) /
                                       ((float)LOOKAHEAD_FULL_URGENCY_PX - (float)LOOKAHEAD_DEADBAND_PX);
            if (result.lookahead_urgency > 1.0f) result.lookahead_urgency = 1.0f;
        } else {
            result.lookahead_urgency = 0.0f;
        }
        result.curvature = result.lookahead_urgency;
        return result;
    }

    int single_idx = (best_left_idx >= 0) ? best_left_idx : best_right_idx;
    if (single_idx >= 0) {
        result.found = 1;
        float s_dy = ny[single_idx] - fy[single_idx];
        float s_dx = fx[single_idx] - nx[single_idx];
        if (s_dy > 0.5f) {
            result.trajectory_angle_deg = atan2f(s_dx, s_dy) * (180.0f / (float)M_PI);
            float t = (ny[single_idx] - sample_y) / s_dy;
            float x_at_sample = nx[single_idx] + t * s_dx;
            if (single_idx == best_left_idx) {
                result.left_root_found = 1;
                result.lookahead_x = x_at_sample + 17.0f;
            } else {
                result.right_root_found = 1;
                result.lookahead_x = x_at_sample - 17.0f;
            }
            result.trajectory_offset_px = result.lookahead_x - (float)PIXY_CENTER_X;
            float abs_offset = fabsf(result.trajectory_offset_px);
            if (abs_offset > (float)LOOKAHEAD_DEADBAND_PX) {
                result.lookahead_urgency = (abs_offset - (float)LOOKAHEAD_DEADBAND_PX) /
                                           ((float)LOOKAHEAD_FULL_URGENCY_PX - (float)LOOKAHEAD_DEADBAND_PX);
                if (result.lookahead_urgency > 1.0f) result.lookahead_urgency = 1.0f;
            }
            result.curvature = result.lookahead_urgency;
        }
    }

    return result;
}

int main(void) {
    printf("Running preview curvature tests...\n");

    // Test 1: Curve ahead in mid-far horizon (Y between 16 and 28, NO vectors at Y >= 30)
    // Left boundary: (15, 28) -> (10, 16)
    // Right boundary: (45, 28) -> (28, 16) -- strongly curving LEFT
    uint16_t curve_left[8] = {15, 28, 10, 16,  45, 28, 28, 16};
    trajectory_result_t r1 = compute_lookahead_curvature(curve_left, 2);
    printf("Test 1 (Far Curve Left): urgency=%.2f, angle=%.1f deg, found=%d\n",
           r1.lookahead_urgency, r1.trajectory_angle_deg, r1.found);
    assert(r1.found >= 1);
    assert(r1.lookahead_urgency > 0.60f);
    assert(r1.trajectory_angle_deg < -10.0f);

    // Test 2: Straightaway ahead (centered lines at Y=15..35)
    uint16_t straight[8] = {25, 35, 28, 15,  55, 35, 52, 15};
    trajectory_result_t r2 = compute_lookahead_curvature(straight, 2);
    printf("Test 2 (Straight): urgency=%.2f, angle=%.1f deg, found=%d\n",
           r2.lookahead_urgency, r2.trajectory_angle_deg, r2.found);
    assert(r2.lookahead_urgency < 0.20f);

    // Test 3: Transverse tape wall ahead (90 deg turn wall: dx=32, dy=2 at Y=24)
    uint16_t wall[4] = {20, 24, 52, 22};
    trajectory_result_t r3 = compute_lookahead_curvature(wall, 1);
    printf("Test 3 (Transverse Wall): urgency=%.2f, found=%d\n",
           r3.lookahead_urgency, r3.found);
    assert(r3.lookahead_urgency >= 0.95f);

    printf("ALL PREVIEW CURVATURE TESTS PASSED!\n");
    return 0;
}
