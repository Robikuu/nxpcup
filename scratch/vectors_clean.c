#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "vectors.h"
#include "Config.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

#define MAX_FILTER_VECTORS 32

/* ---- Helper Utilities ---- */
static inline float clamp_f(float val, float min_val, float max_val)
{
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

static inline float project_x_at_y(float nx, float ny, float fx, float fy, float target_y)
{
    float dy = ny - fy;
    if (dy > 0.5f) {
        float t = (ny - target_y) / dy;
        return clamp_f(nx + t * (fx - nx), 0.0f, (float)PIXY_FRAME_WIDTH);
    }
    return clamp_f(nx, 0.0f, (float)PIXY_FRAME_WIDTH);
}

/* ---- Temporal Tracking Memory ---- */
static float s_last_left_near_x       = (PIXY_CENTER_X - PIXY_TRACK_HALF_WIDTH);
static float s_last_right_near_x      = (PIXY_CENTER_X + PIXY_TRACK_HALF_WIDTH);
static float s_last_heading           = 0.0f;
static float s_last_lookahead_left_x  = (PIXY_CENTER_X - PIXY_TRACK_HALF_WIDTH);
static float s_last_lookahead_right_x = (PIXY_CENTER_X + PIXY_TRACK_HALF_WIDTH);

void reset_steering_tracking(void)
{
    s_last_left_near_x       = (float)(PIXY_CENTER_X - PIXY_TRACK_HALF_WIDTH);
    s_last_right_near_x      = (float)(PIXY_CENTER_X + PIXY_TRACK_HALF_WIDTH);
    s_last_heading           = 0.0f;
    s_last_lookahead_left_x  = (float)(PIXY_CENTER_X - PIXY_TRACK_HALF_WIDTH);
    s_last_lookahead_right_x = (float)(PIXY_CENTER_X + PIXY_TRACK_HALF_WIDTH);
}

/* ------------------------------------------------------------------ */
/* vector_filtering - Geometric pair validation & continuity tracking */
/* ------------------------------------------------------------------ */
void vector_filtering(uint16_t *vectors, size_t *num_vectors)
{
    size_t n = *num_vectors;
    if (n == 0) return;

    /* ---- Pass 1: Chassis exclusion zone filter (Top Camera)
     * Rejects spurious reflections that lie entirely inside the front bumper/chassis zone.
     * Legitimate road boundaries extending into the zone have their near endpoint trimmed. ---- */
    size_t write = 0;
    for (size_t i = 0; i < n; i++) {
        uint16_t x0 = vectors[4 * i + 0];
        uint16_t y0 = vectors[4 * i + 1];
        uint16_t x1 = vectors[4 * i + 2];
        uint16_t y1 = vectors[4 * i + 3];

        bool pt0_in = (x0 >= TOP_CAM_EXCLUDE_X_MIN && x0 <= TOP_CAM_EXCLUDE_X_MAX && y0 >= TOP_CAM_EXCLUDE_Y_MIN);
        bool pt1_in = (x1 >= TOP_CAM_EXCLUDE_X_MIN && x1 <= TOP_CAM_EXCLUDE_X_MAX && y1 >= TOP_CAM_EXCLUDE_Y_MIN);

        if (pt0_in && pt1_in) {
            continue; /* Entirely inside chassis zone */
        }

        if (pt0_in) y0 = TOP_CAM_EXCLUDE_Y_MIN - 1;
        if (pt1_in) y1 = TOP_CAM_EXCLUDE_Y_MIN - 1;

        int dx = (int)x1 - (int)x0;
        int dy = (int)y1 - (int)y0;
        if ((dx * dx + dy * dy) < 16) {
            continue; /* Vector became degenerate or too short (< 4 px) */
        }

        vectors[4 * write + 0] = x0;
        vectors[4 * write + 1] = y0;
        vectors[4 * write + 2] = x1;
        vectors[4 * write + 3] = y1;
        write++;
    }
    n = write;
    *num_vectors = n;
    if (n == 0) return;

    /* ---- Pass 2: Reject near-horizontal vectors (|dx| > 0.85*|dy|, angle > 40.3 deg)
     * Rejects 4-way intersection crossing lines and sharp perpendicular branches ---- */
    write = 0;
    for (size_t i = 0; i < n; i++) {
        int dx = (int)vectors[4 * i + 2] - (int)vectors[4 * i + 0];
        int dy = (int)vectors[4 * i + 3] - (int)vectors[4 * i + 1];
        if (dx < 0) dx = -dx;
        if (dy < 0) dy = -dy;

        if (dy > 0 && (dx * 20 <= dy * 17)) {
            if (write != i) {
                for (int j = 0; j < 4; j++) {
                    vectors[4 * write + j] = vectors[4 * i + j];
                }
            }
            write++;
        }
    }
    n = write;
    *num_vectors = n;

    if (n <= 1) return;
    if (n > MAX_FILTER_VECTORS) n = MAX_FILTER_VECTORS;

    /* Sort endpoints so near point (higher Y, closer to car) is (near_x, near_y) */
    uint16_t near_x[MAX_FILTER_VECTORS], near_y[MAX_FILTER_VECTORS];
    uint16_t far_x[MAX_FILTER_VECTORS], far_y[MAX_FILTER_VECTORS];
    uint32_t len_sq[MAX_FILTER_VECTORS];
    float    heading[MAX_FILTER_VECTORS];

    for (size_t i = 0; i < n; i++) {
        uint16_t x0 = vectors[4 * i + 0], y0 = vectors[4 * i + 1];
        uint16_t x1 = vectors[4 * i + 2], y1 = vectors[4 * i + 3];

        if (y0 >= y1) {
            near_x[i] = x0; near_y[i] = y0;
            far_x[i]  = x1; far_y[i]  = y1;
        } else {
            near_x[i] = x1; near_y[i] = y1;
            far_x[i]  = x0; far_y[i]  = y0;
        }

        int32_t dx = (int32_t)x1 - (int32_t)x0;
        int32_t dy = (int32_t)y1 - (int32_t)y0;
        len_sq[i] = (uint32_t)(dx * dx + dy * dy);

        float fdy = (float)near_y[i] - (float)far_y[i];
        float fdx = (float)far_x[i]  - (float)near_x[i];
        heading[i] = (fdy > 0.5f) ? (atan2f(fdx, fdy) * (180.0f / (float)M_PI)) : 0.0f;
    }

    /* ---- Pass 3: Candidate Pair Selection ---- */
    int best_left_idx = -1;
    int best_right_idx = -1;
    float best_pair_score = -99999.0f;
    const float nominal_width = 2.0f * (float)PIXY_TRACK_HALF_WIDTH; // 50.0f
    const float connect_dist_sq = (float)CONNECT_DIST_PX * (float)CONNECT_DIST_PX;

    for (size_t i = 0; i < n; i++) {
        for (size_t j = i + 1; j < n; j++) {
            size_t left_i  = (near_x[i] <= near_x[j]) ? i : j;
            size_t right_j = (near_x[i] <= near_x[j]) ? j : i;

            float sep = (float)near_x[right_j] - (float)near_x[left_i];
            if (sep < (float)MIN_LANE_WIDTH_PX || sep > (float)MAX_LANE_WIDTH_PX) {
                continue;
            }

            /* Reject diverging perspective angles */
            float h_diff = heading[left_i] - heading[right_j];
            if (h_diff < -10.0f || h_diff > 62.0f) {
                continue;
            }

            /* Reject connected segments of the same curving boundary */
            float d3_sq = (float)((int32_t)near_x[i] - (int32_t)far_x[j]) * ((int32_t)near_x[i] - (int32_t)far_x[j]) +
                          (float)((int32_t)near_y[i] - (int32_t)far_y[j]) * ((int32_t)near_y[i] - (int32_t)far_y[j]);
            float d4_sq = (float)((int32_t)far_x[i]  - (int32_t)near_x[j]) * ((int32_t)far_x[i]  - (int32_t)near_x[j]) +
                          (float)((int32_t)far_y[i]  - (int32_t)near_y[j]) * ((int32_t)far_y[i]  - (int32_t)near_y[j]);
            if (d3_sq < connect_dist_sq || d4_sq < connect_dist_sq) {
                continue;
            }

            float width_err = fabsf(sep - nominal_width);
            float min_ny = (float)((near_y[left_i] < near_y[right_j]) ? near_y[left_i] : near_y[right_j]);
            float len_sum = sqrtf((float)len_sq[left_i]) + sqrtf((float)len_sq[right_j]);

            /* Heading continuity penalty */
            float mid_near_x = (float)(near_x[left_i] + near_x[right_j]) * 0.5f;
            float mid_far_x  = (float)(far_x[left_i]  + far_x[right_j])  * 0.5f;
            float mid_near_y = (float)(near_y[left_i] + near_y[right_j]) * 0.5f;
            float mid_far_y  = (float)(far_y[left_i]  + far_y[right_j])  * 0.5f;
            float pdy = mid_near_y - mid_far_y;
            float pdx = mid_far_x  - mid_near_x;
            float pair_heading = (pdy > 0.5f) ? (atan2f(pdx, pdy) * (180.0f / (float)M_PI)) : 0.0f;
            float h_dev = fabsf(pair_heading - s_last_heading);
            float heading_penalty = (h_dev > 15.0f) ? ((h_dev - 15.0f) * 6.0f) : 0.0f;

            /* Lateral deviation penalty from previous track boundaries */
            float expected_mid_x = (s_last_left_near_x + s_last_right_near_x) * 0.5f;
            float lat_dev = fabsf(mid_near_x - expected_mid_x);
            float lateral_penalty = (lat_dev > 18.0f) ? ((lat_dev - 18.0f) * 2.5f) : 0.0f;

            float score = min_ny * 10.0f - width_err * 8.0f + len_sum * 0.5f - heading_penalty - lateral_penalty;
            if (score > best_pair_score) {
                best_pair_score = score;
                best_left_idx  = (int)left_i;
                best_right_idx = (int)right_j;
            }
        }
    }

    if (best_left_idx >= 0 && best_right_idx >= 0) {
        uint16_t temp[8];
        for (int k = 0; k < 4; k++) {
            temp[k]     = vectors[4 * best_left_idx + k];
            temp[4 + k] = vectors[4 * best_right_idx + k];
        }
        for (int k = 0; k < 8; k++) {
            vectors[k] = temp[k];
        }
        *num_vectors = 2;
        return;
    }

    /* ---- Pass 4: Single Boundary Fallback ---- */
    int best_single_idx = -1;
    float best_single_score = -99999.0f;

    for (size_t i = 0; i < n; i++) {
        float xn = (float)near_x[i];
        float dist_l = fabsf(xn - s_last_left_near_x);
        float dist_r = fabsf(xn - s_last_right_near_x);
        float min_bound_dist = (dist_l < dist_r) ? dist_l : dist_r;

        float lateral_penalty = min_bound_dist * 2.5f;
        float h_dev = fabsf(heading[i] - s_last_heading);
        float heading_penalty = (h_dev > 18.0f) ? ((h_dev - 18.0f) * 6.0f) : 0.0f;

        float score = (float)near_y[i] * 5.0f + sqrtf((float)len_sq[i]) * 2.0f
                      - lateral_penalty - heading_penalty;

        if (score > best_single_score) {
            best_single_score = score;
            best_single_idx = (int)i;
        }
    }

    if (best_single_idx >= 0) {
        for (int k = 0; k < 4; k++) {
            vectors[k] = vectors[4 * best_single_idx + k];
        }
        *num_vectors = 1;
    }
}

/* ------------------------------------------------------------------ */
/* compute_steering_error - CTE & heading with perspective synthesis   */
/* ------------------------------------------------------------------ */
steering_result_t compute_steering_error(const uint16_t *vectors, size_t num_vectors)
{
    steering_result_t result;
    result.cte         = 0.0f;
    result.heading_deg = 0.0f;
    result.found       = 0;

    if (num_vectors == 0) return result;

    /* ---- Case A: Two vectors (left + right boundary) ---- */
    if (num_vectors >= 2) {
        float lx_near, ly_near, lx_far, ly_far;
        float rx_near, ry_near, rx_far, ry_far;

        if (vectors[1] >= vectors[3]) {
            lx_near = (float)vectors[0];  ly_near = (float)vectors[1];
            lx_far  = (float)vectors[2];  ly_far  = (float)vectors[3];
        } else {
            lx_near = (float)vectors[2];  ly_near = (float)vectors[3];
            lx_far  = (float)vectors[0];  ly_far  = (float)vectors[1];
        }

        if (vectors[5] >= vectors[7]) {
            rx_near = (float)vectors[4];  ry_near = (float)vectors[5];
            rx_far  = (float)vectors[6];  ry_far  = (float)vectors[7];
        } else {
            rx_near = (float)vectors[6];  ry_near = (float)vectors[7];
            rx_far  = (float)vectors[4];  ry_far  = (float)vectors[5];
        }

        s_last_left_near_x  = clamp_f(lx_near, 0.0f, (float)PIXY_CENTER_X - 5.0f);
        s_last_right_near_x = clamp_f(rx_near, (float)PIXY_CENTER_X + 5.0f, (float)PIXY_FRAME_WIDTH);

        float mid_near_x = (lx_near + rx_near) * 0.5f;
        float mid_far_x  = (lx_far  + rx_far)  * 0.5f;
        float mid_near_y = (ly_near + ry_near) * 0.5f;
        float mid_far_y  = (ly_far  + ry_far)  * 0.5f;

        result.cte = mid_near_x - (float)PIXY_CENTER_X;

        float dy = mid_near_y - mid_far_y;
        float dx = mid_far_x  - mid_near_x;
        if (dy > 0.5f) {
            result.heading_deg = atan2f(dx, dy) * (180.0f / (float)M_PI);
        }

        result.heading_deg = clamp_f(result.heading_deg, -55.0f, 55.0f);
        result.found = 2;

        float dh = clamp_f(result.heading_deg - s_last_heading, -12.0f, 12.0f);
        s_last_heading += dh;
        return result;
    }

    /* ---- Case B: One vector (single boundary fallback) ---- */
    float x0 = (float)vectors[0], y0 = (float)vectors[1];
    float x1 = (float)vectors[2], y1 = (float)vectors[3];

    float xn, yn, xf, yf;
    if (y0 >= y1) {
        xn = x0;  yn = y0;  xf = x1;  yf = y1;
    } else {
        xn = x1;  yn = y1;  xf = x0;  yf = y0;
    }

    bool is_right;
    if (xn >= ((float)PIXY_CENTER_X + 3.0f)) {
        is_right = true;
    } else if (xn <= ((float)PIXY_CENTER_X - 3.0f)) {
        is_right = false;
    } else {
        float dist_to_left  = fabsf(xn - s_last_left_near_x);
        float dist_to_right = fabsf(xn - s_last_right_near_x);
        is_right = (dist_to_right < dist_to_left);
    }

    if (is_right) {
        s_last_right_near_x = clamp_f(xn, (float)PIXY_CENTER_X + 5.0f, (float)PIXY_FRAME_WIDTH);
        s_last_left_near_x  = clamp_f(xn - (2.0f * (float)PIXY_TRACK_HALF_WIDTH), 0.0f, (float)PIXY_CENTER_X - 5.0f);
    } else {
        s_last_left_near_x  = clamp_f(xn, 0.0f, (float)PIXY_CENTER_X - 5.0f);
        s_last_right_near_x = clamp_f(xn + (2.0f * (float)PIXY_TRACK_HALF_WIDTH), (float)PIXY_CENTER_X + 5.0f, (float)PIXY_FRAME_WIDTH);
    }

    float sign = is_right ? -1.0f : 1.0f;
    float mid_near_x = xn + sign * (float)PIXY_TRACK_HALF_WIDTH;

    result.cte = mid_near_x - (float)PIXY_CENTER_X;

    float dy = yn - yf;
    float dx = xf - xn;
    if (dy > 0.5f) {
        result.heading_deg = atan2f(dx, dy) * (180.0f / (float)M_PI);
    }

    result.heading_deg = clamp_f(result.heading_deg, -55.0f, 55.0f);
    result.found = 1;

    float dh = clamp_f(result.heading_deg - s_last_heading, -12.0f, 12.0f);
    s_last_heading += dh;
    return result;
}

/* ------------------------------------------------------------------ */
/* compute_lookahead_curvature - Extended 1.0m Lookahead Estimator    */
/* Discards Y >= 30 bumper restriction to see curves 0.5m-1.2m ahead  */
/* ------------------------------------------------------------------ */
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
    result.has_transverse_tape  = 0;
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
        float x0 = (float)vectors[4 * i + 0], y0 = (float)vectors[4 * i + 1];
        float x1 = (float)vectors[4 * i + 2], y1 = (float)vectors[4 * i + 3];

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

        nx[count] = n_x; ny[count] = n_y;
        fx[count] = f_x; fy[count] = f_y;
        len[count] = sqrtf(dx * dx + dy * dy);
        count++;
    }

    if (transverse_wall_detected) {
        result.has_transverse_tape = 1;
        result.found = 2;
        result.lookahead_urgency = 1.0f;
        result.curvature = 1.0f;
        if (transverse_wall_mid_x > (float)PIXY_CENTER_X + 2.0f) {
            result.lookahead_x          = 10.0f; /* Clear path is on left */
            result.trajectory_angle_deg = -30.0f;
            result.trajectory_offset_px = -15.0f;
        } else {
            result.lookahead_x          = 70.0f; /* Clear path is on right */
            result.trajectory_angle_deg = 30.0f;
            result.trajectory_offset_px = 15.0f;
        }
        return result;
    }

    if (count == 0) return result;

    const float far_y  = (float)FORWARD_CAM_SAMPLE_HORIZON_Y; // 20.0f (preview pre-braking)
    const float near_y = (float)FORWARD_CAM_SAMPLE_NEAR_Y;    // 38.0f (fallback steering)

    int best_left_idx = -1;
    float best_left_score = -99999.0f;
    int best_right_idx = -1;
    float best_right_score = -99999.0f;

    for (size_t i = 0; i < count; i++) {
        float x_near = project_x_at_y(nx[i], ny[i], fx[i], fy[i], near_y);
        float score = len[i] + ny[i] * 1.5f;

        if (x_near < (float)PIXY_CENTER_X) {
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

    /* ---- Case 1: Dual Root Boundaries Found ---- */
    if (best_left_idx >= 0 && best_right_idx >= 0) {
        result.left_root_found  = 1;
        result.right_root_found = 1;
        result.found            = 2;

        /* 1. Near Horizon (Y=38) for Fallback Steering */
        float xl_near = project_x_at_y(nx[best_left_idx], ny[best_left_idx], fx[best_left_idx], fy[best_left_idx], near_y);
        float xr_near = project_x_at_y(nx[best_right_idx], ny[best_right_idx], fx[best_right_idx], fy[best_right_idx], near_y);
        float near_mid_x = (xl_near + xr_near) * 0.5f;
        result.trajectory_offset_px = near_mid_x - (float)PIXY_CENTER_X;

        /* Near-zone local trajectory angle */
        float l_dx = fx[best_left_idx] - nx[best_left_idx], l_dy = ny[best_left_idx] - fy[best_left_idx];
        float r_dx = fx[best_right_idx] - nx[best_right_idx], r_dy = ny[best_right_idx] - fy[best_right_idx];
        float avg_dx = (l_dx + r_dx) * 0.5f;
        float avg_dy = (l_dy + r_dy) * 0.5f;
        if (avg_dy > 0.5f) {
            result.trajectory_angle_deg = atan2f(avg_dx, avg_dy) * (180.0f / (float)M_PI);
        }

        /* 2. Far Horizon (Y=20) for Preview Pre-Braking */
        float xl_far = project_x_at_y(nx[best_left_idx], ny[best_left_idx], fx[best_left_idx], fy[best_left_idx], far_y);
        float xr_far = project_x_at_y(nx[best_right_idx], ny[best_right_idx], fx[best_right_idx], fy[best_right_idx], far_y);
        float far_mid_x = (xl_far + xr_far) * 0.5f;
        result.lookahead_x = far_mid_x;

        /* Lookahead Urgency: Curvature displacement between far preview and near bumper */
        float mid_dx = far_mid_x - near_mid_x;
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

    /* ---- Case 2: Single Boundary Fallback ---- */
    int single_idx = (best_left_idx >= 0) ? best_left_idx : best_right_idx;
    if (single_idx >= 0) {
        result.found = 1;
        float s_dy = ny[single_idx] - fy[single_idx];
        float s_dx = fx[single_idx] - nx[single_idx];
        if (s_dy > 0.5f) {
            result.trajectory_angle_deg = atan2f(s_dx, s_dy) * (180.0f / (float)M_PI);

            /* Near Horizon (Y=38) for Fallback Steering */
            float x_near = project_x_at_y(nx[single_idx], ny[single_idx], fx[single_idx], fy[single_idx], near_y);
            float near_center_x;
            if (single_idx == best_left_idx) {
                result.left_root_found = 1;
                near_center_x = x_near + 22.0f; /* Half lane width at near bumper */
            } else {
                result.right_root_found = 1;
                near_center_x = x_near - 22.0f;
            }
            result.trajectory_offset_px = near_center_x - (float)PIXY_CENTER_X;

            /* Far Horizon (Y=20) for Preview Pre-Braking */
            float x_far = project_x_at_y(nx[single_idx], ny[single_idx], fx[single_idx], fy[single_idx], far_y);
            float far_mid_x;
            if (single_idx == best_left_idx) {
                far_mid_x = x_far + 17.0f; /* Half lane width at far horizon */
            } else {
                far_mid_x = x_far - 17.0f;
            }
            result.lookahead_x = far_mid_x;

            /* Curvature displacement */
            float mid_dx = far_mid_x - near_center_x;
            float abs_curvature = fabsf(mid_dx);
            float abs_far_offset = fabsf(far_mid_x - (float)PIXY_CENTER_X);
            float max_curve = (abs_curvature > abs_far_offset) ? abs_curvature : abs_far_offset;
            if (max_curve > (float)LOOKAHEAD_DEADBAND_PX) {
                result.lookahead_urgency = (max_curve - (float)LOOKAHEAD_DEADBAND_PX) /
                                           ((float)LOOKAHEAD_FULL_URGENCY_PX - (float)LOOKAHEAD_DEADBAND_PX);
                if (result.lookahead_urgency > 1.0f) result.lookahead_urgency = 1.0f;
            } else {
                result.lookahead_urgency = 0.0f;
            }
            result.curvature = result.lookahead_urgency;
        }
    }

    return result;
}
