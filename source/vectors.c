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
/* vector_filtering - Geometric pair validation                       */
/* ------------------------------------------------------------------ */
void vector_filtering(uint16_t *vectors, size_t *num_vectors)
{
    size_t n = *num_vectors;
    if (n == 0) return;

    /* ---- Pass 1: Chassis exclusion zone filter (Top Camera)
     * Rejects spurious reflections that lie entirely inside the front bumper/chassis zone:
     * PixyMon (640x400) X in [200, 450], Y in [360, max] -> Pixy2 frame (79x52) X in [24, 56], Y in [46, 51].
     * Legitimate road boundaries extending into the zone have their near endpoint trimmed to Y=45. ---- */
    size_t write = 0;
    for (size_t i = 0; i < n; i++) {
        uint16_t x0 = vectors[4 * i + 0];
        uint16_t y0 = vectors[4 * i + 1];
        uint16_t x1 = vectors[4 * i + 2];
        uint16_t y1 = vectors[4 * i + 3];

        bool pt0_in = (x0 >= TOP_CAM_EXCLUDE_X_MIN && x0 <= TOP_CAM_EXCLUDE_X_MAX && y0 >= TOP_CAM_EXCLUDE_Y_MIN);
        bool pt1_in = (x1 >= TOP_CAM_EXCLUDE_X_MIN && x1 <= TOP_CAM_EXCLUDE_X_MAX && y1 >= TOP_CAM_EXCLUDE_Y_MIN);

        /* Spurious reflection entirely on chassis/bumper: reject */
        if (pt0_in && pt1_in) {
            continue;
        }

        /* Legitimate track boundary: trim any tip dipping into the bumper zone */
        if (pt0_in) {
            y0 = TOP_CAM_EXCLUDE_Y_MIN - 1;
        }
        if (pt1_in) {
            y1 = TOP_CAM_EXCLUDE_Y_MIN - 1;
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

    /* ---- Pass 2: reject near-horizontal vectors (|dx| > 0.85*|dy|, angle > 40.3 deg)
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
    uint16_t near_x[MAX_FILTER_VECTORS];
    uint16_t near_y[MAX_FILTER_VECTORS];
    uint16_t far_x[MAX_FILTER_VECTORS];
    uint16_t far_y[MAX_FILTER_VECTORS];
    uint32_t len_sq[MAX_FILTER_VECTORS];
    float    heading[MAX_FILTER_VECTORS];

    for (size_t i = 0; i < n; i++) {
        uint16_t x0 = vectors[4 * i + 0];
        uint16_t y0 = vectors[4 * i + 1];
        uint16_t x1 = vectors[4 * i + 2];
        uint16_t y1 = vectors[4 * i + 3];

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
        if (fdy > 0.5f) {
            heading[i] = atan2f(fdx, fdy) * (180.0f / (float)M_PI);
        } else {
            heading[i] = 0.0f;
        }
    }

    /*
     * 1. Candidate Pair Selection
     * Evaluate all unique pairs (i, j).
     * Rejects pairs with separation < MIN_LANE_WIDTH_PX (32.0f) or > MAX_LANE_WIDTH_PX (68.0f),
     * which immediately disqualifies any pairs formed with the center dashed line (~25px separation).
     * Rejects pairs where endpoints are within CONNECT_DIST_PX (chained fragments of the same curve).
     * Rejects pairs with diverging perspective headings (hl - hr outside [-10.0 deg, +62.0 deg]).
     * Scores candidates by proximity to bumper (min near_y) and closeness to 50px nominal width.
     */
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

            /* Reject candidate pairs whose headings diverge or form invalid perspective angles.
             * Legitimate lanes have (heading_left - heading_right) in [-10.0 deg, +62.0 deg]. */
            float h_diff = heading[left_i] - heading[right_j];
            if (h_diff < -10.0f || h_diff > 62.0f) {
                continue;
            }

            /* Reject connected segments of the same curving boundary (head-to-tail connection) */
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

            float score = min_ny * 10.0f - width_err * 8.0f + len_sum * 0.5f;
            if (score > best_pair_score) {
                best_pair_score = score;
                best_left_idx  = (int)left_i;
                best_right_idx = (int)right_j;
            }
        }
    }

    if (best_left_idx >= 0 && best_right_idx >= 0) {
        /* Valid boundary pair! Vector 0 = Left, Vector 1 = Right */
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

    /*
     * 2. Single Boundary Fallback
     * Pick vector that best matches temporal boundary positions and heading continuity.
     * Penalizes candidate vectors that deviate sharply from recent vehicle heading (branches).
     * Grants bonus to continuous outer track boundary (right side) to prevent inner-loop trapping.
     */
    int best_single_idx = -1;
    float best_single_score = -99999.0f;

    for (size_t i = 0; i < n; i++) {
        float xn = (float)near_x[i];
        float dist_l = fabsf(xn - s_last_left_near_x);
        float dist_r = fabsf(xn - s_last_right_near_x);
        float min_bound_dist = (dist_l < dist_r) ? dist_l : dist_r;

        /* 1. Continuous lateral tracking penalty */
        float lateral_penalty = min_bound_dist * 2.5f;

        /* 2. Heading continuity penalty: heavily penalize vectors that deviate sharply
         * from recent vehicle heading (e.g. branch lines peeling off at intersections) */
        float h_dev = fabsf(heading[i] - s_last_heading);
        float heading_penalty = (h_dev > 18.0f) ? ((h_dev - 18.0f) * 6.0f) : 0.0f;

        /* 3. Unbiased continuity scoring: evaluate proximity to bumper, length,
         * continuity with previous lateral boundaries, and heading smoothness. */
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

    /* ---- Two vectors: left + right boundary ---- */
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

        /* Update temporal memory with known boundary coordinates (clamped to physical bounds) */
        s_last_left_near_x  = lx_near;
        s_last_right_near_x = rx_near;
        if (s_last_left_near_x > ((float)PIXY_CENTER_X - 5.0f)) {
            s_last_left_near_x = (float)PIXY_CENTER_X - 5.0f;
        }
        if (s_last_right_near_x < ((float)PIXY_CENTER_X + 5.0f)) {
            s_last_right_near_x = (float)PIXY_CENTER_X + 5.0f;
        }

        /* Midpoints of the two boundaries */
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

        /* Clamp heading to physical camera geometry limits */
        if (result.heading_deg > 55.0f)  result.heading_deg = 55.0f;
        if (result.heading_deg < -55.0f) result.heading_deg = -55.0f;

        result.found = 2;
        s_last_heading = result.heading_deg;
        return result;
    }

    /* ---- One vector: single boundary visible ---- */
    float x0 = (float)vectors[0];
    float y0 = (float)vectors[1];
    float x1 = (float)vectors[2];
    float y1 = (float)vectors[3];

    float xn, yn, xf, yf;
    if (y0 >= y1) {
        xn = x0;  yn = y0;  xf = x1;  yf = y1;
    } else {
        xn = x1;  yn = y1;  xf = x0;  yf = y0;
    }

    /* Robust physical side classification:
     * 1. If line is clearly to the right of camera center, it is the right boundary.
     * 2. If line is clearly to the left of camera center, it is the left boundary.
     * 3. In the narrow transition band around center [36.5, 42.5], use temporal memory. */
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

    /* Update memory for continuity (with physical sanity clamping) */
    if (is_right) {
        s_last_right_near_x = xn;
        s_last_left_near_x  = xn - (2.0f * (float)PIXY_TRACK_HALF_WIDTH);
    } else {
        s_last_left_near_x  = xn;
        s_last_right_near_x = xn + (2.0f * (float)PIXY_TRACK_HALF_WIDTH);
    }
    if (s_last_left_near_x > ((float)PIXY_CENTER_X - 5.0f)) {
        s_last_left_near_x = (float)PIXY_CENTER_X - 5.0f;
    }
    if (s_last_right_near_x < ((float)PIXY_CENTER_X + 5.0f)) {
        s_last_right_near_x = (float)PIXY_CENTER_X + 5.0f;
    }

    /* Centerline synthesis from single boundary:
     * 1. Lateral position: offset from single boundary by nominal track half-width.
     * 2. Heading: The lane centerline runs parallel to the single boundary vector.
     *    Heading is computed directly from the line vector (dx = xf - xn, dy = yn - yf),
     *    completely removing the artificial trapezoid tilt that caused straightaway weaving. */
    float sign = is_right ? -1.0f : 1.0f;
    float mid_near_x = xn + sign * (float)PIXY_TRACK_HALF_WIDTH;

    result.cte = mid_near_x - (float)PIXY_CENTER_X;

    float dy = yn - yf;
    float dx = xf - xn;
    if (dy > 0.5f) {
        result.heading_deg = atan2f(dx, dy) * (180.0f / (float)M_PI);
    }

    /* Clamp heading to physical camera geometry limits */
    if (result.heading_deg > 55.0f)  result.heading_deg = 55.0f;
    if (result.heading_deg < -55.0f) result.heading_deg = -55.0f;

    result.found = 1;
    s_last_heading = result.heading_deg;
    return result;
}

/* ------------------------------------------------------------------ */
/* compute_lookahead_trajectory - Foreground Root Anchor Algorithm     */
/* ------------------------------------------------------------------ */
trajectory_result_t compute_lookahead_trajectory(const uint16_t *vectors, size_t num_vectors)
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

    /* Orient each vector from near (yn >= yf, higher y is closer to bumper) to far (yf, horizon)
     * and filter out room background clutter (upper quarter Y < 13) and horizontal cross lines */
    size_t max_in = (num_vectors > (size_t)MAX_FILTER_VECTORS) ? (size_t)MAX_FILTER_VECTORS : num_vectors;
    size_t count = 0;
    float nx[MAX_FILTER_VECTORS], ny[MAX_FILTER_VECTORS], fx[MAX_FILTER_VECTORS], fy[MAX_FILTER_VECTORS];
    float len[MAX_FILTER_VECTORS];

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

        /* 1. Upper quarter horizon discard:
         * If both endpoints are in the upper quarter (y < 13), it's background clutter */
        if (n_y < (float)FORWARD_CAM_HORIZON_Y_MIN && f_y < (float)FORWARD_CAM_HORIZON_Y_MIN) {
            continue;
        }

        /* 2. Clip far endpoint if it extends into the upper quarter room ceiling */
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

        /* 3. Reject vectors pointing near-horizontal or backwards (dy < 6.0px) */
        if (dy < 6.0f) {
            continue;
        }

        nx[count] = n_x;
        ny[count] = n_y;
        fx[count] = f_x;
        fy[count] = f_y;
        len[count] = sqrtf(dx * dx + dy * dy);
        count++;
    }

    if (count == 0) return result;

    /*
     * Foreground Root Vector Selection:
     * A real track boundary currently entering under the vehicle must anchor on the foreground
     * track floor (Y_near >= FORWARD_CAM_ROOT_ANCHOR_Y_MIN, 30U).
     * Any vector originating higher up (e.g. peeling horizon branches) is excluded.
     * The center zone [35.0, 44.0] is excluded at the bumper, rejecting center dashed lines.
     */
    int best_left_idx = -1;
    float best_left_score = -99999.0f;
    int best_right_idx = -1;
    float best_right_score = -99999.0f;

    for (size_t i = 0; i < count; i++) {
        /* Must anchor in foreground bumper zone */
        if (ny[i] < (float)FORWARD_CAM_ROOT_ANCHOR_Y_MIN) {
            continue;
        }

        float score = ny[i] * 2.0f + len[i];

        /* Left Root Candidate (X_near <= 34.0) */
        if (nx[i] <= (float)FORWARD_CAM_LEFT_MAX_X) {
            if (score > best_left_score) {
                best_left_score = score;
                best_left_idx = (int)i;
            }
        }
        /* Right Root Candidate (X_near >= 45.0) */
        else if (nx[i] >= (float)FORWARD_CAM_RIGHT_MIN_X) {
            if (score > best_right_score) {
                best_right_score = score;
                best_right_idx = (int)i;
            }
        }
    }

    const float sample_y = (float)FORWARD_CAM_SAMPLE_HORIZON_Y; // 20.0f

    /* Case A: Both Left and Right Root Vectors Found */
    if (best_left_idx >= 0 && best_right_idx >= 0) {
        result.left_root_found  = 1;
        result.right_root_found = 1;
        result.found            = 2;

        /* Project Left Root to sample horizon Y = 20.0f */
        float l_dy = ny[best_left_idx] - fy[best_left_idx];
        float xl_20 = nx[best_left_idx];
        if (l_dy > 0.5f) {
            float t_l = (ny[best_left_idx] - sample_y) / l_dy;
            xl_20 = nx[best_left_idx] + t_l * (fx[best_left_idx] - nx[best_left_idx]);
        }

        /* Project Right Root to sample horizon Y = 20.0f */
        float r_dy = ny[best_right_idx] - fy[best_right_idx];
        float xr_20 = nx[best_right_idx];
        if (r_dy > 0.5f) {
            float t_r = (ny[best_right_idx] - sample_y) / r_dy;
            xr_20 = nx[best_right_idx] + t_r * (fx[best_right_idx] - nx[best_right_idx]);
        }

        /* Calculate Lookahead Point as the midpoint at horizon Y = 20.0f */
        result.lookahead_x = (xl_20 + xr_20) * 0.5f;
        result.trajectory_offset_px = result.lookahead_x - (float)PIXY_CENTER_X;

        /* Trajectory angle from near midpoint to lookahead point */
        float near_mid_x = (nx[best_left_idx] + nx[best_right_idx]) * 0.5f;
        float near_mid_y = (ny[best_left_idx] + ny[best_right_idx]) * 0.5f;
        float mid_dx = result.lookahead_x - near_mid_x;
        float mid_dy = near_mid_y - sample_y;
        if (mid_dy > 0.5f) {
            result.trajectory_angle_deg = atan2f(mid_dx, mid_dy) * (180.0f / (float)M_PI);
        }

        /* Lookahead Urgency with +/- 2.5px Deadband and Full Urgency at LOOKAHEAD_FULL_URGENCY_PX */
        float abs_offset = fabsf(result.trajectory_offset_px);
        if (abs_offset > (float)LOOKAHEAD_DEADBAND_PX) {
            result.lookahead_urgency = (abs_offset - (float)LOOKAHEAD_DEADBAND_PX) / ((float)LOOKAHEAD_FULL_URGENCY_PX - (float)LOOKAHEAD_DEADBAND_PX);
            if (result.lookahead_urgency > 1.0f) result.lookahead_urgency = 1.0f;
        } else {
            result.lookahead_urgency = 0.0f;
        }

        /* Update temporal memory */
        s_last_lookahead_left_x  = nx[best_left_idx];
        s_last_lookahead_right_x = nx[best_right_idx];
        return result;
    }

    /* Case B: Single Root Vector Found */
    if (best_left_idx >= 0) {
        result.left_root_found  = 1;
        result.right_root_found = 0;
        result.found            = 1;

        float l_dy = ny[best_left_idx] - fy[best_left_idx];
        float xl_20 = nx[best_left_idx];
        if (l_dy > 0.5f) {
            float t_l = (ny[best_left_idx] - sample_y) / l_dy;
            xl_20 = nx[best_left_idx] + t_l * (fx[best_left_idx] - nx[best_left_idx]);
        }

        /* Synthesize lookahead point by adding nominal half-width at horizon (~15px) */
        result.lookahead_x = xl_20 + 15.0f;
        result.trajectory_offset_px = result.lookahead_x - (float)PIXY_CENTER_X;

        float mid_dx = (fx[best_left_idx] - nx[best_left_idx]);
        float mid_dy = l_dy;
        if (mid_dy > 0.5f) {
            result.trajectory_angle_deg = atan2f(mid_dx, mid_dy) * (180.0f / (float)M_PI);
        }

        float abs_offset = fabsf(result.trajectory_offset_px);
        if (abs_offset > (float)LOOKAHEAD_DEADBAND_PX) {
            result.lookahead_urgency = (abs_offset - (float)LOOKAHEAD_DEADBAND_PX) / ((float)LOOKAHEAD_FULL_URGENCY_PX - (float)LOOKAHEAD_DEADBAND_PX);
            if (result.lookahead_urgency > 1.0f) result.lookahead_urgency = 1.0f;
        } else {
            result.lookahead_urgency = 0.0f;
        }

        s_last_lookahead_left_x = nx[best_left_idx];
        return result;
    }

    if (best_right_idx >= 0) {
        result.left_root_found  = 0;
        result.right_root_found = 1;
        result.found            = 1;

        float r_dy = ny[best_right_idx] - fy[best_right_idx];
        float xr_20 = nx[best_right_idx];
        if (r_dy > 0.5f) {
            float t_r = (ny[best_right_idx] - sample_y) / r_dy;
            xr_20 = nx[best_right_idx] + t_r * (fx[best_right_idx] - nx[best_right_idx]);
        }

        /* Synthesize lookahead point by subtracting nominal half-width at horizon (~15px) */
        result.lookahead_x = xr_20 - 15.0f;
        result.trajectory_offset_px = result.lookahead_x - (float)PIXY_CENTER_X;

        float mid_dx = (fx[best_right_idx] - nx[best_right_idx]);
        float mid_dy = r_dy;
        if (mid_dy > 0.5f) {
            result.trajectory_angle_deg = atan2f(mid_dx, mid_dy) * (180.0f / (float)M_PI);
        }

        float abs_offset = fabsf(result.trajectory_offset_px);
        if (abs_offset > (float)LOOKAHEAD_DEADBAND_PX) {
            result.lookahead_urgency = (abs_offset - (float)LOOKAHEAD_DEADBAND_PX) / ((float)LOOKAHEAD_FULL_URGENCY_PX - (float)LOOKAHEAD_DEADBAND_PX);
            if (result.lookahead_urgency > 1.0f) result.lookahead_urgency = 1.0f;
        } else {
            result.lookahead_urgency = 0.0f;
        }

        s_last_lookahead_right_x = nx[best_right_idx];
        return result;
    }

    /* Case C: No Root Vectors Found */
    return result;
}