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
/* compute_lookahead_trajectory - Trajectory planning from front cam  */
/* ------------------------------------------------------------------ */
trajectory_result_t compute_lookahead_trajectory(const uint16_t *vectors, size_t num_vectors)
{
    trajectory_result_t result;
    result.trajectory_angle_deg = 0.0f;
    result.trajectory_offset_px = 0.0f;
    result.curvature           = 0.0f;
    result.lookahead_urgency   = 0.0f;
    result.found               = 0;

    if (num_vectors == 0) return result;

    /* Orient each vector from near (yn >= yf, higher y is closer to bumper) to far (yf, horizon)
     * and filter out room background clutter (upper quarter Y < 13 and ungrounded vectors Ynear < 24) */
    size_t max_in = (num_vectors > (size_t)MAX_FILTER_VECTORS) ? (size_t)MAX_FILTER_VECTORS : num_vectors;
    size_t count = 0;
    float nx[MAX_FILTER_VECTORS], ny[MAX_FILTER_VECTORS], fx[MAX_FILTER_VECTORS], fy[MAX_FILTER_VECTORS];
    float vdx[MAX_FILTER_VECTORS], vdy[MAX_FILTER_VECTORS], len_sq[MAX_FILTER_VECTORS], heading[MAX_FILTER_VECTORS];

    for (size_t i = 0; i < max_in; i++) {
        float x0 = (float)vectors[4 * i];
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
         * If both endpoints are in the upper quarter (y < 13), it's background walls / ceiling / chairs */
        if (n_y < (float)FORWARD_CAM_HORIZON_Y_MIN && f_y < (float)FORWARD_CAM_HORIZON_Y_MIN) {
            continue;
        }

        /* 2. Track floor connection gate:
         * Candidate lookahead vectors must have their base anchored on the track floor (n_y >= 24).
         * Rejects floating background furniture that does not connect to the track floor near the vehicle */
        if (n_y < (float)FORWARD_CAM_FLOOR_Y_MIN) {
            continue;
        }

        /* 3. Clip far endpoint if it extends into the upper quarter room ceiling */
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

        /* Reject vectors pointing near-horizontal or backwards (dy < 6.0px) */
        if (dy < 6.0f) {
            continue;
        }

        nx[count] = n_x;
        ny[count] = n_y;
        fx[count] = f_x;
        fy[count] = f_y;
        vdx[count] = dx;
        vdy[count] = dy;
        len_sq[count] = dx * dx + dy * dy;
        heading[count] = (dy > 1.0f) ? (atan2f(dx, dy) * (180.0f / (float)M_PI)) : 0.0f;
        count++;
    }

    if (count == 0) return result;

    /* 1. Search for a valid Left + Right boundary pair */
    int best_l = -1;
    int best_r = -1;
    float best_pair_score = -99999.0f;

    for (size_t i = 0; i < count; i++) {
        for (size_t j = 0; j < count; j++) {
            if (i == j) continue;

            /* i must be strictly left of j at near end */
            if (nx[i] >= nx[j]) continue;

            float sep = nx[j] - nx[i];
            /* Separation must be physically plausible for full track width in lookahead view.
             * Full track width is ~46-52px. The center dashed line to an outer boundary is ~24px.
             * Reject any pair with sep < 34.0px to strictly prevent pairing with the center line! */
            if (sep < 34.0f || sep > 65.0f) continue;

            /* Boundaries must not cross each other at far end */
            if (fx[i] >= fx[j]) continue;

            /* Directional heading compatibility (matching top camera logic):
             * On a straight road, perspective convergence causes left boundary to lean right
             * (+18.9 deg) and right boundary to lean left (-18.9 deg), giving h_diff ~ +37.8 deg.
             * The old fabsf() check rejected this at 32 deg. Use directional check:
             * h_diff = heading[left] - heading[right] should be in [-10, +62] deg. */
            float h_diff = heading[i] - heading[j];
            if (h_diff < -10.0f || h_diff > 62.0f) continue;

            /* Pair score: near-camera presence, realistic separation (~46px), and vector length */
            float score = (ny[i] + ny[j]) * 0.5f - fabsf(sep - 46.0f) * 1.5f + (sqrtf(len_sq[i]) + sqrtf(len_sq[j])) * 0.5f;
            if (score > best_pair_score) {
                best_pair_score = score;
                best_l = (int)i;
                best_r = (int)j;
            }
        }
    }

    if (best_l >= 0 && best_r >= 0) {
        /* Two valid boundaries found: Vector best_l = Left, Vector best_r = Right */
        float lx_near = nx[best_l];  float ly_near = ny[best_l];
        float lx_far  = fx[best_l];  float ly_far  = fy[best_l];
        float rx_near = nx[best_r];  float ry_near = ny[best_r];
        float rx_far  = fx[best_r];  float ry_far  = fy[best_r];

        /* Update temporal memory */
        s_last_lookahead_left_x  = lx_near;
        s_last_lookahead_right_x = rx_near;
        if (s_last_lookahead_left_x > ((float)PIXY_CENTER_X - 5.0f)) {
            s_last_lookahead_left_x = (float)PIXY_CENTER_X - 5.0f;
        }
        if (s_last_lookahead_right_x < ((float)PIXY_CENTER_X + 5.0f)) {
            s_last_lookahead_right_x = (float)PIXY_CENTER_X + 5.0f;
        }

        /* Centerline trajectory midpoints */
        float mid_near_x = (lx_near + rx_near) * 0.5f;
        float mid_far_x  = (lx_far  + rx_far)  * 0.5f;
        float mid_near_y = (ly_near + ry_near) * 0.5f;
        float mid_far_y  = (ly_far  + ry_far)  * 0.5f;

        float dy = mid_near_y - mid_far_y;
        float dx = mid_far_x  - mid_near_x;

        /* Apex-cut bias: shift far target toward inside of corner.
         * On straights (|dx| <= 2.5f), no bias applied. */
        if (fabsf(dx) > 2.5f && dy > 1.0f) {
            float lane_half = (rx_far - lx_far) * 0.5f;
            if (lane_half < 5.0f)  lane_half = 5.0f;
            if (lane_half > 25.0f) lane_half = 25.0f;
            float apex_shift = (dx > 0.0f ? 1.0f : -1.0f) * lane_half * (float)APEX_CUT_RATIO;
            mid_far_x += apex_shift;
            dx = mid_far_x - mid_near_x;
        }

        if (dy > 1.0f) {
            result.trajectory_angle_deg = atan2f(dx, dy) * (180.0f / (float)M_PI);
        }
        result.trajectory_offset_px = mid_far_x - (float)PIXY_CENTER_X;

        float seg_len = sqrtf(dx * dx + dy * dy);
        if (seg_len > 1.0f) {
            result.curvature = fabsf(dx) / (seg_len * seg_len);
        }

        result.found = 2;
    } else {
        /* Single boundary visible: pick the best single forward vector */
        int best_single = -1;
        float best_single_score = -99999.0f;
        for (size_t i = 0; i < count; i++) {
            if (vdy[i] < 6.0f) continue; /* Reject cross-lines / horizontal artifacts */
            /* Reject center dashed line fragments (near camera center x ~ 39.5).
             * Outer track boundaries are at x ~ 15 (left) or x ~ 64 (right). */
            if (fabsf(nx[i] - (float)PIXY_CENTER_X) < 8.0f) continue;
            float s = ny[i] * 2.0f + sqrtf(len_sq[i]);
            if (s > best_single_score) {
                best_single_score = s;
                best_single = (int)i;
            }
        }

        if (best_single < 0) {
            return result;
        }

        float xn = nx[best_single];
        float yn = ny[best_single];
        float xf = fx[best_single];
        float yf = fy[best_single];

        /* Side classification with deadband and temporal memory:
         * 1. Clearly right of center -> right boundary
         * 2. Clearly left of center  -> left boundary
         * 3. Transition zone [36.5, 42.5] -> use temporal memory */
        bool is_right;
        if (xn >= ((float)PIXY_CENTER_X + 3.0f)) {
            is_right = true;
        } else if (xn <= ((float)PIXY_CENTER_X - 3.0f)) {
            is_right = false;
        } else {
            float dist_to_left  = fabsf(xn - s_last_lookahead_left_x);
            float dist_to_right = fabsf(xn - s_last_lookahead_right_x);
            is_right = (dist_to_right < dist_to_left);
        }

        /* Update temporal memory */
        if (is_right) {
            s_last_lookahead_right_x = xn;
            s_last_lookahead_left_x  = xn - (2.0f * (float)PIXY_TRACK_HALF_WIDTH);
        } else {
            s_last_lookahead_left_x  = xn;
            s_last_lookahead_right_x = xn + (2.0f * (float)PIXY_TRACK_HALF_WIDTH);
        }
        if (s_last_lookahead_left_x > ((float)PIXY_CENTER_X - 5.0f)) {
            s_last_lookahead_left_x = (float)PIXY_CENTER_X - 5.0f;
        }
        if (s_last_lookahead_right_x < ((float)PIXY_CENTER_X + 5.0f)) {
            s_last_lookahead_right_x = (float)PIXY_CENTER_X + 5.0f;
        }

        /* Centerline synthesis from single boundary:
         * The forward camera is tilted looking down the track. On a straight track,
         * perspective foreshortening causes each boundary to converge toward the
         * optical vanishing point (PIXY_CENTER_X, Y_vanish ~ 0).
         * If the raw line slope atan2(dx, dy) is used without perspective compensation,
         * a straight boundary falsely looks like a 26-28 deg curve, permanently triggering
         * maximum urgency and killing straightaway speed.
         *
         * For a straight boundary starting at (xn, yn), its expected perspective position at yf is:
         *   x_expected = PIXY_CENTER_X + (xn - PIXY_CENTER_X) * (yf / yn)
         * The true turn curvature is the departure of xf from this straight perspective line:
         *   dx_turn = xf - x_expected
         */
        float yn_clamped = (yn > 5.0f) ? yn : 5.0f;
        float depth_ratio = yf / yn_clamped;
        if (depth_ratio < 0.0f) depth_ratio = 0.0f;
        if (depth_ratio > 1.0f) depth_ratio = 1.0f;

        float x_expected = (float)PIXY_CENTER_X + (xn - (float)PIXY_CENTER_X) * depth_ratio;
        float dx_turn = xf - x_expected;
        float dy = yn - yf;

        /* Apparent track half-width at distance yf */
        float track_half_far = (float)PIXY_TRACK_HALF_WIDTH * depth_ratio;
        if (track_half_far < 5.0f)  track_half_far = 5.0f;
        if (track_half_far > (float)PIXY_TRACK_HALF_WIDTH) track_half_far = (float)PIXY_TRACK_HALF_WIDTH;

        float sign = is_right ? -1.0f : 1.0f;
        float mid_far_x = xf + sign * track_half_far;

        if (dy > 0.5f) {
            result.trajectory_angle_deg = atan2f(dx_turn, dy) * (180.0f / (float)M_PI);
        }
        result.trajectory_offset_px = mid_far_x - (float)PIXY_CENTER_X;

        float seg_len = sqrtf(dx_turn * dx_turn + dy * dy);
        if (seg_len > 1.0f) {
            result.curvature = fabsf(dx_turn) / (seg_len * seg_len);
        }

        result.found = 1;
    }

    /* Clamp trajectory angle to realistic lookahead limits */
    if (result.trajectory_angle_deg > 35.0f)  result.trajectory_angle_deg = 35.0f;
    if (result.trajectory_angle_deg < -35.0f) result.trajectory_angle_deg = -35.0f;

    /* Compute normalized lookahead urgency (0.0 to 1.0) with deadbands to prevent straight pre-braking:
     * - Ignore trajectory angles <= 6.0 deg (straightaway horizon jitter/alignment)
     * - Ignore lateral offset <= 15.0 px (matches CTE straight deadband) */
    float abs_angle = fabsf(result.trajectory_angle_deg);
    float angle_urgency = 0.0f;
    if (abs_angle > 6.0f) {
        angle_urgency = (abs_angle - 6.0f) / ((float)HEADING_FULL_BRAKE_DEG - 6.0f);
        if (angle_urgency > 1.0f) angle_urgency = 1.0f;
    }

    float abs_offset = fabsf(result.trajectory_offset_px);
    float offset_urgency = 0.0f;
    if (abs_offset > 15.0f) {
        offset_urgency = (abs_offset - 15.0f) / ((float)PIXY_CENTER_X - 15.0f);
        if (offset_urgency > 1.0f) offset_urgency = 1.0f;
    }

    result.lookahead_urgency = (angle_urgency > offset_urgency) ? angle_urgency : offset_urgency;

    /* Single-boundary gating: An isolated single line can have jitter or partial perspective mismatch.
     * Scale its urgency by 0.70x so single-boundary noise never falsely cuts straightaway speed.
     * Full 1.0x urgency requires dual-boundary geometric confirmation (found == 2). */
    if (result.found == 1) {
        result.lookahead_urgency *= 0.70f;
    }

    return result;
}