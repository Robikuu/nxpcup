#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "vectors.h"
#include "Config.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

/* Temporal memory for loss of line continuity */
static float s_last_center_x = (float)PIXY_CENTER_X;
static float s_last_heading  = 0.0f;

void reset_steering_tracking(void)
{
    s_last_center_x = (float)PIXY_CENTER_X;
    s_last_heading  = 0.0f;
}

/* ------------------------------------------------------------------ */
/* compute_top_camera_lane: Find closest left and right lines to car  */
/* ------------------------------------------------------------------ */
void compute_top_camera_lane(const uint16_t *vectors, size_t num_vectors, steering_result_t *out)
{
    if (out == NULL) return;

    memset(out, 0, sizeof(steering_result_t));

    if (num_vectors == 0 || vectors == NULL) {
        out->cte         = s_last_center_x - (float)PIXY_CENTER_X;
        out->heading_deg = s_last_heading;
        out->found       = 0;
        return;
    }

    bool  left_found       = false;
    bool  right_found      = false;
    float best_left_x      = -1000.0f; /* We want max x < 39 (closest to center from left) */
    float best_left_angle  = 0.0f;
    float best_right_x     = 1000.0f;  /* We want min x > 39 (closest to center from right) */
    float best_right_angle = 0.0f;

    for (size_t i = 0; i < num_vectors; i++) {
        uint16_t x0 = vectors[4 * i + 0];
        uint16_t y0 = vectors[4 * i + 1];
        uint16_t x1 = vectors[4 * i + 2];
        uint16_t y1 = vectors[4 * i + 3];

        /* Determine near (closest to bumper) and far endpoints */
        float x_near, y_near, x_far, y_far;
        if (y0 >= y1) {
            x_near = (float)x0; y_near = (float)y0;
            x_far  = (float)x1; y_far  = (float)y1;
        } else {
            x_near = (float)x1; y_near = (float)y1;
            x_far  = (float)x0; y_far  = (float)y0;
        }

        float dy = y_near - y_far;
        float dx = x_far - x_near;
        float abs_dx = fabsf(dx);

        /* Stateless transverse tape filtering (ratio test) */
        if (dy <= 0.27f * abs_dx && abs_dx >= 16.0f) {
            out->has_transverse_tape = 1U;
            float min_x = (x_near < x_far) ? x_near : x_far;
            float max_x = (x_near > x_far) ? x_near : x_far;
            if (min_x <= 46.0f && max_x >= 32.0f) {
                out->transverse_crosses_center = 1U;
            }
            continue; /* Discard from left/right boundary selection */
        }

        /* Must extend into lower visible field near car */
        if (y_near < 16.0f) continue;

        /* Must have minimum vertical length (reject horizontal crossbars/noise) */
        if (dy < 4.0f) continue;

        float angle = atan2f(dx, dy) * (180.0f / M_PI);

        /* Classify: Left of vehicle center */
        if (x_near < (float)PIXY_CENTER_X) {
            if (x_near > best_left_x) {
                best_left_x     = x_near;
                best_left_angle = angle;
                left_found      = true;
            }
        }
        /* Classify: Right of vehicle center */
        else if (x_near > (float)PIXY_CENTER_X) {
            if (x_near < best_right_x) {
                best_right_x     = x_near;
                best_right_angle = angle;
                right_found      = true;
            }
        }
    }

    float center_x;
    float heading;

    if (left_found && right_found) {
        center_x = (best_left_x + best_right_x) * 0.5f;
        heading  = (best_left_angle + best_right_angle) * 0.5f;
        out->found = 2;
    } else if (left_found) {
        center_x = best_left_x + (float)TRACK_HALF_WIDTH_PX;
        heading  = best_left_angle;
        out->found = 1;
    } else if (right_found) {
        center_x = best_right_x - (float)TRACK_HALF_WIDTH_PX;
        heading  = best_right_angle;
        out->found = 1;
    } else {
        center_x = s_last_center_x;
        heading  = s_last_heading;
        out->found = 0;
    }

    if (out->found > 0) {
        s_last_center_x = center_x;
        s_last_heading  = heading;
    }

    out->left_found  = left_found ? 1U : 0U;
    out->right_found = right_found ? 1U : 0U;
    out->cte         = center_x - (float)PIXY_CENTER_X;
    out->heading_deg = heading;
}

/* ------------------------------------------------------------------ */
/* compute_front_camera_horizon: Strategic Classifier & Horizon Angle */
/* ------------------------------------------------------------------ */
typedef struct {
    float x_near;
    float y_near;
    float x_far;
    float y_far;
    float dx;
    float dy;
    float len;
    float angle;
    bool  is_transverse;
} front_vector_desc_t;

void compute_front_camera_horizon(const uint16_t *vectors, size_t num_vectors, horizon_result_t *out)
{
    if (out == NULL) return;

    memset(out, 0, sizeof(horizon_result_t));

    if (num_vectors == 0 || vectors == NULL) {
        return;
    }

    front_vector_desc_t parsed[32];
    size_t count = (num_vectors > 32) ? 32 : num_vectors;

    float sum_weighted_angle = 0.0f;
    float sum_length         = 0.0f;
    uint8_t valid_count      = 0;
    float trans_y_near_max   = 0.0f;
    float trans_y_far_min    = 999.0f;

    /* Pass 1: Parse vectors, detect transverse tape, test center gap */
    for (size_t i = 0; i < count; i++) {
        uint16_t x0 = vectors[4 * i + 0];
        uint16_t y0 = vectors[4 * i + 1];
        uint16_t x1 = vectors[4 * i + 2];
        uint16_t y1 = vectors[4 * i + 3];

        float x_near, y_near, x_far, y_far;
        if (y0 >= y1) {
            x_near = (float)x0; y_near = (float)y0;
            x_far  = (float)x1; y_far  = (float)y1;
        } else {
            x_near = (float)x1; y_near = (float)y1;
            x_far  = (float)x0; y_far  = (float)y0;
        }

        float dx = x_far - x_near;
        float dy = y_near - y_far;
        float abs_dx = fabsf(dx);
        float len = sqrtf(dx * dx + dy * dy);
        float angle = atan2f(dx, dy) * (180.0f / M_PI);

        parsed[i].x_near = x_near;
        parsed[i].y_near = y_near;
        parsed[i].x_far  = x_far;
        parsed[i].y_far  = y_far;
        parsed[i].dx     = dx;
        parsed[i].dy     = dy;
        parsed[i].len    = len;
        parsed[i].angle  = angle;

        /* Transverse tape aspect ratio test: dy <= 0.27 * |dx| && |dx| >= 16.0 */
        if (dy <= 0.27f * abs_dx && abs_dx >= 16.0f) {
            parsed[i].is_transverse = true;
            out->has_transverse_tape = 1U;

            if (y_near > trans_y_near_max) {
                trans_y_near_max = y_near;
            }
            if (y_far < trans_y_far_min) {
                trans_y_far_min = y_far;
            }

            /* Center Gap Test: check if transverse tape enters center band |x - 39| <= 14.0 (x in [25, 53]) */
            float min_x = (x_near < x_far) ? x_near : x_far;
            float max_x = (x_near > x_far) ? x_near : x_far;
            if (min_x <= 53.0f && max_x >= 25.0f) {
                out->transverse_crosses_center = 1U;
            }
        } else {
            parsed[i].is_transverse = false;

            /* Exclude transverse tape from horizon angle calculation to avoid pivot drag */
            if (dy >= 2.0f && len >= 5.0f) {
                sum_weighted_angle += angle * len;
                sum_length         += len;
                valid_count++;
            }
        }
    }

    /* Pass 2: Forward Exit Pair Test beyond transverse tape */
    bool has_exit_pair = false;
    for (size_t j = 0; j < count; j++) {
        if (parsed[j].is_transverse || parsed[j].dy < 3.0f) continue;
        if (fabsf(parsed[j].angle) > 20.0f) continue;

        float x_mid_j = (parsed[j].x_near + parsed[j].x_far) * 0.5f;
        if (x_mid_j < 15.0f || x_mid_j > 36.0f) continue;

        /* Exit line must extend beyond transverse tape into distance (y_far < trans_y_far_min)
           and must not start back on the entrance side (y_near <= trans_y_near_max + 3.0f) */
        if (out->has_transverse_tape) {
            if (parsed[j].y_far >= trans_y_far_min) continue;
            if (parsed[j].y_near > (trans_y_near_max + 3.0f)) continue;
        }

        for (size_t k = 0; k < count; k++) {
            if (j == k) continue;
            if (parsed[k].is_transverse || parsed[k].dy < 3.0f) continue;
            if (fabsf(parsed[k].angle) > 20.0f) continue;

            float x_mid_k = (parsed[k].x_near + parsed[k].x_far) * 0.5f;
            if (x_mid_k < 42.0f || x_mid_k > 65.0f) continue;

            if (out->has_transverse_tape) {
                if (parsed[k].y_far >= trans_y_far_min) continue;
                if (parsed[k].y_near > (trans_y_near_max + 3.0f)) continue;
            }

            float sep = x_mid_k - x_mid_j;
            if (sep >= 22.0f && sep <= 48.0f) {
                has_exit_pair = true;
                break;
            }
        }
        if (has_exit_pair) break;
    }

    /* Strategic Decision */
    if (out->has_transverse_tape) {
        if (out->transverse_crosses_center == 0U && has_exit_pair) {
            out->is_intersection_ahead = 1U;
            out->is_sharp_corner       = 0U; /* Open intersection -> do not brake or trigger pivot drag */
        } else {
            out->is_intersection_ahead = 0U;
            out->is_sharp_corner       = 1U; /* Fail-safe default to corner, pre-brake! */
        }
    } else {
        out->is_intersection_ahead = 0U;
        if (sum_length > 0.1f && valid_count > 0) {
            float avg_angle = sum_weighted_angle / sum_length;
            out->is_sharp_corner = (fabsf(avg_angle) > (float)FRONT_CORNER_ANGLE_THRESH_DEG) ? 1U : 0U;
        } else {
            out->is_sharp_corner = 0U;
        }
    }

    if (sum_length > 0.1f && valid_count > 0) {
        out->horizon_angle_deg = sum_weighted_angle / sum_length;
        out->found             = valid_count;
    } else if (out->has_transverse_tape) {
        out->horizon_angle_deg = 0.0f;
        out->found             = (uint8_t)count;
    }
}