#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "vectors.h"
#include "Config.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

#define TOP_CAM_LOOKAHEAD_Y_REF      44.0f
#define TOP_CAM_TRACK_GATE_PX        22.0f
#define TOP_CAM_MAX_BLIND_TRACK_TICKS 8U
#define TOP_SINGLE_LINE_PERSP_DEG    14.0f
#define FRONT_SINGLE_LINE_PERSP_DEG  18.0f

/* Stateful temporal memory for lane continuity and left/right boundary tracking */
static float   s_last_center_x = (float)PIXY_CENTER_X;
static float   s_last_heading  = 0.0f;
static uint8_t s_has_tracking  = 0U;
static float   s_last_left_x   = (float)PIXY_CENTER_X - (float)TRACK_HALF_WIDTH_PX;
static float   s_last_right_x  = (float)PIXY_CENTER_X + (float)TRACK_HALF_WIDTH_PX;
static uint8_t s_blind_frames  = 0U;

void reset_steering_tracking(void)
{
    s_last_center_x = (float)PIXY_CENTER_X;
    s_last_heading  = 0.0f;
    s_has_tracking  = 0U;
    s_last_left_x   = (float)PIXY_CENTER_X - (float)TRACK_HALF_WIDTH_PX;
    s_last_right_x  = (float)PIXY_CENTER_X + (float)TRACK_HALF_WIDTH_PX;
    s_blind_frames  = 0U;
}

/* ------------------------------------------------------------------ */
/* compute_top_camera_lane: Find closest left and right lines to car  */
/* ------------------------------------------------------------------ */
void compute_top_camera_lane(const uint16_t *vectors, size_t num_vectors, steering_result_t *out)
{
    if (out == NULL) return;

    memset(out, 0, sizeof(steering_result_t));

    if (num_vectors == 0 || vectors == NULL) {
        if (s_blind_frames < 255U) {
            s_blind_frames++;
        }
        out->cte         = s_last_center_x - (float)PIXY_CENTER_X;
        out->heading_deg = s_last_heading;
        out->found       = 0;
        return;
    }

    bool  left_found       = false;
    bool  right_found      = false;
    float best_left_x      = -1000.0f;
    float best_left_x_far  = 0.0f;
    float best_left_angle  = 0.0f;
    float best_left_dy     = 0.0f;
    float best_left_dist   = 1000.0f;
    float best_right_x     = 1000.0f;
    float best_right_x_far = 0.0f;
    float best_right_angle = 0.0f;
    float best_right_dy    = 0.0f;
    float best_right_dist  = 1000.0f;

    bool use_tracking = (s_has_tracking != 0U) && (s_blind_frames < TOP_CAM_MAX_BLIND_TRACK_TICKS);

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

        /* Chassis / Wheel / Tie-Rod Exclusion Zone at bottom of top camera frame:
         * Rejects short/medium reflections starting in the lower exclusion box unless the vector
         * is a genuine long lane boundary extending into the upper half of the view.
         */
        if (y_near >= (float)TOP_CAM_EXCLUDE_Y_MIN &&
            x_near >= (float)TOP_CAM_EXCLUDE_X_MIN &&
            x_near <= (float)TOP_CAM_EXCLUDE_X_MAX &&
            (y_far >= 28.0f || dy < 16.0f)) {
            continue;
        }

        /* Stateless transverse tape filtering (ratio test) */
        if (dy <= 0.27f * abs_dx && abs_dx >= 16.0f) {
            out->has_transverse_tape = 1U;
            float min_x = (x_near < x_far) ? x_near : x_far;
            float max_x = (x_near > x_far) ? x_near : x_far;
            if (min_x < 35.0f && max_x > 43.0f) {
                out->transverse_crosses_center = 1U;
            }
            continue; /* Discard from left/right boundary selection */
        }

        /* Must extend into lower visible field near car */
        if (y_near < 16.0f) continue;

        /* Must have minimum vertical length (reject horizontal crossbars/noise) */
        if (dy < 4.0f) continue;

        /* Diagonal Chord Filter: reject short corner-merged diagonal chords */
        if (abs_dx >= dy && dy < 10.0f) {
            continue;
        }

        float angle = atan2f(dx, dy) * (180.0f / M_PI);
        if (fabsf(angle) > 55.0f && dy < 12.0f) {
            continue;
        }

        /* Project candidate along its slope to fixed lookahead reference row Y_REF = 35.0f */
        float slope  = dx / dy;
        float x_proj = x_near + slope * (y_near - TOP_CAM_LOOKAHEAD_Y_REF);
        if (x_proj < -20.0f) x_proj = -20.0f;
        if (x_proj > 100.0f) x_proj = 100.0f;

        /* Classify candidate into Left or Right boundary using temporal tracking when available */
        bool is_left_candidate = false;
        if (use_tracking) {
            float d_left  = fabsf(x_proj - s_last_left_x);
            float d_right = fabsf(x_proj - s_last_right_x);
            if (d_left <= d_right && d_left <= TOP_CAM_TRACK_GATE_PX) {
                is_left_candidate = true;
            } else if (d_right < d_left && d_right <= TOP_CAM_TRACK_GATE_PX) {
                is_left_candidate = false;
            } else {
                is_left_candidate = (x_proj < (float)PIXY_CENTER_X);
            }
        } else {
            is_left_candidate = (x_proj < (float)PIXY_CENTER_X);
        }

        if (is_left_candidate) {
            float ref_dist = use_tracking ? fabsf(x_proj - s_last_left_x) : fabsf((float)PIXY_CENTER_X - x_proj);
            if (!left_found || ref_dist < best_left_dist) {
                best_left_x     = x_proj;
                best_left_x_far = x_far;
                best_left_angle = angle;
                best_left_dy    = dy;
                best_left_dist  = ref_dist;
                left_found      = true;
            }
        } else {
            float ref_dist = use_tracking ? fabsf(x_proj - s_last_right_x) : fabsf(x_proj - (float)PIXY_CENTER_X);
            if (!right_found || ref_dist < best_right_dist) {
                best_right_x     = x_proj;
                best_right_x_far = x_far;
                best_right_angle = angle;
                best_right_dy    = dy;
                best_right_dist  = ref_dist;
                right_found      = true;
            }
        }
    }

    /* Minimum Lane Width Validation: reject duplicate split segments of the same line */
    if (left_found && right_found && (best_right_x - best_left_x) < (float)MIN_LANE_WIDTH_PX) {
        if (use_tracking) {
            if (best_left_dist <= best_right_dist) {
                right_found = false;
            } else {
                left_found = false;
            }
        } else if (best_left_dy >= best_right_dy) {
            right_found = false;
        } else {
            left_found = false;
        }
    }

    float center_x;
    float heading;

    if (left_found && right_found) {
        center_x       = (best_left_x + best_right_x) * 0.5f;
        heading        = (best_left_angle + best_right_angle) * 0.5f;
        out->found     = 2;
        s_last_left_x  = best_left_x;
        s_last_right_x = best_right_x;
        s_has_tracking = 1U;
        s_blind_frames = 0U;
    } else if (left_found) {
        heading = best_left_angle;
        if (best_left_x_far < (float)PIXY_CENTER_X && heading > 0.0f) {
            float persp_scale = (30.0f - fabsf(heading)) / (30.0f - 20.0f);
            if (persp_scale < 0.0f) persp_scale = 0.0f;
            if (persp_scale > 1.0f) persp_scale = 1.0f;
            float comp = TOP_SINGLE_LINE_PERSP_DEG * persp_scale;
            heading = (heading > comp) ? (heading - comp) : 0.0f;
        }
        float cos_h = cosf(heading * (M_PI / 180.0f));
        if (cos_h < 0.55f) cos_h = 0.55f;
        float perp_offset = (float)TRACK_HALF_WIDTH_PX / cos_h;
        center_x       = best_left_x + perp_offset;
        out->found     = 1;
        s_last_left_x  = best_left_x;
        s_last_right_x = best_left_x + 2.0f * perp_offset;
        s_has_tracking = 1U;
        s_blind_frames = 0U;
    } else if (right_found) {
        heading = best_right_angle;
        if (best_right_x_far >= (float)PIXY_CENTER_X && heading < 0.0f) {
            float persp_scale = (30.0f - fabsf(heading)) / (30.0f - 20.0f);
            if (persp_scale < 0.0f) persp_scale = 0.0f;
            if (persp_scale > 1.0f) persp_scale = 1.0f;
            float comp = TOP_SINGLE_LINE_PERSP_DEG * persp_scale;
            heading = (heading < -comp) ? (heading + comp) : 0.0f;
        }
        float cos_h = cosf(heading * (M_PI / 180.0f));
        if (cos_h < 0.55f) cos_h = 0.55f;
        float perp_offset = (float)TRACK_HALF_WIDTH_PX / cos_h;
        center_x       = best_right_x - perp_offset;
        out->found     = 1;
        s_last_right_x = best_right_x;
        s_last_left_x  = best_right_x - 2.0f * perp_offset;
        s_has_tracking = 1U;
        s_blind_frames = 0U;
    } else {
        if (s_blind_frames < 255U) {
            s_blind_frames++;
        }
        center_x   = s_last_center_x;
        heading    = s_last_heading;
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
    size_t last_valid_idx    = 0;
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
            if (y_near < 22.0f) {
                parsed[i].is_transverse = false;
                continue; /* Ignore distant transverse walls in top 43% of horizon frame */
            }
            parsed[i].is_transverse = true;
            out->has_transverse_tape = 1U;

            if (y_near > trans_y_near_max) {
                trans_y_near_max = y_near;
            }
            if (y_far < trans_y_far_min) {
                trans_y_far_min = y_far;
            }

            /* Center Gap Test: a transverse wall crosses center only if it actually straddles the centerline */
            float min_x = (x_near < x_far) ? x_near : x_far;
            float max_x = (x_near > x_far) ? x_near : x_far;
            if (min_x < 35.0f && max_x > 43.0f) {
                out->transverse_crosses_center = 1U;
            }
        } else {
            parsed[i].is_transverse = false;

            /* Exclude transverse tape from horizon angle calculation to avoid pivot drag */
            if (dy >= 2.0f && len >= 5.0f) {
                sum_weighted_angle += angle * len;
                sum_length         += len;
                valid_count++;
                last_valid_idx = i;
            }
        }
    }

    /* Transverse tape union check for segmented/broken walls crossing center */
    for (size_t j = 0; j < count; j++) {
        if (!parsed[j].is_transverse) continue;
        float min_j = (parsed[j].x_near < parsed[j].x_far) ? parsed[j].x_near : parsed[j].x_far;
        float max_j = (parsed[j].x_near > parsed[j].x_far) ? parsed[j].x_near : parsed[j].x_far;
        float y_mid_j = (parsed[j].y_near + parsed[j].y_far) * 0.5f;
        for (size_t k = j + 1; k < count; k++) {
            if (!parsed[k].is_transverse) continue;
            float y_mid_k = (parsed[k].y_near + parsed[k].y_far) * 0.5f;
            if (fabsf(y_mid_j - y_mid_k) <= 6.0f) {
                float min_k = (parsed[k].x_near < parsed[k].x_far) ? parsed[k].x_near : parsed[k].x_far;
                float max_k = (parsed[k].x_near > parsed[k].x_far) ? parsed[k].x_near : parsed[k].x_far;
                /* Check if intervals overlap or touch within 4 pixels */
                if (min_k <= (max_j + 4.0f) && min_j <= (max_k + 4.0f)) {
                    float union_min = (min_j < min_k) ? min_j : min_k;
                    float union_max = (max_j > max_k) ? max_j : max_k;
                    if (union_min < 35.0f && union_max > 43.0f) {
                        out->transverse_crosses_center = 1U;
                    }
                }
            }
        }
    }

    /* Pass 2: Forward Exit Recognition (Pair or Single Forward Exit Line) */
    bool has_exit_line = false;
    if (out->has_transverse_tape) {
        bool found_diverging_pair = false;
        for (size_t j = 0; j < count; j++) {
            if (parsed[j].is_transverse || parsed[j].dy < 3.0f) continue;
            if (fabsf(parsed[j].angle) > 45.0f) continue;

            /* Exit line must extend beyond transverse tape into distance */
            if (parsed[j].y_far >= trans_y_far_min) continue;
            if (parsed[j].y_near > (trans_y_near_max + 5.0f)) continue;

            float mid_j = (parsed[j].x_near + parsed[j].x_far) * 0.5f;

            for (size_t k = j + 1; k < count; k++) {
                if (parsed[k].is_transverse || parsed[k].dy < 3.0f) continue;
                if (fabsf(parsed[k].angle) > 45.0f) continue;

                if (parsed[k].y_far >= trans_y_far_min) continue;
                if (parsed[k].y_near > (trans_y_near_max + 5.0f)) continue;

                float mid_k = (parsed[k].x_near + parsed[k].x_far) * 0.5f;

                /* Order left and right: idx_left on left, idx_right on right */
                size_t idx_left  = (mid_j <= mid_k) ? j : k;
                size_t idx_right = (mid_j <= mid_k) ? k : j;
                float left_mid   = (mid_j <= mid_k) ? mid_j : mid_k;
                float right_mid  = (mid_j <= mid_k) ? mid_k : mid_j;

                float sep = right_mid - left_mid;
                float pair_center = (left_mid + right_mid) * 0.5f;

                /* Separation between 20.0f and 45.0f, pair midpoint in [24.0f, 54.0f] */
                if (sep < 20.0f || sep > 45.0f) continue;
                if (pair_center < 24.0f || pair_center > 54.0f) continue;

                /* Vanishing-Point Convergence Test:
                 * Line equation: x(y) = x_near + s * (y_near - y)
                 * At y = 0: x(0) = x_near + s * y_near
                 */
                float s_l = parsed[idx_left].dx / parsed[idx_left].dy;
                float s_r = parsed[idx_right].dx / parsed[idx_right].dy;
                float x0_l = parsed[idx_left].x_near + s_l * parsed[idx_left].y_near;
                float x0_r = parsed[idx_right].x_near + s_r * parsed[idx_right].y_near;

                float denom = s_l - s_r;
                float x_v, y_v;
                float ang_l = fabsf(parsed[idx_left].angle);
                float ang_r = fabsf(parsed[idx_right].angle);

                /* Forward diverging lines (s_l < s_r) cannot form a converging exit pair */
                if (denom < -1e-3f) {
                    found_diverging_pair = true;
                    continue;
                }

                if (denom < 0.10f) {
                    /* Nearly parallel lines: converge at infinity, valid if forward-facing corridor */
                    if (ang_l <= 20.0f && ang_r <= 20.0f) {
                        y_v = -999.0f;
                        x_v = pair_center;
                    } else {
                        continue;
                    }
                } else {
                    y_v = (x0_l - x0_r) / denom;
                    /* For far vanishing points above the image, use horizon center to prevent lateral divergence */
                    x_v = (y_v >= -60.0f) ? (x0_l - s_l * y_v) : ((x0_l + x0_r) * 0.5f);
                }

                /* Convergence: vanishing point above image (Y_v <= 5.0f) and in [15.0f, 60.0f] */
                if (y_v <= 5.0f && x_v >= 15.0f && x_v <= 60.0f) {
                    has_exit_line = true;
                    break;
                }
            }
            if (has_exit_line) break;
        }

        /* Check 2: Single forward exit line aligned through central corridor
         * Only permitted if no diverging pair was detected!
         */
        if (!has_exit_line && !found_diverging_pair) {
            for (size_t j = 0; j < count; j++) {
                if (parsed[j].is_transverse || parsed[j].dy < 4.0f) continue;
                if (fabsf(parsed[j].angle) > 22.0f) continue;
                if (parsed[j].y_far >= trans_y_far_min) continue;
                if (parsed[j].y_near > (trans_y_near_max + 5.0f)) continue;

                float mid_x = (parsed[j].x_near + parsed[j].x_far) * 0.5f;
                if (mid_x >= 20.0f && mid_x <= 58.0f) {
                    has_exit_line = true;
                    break;
                }
            }
        }
    }

    /* Compute weighted foreground/longitudinal road angle from non-transverse vectors */
    float avg_angle = 0.0f;
    if (sum_length > 0.1f && valid_count > 0) {
        avg_angle = sum_weighted_angle / sum_length;

        /* Single-line perspective convergence compensation:
         * Only subtract the natural ~18 deg straight-line convergence when a single vector
         * stays on its own side of PIXY_CENTER_X (not curving across the track center).
         */
        if (valid_count == 1) {
            float sx_near = parsed[last_valid_idx].x_near;
            float sx_far  = parsed[last_valid_idx].x_far;
            if (sx_near < (float)PIXY_CENTER_X && sx_far < (float)PIXY_CENTER_X && avg_angle > 0.0f) {
                avg_angle = (avg_angle > FRONT_SINGLE_LINE_PERSP_DEG) ? (avg_angle - FRONT_SINGLE_LINE_PERSP_DEG) : 0.0f;
            } else if (sx_near >= (float)PIXY_CENTER_X && sx_far >= (float)PIXY_CENTER_X && avg_angle < 0.0f) {
                avg_angle = (avg_angle < -FRONT_SINGLE_LINE_PERSP_DEG) ? (avg_angle + FRONT_SINGLE_LINE_PERSP_DEG) : 0.0f;
            }
        }
    }

    /* Strategic Decision */
    if (out->has_transverse_tape) {
        if (out->transverse_crosses_center == 0U && has_exit_line) {
            out->is_intersection_ahead = 1U;
            out->is_sharp_corner       = 0U; /* Open intersection -> do not brake or trigger pivot drag */
            out->horizon_angle_deg     = 0.0f;
        } else {
            out->is_intersection_ahead = 0U;
            out->is_sharp_corner       = 1U; /* Fail-safe default to corner, pre-brake! */
            /* Preserve foreground curving lane angle when available so inner-loop corners
             * keep front-camera steering feedforward (top_turn_gate in main.c blocks early
             * turn-in on straights). */
            out->horizon_angle_deg     = (valid_count > 0) ? avg_angle : 0.0f;
        }
        out->found = (uint8_t)count;
    } else {
        out->is_intersection_ahead = 0U;
        if (sum_length > 0.1f && valid_count > 0) {
            out->is_sharp_corner   = (fabsf(avg_angle) > (float)FRONT_CORNER_ANGLE_THRESH_DEG) ? 1U : 0U;
            out->horizon_angle_deg = avg_angle;
            out->found             = valid_count;
        } else {
            out->is_sharp_corner   = 0U;
            out->horizon_angle_deg = 0.0f;
            out->found             = 0;
        }
    }
}
