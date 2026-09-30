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

        /* Must extend into lower visible field near car */
        if (y_near < 16.0f) continue;

        /* Must have minimum vertical length (reject horizontal crossbars/noise) */
        float dy = y_near - y_far;
        if (dy < 4.0f) continue;

        float dx = x_far - x_near;
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
/* compute_front_camera_horizon: Length-weighted horizon road angle   */
/* ------------------------------------------------------------------ */
void compute_front_camera_horizon(const uint16_t *vectors, size_t num_vectors, horizon_result_t *out)
{
    if (out == NULL) return;

    memset(out, 0, sizeof(horizon_result_t));

    if (num_vectors == 0 || vectors == NULL) {
        return;
    }

    float sum_weighted_angle = 0.0f;
    float sum_length         = 0.0f;
    uint8_t valid_count      = 0;

    for (size_t i = 0; i < num_vectors; i++) {
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

        /* Reject pure horizontal artifacts */
        if (dy < 2.0f) continue;

        float len = sqrtf(dx * dx + dy * dy);
        if (len < 5.0f) continue;

        float angle = atan2f(dx, dy) * (180.0f / M_PI);

        sum_weighted_angle += angle * len;
        sum_length         += len;
        valid_count++;
    }

    if (sum_length > 0.1f && valid_count > 0) {
        float avg_angle = sum_weighted_angle / sum_length;
        out->horizon_angle_deg = avg_angle;
        out->is_sharp_corner   = (fabsf(avg_angle) > (float)FRONT_CORNER_ANGLE_THRESH_DEG) ? 1U : 0U;
        out->found             = valid_count;
    }
}