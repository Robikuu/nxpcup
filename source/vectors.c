#include <math.h>
#include <stdint.h>
#include <string.h>
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "wifi_config.h"
#include "vectors.h"
#include "stdbool.h"
#include "Config.h"
#include "pixy.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

void vector_filtering(uint16_t *vectors, size_t *num_vectors) {
    size_t n = *num_vectors;
    if (n == 0) return;

    // Normalize all vectors so (x0, y0) is ALWAYS the near point (highest Y)
    for (size_t i = 0; i < n; i++) {
        if (vectors[4*i + 1] < vectors[4*i + 3]) {
            uint16_t tmp_x = vectors[4*i + 0];
            uint16_t tmp_y = vectors[4*i + 1];
            vectors[4*i + 0] = vectors[4*i + 2];
            vectors[4*i + 1] = vectors[4*i + 3];
            vectors[4*i + 2] = tmp_x;
            vectors[4*i + 3] = tmp_y;
        }
    }

    if (n <= 1) return;

    size_t write = 0;
    for (size_t i = 0; i < n; i++) {
        int dx = (int)vectors[4*i + 2] - (int)vectors[4*i + 0];
        int dy = (int)vectors[4*i + 3] - (int)vectors[4*i + 1];
        if (dx < 0) dx = -dx;
        if (dy < 0) dy = -dy;
        if (dy > 0 && dx <= 2 * dy) {
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

    int best_left  = -1;
    int best_right = -1;
    for (size_t i = 0; i < n; i++) {
        uint16_t x0 = vectors[4 * i + 0];
        uint16_t y0 = vectors[4 * i + 1];
        if (x0 < PIXY_CENTER_X) {
            if (best_left < 0) {
                best_left = (int)i;
            } else {
                uint16_t by = vectors[4 * best_left + 1];
                uint16_t bx = vectors[4 * best_left + 0];
                int cur_dist  = PIXY_CENTER_X - (int)x0;
                int best_dist = PIXY_CENTER_X - (int)bx;
                if (y0 > by || (y0 == by && cur_dist < best_dist)) {
                    best_left = (int)i;
                }
            }
        } else {
            if (best_right < 0) {
                best_right = (int)i;
            } else {
                uint16_t by = vectors[4 * best_right + 1];
                uint16_t bx = vectors[4 * best_right + 0];
                int cur_dist  = (int)x0 - PIXY_CENTER_X;
                int best_dist = (int)bx - PIXY_CENTER_X;
                if (y0 > by || (y0 == by && cur_dist < best_dist)) {
                    best_right = (int)i;
                }
            }
        }
    }

    uint16_t left_vec[4], right_vec[4];
    bool has_left  = (best_left  >= 0);
    bool has_right = (best_right >= 0);

    if (has_left) {
        for (int j = 0; j < 4; j++) left_vec[j] = vectors[4 * best_left + j];
    }
    if (has_right) {
        for (int j = 0; j < 4; j++) right_vec[j] = vectors[4 * best_right + j];
    }

    size_t out = 0;
    if (has_left) {
        for (int j = 0; j < 4; j++) vectors[4 * out + j] = left_vec[j];
        out++;
    }
    if (has_right) {
        for (int j = 0; j < 4; j++) vectors[4 * out + j] = right_vec[j];
        out++;
    }
    *num_vectors = out;
}

steering_result_t compute_steering_error(const uint16_t *vectors, size_t num_vectors) {
    steering_result_t result = {0.0f, 0.0f, 0};
    if (num_vectors == 0) return result;

    float lines[2][4];
    float lengths[2];
    size_t n_lines = (num_vectors > 2) ? 2 : num_vectors;

    for (size_t i = 0; i < n_lines; i++) {
        float x0 = (float)vectors[4*i + 0];
        float y0 = (float)vectors[4*i + 1];
        float x1 = (float)vectors[4*i + 2];
        float y1 = (float)vectors[4*i + 3];
        if (y0 >= y1) {
            lines[i][0] = x0;  lines[i][1] = y0;
            lines[i][2] = x1;  lines[i][3] = y1;
        } else {
            lines[i][0] = x1;  lines[i][1] = y1;
            lines[i][2] = x0;  lines[i][3] = y0;
        }
        float dx = lines[i][2] - lines[i][0];
        float dy = lines[i][1] - lines[i][3];
        lengths[i] = sqrtf(dx * dx + dy * dy);
    }

    uint8_t use_two = 0;
    if (n_lines == 2) {
        float near_sep = fabsf(lines[0][0] - lines[1][0]);
        float dx_fn, dy_fn;

        dx_fn = lines[0][2] - lines[1][0];
        dy_fn = lines[0][3] - lines[1][1];
        float d_far_near_01 = sqrtf(dx_fn * dx_fn + dy_fn * dy_fn);

        dx_fn = lines[1][2] - lines[0][0];
        dy_fn = lines[1][3] - lines[0][1];
        float d_far_near_10 = sqrtf(dx_fn * dx_fn + dy_fn * dy_fn);

        dx_fn = lines[0][2] - lines[1][2];
        dy_fn = lines[0][3] - lines[1][3];
        float d_far_far = sqrtf(dx_fn * dx_fn + dy_fn * dy_fn);

        if (near_sep >= MIN_LANE_WIDTH_PX && d_far_near_01 >= CONNECT_DIST_PX
            && d_far_near_10 >= CONNECT_DIST_PX && d_far_far >= CONNECT_DIST_PX) {
            use_two = 1;
        }
    }

    if (use_two) {
        float mid_near_x = (lines[0][0] + lines[1][0]) * 0.5f;
        float mid_far_x  = (lines[0][2] + lines[1][2]) * 0.5f;
        float mid_near_y = (lines[0][1] + lines[1][1]) * 0.5f;
        float mid_far_y  = (lines[0][3] + lines[1][3]) * 0.5f;

        result.cte = mid_near_x - (float)PIXY_CENTER_X;
        float dy = mid_near_y - mid_far_y;
        float dx = mid_far_x  - mid_near_x;
        if (dy > 0.5f) {
            result.heading_deg = atan2f(dx, dy) * (180.0f / (float)M_PI);
        }
        result.found = 2;
        return result;
    }

   // Fix: Pick the line segment closest to the car bumper (highest Y), NOT the longest one.
    // Length fluctuates wildly with camera noise, but the bottom anchor is stable.
    size_t best = 0;
    if (n_lines == 2 && lines[1][1] > lines[0][1]) {
        best = 1;
    }
    
    float xn = lines[best][0];
    float yn = lines[best][1];
    float xf = lines[best][2];
    float yf = lines[best][3];
    
    float dy = yn - yf;
    float dx = xf - xn;
    float side_offset;

    // Trajectory is the absolute truth because of vanishing points and corner sweeping.
    // Left lines always vanish towards the top-right (dx > 0). 
    // Right lines vanish towards the top-left (dx < 0).
    if (dx > 4.0f) {
        side_offset = (float)PIXY_TRACK_HALF_WIDTH;
    } else if (dx < -4.0f) {
        side_offset = -(float)PIXY_TRACK_HALF_WIDTH;
    } else {
        side_offset = (xn >= (float)PIXY_CENTER_X) ? -(float)PIXY_TRACK_HALF_WIDTH : (float)PIXY_TRACK_HALF_WIDTH;
    }
    
    result.cte = (xn + side_offset) - (float)PIXY_CENTER_X;
    
    if (dy > 0.5f) {
        result.heading_deg = atan2f(dx, dy) * (180.0f / (float)M_PI);
    }
    
    result.found = 1;
    return result;
}