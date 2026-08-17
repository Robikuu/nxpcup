#include <math.h>
#include <stdint.h>
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "wifi_config.h"
#include "vectors.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

//transformare homography pt vectori
void pixel_to_ground(float u, float v, float *X, float *Y)
{
    float Wp = H31 * u + H32 * v + H33;

    if (fabsf(Wp) < 1e-6f) {
        *X = 0.0f;
        *Y = 0.0f;
        return;
    }

    float Xp = H11 * u + H12 * v + H13;
    float Yp = H21 * u + H22 * v + H23;

    *X = Xp / Wp;
    *Y = Yp / Wp;
}

void print_vector_details(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, size_t index)
{
    int dx = (int)x1 - (int)x0;
    int dy = (int)y1 - (int)y0;

    // Calculate vector length (magnitude)
    float length = sqrtf((float)(dx * dx + dy * dy));

    // Calculate direction angle in degrees relative to the vertical axis (-dy).
    // Straight forward is 0 degrees. Left is negative, right is positive.
    float angle_deg = 0.0f;
    if (dy != 0) {
        angle_deg = atan2f((float)dx, (float)(-dy)) * 180.0f / 3.141592653589793f;
    } else {
        angle_deg = (dx >= 0) ? 90.0f : -90.0f;
    }

    // Split into integer and fractional parts for printing (avoiding %f format)
    int len_int = (int)length;
    int len_frac = (int)((length - len_int) * 100.0f);
    if (len_frac < 0) len_frac = -len_frac;

    int ang_int = (int)angle_deg;
    int ang_frac = (int)((angle_deg - (float)ang_int) * 100.0f);
    if (ang_frac < 0) ang_frac = -ang_frac;

    esp_printf("Vector [%u] details:\r\n", (unsigned)index);
    esp_printf("  Start: (%u, %u) -> End: (%u, %u)\r\n", (unsigned)x0, (unsigned)y0, (unsigned)x1, (unsigned)y1);
    esp_printf("  dx: %d, dy: %d\r\n", dx, dy);
    esp_printf("  Length: %d.%02d px\r\n", len_int, len_frac);
    esp_printf("  Angle: %d.%02d deg\r\n", ang_int, ang_frac);
}

/* ------------------------------------------------------------------ */
/*  Raw-Vector Virtual Centerline Algorithm (No Extrapolation)        */
/*  vectors[] layout: 6 fields per vector                             */
/*    [0]=x0  [1]=y0  [2]=x1  [3]=y1  [4]=index  [5]=flags           */
/* ------------------------------------------------------------------ */
lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors)
{
    lane_result_t result = {0.0f, 0.0f, 0};
    if (num_vectors == 0) return result;

    int   left_found  = 0;
    int   right_found = 0;
    float left_x0     = -1e6f, left_x1  = 0.0f;
    float right_x0    =  1e6f, right_x1 = 0.0f;

    for (size_t i = 0; i < num_vectors; i++) {
        float x0 = (float)vectors[6*i + 0];
        float y0 = (float)vectors[6*i + 1];
        float x1 = (float)vectors[6*i + 2];
        float y1 = (float)vectors[6*i + 3];

        /* Ensure (x0, y0) is the base/near point (larger Y)
           and (x1, y1) is the tip/far point (smaller Y) */
        if (y0 < y1) {
            float tx = x0; float ty = y0;
            x0 = x1; y0 = y1;
            x1 = tx; y1 = ty;
        }

        /* Ignore purely horizontal or 1-pixel glitch lines */
        if ((y0 - y1) < 1.0f) {
            continue;
        }

        /* Classify by actual vector base position on screen (no extrapolation!) */
        if (x0 < PIXY_CENTER_X) {
            /* Left boundary: pick the one closest to center (largest x0) */
            if (!left_found || x0 > left_x0) {
                left_x0     = x0;
                left_x1     = x1;
                left_found  = 1;
            }
        } else {
            /* Right boundary: pick the one closest to center (smallest x0) */
            if (!right_found || x0 < right_x0) {
                right_x0    = x0;
                right_x1    = x1;
                right_found = 1;
            }
        }
    }

    if (left_found && right_found) {
        /* Case 1: Both boundaries found -> true midpoint */
        float center_x0 = (left_x0 + right_x0) * 0.5f;
        float center_x1 = (left_x1 + right_x1) * 0.5f;

        result.cte         = center_x0 - PIXY_CENTER_X;
        result.heading     = center_x1 - center_x0;
        result.lines_found = 2;
    }
    else if (left_found) {
        /* Case 2: Only left boundary -> shift right by half lane width */
        float center_x0 = left_x0 + HALF_LANE_OFFSET_PX;
        float center_x1 = left_x1 + HALF_LANE_OFFSET_PX;

        result.cte         = center_x0 - PIXY_CENTER_X;
        result.heading     = left_x1 - left_x0;  /* pure vector direction */
        result.lines_found = 1;
    }
    else if (right_found) {
        /* Case 3: Only right boundary -> shift left by half lane width */
        float center_x0 = right_x0 - HALF_LANE_OFFSET_PX;
        float center_x1 = right_x1 - HALF_LANE_OFFSET_PX;

        result.cte         = center_x0 - PIXY_CENTER_X;
        result.heading     = right_x1 - right_x0;  /* pure vector direction */
        result.lines_found = 1;
    }

    return result;
}

