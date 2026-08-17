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
/*  Pure Pixel-Space Virtual Centerline Algorithm                     */
/*  vectors[] layout: 6 fields per vector                             */
/*    [0]=x0  [1]=y0  [2]=x1  [3]=y1  [4]=index  [5]=flags           */
/* ------------------------------------------------------------------ */
#define LC_MAX 10

lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors)
{
    lane_result_t result = {0.0f, 0.0f, 0};
    if (num_vectors == 0) return result;

    typedef struct {
        float x_near;  /* X coordinate evaluated at PIXY_Y_NEAR (bottom) */
        float x_far;   /* X coordinate evaluated at PIXY_Y_FAR (top)    */
    } line_cand_t;

    line_cand_t cands[LC_MAX];
    size_t n_cands = 0;

    for (size_t i = 0; i < num_vectors && n_cands < LC_MAX; i++) {
        float x0 = (float)vectors[6*i + 0];
        float y0 = (float)vectors[6*i + 1];
        float x1 = (float)vectors[6*i + 2];
        float y1 = (float)vectors[6*i + 3];

        /* Ensure (x0, y0) is the near point (larger Y)
           and (x1, y1) is the far point (smaller Y) */
        if (y0 < y1) {
            float tx = x0; float ty = y0;
            x0 = x1; y0 = y1;
            x1 = tx; y1 = ty;
        }

        float dy = y0 - y1;
        if (dy < 2.0f) {
            /* Ignore purely horizontal or glitch lines */
            continue;
        }

        /* Calculate line intersection with near and far scanlines */
        float inv_slope = (x1 - x0) / dy;
        float x_near    = x0 + inv_slope * (y0 - PIXY_Y_NEAR);
        float x_far     = x0 + inv_slope * (y0 - PIXY_Y_FAR);

        /* Clamp to reasonable image bounds */
        if (x_near < -20.0f) x_near = -20.0f;
        if (x_near > 100.0f) x_near = 100.0f;
        if (x_far  < -20.0f) x_far  = -20.0f;
        if (x_far  > 100.0f) x_far  = 100.0f;

        cands[n_cands].x_near = x_near;
        cands[n_cands].x_far  = x_far;
        n_cands++;
    }

    if (n_cands == 0) return result;

    /* Partition into Left (x_near < 39) and Right (x_near >= 39) boundaries */
    int   best_left  = -1;
    int   best_right = -1;
    float max_lx     = -1e6f;  /* closest to center from left (max X) */
    float min_rx     =  1e6f;  /* closest to center from right (min X) */

    for (size_t i = 0; i < n_cands; i++) {
        if (cands[i].x_near < PIXY_CENTER_X) {
            if (cands[i].x_near > max_lx) {
                max_lx    = cands[i].x_near;
                best_left = (int)i;
            }
        } else {
            if (cands[i].x_near < min_rx) {
                min_rx     = cands[i].x_near;
                best_right = (int)i;
            }
        }
    }

    float center_near;
    float center_far;

    if (best_left >= 0 && best_right >= 0) {
        /* Case 1: Both boundaries found -> true midpoint */
        center_near = (cands[best_left].x_near + cands[best_right].x_near) * 0.5f;
        center_far  = (cands[best_left].x_far  + cands[best_right].x_far)  * 0.5f;
        result.lines_found = 2;
    }
    else if (best_left >= 0) {
        /* Case 2: Only left boundary -> shift right by half lane width */
        center_near = cands[best_left].x_near + (LANE_WIDTH_NEAR_PX * 0.5f);
        center_far  = cands[best_left].x_far  + (LANE_WIDTH_FAR_PX  * 0.5f);
        result.lines_found = 1;
    }
    else if (best_right >= 0) {
        /* Case 3: Only right boundary -> shift left by half lane width */
        center_near = cands[best_right].x_near - (LANE_WIDTH_NEAR_PX * 0.5f);
        center_far  = cands[best_right].x_far  - (LANE_WIDTH_FAR_PX  * 0.5f);
        result.lines_found = 1;
    }
    else {
        return result;
    }

    /* CTE: Lateral offset from image center (39.0) at the near scanline */
    result.cte = center_near - PIXY_CENTER_X;

    /* Heading: Virtual centerline tilt / direction (far - near) */
    result.heading = center_far - center_near;

    return result;
}

