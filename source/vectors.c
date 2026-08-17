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
/*  Two-boundary lane centering                                       */
/* ------------------------------------------------------------------ */
#define LC_MAX_CANDIDATES 10

lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors)
{
    lane_result_t result = {0.0f, 0.0f, 0};

    if (num_vectors == 0) return result;

    /* --- 1. Transform all vectors to ground space (no angle filter) --- */
    typedef struct {
        float x_mid;          /* midpoint X in ground space              */
        float angle;          /* heading from forward (radians)          */
    } candidate_t;

    candidate_t cands[LC_MAX_CANDIDATES];
    size_t      n_cands = 0;

    for (size_t i = 0; i < num_vectors && i < LC_MAX_CANDIDATES; i++) {
        float u0 = vectors[4*i + 0] * PIXY_GRID_SCALE;
        float v0 = vectors[4*i + 1] * PIXY_GRID_SCALE;
        float u1 = vectors[4*i + 2] * PIXY_GRID_SCALE;
        float v1 = vectors[4*i + 3] * PIXY_GRID_SCALE;

        float X0, Y0, X1, Y1;
        pixel_to_ground(u0, v0, &X0, &Y0);
        pixel_to_ground(u1, v1, &X1, &Y1);

        cands[n_cands].x_mid = (X0 + X1) * 0.5f;
        cands[n_cands].angle = atan2f(X1 - X0, Y1 - Y0);
        n_cands++;
    }

    if (n_cands == 0) return result;

    /* --- 2. Single vector: follow it directly, no offset guessing --- */
    if (n_cands == 1) {
        result.cte         = cands[0].x_mid;
        result.heading     = cands[0].angle;
        result.lines_found = 1;
        return result;
    }

    /* --- 3. Multiple vectors: try to find one on each side --- */
    int   best_left  = -1,  best_right  = -1;
    float best_lx    = -1e6f;          /* want largest  X (closest to center from left)  */
    float best_rx    =  1e6f;          /* want smallest X (closest to center from right) */

    for (size_t i = 0; i < n_cands; i++) {
        if (cands[i].x_mid < 0.0f) {
            if (cands[i].x_mid > best_lx) {
                best_lx   = cands[i].x_mid;
                best_left = (int)i;
            }
        } else {
            if (cands[i].x_mid < best_rx) {
                best_rx    = cands[i].x_mid;
                best_right = (int)i;
            }
        }
    }

    if (best_left >= 0 && best_right >= 0) {
        /* Both sides found — center between them */
        result.cte     = (cands[best_left].x_mid + cands[best_right].x_mid) * 0.5f;
        result.heading = (cands[best_left].angle  + cands[best_right].angle)  * 0.5f;
        result.lines_found = 2;
    } else {
        /* All vectors on one side — follow the one closest to center */
        int   best     = 0;
        float best_abs = fabsf(cands[0].x_mid);
        for (size_t i = 1; i < n_cands; i++) {
            float d = fabsf(cands[i].x_mid);
            if (d < best_abs) {
                best_abs = d;
                best     = (int)i;
            }
        }
        result.cte         = cands[best].x_mid;
        result.heading     = cands[best].angle;
        result.lines_found = 1;
    }

    return result;
}
