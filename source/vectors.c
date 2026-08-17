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
/*  Two-boundary lane centering (direction-normalized & fixed lookahead) */
/*  vectors[] layout: 6 fields per vector                             */
/*    [0]=x0  [1]=y0  [2]=x1  [3]=y1  [4]=index  [5]=flags           */
/* ------------------------------------------------------------------ */
#define LC_MAX 10

lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors)
{
    lane_result_t result = {0.0f, 0.0f, 0};
    if (num_vectors == 0) return result;

    typedef struct {
        float   x_eval;   /* lateral position X evaluated at LOOKAHEAD_Y_M */
        float   angle;    /* forward-facing heading angle in radians       */
        uint8_t idx;
    } cand_t;

    cand_t cands[LC_MAX];
    size_t n_cands = 0;

    for (size_t i = 0; i < num_vectors && n_cands < LC_MAX; i++) {
        float u0 = vectors[6*i + 0] * PIXY_GRID_SCALE;
        float v0 = vectors[6*i + 1] * PIXY_GRID_SCALE;
        float u1 = vectors[6*i + 2] * PIXY_GRID_SCALE;
        float v1 = vectors[6*i + 3] * PIXY_GRID_SCALE;

        float X0, Y0, X1, Y1;
        pixel_to_ground(u0, v0, &X0, &Y0);
        pixel_to_ground(u1, v1, &X1, &Y1);

        /* 1. Direction Normalization:
              Ensure (X0, Y0) is ALWAYS the near point (smaller Y)
              and (X1, Y1) is the far point (larger Y).
              This guarantees the vector strictly points forward. */
        if (Y0 > Y1) {
            float tx = X0; float ty = Y0;
            X0 = X1; Y0 = Y1;
            X1 = tx; Y1 = ty;
        }

        /* 2. Discard if entire vector is beyond horizon limit */
        if (Y0 > MAX_GROUND_Y_M) {
            continue;
        }

        float dy = Y1 - Y0;
        float dx = X1 - X0;

        /* 3. Compute forward heading angle (strictly in [-pi/2, pi/2]) */
        float angle = atan2f(dx, dy + 1e-5f);

        /* 4. Evaluate lateral position at fixed lookahead distance */
        float x_eval;
        if (dy > 0.005f) {
            x_eval = X0 + (dx / dy) * (LOOKAHEAD_Y_M - Y0);
        } else {
            x_eval = (X0 + X1) * 0.5f;
        }

        /* Clamp x_eval to sane physical bounds (+/- 50 cm) */
        if (x_eval > 0.50f)  x_eval = 0.50f;
        if (x_eval < -0.50f) x_eval = -0.50f;

        cands[n_cands].x_eval = x_eval;
        cands[n_cands].angle  = angle;
        cands[n_cands].idx    = (uint8_t)vectors[6*i + 4];
        n_cands++;
    }

    if (n_cands == 0) return result;

    /* 5. Partition into left (x_eval < 0) and right (x_eval >= 0) */
    int   best_left  = -1;
    int   best_right = -1;
    float max_lx     = -1e6f;  /* closest to center from left (max negative) */
    float min_rx     =  1e6f;  /* closest to center from right (min positive) */

    for (size_t i = 0; i < n_cands; i++) {
        if (cands[i].x_eval < 0.0f) {
            if (cands[i].x_eval > max_lx) {
                max_lx    = cands[i].x_eval;
                best_left = (int)i;
            }
        } else {
            if (cands[i].x_eval < min_rx) {
                min_rx     = cands[i].x_eval;
                best_right = (int)i;
            }
        }
    }

    /* 6. Compute lane center */
    if (best_left >= 0 && best_right >= 0) {
        /* Both lane boundaries visible -> true center between them */
        result.cte     = (cands[best_left].x_eval + cands[best_right].x_eval) * 0.5f;
        result.heading = (cands[best_left].angle  + cands[best_right].angle)  * 0.5f;
        result.lines_found = 2;
    }
    else if (best_left >= 0) {
        /* Only left boundary -> center is half-lane to the right */
        result.cte     = cands[best_left].x_eval + (LANE_WIDTH_M * 0.5f);
        result.heading = cands[best_left].angle;
        result.lines_found = 1;
    }
    else if (best_right >= 0) {
        /* Only right boundary -> center is half-lane to the left */
        result.cte     = cands[best_right].x_eval - (LANE_WIDTH_M * 0.5f);
        result.heading = cands[best_right].angle;
        result.lines_found = 1;
    }

    return result;
}

