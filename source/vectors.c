#include <math.h>
#include <stdint.h>
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "wifi_config.h"
#include "vectors.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

//TODO: look over the entire vectors.c file
//testeaza cum sunt vazuti vectorii de la pixy, si cum sunt transformati in coordonate (maybe foloseste python pt vizualizare)
//refa transformarea homograpfica

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
/*  Two-boundary lane centering (index-aware)                         */
/*  vectors[] layout: 6 fields per vector                             */
/*    [0]=x0  [1]=y0  [2]=x1  [3]=y1  [4]=index  [5]=flags           */
/* ------------------------------------------------------------------ */
#define LC_MAX 10

lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors)
{
    lane_result_t result = {0.0f, 0.0f, 0};
    if (num_vectors == 0) return result;

    /* --- 1. Transform to ground space, keep Pixy index --- */
    struct {
        float   x_mid;
        float   angle;
        uint8_t idx;
    } v[LC_MAX];
    size_t n = 0;

    for (size_t i = 0; i < num_vectors && i < LC_MAX; i++) {
        float u0 = vectors[6*i + 0] * PIXY_GRID_SCALE;
        float v0 = vectors[6*i + 1] * PIXY_GRID_SCALE;
        float u1 = vectors[6*i + 2] * PIXY_GRID_SCALE;
        float v1 = vectors[6*i + 3] * PIXY_GRID_SCALE;

        float X0, Y0, X1, Y1;
        pixel_to_ground(u0, v0, &X0, &Y0);
        pixel_to_ground(u1, v1, &X1, &Y1);

        v[n].x_mid = (X0 + X1) * 0.5f;
        v[n].angle = atan2f(X1 - X0, Y1 - Y0);
        v[n].idx   = (uint8_t)vectors[6*i + 4];
        n++;
    }

    /* --- 2. Single vector: follow directly --- */
    if (n == 1) {
        result.cte         = v[0].x_mid;
        result.heading     = v[0].angle;
        result.lines_found = 1;
        return result;
    }

    /* --- 3. Group by Pixy index, compute average x_mid per group --- */
    struct {
        uint8_t idx;
        float   x_sum;
        float   a_sum;
        uint8_t cnt;
    } grp[LC_MAX];
    size_t ng = 0;

    for (size_t i = 0; i < n; i++) {
        int found = -1;
        for (size_t g = 0; g < ng; g++) {
            if (grp[g].idx == v[i].idx) { found = (int)g; break; }
        }
        if (found >= 0) {
            grp[found].x_sum += v[i].x_mid;
            grp[found].a_sum += v[i].angle;
            grp[found].cnt++;
        } else {
            grp[ng].idx   = v[i].idx;
            grp[ng].x_sum = v[i].x_mid;
            grp[ng].a_sum = v[i].angle;
            grp[ng].cnt   = 1;
            ng++;
        }
    }

    if (ng >= 2) {
        /* Two or more distinct lines — find leftmost and rightmost */
        int   gl = 0, gr = 0;
        float xl = grp[0].x_sum / grp[0].cnt;
        float xr = xl;

        for (size_t g = 1; g < ng; g++) {
            float avg = grp[g].x_sum / grp[g].cnt;
            if (avg < xl) { xl = avg; gl = (int)g; }
            if (avg > xr) { xr = avg; gr = (int)g; }
        }

        result.cte     = (xl + xr) * 0.5f;
        result.heading = (grp[gl].a_sum / grp[gl].cnt +
                          grp[gr].a_sum / grp[gr].cnt) * 0.5f;
        result.lines_found = 2;
    } else {
        /* All vectors belong to one line — follow closest to center */
        int   best   = 0;
        float best_d = fabsf(v[0].x_mid);
        for (size_t i = 1; i < n; i++) {
            float d = fabsf(v[i].x_mid);
            if (d < best_d) { best_d = d; best = (int)i; }
        }
        result.cte         = v[best].x_mid;
        result.heading     = v[best].angle;
        result.lines_found = 1;
    }

    return result;
}

