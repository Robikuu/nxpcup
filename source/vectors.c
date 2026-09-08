#include <math.h>
#include <stdint.h>
#include <string.h>
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "wifi_config.h"
#include "vectors.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#define SBV_MAX 10

/* --- Pixel-Space Configuration Thresholds --- */
#define PIXY_CENTER_X      39.5f  /* Half of 79px width */
#define PIXY_HEIGHT        52.0f  /* Total Pixy height */

#define MAX_START_Y_PX     18.0f  /* Must start in the bottom 35% of the screen */
#define MAX_SLOPE_RATIO    3.0f   /* Reject lines flatter than ~71 degrees */
#define MIN_DY_PX          5.0f   /* Reject lines that lack vertical depth (horizontal/distant noise) */

#define LANE_WIDTH_PX      75.0f  /* Expected lane width in pixels at the absolute bottom */
#define MIN_PAIR_SEP_PX    20.0f  /* Keep this small to allow for high-perspective pinch */
#define MAX_PAIR_SEP_PX    80.0f  /* Must be 80 to allow lines on both extreme edges */
#define MAX_SINGLE_DIST_PX 40.0f  /* Must be 40 to allow a single line on the extreme edge */

static void float_parts(float val, const char **sign, int *ip, int *fp)
{
    *sign = (val < 0.0f) ? "-" : "";
    float a = fabsf(val);
    *ip = (int)a;
    *fp = (int)((a - (float)(*ip)) * 100.0f + 0.5f);
    if (*fp >= 100) { *fp -= 100; (*ip)++; }
}

uint8_t select_boundary_vectors(const uint16_t *vectors, size_t num_vectors,
                                 ground_vector_t *left, ground_vector_t *right)
{
    left->valid = 0;
    right->valid = 0;

    if (num_vectors == 0) return 0;

    ground_vector_t v[SBV_MAX];
    size_t n = 0;

    for (size_t i = 0; i < num_vectors && i < SBV_MAX; i++) {
        float u0 = (float)vectors[6*i + 0];
        float v0 = (float)vectors[6*i + 1];
        float u1 = (float)vectors[6*i + 2];
        float v1 = (float)vectors[6*i + 3];

        /* Shift Pixy's 79x52 grid: X is 0 at center, Y is 0 at bottom (nearest to car) */
        float X0 = u0 - PIXY_CENTER_X;
        float Y0 = PIXY_HEIGHT - v0;
        float X1 = u1 - PIXY_CENTER_X;
        float Y1 = PIXY_HEIGHT - v1;

        /* Standardize so (X0, Y0) is always the point closest to the car */
        if (Y0 > Y1) {
            float tmpX = X0; X0 = X1; X1 = tmpX;
            float tmpY = Y0; Y0 = Y1; Y1 = tmpY;
        }

        /* 1. Bottom-Anchored ROI: Reject distant background noise */
        if (Y0 > MAX_START_Y_PX) continue;

        float abs_dX = fabsf(X1 - X0);
        float abs_dY = fabsf(Y1 - Y0);

        /* 2. Vertical Depth Filter: True lane lines travel away from the camera (up the Y axis).
           Distant cross-tracks appear horizontally compressed. */
        if (abs_dY < MIN_DY_PX) continue;

        /* 3. Division-Free Slope Filter: Reject lines that are too horizontal */
        if (abs_dX > MAX_SLOPE_RATIO * abs_dY) continue;

        v[n].X0 = X0; v[n].Y0 = Y0;
        v[n].X1 = X1; v[n].Y1 = Y1;
        v[n].valid = 1;
        n++;
    }

    if (n == 0) return 0;
    if (n == 1) {
        *left = v[0];
        return 1;
    }

    /* 5. Select candidates by NEAREST Y0 (closest to car bumper), NOT widest X0 */
    int best_left = -1;
    int best_right = -1;
    float min_y_left  = PIXY_HEIGHT + 1.0f;
    float min_y_right = PIXY_HEIGHT + 1.0f;

    for (size_t i = 0; i < n; i++) {
        /* Split the screen into left and right halves based on X0 */
        if (v[i].X0 < 0.0f) {
            if (v[i].Y0 < min_y_left) {
                min_y_left = v[i].Y0;
                best_left  = (int)i;
            }
        } else {
            if (v[i].Y0 < min_y_right) {
                min_y_right = v[i].Y0;
                best_right  = (int)i;
            }
        }
    }

    /* 6. Validate pair separation at the bottom of the screen */
    if (best_left != -1 && best_right != -1) {
        float bottom_sep = v[best_right].X0 - v[best_left].X0; 
        if (bottom_sep >= MIN_PAIR_SEP_PX && bottom_sep <= MAX_PAIR_SEP_PX) {
            *left  = v[best_left];
            *right = v[best_right];
            return 2;
        }
    }

    /* 7. Fallback: Pair failed. Pick the single line physically closest to the bottom */
    int best_single = -1;
    float min_y_single = PIXY_HEIGHT + 1.0f;
    
    for (size_t i = 0; i < n; i++) {
        if (v[i].Y0 < min_y_single) { 
            min_y_single = v[i].Y0; 
            best_single  = (int)i; 
        }
    }
    
    if (best_single != -1) {
        /* Curb-Dive Protection */
        if (fabsf(v[best_single].X0) > MAX_SINGLE_DIST_PX) {
            return 0; 
        }
        *left = v[best_single];
        return 1;
    }

    return 0;

}

static uint8_t interp_x_at_y(const ground_vector_t *v, float Y_ref, float *X_out)
{
    float Ya = v->Y0, Xa = v->X0;
    float Yb = v->Y1, Xb = v->X1;

    if (Ya > Yb) {
        float t = Ya; Ya = Yb; Yb = t;
        t = Xa; Xa = Xb; Xb = t;
    }

    float dY = Yb - Ya;
    if (fabsf(dY) < 1e-3f) return 0; 

    float t = (Y_ref - Ya) / dY;
    *X_out = Xa + t * (Xb - Xa);
    return 1;
}

center_line_t compute_center_line(const ground_vector_t *left, const ground_vector_t *right)
{
    center_line_t result;
    memset(&result, 0, sizeof(result));

    if (!left->valid || !right->valid) return result; 

    float lY_near = fminf(left->Y0, left->Y1);
    float lY_far  = fmaxf(left->Y0, left->Y1);
    float rY_near = fminf(right->Y0, right->Y1);
    float rY_far  = fmaxf(right->Y0, right->Y1);

    float Y_near = fmaxf(lY_near, rY_near); 
    float Y_far  = fminf(lY_far, rY_far);   

    if (Y_far <= Y_near) return result; 

    float Xl_near, Xl_far, Xr_near, Xr_far;
    if (!interp_x_at_y(left,  Y_near, &Xl_near)) return result;
    if (!interp_x_at_y(left,  Y_far,  &Xl_far))  return result;
    if (!interp_x_at_y(right, Y_near, &Xr_near)) return result;
    if (!interp_x_at_y(right, Y_far,  &Xr_far))  return result;

    result.X0 = (Xl_near + Xr_near) * 0.5f;
    result.Y0 = Y_near;
    result.X1 = (Xl_far + Xr_far) * 0.5f;
    result.Y1 = Y_far;

    result.cte     = result.X0;
    result.heading = atan2f(result.X1 - result.X0, result.Y1 - result.Y0);
    result.valid   = 1;

    return result;
}

center_line_t compute_center_line_single(const ground_vector_t *line)
{
    center_line_t result;
    memset(&result, 0, sizeof(result));

    if (!line->valid) return result;

    float x_near = line->X0, y_near = line->Y0;
    float x_far  = line->X1, y_far  = line->Y1;
    
    if (y_near > y_far) {
        x_near = line->X1; y_near = line->Y1;
        x_far  = line->X0; y_far  = line->Y0;
    }

    float heading = atan2f(x_far - x_near, y_far - y_near);
    
    /* 
     * PIXEL SPACE FIX:
     * Shift strictly horizontally. Do not multiply by cosf().
     */
    float shift_x = LANE_WIDTH_PX / 2.0f;
    
    /* Determine left vs right strictly by bottom anchor */
    int is_right_line = (x_near > 0.0f);

    if (is_right_line) {
        shift_x = -shift_x;
    }

    /* Apply flat shift */
    result.X0 = x_near + shift_x;
    result.Y0 = y_near; 
    result.X1 = x_far + shift_x;
    result.Y1 = y_far;  

    result.cte = result.X0;
    result.heading = heading;
    result.valid = 1;

    return result;
}

void print_frame_debug(const uint16_t *vectors, size_t num_vectors)
{
    for (size_t i = 0; i < num_vectors && i < SBV_MAX; i++) {
        float u0 = (float)vectors[6*i + 0];
        float v0 = (float)vectors[6*i + 1];
        float u1 = (float)vectors[6*i + 2];
        float v1 = (float)vectors[6*i + 3];
        float X0 = u0 - PIXY_CENTER_X;
        float Y0 = PIXY_HEIGHT - v0;
        float X1 = u1 - PIXY_CENTER_X;
        float Y1 = PIXY_HEIGHT - v1;

        const char *s; int ip, fp;
        esp_printf("Vec[%u]: Raw=(%.1f,%.1f)->(%.1f,%.1f)\r\n",
                   (unsigned)vectors[6*i + 4], u0, v0, u1, v1);
        
        float_parts(X0, &s, &ip, &fp); esp_printf("  Shifted Start: (%s%d.%02d, ", s, ip, fp);
        float_parts(Y0, &s, &ip, &fp); esp_printf("%s%d.%02d) px\r\n", s, ip, fp);
    }

    ground_vector_t left, right;
    uint8_t found = select_boundary_vectors(vectors, num_vectors, &left, &right);
    esp_printf("Boundaries found: %u\r\n", (unsigned)found);

    if (found == 2) {
        center_line_t c = compute_center_line(&left, &right);
        if (c.valid) {
            esp_printf("  Center cte: %d px\r\n", (int)c.cte);
            esp_printf("  Center heading: %d deg\r\n", (int)(c.heading * 180.0f / M_PI));
        } else {
            esp_printf("  Center: no Y overlap\r\n");
        }
    }
}