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

//TODO: look over the entire vectors.c file
//testeaza cum sunt vazuti vectorii de la pixy, si cum sunt transformati in coordonate (maybe foloseste python pt vizualizare)
//refa transformarea homograpfica

/* ------------------------------------------------------------------ */
/*  Types                                                              */
/*  NOTE: defined here because vectors.h wasn't available to edit —    */
/*  move these into vectors.h alongside lane_result_t so other files   */
/*  (e.g. wherever the PID/steering code ends up) can use them too.    */
/* ------------------------------------------------------------------ */






static void float_parts(float val, const char **sign, int *ip, int *fp)
{
    *sign = (val < 0.0f) ? "-" : "";
    float a = fabsf(val);
    *ip = (int)a;
    *fp = (int)((a - (float)(*ip)) * 100.0f + 0.5f); // +0.5 = round, not truncate
    if (*fp >= 100) { *fp -= 100; (*ip)++; }          // carry (e.g. 2.999 -> 3.00)
}


/* ------------------------------------------------------------------ */
/*  Homography: pixel -> ground plane (cm)                             */
/*                                                                      */
/*  Returns 1 on success, 0 on failure. On failure X/Y are NOT written  */
/*  — callers must check the return value, not assume (0,0) means      */
/*  "point is under the camera." A degenerate Wp or an extrapolated    */
/*  point far outside the calibrated grid both fail closed instead of   */
/*  silently returning a plausible-looking wrong answer.                */
/*                                                                      */
/*  BREAKING CHANGE from the previous void version: any other call     */
/*  site (main.c, elsewhere) needs to check this return value now.      */
/*  This file's own compute_lane_center() is patched below to do so.    */
/*                                                                        */
/*  RECALIBRATION NOTE: H below must be calibrated using u,v values      */
/*  read directly from vectors[] (0-79, 0-52 range — Pixy2's native      */
/*  line-tracking grid), NOT PixyMon's on-screen cursor coordinates.     */
/*  Those are two different coordinate systems with no documented       */
/*  conversion between them. If H was fit against PixyMon-cursor         */
/*  values, every output here will be extrapolated garbage that still    */
/*  looks plausible (smooth, parallel-ish, wrong).                       */
/* ------------------------------------------------------------------ */
uint8_t pixel_to_ground(float u, float v, float *X, float *Y)
{
    float Wp = H31 * u + H32 * v + H33;

    if (fabsf(Wp) < 1e-6f) {
        return 0; // degenerate — do not treat as a valid (0,0)
    }

    float Xr = (H11 * u + H12 * v + H13) / Wp;
    float Yr = (H21 * u + H22 * v + H23) / Wp;

    // Bounds are in ground-space cm, unaffected by the grid-vs-display
    // pixel issue above. Re-check these against your new calibration's
    // reprojection error once you have it — the plausible X/Y range may
    // shift slightly with the new, correctly-calibrated H.
    if (Yr < 8.4f || Yr > 135.6f || fabsf(Xr) > (0.330f * Yr + 59.25f)) {
        return 0;
    }

    *X = Xr;
    *Y = Yr;
    return 1;
}


void print_vector_details(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, size_t index)
{
    int dx = (int)x1 - (int)x0;
    int dy = (int)y1 - (int)y0;

    float length = sqrtf((float)(dx * dx + dy * dy));

    float angle_deg = 0.0f;
    if (dy != 0) {
        angle_deg = atan2f((float)dx, (float)(-dy)) * 180.0f / (float)M_PI;
    } else {
        angle_deg = (dx >= 0) ? 90.0f : -90.0f;
    }

    const char *s;
    int ip, fp;

    esp_printf("Vector [%u] details:\r\n", (unsigned)index);
    esp_printf("  Pixel: (%u, %u) -> (%u, %u)\r\n", (unsigned)x0, (unsigned)y0, (unsigned)x1, (unsigned)y1);

    float_parts(length, &s, &ip, &fp);
    esp_printf("  Length: %s%d.%02d px\r\n", s, ip, fp);

    float_parts(angle_deg, &s, &ip, &fp);
    esp_printf("  Pixel angle: %s%d.%02d deg\r\n", s, ip, fp);

    /* Ground-space (homography-transformed) coordinates. This is the
     * physically meaningful space for slope — pixel-space slope isn't,
     * since perspective compresses it non-linearly with distance
     * (this is the whole reason the homography step exists). */

    // No scaling here on purpose — see select_boundary_vectors() below
    // for why. x0..y1 are Pixy's native 79x52 grid values, fed straight
    // into a homography now calibrated for that same grid.
    float gu0 = (float)x0;
    float gv0 = (float)y0;
    float gu1 = (float)x1;
    float gv1 = (float)y1;

    float X0, Y0, X1, Y1;
    uint8_t ok0 = pixel_to_ground(gu0, gv0, &X0, &Y0);
    uint8_t ok1 = pixel_to_ground(gu1, gv1, &X1, &Y1);

    if (!ok0 || !ok1) {
        esp_printf("  Ground: OUT OF CALIBRATED RANGE\r\n");
        return;
    }
    /* reject near-horizontal vectors so Python doesn't plot them */
    float dX_g = X1 - X0;
    float dY_g = Y1 - Y0;
    if (fabsf(dY_g) < 1e-3f || fabsf(dX_g / dY_g) > 1.73f) {
        esp_printf("  Ground: FILTERED (too horizontal)\r\n");
        return;
    }

    float_parts(X0, &s, &ip, &fp);
    esp_printf("  Ground start: (%s%d.%02d, ", s, ip, fp);
    float_parts(Y0, &s, &ip, &fp);
    esp_printf("%s%d.%02d) cm\r\n", s, ip, fp);

    float_parts(X1, &s, &ip, &fp);
    esp_printf("  Ground end:   (%s%d.%02d, ", s, ip, fp);
    float_parts(Y1, &s, &ip, &fp);
    esp_printf("%s%d.%02d) cm\r\n", s, ip, fp);

    // Slope = dX/dY (lateral cm per forward cm), as literally asked.
    // Caveat: this blows up toward infinity for a line that's nearly
    // horizontal in ground space (perpendicular to the direction of
    // travel) — that's a real singularity, not a bug. Fine for
    // printing/plotting in Python; don't feed it into control logic
    // downstream without a guard. Ground angle doesn't have this
    // problem if you need something robust instead.
    float dY = Y1 - Y0;
    if (fabsf(dY) > 1e-3f) {
        float slope = (X1 - X0) / dY;
        float_parts(slope, &s, &ip, &fp);
        esp_printf("  Ground slope (dX/dY): %s%d.%02d\r\n", s, ip, fp);
    } else {
        esp_printf("  Ground slope: undefined (near-horizontal in ground space)\r\n");
    }
}


/* ------------------------------------------------------------------ */
/*  Step 1: transform raw Pixy vectors to ground space, pick the two   */
/*  boundary lines (leftmost / rightmost by ground-space X).           */
/*                                                                      */
/*  This is deliberately "dumb" — no left/right identity logic, no     */
/*  handling of intersections/gaps/single-line cases beyond what's     */
/*  described below. That's the classification work you said you'd    */
/*  do later; this just answers "of what Pixy saw this frame, which    */
/*  two vectors are the outermost."                                     */
/*                                                                      */
/*  Returns count of boundary vectors found (0, 1, or 2).               */
/*  n==1: the single vector found is placed in *left by convention —   */
/*  no actual side classification happened. Treat it as "the one line  */
/*  we saw," not "confirmed left boundary."                             */
/* ------------------------------------------------------------------ */
#define SBV_MAX 10

uint8_t select_boundary_vectors(const uint16_t *vectors, size_t num_vectors,
                                 ground_vector_t *left, ground_vector_t *right)
{
    left->valid = 0;
    right->valid = 0;

    if (num_vectors == 0) return 0;

    ground_vector_t v[SBV_MAX];
    float x_mid[SBV_MAX];
    size_t n = 0;

    for (size_t i = 0; i < num_vectors && i < SBV_MAX; i++) {
        // No scaling here on purpose. vectors[] is Pixy2's native line-
        // tracking grid — confirmed 79 wide x 52 tall by Pixy's own docs
        // (pixy.frameWidth/frameHeight in line-tracking mode). H is now
        // calibrated directly against that grid (see calibration note in
        // pixel_to_ground below), so these raw values go straight in.
        float u0 = (float)vectors[6*i + 0];
        float v0 = (float)vectors[6*i + 1];
        float u1 = (float)vectors[6*i + 2];
        float v1 = (float)vectors[6*i + 3];

        float X0, Y0, X1, Y1;
        uint8_t ok0 = pixel_to_ground(u0, v0, &X0, &Y0);
        uint8_t ok1 = pixel_to_ground(u1, v1, &X1, &Y1);

        if (!ok0 || !ok1) continue; // drop vectors outside the calibrated/plausible ground region

        float dX = X1 - X0;
        float dY_vec = Y1 - Y0;
        if (fabsf(dY_vec) < 1e-3f || fabsf(dX / dY_vec) > 1.73f) {
            continue;   /* slope > ~60° from forward — not a lane line */
        }

        v[n].X0 = X0; v[n].Y0 = Y0;
        v[n].X1 = X1; v[n].Y1 = Y1;
        v[n].valid = 1;
        x_mid[n] = (X0 + X1) * 0.5f;
        n++;
    }

    if (n == 0) return 0;

    if (n == 1) {
        *left = v[0];
        return 1;
    }

    size_t li = 0, ri = 0;
    for (size_t i = 1; i < n; i++) {
        if (x_mid[i] < x_mid[li]) li = i;
        if (x_mid[i] > x_mid[ri]) ri = i;
    }

    *left  = v[li];
    *right = v[ri];
    return 2;
}


/* ------------------------------------------------------------------ */
/*  Step 3: build the centerline from the two boundary vectors.        */
/*  No curvature — straight-line interpolation only, as asked.         */
/*                                                                      */
/*  Left and right vectors will generally NOT span the same Y range    */
/*  (Pixy detects each independently). Naively averaging endpoint-to-  */
/*  endpoint would compare points at different forward distances,      */
/*  which isn't a meaningful "center." Instead this finds the Y range  */
/*  BOTH lines actually cover and interpolates each line to match.     */
/* ------------------------------------------------------------------ */
static uint8_t interp_x_at_y(const ground_vector_t *v, float Y_ref, float *X_out)
{
    float Ya = v->Y0, Xa = v->X0;
    float Yb = v->Y1, Xb = v->X1;

    if (Ya > Yb) {
        float t;
        t = Ya; Ya = Yb; Yb = t;
        t = Xa; Xa = Xb; Xb = t;
    }

    float dY = Yb - Ya;
    if (fabsf(dY) < 1e-3f) {
        return 0; // line is near-horizontal in ground space — can't interpolate by Y
    }

    float t = (Y_ref - Ya) / dY;
    *X_out = Xa + t * (Xb - Xa);
    return 1;
}

center_line_t compute_center_line(const ground_vector_t *left, const ground_vector_t *right)
{
    center_line_t result;
    memset(&result, 0, sizeof(result));

    if (!left->valid || !right->valid) {
        return result; // need both boundaries — single-line fallback is future work
    }

    float lY_near = fminf(left->Y0, left->Y1);
    float lY_far  = fmaxf(left->Y0, left->Y1);
    float rY_near = fminf(right->Y0, right->Y1);
    float rY_far  = fmaxf(right->Y0, right->Y1);

    float Y_near = fmaxf(lY_near, rY_near); // overlapping forward-distance
    float Y_far  = fminf(lY_far, rY_far);   // range both lines cover

    if (Y_far <= Y_near) {
        return result; // no usable overlap between the two lines
    }

    float Xl_near, Xl_far, Xr_near, Xr_far;
    if (!interp_x_at_y(left,  Y_near, &Xl_near)) return result;
    if (!interp_x_at_y(left,  Y_far,  &Xl_far))  return result;
    if (!interp_x_at_y(right, Y_near, &Xr_near)) return result;
    if (!interp_x_at_y(right, Y_far,  &Xr_far))  return result;

    result.X0 = (Xl_near + Xr_near) * 0.5f;
    result.Y0 = Y_near;
    result.X1 = (Xl_far + Xr_far) * 0.5f;
    result.Y1 = Y_far;

    // cte is the lateral offset at the NEAREST point both lines actually
    // covered — deliberately not extrapolated out to Y=0 (under the car).
    // Extrapolating past the calibrated/validated homography range is
    // exactly the risk flagged earlier in this whole homography exercise.
    // Push this to Y=0 later if you find you need it; starting
    // conservative here.
    result.cte     = result.X0;
    result.heading = atan2f(result.X1 - result.X0, result.Y1 - result.Y0); // radians
    result.valid   = 1;

    return result;
}


/* ------------------------------------------------------------------ */
/*  Debug dump: every raw vector Pixy saw this frame (pixel + ground +  */
/*  slope, via print_vector_details), plus what select_boundary_vectors */
/*  + compute_center_line currently conclude from them.                 */
/*                                                                        */
/*  Call this once per frame to see raw vector behavior before designing */
/*  intersection / corner / left-right logic. Needs a prototype in       */
/*  vectors.h to be callable from main.c.                                */
/*                                                                        */
/*  Heavy: this is several blocking esp_printf calls per vector, every   */
/*  frame. See the note where this gets called in main.c about what      */
/*  that does to your fixed-dt PID loop.                                 */
/* ------------------------------------------------------------------ */
void print_frame_debug(const uint16_t *vectors, size_t num_vectors)
{
    for (size_t i = 0; i < num_vectors && i < SBV_MAX; i++) {
        // vectors[6*i+4] is Pixy's own persistent tracking idx for this
        // line, not a loop counter — lets you see whether Pixy keeps
        // calling the same physical line the same idx frame to frame.
        print_vector_details(vectors[6*i + 0], vectors[6*i + 1],
                              vectors[6*i + 2], vectors[6*i + 3],
                              (size_t)vectors[6*i + 4]);
    }

    ground_vector_t left, right;
    uint8_t found = select_boundary_vectors(vectors, num_vectors, &left, &right);
    esp_printf("Boundary vectors selected: %u\r\n", (unsigned)found);

    if (found == 2) {
        center_line_t c = compute_center_line(&left, &right);
        const char *s;
        int ip, fp;

        if (c.valid) {
            float_parts(c.cte, &s, &ip, &fp);
            esp_printf("  Center cte: %s%d.%02d cm\r\n", s, ip, fp);

            float_parts(c.heading * 180.0f / (float)M_PI, &s, &ip, &fp);
            esp_printf("  Center heading: %s%d.%02d deg\r\n", s, ip, fp);
        } else {
            esp_printf("  Center: boundaries found but no Y overlap between them\r\n");
        }
        esp_printf("---FRAME_END---\r\n");
    }
}


/* ------------------------------------------------------------------ */
/*  DEPRECATED — superseded by select_boundary_vectors() +             */
/*  compute_center_line() above, which do the same job split into      */
/*  the pieces you asked for, plus: bounds-checked pixel_to_ground      */
/*  (this one didn't check it), and Y-matched interpolation instead     */
/*  of raw per-line X-midpoint averaging.                               */
/*                                                                      */
/*  Left in place since other code may still call it — patched only     */
/*  so it doesn't read uninitialized X/Y now that pixel_to_ground can    */
/*  fail without writing its output params. Recommend migrating call    */
/*  sites and removing this once confirmed nothing else needs it.       */
/* ------------------------------------------------------------------ */
#define LC_MAX 10

lane_result_t compute_lane_center(uint16_t *vectors, size_t num_vectors)
{
    lane_result_t result = {0.0f, 0.0f, 0};
    if (num_vectors == 0) return result;

    struct {
        float   x_mid;
        float   angle;
        uint8_t idx;
    } v[LC_MAX];
    size_t n = 0;

    for (size_t i = 0; i < num_vectors && i < LC_MAX; i++) {
        // No scaling — see select_boundary_vectors() above for why.
        float u0 = (float)vectors[6*i + 0];
        float v0 = (float)vectors[6*i + 1];
        float u1 = (float)vectors[6*i + 2];
        float v1 = (float)vectors[6*i + 3];

        float X0 = 0.0f, Y0 = 0.0f, X1 = 0.0f, Y1 = 0.0f;
        uint8_t ok0 = pixel_to_ground(u0, v0, &X0, &Y0);
        uint8_t ok1 = pixel_to_ground(u1, v1, &X1, &Y1);
        if (!ok0 || !ok1) continue; // patched: skip rather than use uninitialized X/Y

        v[n].x_mid = (X0 + X1) * 0.5f;
        v[n].angle = atan2f(X1 - X0, Y1 - Y0);
        v[n].idx   = (uint8_t)vectors[6*i + 4];
        n++;
    }

    if (n == 0) return result;

    if (n == 1) {
        result.cte         = v[0].x_mid;
        result.heading     = v[0].angle;
        result.lines_found = 1;
        return result;
    }

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
