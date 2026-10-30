/* Regression test for the corner-exit / off-center recovery vision fixes in source/vectors.c.
 * Frames are copied verbatim from the on-track `dump 1` logs (Run 3 CCW, Run 4 CW).
 * Build (host clang):  clang -std=gnu99 -Iinclude -Isource scratch/test_exit_recovery.c source/vectors.c
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "Config.h"
#include "vectors.h"

static int g_fail = 0;
#define CHECK(cond, ...) do { if (cond) { printf("[PASS] "); } else { printf("[FAIL] "); g_fail++; } \
                              printf(__VA_ARGS__); printf("\n"); } while (0)

#define STEP(arr, sr) compute_top_camera_lane((arr), sizeof(arr) / (4 * sizeof((arr)[0])), &(sr))

int main(void)
{
    steering_result_t sr;

    /* ---- Straight, 2 boundaries: lane width must be exposed ---- */
    reset_steering_tracking();
    { uint16_t v[] = {14,50,14,16, 64,50,64,16}; STEP(v, sr); }
    CHECK(sr.found == 2 && fabsf(sr.lane_width_px - 50.0f) < 0.5f,
          "straight lane found=%u width=%.1f (expect 2 / 50)", sr.found, sr.lane_width_px);

    /* ---- Chassis / tie-rod / wheel exclusion must still reject interior noise ---- */
    reset_steering_tracking();
    { uint16_t v[] = {32,43,45,42}; STEP(v, sr); }
    CHECK(sr.found == 0, "chassis-only artifact rejected (found=%u)", sr.found);
    reset_steering_tracking();
    { uint16_t v[] = {35,41,52,42, 65,50,63,15}; STEP(v, sr); }
    CHECK(sr.found == 1 && sr.right_found == 1 && sr.left_found == 0,
          "tie-rod rejected, right lane kept (found=%u)", sr.found);

    /* ---- Run 3 Frame 46: boundary leaving through left border (x_far=0) must be accepted ---- */
    reset_steering_tracking();
    { uint16_t v[] = {32,51,0,17, 78,29,52,0}; STEP(v, sr); }   /* pre-frame: establishes tracking */
    { uint16_t v[] = {78,32,51,6, 30,51,0,28}; STEP(v, sr); }
    CHECK(sr.found == 2 && sr.left_found == 1,
          "Run3 F46 left border-exit boundary (30,51)->(0,28) accepted (found=%u L=%u)", sr.found, sr.left_found);

    /* ---- Interior (non-border) long-ish vector in the chassis box must STILL be rejected ---- */
    reset_steering_tracking();
    { uint16_t v[] = {42,51,44,30}; STEP(v, sr); }              /* Run 3 F54 shape: y_far=30, x_far=44 */
    CHECK(sr.found == 0, "interior vector (42,51)->(44,30) still excluded (found=%u)", sr.found);

    /* ---- Run 4 CW corner exit: sideways car frames 24..33 ---- */
    reset_steering_tracking();
    { uint16_t v[] = {12,51,38,0, 70,51,78,3};  STEP(v, sr); }  /* F24 */
    { uint16_t v[] = {12,51,26,0, 70,51,57,0};  STEP(v, sr); }  /* F25 straight */
    CHECK(sr.found == 2 && fabsf(sr.heading_deg) < 5.0f && fabsf(sr.lane_width_px) > 35.0f,
          "Run4 F25 straight exit: found=%u H=%.1f width=%.1f", sr.found, sr.heading_deg, sr.lane_width_px);
    { uint16_t v[] = {10,51,13,0, 74,51,44,0};  STEP(v, sr); }  /* F26 */
    { uint16_t v[] = {3,51,0,42, 70,43,26,0, 77,25,64,0, 71,43,78,41}; STEP(v, sr); } /* F27 */

    float heading[6]; uint8_t found[6]; uint8_t lw[6]; int n = 0;
    { uint16_t v[] = {62,39,10,4, 67,17,53,0, 72,37,78,18, 63,39,72,37};          STEP(v, sr); heading[n]=sr.heading_deg; found[n]=sr.found; lw[n++]=sr.left_found; } /* F28 */
    { uint16_t v[] = {60,41,2,12, 63,16,44,0, 68,39,74,10, 61,41,68,39, 2,10,9,1}; STEP(v, sr); heading[n]=sr.heading_deg; found[n]=sr.found; lw[n++]=sr.left_found; } /* F29 */
    { uint16_t v[] = {60,47,2,14, 64,20,42,0, 73,43,76,12, 61,47,72,44, 2,12,8,3}; STEP(v, sr); heading[n]=sr.heading_deg; found[n]=sr.found; lw[n++]=sr.left_found; } /* F30 */
    { uint16_t v[] = {55,51,5,13, 73,32,47,0, 10,4,16,0, 5,13,8,5};               STEP(v, sr); heading[n]=sr.heading_deg; found[n]=sr.found; lw[n++]=sr.left_found; } /* F31 */
    { uint16_t v[] = {48,51,6,16, 75,35,52,1, 6,16,8,7};                          STEP(v, sr); heading[n]=sr.heading_deg; found[n]=sr.found; lw[n++]=sr.left_found; } /* F32 */
    { uint16_t v[] = {28,50,4,22, 78,47,50,7, 8,11,24,0, 4,22,8,11};              STEP(v, sr); heading[n]=sr.heading_deg; found[n]=sr.found; lw[n++]=sr.left_found; } /* F33 */

    int all_neg = 1, no_false_pair = 1;
    for (int i = 0; i < n; i++) {
        if (!(heading[i] <= -35.0f)) all_neg = 0;
        if (found[i] != 1) no_false_pair = 0;
    }
    CHECK(all_neg, "Run4 F28-F33 heading stays strongly negative [%.1f %.1f %.1f %.1f %.1f %.1f] (no flip to +13.5)",
          heading[0], heading[1], heading[2], heading[3], heading[4], heading[5]);
    CHECK(no_false_pair, "Run4 F28-F33 no false two-line pairing from chained fragments (found=%u %u %u %u %u %u)",
          found[0], found[1], found[2], found[3], found[4], found[5]);

    /* ---- Span weighting: long foreground boundary beats short edge fragment nearer to tracking ---- */
    reset_steering_tracking();
    { uint16_t v[] = {12,51,26,0, 70,51,57,0}; STEP(v, sr); }   /* establish tracking, R boundary x_proj~68 */
    { uint16_t v[] = {60,47,50,0,  76,20,76,5, 0,0,0,0}; STEP(v, sr); }
    CHECK(sr.found >= 1, "long boundary + short edge fragment resolves (found=%u)", sr.found);

    printf("\n%s (%d failures)\n", g_fail ? "FAILED" : "ALL EXIT-RECOVERY VISION TESTS PASSED", g_fail);
    return g_fail ? 1 : 0;
}
