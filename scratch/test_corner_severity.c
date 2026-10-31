/* Host unit tests for severity-scaled corner speed (control_theory.c).
 * Build: clang -Iinclude -Isource scratch/test_corner_severity.c source/control_theory.c -o scratch/test_corner_severity.exe
 */
#include <stdio.h>
#include <math.h>
#include "Config.h"
#include "control_theory.h"

static int g_failures = 0;

#define CHECK(cond, msg, ...) do { \
    if (!(cond)) { printf("[FAIL] " msg "\n", ##__VA_ARGS__); g_failures++; } \
    else         { printf("[PASS] " msg "\n", ##__VA_ARGS__); } \
} while (0)

/* Defaults used by the car: fa_th = 20, h_lo = 12, h_hi = 25, spd_s = 100, spd_out = 55 */
#define FA_TH   20.0f
#define H_LO    12.0f
#define H_HI    25.0f
#define SPD_S   100.0f
#define SPD_OUT 55.0f

static float step(corner_severity_t *s, float fs, float h, uint8_t wall, uint8_t top_wall, uint8_t ix)
{
    return corner_severity_update(s, fs, h, wall, top_wall, ix, FA_TH, H_LO, H_HI);
}

int main(void)
{
    corner_severity_t cs;

    printf("=== 1. Straightaway: no severity, full speed ===\n");
    corner_severity_reset(&cs);
    {
        float sev = step(&cs, 10.0f, 5.0f, 0, 0, 0);
        float spd = corner_severity_speed(sev, SPD_S, SPD_OUT);
        CHECK(sev == 0.0f && spd == SPD_S, "FS=10, H=5 -> sev=%.2f, speed=%.1f", sev, spd);
    }

    printf("\n=== 2. 45-deg kink approach: partial braking, not 55%% ===\n");
    corner_severity_reset(&cs);
    {
        /* With FRONT_SEVERITY_FULL_DEG = 40.0f:
         * At FS=30: front_sev = (30 - 20) / (40 - 20) = 0.50 -> 77.5%
         * At FS=35: front_sev = (35 - 20) / (40 - 20) = 0.75 -> 66.25%
         */
        float sev30 = step(&cs, 30.0f, 0.0f, 0, 0, 0);
        float spd30 = corner_severity_speed(sev30, SPD_S, SPD_OUT);
        CHECK(fabsf(sev30 - 0.50f) < 0.01f && fabsf(spd30 - 77.5f) < 0.1f,
              "FS=30 kink -> sev=%.2f, speed=%.1f (expect 0.50 / 77.5)", sev30, spd30);
        float sev35 = step(&cs, 35.0f, 0.0f, 0, 0, 0);
        float spd35 = corner_severity_speed(sev35, SPD_S, SPD_OUT);
        CHECK(fabsf(sev35 - 0.75f) < 0.01f && fabsf(spd35 - 66.25f) < 0.1f,
              "FS=35 kink -> sev=%.2f, speed=%.2f (expect 0.75 / 66.25)", sev35, spd35);
    }

    printf("\n=== 3. Negative angles (left turns) behave the same as right ===\n");
    corner_severity_reset(&cs);
    {
        float sev = step(&cs, -30.0f, 0.0f, 0, 0, 0);
        CHECK(fabsf(sev - 0.50f) < 0.01f, "FS=-30 -> sev=%.2f (expect 0.50)", sev);
    }

    printf("\n=== 4. Front severity saturates at FRONT_SEVERITY_FULL_DEG (40) ===\n");
    corner_severity_reset(&cs);
    {
        float sev40 = step(&cs, 40.0f, 0.0f, 0, 0, 0);
        float sev65 = step(&cs, 65.0f, 0.0f, 0, 0, 0);
        CHECK(sev40 == 1.0f && sev65 == 1.0f, "FS=40 -> %.2f, FS=65 -> %.2f (expect 1.00)", sev40, sev65);
    }

    printf("\n=== 5. Apex: top camera holds speed down while front cam sees the exit ===\n");
    corner_severity_reset(&cs);
    {
        /* top_sev = (18.5 - 12) / (25 - 12) = 0.50 */
        float sev = step(&cs, 5.0f, 18.5f, 0, 0, 0);
        CHECK(fabsf(sev - 0.50f) < 0.01f, "FS=5, H=18.5 -> sev=%.2f (expect 0.50)", sev);
        corner_severity_reset(&cs);
        sev = step(&cs, 5.0f, -40.0f, 0, 0, 0);
        CHECK(sev == 1.0f, "FS=5, H=-40 -> sev=%.2f (expect 1.00)", sev);
    }

    printf("\n=== 6. Severity is max(front, top) ===\n");
    corner_severity_reset(&cs);
    {
        /* front 0.50 (FS=30), top ~0.77 (H=22) */
        float sev = step(&cs, 30.0f, 22.0f, 0, 0, 0);
        float expect = (22.0f - H_LO) / (H_HI - H_LO);
        CHECK(fabsf(sev - expect) < 0.01f, "FS=30, H=22 -> sev=%.2f (expect %.2f)", sev, expect);
    }

    printf("\n=== 7. 90-deg corner wall ahead (front cam) -> full braking ===\n");
    corner_severity_reset(&cs);
    {
        float sev = step(&cs, 0.0f, 0.0f, 1, 0, 0);
        float spd = corner_severity_speed(sev, SPD_S, SPD_OUT);
        CHECK(sev == 1.0f && spd == SPD_OUT, "Front wall -> sev=%.2f, speed=%.1f", sev, spd);
    }

    printf("\n=== 8. Top-cam wall crossing center -> full braking ===\n");
    corner_severity_reset(&cs);
    {
        float sev = step(&cs, 0.0f, 0.0f, 0, 1, 0);
        CHECK(sev == 1.0f, "Top transverse_crosses_center -> sev=%.2f", sev);
    }

    printf("\n=== 9. Wall flicker: peak held for 6 ticks (120ms), then released ===\n");
    corner_severity_reset(&cs);
    {
        float s0 = step(&cs, 0.0f, 0.0f, 1, 0, 0);  /* wall seen */
        float s1 = step(&cs, 0.0f, 0.0f, 0, 0, 0);  /* wall flickers out (tick 1 of 6) */
        float s2 = step(&cs, 0.0f, 0.0f, 0, 0, 0);  /* tick 2 of 6 */
        float s3 = step(&cs, 0.0f, 0.0f, 0, 0, 0);  /* tick 3 of 6 */
        float s4 = step(&cs, 0.0f, 0.0f, 0, 0, 0);  /* tick 4 of 6 */
        float s5 = step(&cs, 0.0f, 0.0f, 0, 0, 0);  /* tick 5 of 6 */
        float s6 = step(&cs, 0.0f, 0.0f, 0, 0, 0);  /* tick 6 of 6 */
        float s7 = step(&cs, 0.0f, 0.0f, 0, 0, 0);  /* hold expired on 7th tick */
        CHECK(s0 == 1.0f && s1 == 1.0f && s2 == 1.0f && s3 == 1.0f &&
              s4 == 1.0f && s5 == 1.0f && s6 == 1.0f,
              "Held through 6 flicker ticks (%.2f %.2f %.2f %.2f %.2f %.2f %.2f)",
              s0, s1, s2, s3, s4, s5, s6);
        CHECK(s7 == 0.0f, "Released on 7th tick after wall lost (sev=%.2f)", s7);
    }

    printf("\n=== 10. Wall re-seen during hold refreshes the 6-tick hold ===\n");
    corner_severity_reset(&cs);
    {
        step(&cs, 0.0f, 0.0f, 1, 0, 0);
        step(&cs, 0.0f, 0.0f, 0, 0, 0);
        step(&cs, 0.0f, 0.0f, 0, 0, 0);
        step(&cs, 0.0f, 0.0f, 1, 0, 0);             /* seen again -> refresh to 6 ticks */
        float a = 1.0f;
        for (int i = 0; i < 6; i++) {
            float v = step(&cs, 0.0f, 0.0f, 0, 0, 0);
            if (v != 1.0f) a = v;
        }
        float after = step(&cs, 0.0f, 0.0f, 0, 0, 0);
        CHECK(a == 1.0f && after == 0.0f, "Hold refreshed for full 6 ticks (min=%.2f, 7th=%.2f)", a, after);
    }

    printf("\n=== 11. Rising severity passes through immediately (no lag on braking) ===\n");
    corner_severity_reset(&cs);
    {
        float a = step(&cs, 25.0f, 0.0f, 0, 0, 0);  /* (25-20)/(40-20) = 0.25 */
        float b = step(&cs, 35.0f, 0.0f, 0, 0, 0);  /* (35-20)/(40-20) = 0.75 */
        CHECK(b > a && fabsf(a - 0.25f) < 0.01f && fabsf(b - 0.75f) < 0.01f,
              "Rising sev follows input (%.2f -> %.2f)", a, b);
    }

    printf("\n=== 12. Latched intersection: zero severity and clears the hold ===\n");
    corner_severity_reset(&cs);
    {
        step(&cs, 0.0f, 0.0f, 1, 0, 0);             /* corner peak */
        float ix = step(&cs, 40.0f, 30.0f, 1, 0, 1); /* intersection latched -> 0 regardless */
        float after = step(&cs, 0.0f, 0.0f, 0, 0, 0);/* latch gone: old peak must NOT come back */
        CHECK(ix == 0.0f, "Intersection latched -> sev=%.2f", ix);
        CHECK(after == 0.0f, "Hold cleared by intersection (sev=%.2f)", after);
    }

    printf("\n=== 13. Degenerate params never produce NaN or out-of-range ===\n");
    corner_severity_reset(&cs);
    {
        float a = corner_severity_update(&cs, 30.0f, 30.0f, 0, 0, 0, 60.0f, 20.0f, 20.0f);
        CHECK(!isnan(a) && a >= 0.0f && a <= 1.0f, "fa_th>=40 and h_hi==h_lo -> sev=%.2f", a);
    }

    printf("\n=== 14. Corner entry bridge with Config.h defaults (no mid-entry re-acceleration) ===\n");
    corner_severity_reset(&cs);
    {
        /* Front camera sees corner wall on approach -> severity=1.0 (65%), then wall leaves FOV
         * while top-camera heading builds from 5 deg to 18 deg over 6 ticks (120 ms).
         * With CORNER_SEVERITY_HOLD_TICKS=6 and DEFAULT_H_LO=12.0, speed stays held at 65%
         * through the entry transition and smoothly hands off to top_sev without spiking to 79-100%. */
        float s0 = corner_severity_update(&cs, 38.0f, 2.0f, 1, 0, 0,
                                          DEFAULT_FRONT_ANGLE_THRESH, DEFAULT_H_LO, DEFAULT_H_HI);
        float min_bridge_sev = s0;
        float headings[6] = { 5.0f, 7.5f, 10.0f, 12.5f, 15.0f, 18.0f };
        for (int i = 0; i < 6; i++) {
            float st = corner_severity_update(&cs, 26.0f, headings[i], 0, 0, 0,
                                              DEFAULT_FRONT_ANGLE_THRESH, DEFAULT_H_LO, DEFAULT_H_HI);
            if (st < min_bridge_sev) min_bridge_sev = st;
        }
        float spd_bridge = corner_severity_speed(min_bridge_sev, SPEED_STRAIGHT, SPEED_OUTSIDE_CORNER);
        CHECK(min_bridge_sev == 1.0f && fabsf(spd_bridge - SPEED_OUTSIDE_CORNER) < 0.01f,
              "6-tick hold bridges front->top handoff at sev=%.2f, speed=%.1f%% (no mid-entry re-accel)",
              min_bridge_sev, spd_bridge);
    }

    printf("\n=== 15. Crooked chassis on straight track (Line 74/97) suppresses corner severity & braking ===\n");
    corner_severity_reset(&cs);
    {
        /* Car is crooked on straight track: front cam rel curvature < 15.0 deg -> crooked_straight_track = true
         * Sev angles are zeroed and hold reset, so severity stays 0.0 and speed stays 100% */
        float sev_74 = corner_severity_update(&cs, 0.0f, 0.0f, 0, 0, 0,
                                              DEFAULT_FRONT_ANGLE_THRESH, DEFAULT_H_LO, DEFAULT_H_HI);
        float spd_74 = corner_severity_speed(sev_74, SPEED_STRAIGHT, SPEED_OUTSIDE_CORNER);
        CHECK(sev_74 == 0.0f && spd_74 == SPEED_STRAIGHT,
              "Crooked straight track produces sev=%.2f, speed=%.1f%% (no false braking)", sev_74, spd_74);
    }

    printf("\nSummarize: %d failures\n", g_failures);
    return (g_failures == 0) ? 0 : 1;
}
