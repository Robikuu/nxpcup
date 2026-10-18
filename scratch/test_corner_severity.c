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
        /* front_sev = (35 - 20) / (50 - 20) = 0.50 -> 77.5% */
        float sev = step(&cs, 35.0f, 0.0f, 0, 0, 0);
        float spd = corner_severity_speed(sev, SPD_S, SPD_OUT);
        CHECK(fabsf(sev - 0.50f) < 0.01f && fabsf(spd - 77.5f) < 0.1f,
              "FS=35 kink -> sev=%.2f, speed=%.1f (expect 0.50 / 77.5)", sev, spd);
    }

    printf("\n=== 3. Negative angles (left turns) behave the same as right ===\n");
    corner_severity_reset(&cs);
    {
        float sev = step(&cs, -35.0f, 0.0f, 0, 0, 0);
        CHECK(fabsf(sev - 0.50f) < 0.01f, "FS=-35 -> sev=%.2f (expect 0.50)", sev);
    }

    printf("\n=== 4. Front severity saturates at FRONT_SEVERITY_FULL_DEG (50) ===\n");
    corner_severity_reset(&cs);
    {
        float sev = step(&cs, 65.0f, 0.0f, 0, 0, 0);
        CHECK(sev == 1.0f, "FS=65 -> sev=%.2f (expect 1.00)", sev);
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
        /* front 0.50, top 0.50 + ... use front 0.50 and top ~0.77 (H=22) */
        float sev = step(&cs, 35.0f, 22.0f, 0, 0, 0);
        float expect = (22.0f - H_LO) / (H_HI - H_LO);
        CHECK(fabsf(sev - expect) < 0.01f, "FS=35, H=22 -> sev=%.2f (expect %.2f)", sev, expect);
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

    printf("\n=== 9. Wall flicker: peak held for 3 ticks, then released ===\n");
    corner_severity_reset(&cs);
    {
        float s0 = step(&cs, 0.0f, 0.0f, 1, 0, 0);  /* wall seen */
        float s1 = step(&cs, 0.0f, 0.0f, 0, 0, 0);  /* wall flickers out (angle forced to 0 by vectors.c) */
        float s2 = step(&cs, 0.0f, 0.0f, 0, 0, 0);
        float s3 = step(&cs, 0.0f, 0.0f, 0, 0, 0);
        float s4 = step(&cs, 0.0f, 0.0f, 0, 0, 0);  /* hold expired */
        CHECK(s0 == 1.0f && s1 == 1.0f && s2 == 1.0f && s3 == 1.0f,
              "Held through 3 flicker ticks (%.2f %.2f %.2f %.2f)", s0, s1, s2, s3);
        CHECK(s4 == 0.0f, "Released on 4th tick after wall lost (sev=%.2f)", s4);
    }

    printf("\n=== 10. Wall re-seen during hold refreshes the hold ===\n");
    corner_severity_reset(&cs);
    {
        step(&cs, 0.0f, 0.0f, 1, 0, 0);
        step(&cs, 0.0f, 0.0f, 0, 0, 0);
        step(&cs, 0.0f, 0.0f, 0, 0, 0);
        step(&cs, 0.0f, 0.0f, 1, 0, 0);             /* seen again -> refresh */
        float a = step(&cs, 0.0f, 0.0f, 0, 0, 0);
        float b = step(&cs, 0.0f, 0.0f, 0, 0, 0);
        float c = step(&cs, 0.0f, 0.0f, 0, 0, 0);
        CHECK(a == 1.0f && b == 1.0f && c == 1.0f, "Hold refreshed (%.2f %.2f %.2f)", a, b, c);
    }

    printf("\n=== 11. Rising severity passes through immediately (no lag on braking) ===\n");
    corner_severity_reset(&cs);
    {
        float a = step(&cs, 30.0f, 0.0f, 0, 0, 0);  /* 0.33 */
        float b = step(&cs, 45.0f, 0.0f, 0, 0, 0);  /* 0.83 */
        CHECK(b > a && fabsf(b - (25.0f / 30.0f)) < 0.01f, "Rising sev follows input (%.2f -> %.2f)", a, b);
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
        CHECK(!isnan(a) && a >= 0.0f && a <= 1.0f, "fa_th>=50 and h_hi==h_lo -> sev=%.2f", a);
    }

    printf("\nSummarize: %d failures\n", g_failures);
    return (g_failures == 0) ? 0 : 1;
}
