#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#include "Config.h"
#include "vectors.h"

/* Include the actual implementation under test */
#include "../source/vectors.c"

/* State machine simulation mimicking source/main.c */
#define MACRO_STATE_NORMAL       0
#define MACRO_STATE_INTERSECTION 1

typedef struct {
    uint8_t  macro_state;
    uint32_t intersection_ticks;
    uint32_t intersection_cooldown_ticks;
    float    steer_cmd;
    float    target_left;
    float    target_right;
} sim_context_t;

void sim_step(sim_context_t *ctx,
              const steering_result_t *sr,
              const horizon_result_t *horizon,
              float filtered_cte,
              float filtered_heading,
              float filtered_front_angle)
{
    if (ctx->intersection_cooldown_ticks > 0) {
        ctx->intersection_cooldown_ticks--;
    }

    /* Step 3: Macro State Machine */
    if (ctx->macro_state == MACRO_STATE_NORMAL) {
        if (sr->has_transverse_tape &&
            !horizon->is_sharp_corner &&
            ctx->intersection_cooldown_ticks == 0 &&
            fabsf(filtered_heading) <= 12.0f &&
            fabsf(ctx->steer_cmd) <= 12.0f) {
            ctx->macro_state = MACRO_STATE_INTERSECTION;
            ctx->intersection_ticks = 0;
        }
    } else if (ctx->macro_state == MACRO_STATE_INTERSECTION) {
        ctx->intersection_ticks++;
        if (ctx->intersection_ticks >= 4 && sr->found >= 1 && fabsf(filtered_heading) <= 25.0f) {
            ctx->macro_state = MACRO_STATE_NORMAL;
            ctx->intersection_cooldown_ticks = 60;
        } else if (ctx->intersection_ticks >= 10) {
            ctx->macro_state = MACRO_STATE_NORMAL;
            ctx->intersection_cooldown_ticks = 60;
        }
    }

    /* Step 4: Steering */
    float steer_top = (DEFAULT_K_CTE * filtered_cte) + (DEFAULT_K_HEADING * filtered_heading);
    ctx->steer_cmd = steer_top;
    if (horizon->is_sharp_corner) {
        ctx->steer_cmd += (float)FRONT_FEEDFORWARD_GAIN * filtered_front_angle;
    }
    if (ctx->macro_state == MACRO_STATE_INTERSECTION) {
        ctx->steer_cmd = 0.0f;
    }

    /* Step 5: Speed */
    float target_speed = SPEED_STRAIGHT;
    if (ctx->macro_state == MACRO_STATE_INTERSECTION) {
        target_speed = SPEED_STRAIGHT;
        ctx->target_left = target_speed;
        ctx->target_right = target_speed;
    } else if (horizon->is_sharp_corner) {
        target_speed = SPEED_OUTSIDE_CORNER;
        if (ctx->steer_cmd > 0.0f) {
            ctx->target_left = SPEED_OUTSIDE_CORNER;
            ctx->target_right = SPEED_INSIDE_MIN;
        } else {
            ctx->target_left = SPEED_INSIDE_MIN;
            ctx->target_right = SPEED_OUTSIDE_CORNER;
        }
    } else {
        ctx->target_left = target_speed;
        ctx->target_right = target_speed;
    }
}

int main(void) {
    printf("=== RUNNING INTERSECTION LOGIC UNIT TESTS ===\n");

    /* ------------------------------------------------------------- */
    /* Test 1: Vectors - True Transverse Tape Detection              */
    /* ------------------------------------------------------------- */
    {
        printf("Test 1: True Transverse Tape Detection... ");
        // A transverse tape vector across center:
        // near_x=20, near_y=30, far_x=50, far_y=28 -> dx=30, dy=2, angle=86.2 deg, y_near=30
        uint16_t vectors[4] = { 20, 30, 50, 28 };
        steering_result_t sr;
        reset_steering_tracking();
        compute_top_camera_lane(vectors, 1, &sr);

        assert(sr.has_transverse_tape == 1U);
        assert(sr.found == 0); // Must be rejected from lane boundaries!
        printf("PASS\n");
    }

    /* ------------------------------------------------------------- */
    /* Test 2: Vectors - Tilted Corner Line Must NOT Flag Tape       */
    /* ------------------------------------------------------------- */
    {
        printf("Test 2: Tilted Corner Line Rejection... ");
        // Corner line tilted at 65 deg:
        // near_x=20, near_y=40, far_x=45, far_y=28 -> dx=25, dy=12, angle=64.4 deg
        uint16_t vectors[4] = { 20, 40, 45, 28 };
        steering_result_t sr;
        reset_steering_tracking();
        compute_top_camera_lane(vectors, 1, &sr);

        assert(sr.has_transverse_tape == 0U);
        assert(sr.found == 0); // angle > 60 rejected from boundary, but NOT transverse tape
        printf("PASS\n");
    }

    /* ------------------------------------------------------------- */
    /* Test 3: Vectors - Diagonal Line with High Angle but High dy   */
    /* ------------------------------------------------------------- */
    {
        printf("Test 3: Diagonal Line with High Angle but dy > 6... ");
        // Angle = 74 deg, but dy = 10 (> 6):
        // dx = 35, dy = 10 -> angle = 74.05 deg
        uint16_t vectors[4] = { 10, 40, 45, 30 };
        steering_result_t sr;
        reset_steering_tracking();
        compute_top_camera_lane(vectors, 1, &sr);

        assert(sr.has_transverse_tape == 0U); // dy > 6 must NOT flag transverse tape!
        printf("PASS\n");
    }

    /* ------------------------------------------------------------- */
    /* Test 4: Vectors - Short Noise Fragment                        */
    /* ------------------------------------------------------------- */
    {
        printf("Test 4: Short Noise Fragment... ");
        // dy = 2, dx = 10 (< 16):
        uint16_t vectors[4] = { 30, 25, 40, 23 };
        steering_result_t sr;
        reset_steering_tracking();
        compute_top_camera_lane(vectors, 1, &sr);

        assert(sr.has_transverse_tape == 0U); // dx < 16 must NOT flag transverse tape!
        printf("PASS\n");
    }

    /* ------------------------------------------------------------- */
    /* Test 5: State Machine - Normal Entry and 4-tick Early Exit    */
    /* ------------------------------------------------------------- */
    {
        printf("Test 5: State Machine - 4-tick Exit on 1 Line Reacquired... ");
        sim_context_t ctx = { .macro_state = MACRO_STATE_NORMAL };
        horizon_result_t horizon = { .is_sharp_corner = 0 };
        steering_result_t sr_transverse = { .has_transverse_tape = 1, .found = 0 };

        // Tick 1: Approaching straight, sees transverse tape
        sim_step(&ctx, &sr_transverse, &horizon, 0.0f, 0.0f, 0.0f);
        assert(ctx.macro_state == MACRO_STATE_INTERSECTION);
        assert(ctx.intersection_ticks == 0);
        assert(ctx.steer_cmd == 0.0f);

        // Ticks 2, 3, 4: In intersection, refractory window
        steering_result_t sr_in_box = { .has_transverse_tape = 0, .found = 1, .heading_deg = 5.0f };
        for (int t = 1; t <= 3; t++) {
            sim_step(&ctx, &sr_in_box, &horizon, 0.0f, 0.0f, 0.0f);
            assert(ctx.macro_state == MACRO_STATE_INTERSECTION);
            assert(ctx.intersection_ticks == t);
            assert(ctx.steer_cmd == 0.0f);
        }

        // Tick 5 (intersection_ticks reaches 4): sees 1 line with clean heading
        sim_step(&ctx, &sr_in_box, &horizon, 0.0f, 0.0f, 0.0f);
        assert(ctx.macro_state == MACRO_STATE_NORMAL);
        assert(ctx.intersection_cooldown_ticks == 60);
        printf("PASS\n");
    }

    /* ------------------------------------------------------------- */
    /* Test 6: State Machine - Timeout Backstop at Tick 10           */
    /* ------------------------------------------------------------- */
    {
        printf("Test 6: State Machine - Timeout Backstop at 10 ticks... ");
        sim_context_t ctx = { .macro_state = MACRO_STATE_NORMAL };
        horizon_result_t horizon = { .is_sharp_corner = 0 };
        steering_result_t sr_transverse = { .has_transverse_tape = 1, .found = 0 };
        steering_result_t sr_empty = { .has_transverse_tape = 0, .found = 0 };

        // Entry
        sim_step(&ctx, &sr_transverse, &horizon, 0.0f, 0.0f, 0.0f);
        assert(ctx.macro_state == MACRO_STATE_INTERSECTION);

        // Ticks 1 to 9: Blind in box
        for (int t = 1; t <= 9; t++) {
            sim_step(&ctx, &sr_empty, &horizon, 0.0f, 0.0f, 0.0f);
            assert(ctx.macro_state == MACRO_STATE_INTERSECTION);
            assert(ctx.intersection_ticks == t);
        }

        // Tick 10: Timeout backstop triggers exit
        sim_step(&ctx, &sr_empty, &horizon, 0.0f, 0.0f, 0.0f);
        assert(ctx.macro_state == MACRO_STATE_NORMAL);
        assert(ctx.intersection_cooldown_ticks == 60);
        printf("PASS\n");
    }

    /* ------------------------------------------------------------- */
    /* Test 7: Cooldown Lockout Prevents False Re-entry              */
    /* ------------------------------------------------------------- */
    {
        printf("Test 7: Cooldown Lockout Blocks Re-entry for 60 ticks... ");
        sim_context_t ctx = { .macro_state = MACRO_STATE_NORMAL, .intersection_cooldown_ticks = 60 };
        horizon_result_t horizon = { .is_sharp_corner = 0 };
        steering_result_t sr_fake_tape = { .has_transverse_tape = 1, .found = 1 };

        // Attempt re-entry while cooldown > 0 (ticks 1 to 59)
        for (int t = 60; t > 1; t--) {
            assert(ctx.intersection_cooldown_ticks == t);
            sim_step(&ctx, &sr_fake_tape, &horizon, 0.0f, 0.0f, 0.0f);
            assert(ctx.macro_state == MACRO_STATE_NORMAL); // Must NOT enter!
        }
        assert(ctx.intersection_cooldown_ticks == 1);

        // On tick 60: cooldown decrements from 1 to 0 and legitimate tape can now trigger entry
        sim_step(&ctx, &sr_fake_tape, &horizon, 0.0f, 0.0f, 0.0f);
        assert(ctx.intersection_cooldown_ticks == 0);
        assert(ctx.macro_state == MACRO_STATE_INTERSECTION);
        printf("PASS\n");
    }

    /* ------------------------------------------------------------- */
    /* Test 8: Front Camera Sharp Corner Inhibit                     */
    /* ------------------------------------------------------------- */
    {
        printf("Test 8: Front Camera Corner Inhibit... ");
        sim_context_t ctx = { .macro_state = MACRO_STATE_NORMAL, .intersection_cooldown_ticks = 0 };
        horizon_result_t horizon = { .is_sharp_corner = 1 }; // Corner ahead!
        steering_result_t sr_tape = { .has_transverse_tape = 1, .found = 1 };

        sim_step(&ctx, &sr_tape, &horizon, 0.0f, 0.0f, 0.0f);
        assert(ctx.macro_state == MACRO_STATE_NORMAL); // Inhibited by horizon!
        printf("PASS\n");
    }

    printf("\n>>> ALL 8 INTERSECTION TESTS PASSED! <<<\n");
    return 0;
}
