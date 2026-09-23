#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#include "Config.h"
#include "vectors.h"
#include "control_theory.h"

#ifndef M_PI
    #define M_PI 3.14159265358979323846f
#endif

/* Mock robot params */
robot_params_t g_params;

void mock_params_init(void) {
    g_params.kp_straight          = KP_STRAIGHT;
    g_params.kp_corner            = KP_CORNER;
    g_params.ki                   = DEFAULT_KI;
    g_params.kd_straight          = KD_STRAIGHT;
    g_params.kd_corner            = KD_CORNER;
    g_params.speed_straight       = SPEED_STRAIGHT;
    g_params.speed_outside_corner = SPEED_OUTSIDE_CORNER;
    g_params.speed_inside_min     = SPEED_INSIDE_MIN;
    g_params.k_cte                = DEFAULT_K_CTE;
    g_params.k_heading            = DEFAULT_K_HEADING;
    g_params.k_lookahead          = DEFAULT_K_LOOKAHEAD;
    g_params.dt                   = DEFAULT_DT;
    g_params.alpha                = DEFAULT_ALPHA;
    g_params.steering_offset      = STEERING_OFFSET;
    g_params.accel_ramp           = ACCEL_RAMP_STEP;
    g_params.decel_ramp           = DECEL_RAMP_STEP;
    g_params.state                = STATE_RUNNING;
}

/* Simulation Context representing the rewritten main.c pipeline */
typedef struct {
    uint8_t macro_state;
    uint32_t intersection_ticks;
    uint32_t line_lost_ticks;
    uint32_t line_reacquire_streak;
    
    pid_struct steer_pid;
    float filtered_cte;
    int   cte_filter_init;
    float filtered_heading;
    int   heading_filter_init;
    
    float applied_speed;
    float applied_speed_left;
    float applied_speed_right;
    float steer_cmd;
    float last_steer_cmd;
    float kappa;
} gov_sim_ctx_t;

void gov_sim_init(gov_sim_ctx_t *ctx) {
    memset(ctx, 0, sizeof(*ctx));
    ctx->macro_state = MACRO_STATE_NORMAL;
    ctx->applied_speed = g_params.speed_straight;
    ctx->applied_speed_left = g_params.speed_straight;
    ctx->applied_speed_right = g_params.speed_straight;
    pid_init(&ctx->steer_pid, g_params.kp_corner, g_params.ki, g_params.kd_corner, 100.0f, g_params.alpha);
}

typedef struct {
    uint8_t macro_state;
    float kappa;
    float target_speed;
    float applied_speed;
    float steer_cmd;
    float left_pwm;
    float right_pwm;
} gov_step_result_t;

gov_step_result_t gov_sim_step(gov_sim_ctx_t *ctx,
                               float raw_cte, float raw_heading,
                               float mid_dx, bool transverse_wall,
                               bool intersection_trigger,
                               uint8_t num_top_vectors)
{
    gov_step_result_t res;
    memset(&res, 0, sizeof(res));

    /* 1. Macro State Transitions */
    if (num_top_vectors == 0) {
        ctx->line_reacquire_streak = 0;
        ctx->line_lost_ticks++;
        if (ctx->line_lost_ticks >= (uint32_t)LINE_LOST_DEBOUNCE_TICKS) {
            ctx->macro_state = MACRO_STATE_LINE_LOST;
        }
    } else {
        if (ctx->macro_state == MACRO_STATE_LINE_LOST) {
            if (++ctx->line_reacquire_streak >= (uint32_t)LINE_REACQUIRE_TICKS) {
                /* Exit LINE_LOST -> NORMAL: Reset filters to prevent kick */
                ctx->macro_state = MACRO_STATE_NORMAL;
                ctx->line_lost_ticks = 0;
                ctx->line_reacquire_streak = 0;
                ctx->cte_filter_init = 0;
                ctx->heading_filter_init = 0;
                reset_pid_state(&ctx->steer_pid);
            }
        } else {
            ctx->line_lost_ticks = 0;
            ctx->line_reacquire_streak = 0;
        }
    }

    if (ctx->macro_state == MACRO_STATE_NORMAL && intersection_trigger) {
        ctx->macro_state = MACRO_STATE_INTERSECTION;
        ctx->intersection_ticks = 0;
        reset_pid_state(&ctx->steer_pid);
    }

    /* 2. Base Camera Line Filtering (when lines present) */
    if (num_top_vectors > 0) {
        if (!ctx->heading_filter_init) {
            ctx->filtered_heading = raw_heading;
            ctx->heading_filter_init = 1;
        } else {
            ctx->filtered_heading = 0.60f * raw_heading + 0.40f * ctx->filtered_heading;
        }

        if (!ctx->cte_filter_init) {
            ctx->filtered_cte = raw_cte;
            ctx->cte_filter_init = 1;
        } else {
            ctx->filtered_cte = 0.40f * raw_cte + 0.60f * ctx->filtered_cte;
        }
    }

    float target_speed = g_params.speed_straight;
    float raw_steer = 0.0f;

    /* 3. State-Specific Logic */
    if (ctx->macro_state == MACRO_STATE_NORMAL) {
        /* A. Preview Curvature Metric (kappa) with Deadbands */
        float abs_h = fabsf(ctx->filtered_heading);
        float k_base = 0.0f;
        if (abs_h > (float)GOVERNOR_HEADING_DEADBAND_DEG) {
            k_base = (abs_h - (float)GOVERNOR_HEADING_DEADBAND_DEG) / ((float)GOVERNOR_HEADING_MAX_DEG - (float)GOVERNOR_HEADING_DEADBAND_DEG);
            if (k_base > 1.0f) k_base = 1.0f;
        }

        float abs_curv = fabsf(mid_dx);
        float k_front = 0.0f;
        if (abs_curv > (float)GOVERNOR_CURVATURE_DEADBAND_PX) {
            k_front = (abs_curv - (float)GOVERNOR_CURVATURE_DEADBAND_PX) / ((float)GOVERNOR_CURVATURE_MAX_PX - (float)GOVERNOR_CURVATURE_DEADBAND_PX);
            if (k_front > 1.0f) k_front = 1.0f;
        }

        if (transverse_wall) {
            k_front = 1.0f;
        }

        ctx->kappa = (k_base > k_front) ? k_base : k_front;
        if (ctx->kappa > 1.0f) ctx->kappa = 1.0f;

        /* B. Continuous Target Speed */
        target_speed = g_params.speed_straight - ctx->kappa * (g_params.speed_straight - g_params.speed_outside_corner);

        /* C. Steering PD + Heading Feedforward */
        float effective_h = 0.0f;
        if (abs_h > (float)HEADING_DEADBAND_DEG) {
            effective_h = (abs_h - (float)HEADING_DEADBAND_DEG) * ((ctx->filtered_heading > 0.0f) ? 1.0f : -1.0f);
        }
        float heading_ff = g_params.k_heading * effective_h;
        float pid_cte = -pid_update(&ctx->steer_pid, 0.0f, g_params.k_cte * ctx->filtered_cte, g_params.dt);
        raw_steer = pid_cte + heading_ff;
    }
    else if (ctx->macro_state == MACRO_STATE_INTERSECTION) {
        ctx->kappa = 0.0f;
        target_speed = 70.0f;
        raw_steer = 0.0f; // Hold straight across intersection

        if (++ctx->intersection_ticks >= (uint32_t)INTERSECTION_TIMEOUT_TICKS) {
            /* Guaranteed Timeout Exit -> NORMAL */
            ctx->macro_state = MACRO_STATE_NORMAL;
            ctx->intersection_ticks = 0;
            ctx->cte_filter_init = 0;
            ctx->heading_filter_init = 0;
            reset_pid_state(&ctx->steer_pid);
        }
    }
    else if (ctx->macro_state == MACRO_STATE_LINE_LOST) {
        ctx->kappa = 0.0f;
        raw_steer = 0.0f; // Hold straight

        if (ctx->line_lost_ticks <= (uint32_t)LINE_LOST_BRAKE_TICKS) {
            /* Phase 1: Coast */
            target_speed = ctx->applied_speed; // Hold speed
        } else if (ctx->line_lost_ticks <= (uint32_t)LINE_LOST_STOP_TICKS) {
            /* Phase 2: Dynamic Brake */
            target_speed = 0.0f;
        } else {
            /* Phase 3: Emergency Stop */
            target_speed = 0.0f;
            g_params.state = STATE_IDLE;
        }
    }

    /* 4. Global Steering Slew Rate Limiter */
    const float max_steer_step = (float)MAX_STEER_RATE_PER_SEC * g_params.dt;
    float steer_diff = raw_steer - ctx->last_steer_cmd;
    if (steer_diff > max_steer_step) raw_steer = ctx->last_steer_cmd + max_steer_step;
    else if (steer_diff < -max_steer_step) raw_steer = ctx->last_steer_cmd - max_steer_step;

    if (raw_steer > (float)STEER_CMD_MAX_RIGHT) raw_steer = (float)STEER_CMD_MAX_RIGHT;
    if (raw_steer < (float)STEER_CMD_MAX_LEFT) raw_steer = (float)STEER_CMD_MAX_LEFT;
    ctx->steer_cmd = raw_steer;
    ctx->last_steer_cmd = raw_steer;

    /* 5. Global Longitudinal Speed Slew Rate Limiter */
    float speed_diff = target_speed - ctx->applied_speed;
    if (speed_diff > g_params.accel_ramp) {
        ctx->applied_speed += g_params.accel_ramp;
    } else if (speed_diff < -g_params.decel_ramp) {
        ctx->applied_speed -= g_params.decel_ramp;
    } else {
        ctx->applied_speed = target_speed;
    }

    /* 6. Dynamic Rear Differential Drag (Tank Steering Bite) */
    float abs_steer = fabsf(ctx->steer_cmd);
    float steer_norm = 0.0f;
    if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
        steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) / ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
        if (steer_norm > 1.0f) steer_norm = 1.0f;
    }

    float diff_delta = steer_norm * (ctx->applied_speed - g_params.speed_inside_min);
    if (ctx->steer_cmd >= 0.0f) {
        /* Turning Right */
        ctx->applied_speed_left  = ctx->applied_speed;
        ctx->applied_speed_right = ctx->applied_speed - diff_delta;
    } else {
        /* Turning Left */
        ctx->applied_speed_left  = ctx->applied_speed - diff_delta;
        ctx->applied_speed_right = ctx->applied_speed;
    }

    res.macro_state   = ctx->macro_state;
    res.kappa         = ctx->kappa;
    res.target_speed  = target_speed;
    res.applied_speed = ctx->applied_speed;
    res.steer_cmd     = ctx->steer_cmd;
    res.left_pwm      = ctx->applied_speed_left;
    res.right_pwm     = ctx->applied_speed_right;
    return res;
}

int main(void) {
    printf("=================================================================\n");
    printf("   CONTINUOUS CURVATURE GOVERNOR & 3-MACRO-STATE TEST SUITE      \n");
    printf("=================================================================\n\n");

    mock_params_init();

    /* ----------------------------------------------------------------- */
    /* TEST 1: Straightaway Noise Deadband & True 100% Sprint            */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 1: Straightaway Noise Deadband (True 100%% Speed)\n");
    gov_sim_ctx_t ctx1;
    gov_sim_init(&ctx1);

    for (int i = 0; i < 50; i++) {
        /* Normal straightaway camera noise: H in [-1.5, +1.5], mid_dx in [-3.0, +3.0] */
        float noise_h = (float)(i % 7 - 3) * 0.5f;       // -1.5 to +1.5 deg
        float noise_dx = (float)(i % 5 - 2) * 1.5f;      // -3.0 to +3.0 px
        gov_step_result_t r = gov_sim_step(&ctx1, 5.0f, noise_h, noise_dx, false, false, 2);

        assert(r.macro_state == MACRO_STATE_NORMAL);
        assert(r.kappa == 0.0f);
        assert(r.target_speed == 100.0f);
        assert(r.applied_speed >= 99.0f);
    }
    printf("  [PASS] Test 1: Straightaway camera noise is completely zeroed by deadbands; 100%% speed confirmed!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 2: Smooth Continuous Curvature Scaling & Corner Pre-Braking  */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 2: Continuous Curvature Scaling & Corner Pre-Braking\n");
    gov_sim_ctx_t ctx2;
    gov_sim_init(&ctx2);

    /* 1. Approaching 90 deg corner: front camera detects curvature ramping */
    float prev_speed = 100.0f;
    for (int i = 1; i <= 10; i++) {
        float dx = 4.0f + (float)i * 1.0f; // mid_dx ramps from 5px to 14px
        gov_step_result_t r = gov_sim_step(&ctx2, 0.0f, 0.0f, dx, false, false, 2);
        
        assert(r.macro_state == MACRO_STATE_NORMAL);
        assert(r.kappa > 0.0f);
        assert(r.target_speed < 100.0f);
        assert(r.applied_speed <= prev_speed); // Strictly monotonic deceleration
        prev_speed = r.applied_speed;
    }
    printf("  Decelerated smoothly from 100%% to %.1f%% on corner approach.\n", prev_speed);
    assert(prev_speed <= 70.0f);

    /* 2. Transverse wall detection dead ahead (Corner 1) */
    gov_step_result_t r_wall = gov_sim_step(&ctx2, 0.0f, 0.0f, 15.0f, true, false, 2);
    assert(r_wall.kappa == 1.0f);
    assert(r_wall.target_speed == 58.0f);
    printf("  [PASS] Test 2: Continuous governor pre-brakes smoothly to %.1f%% with zero state jumping!\n\n", r_wall.target_speed);

    /* ----------------------------------------------------------------- */
    /* TEST 3: Global Speed Slew Limiter Across Transitions              */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 3: Global Slew Limiter Discontinuity Immunity\n");
    gov_sim_ctx_t ctx3;
    gov_sim_init(&ctx3);

    /* Instantaneous step change from 100% to 58% */
    for (int i = 0; i < 10; i++) {
        float speed_before = ctx3.applied_speed;
        gov_step_result_t r = gov_sim_step(&ctx3, 0.0f, 25.0f, 15.0f, false, false, 2);
        float step = speed_before - r.applied_speed;
        assert(step <= g_params.decel_ramp + 0.01f); // Strictly rate-limited
    }
    printf("  [PASS] Test 3: Slew rate limiter strictly protects motors against step-changes!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 4: Intersection Crossing & Guaranteed 18-Tick Timeout        */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 4: Intersection Crossing & Timeout Guarantee\n");
    gov_sim_ctx_t ctx4;
    gov_sim_init(&ctx4);

    /* Step 1: Normal straight sprint */
    gov_step_result_t r4_norm = gov_sim_step(&ctx4, 0.0f, 0.0f, 0.0f, false, false, 2);
    assert(r4_norm.macro_state == MACRO_STATE_NORMAL);

    /* Step 2: Trigger Intersection (Tick 1) */
    gov_step_result_t r4_int = gov_sim_step(&ctx4, 0.0f, 0.0f, 0.0f, false, true, 2);
    assert(r4_int.macro_state == MACRO_STATE_INTERSECTION);
    assert(r4_int.steer_cmd == 0.0f); // Held straight

    /* Step 3: Run for ticks 2 through 17 (16 ticks, total 17 ticks in INTERSECTION) */
    for (int t = 2; t <= 17; t++) {
        gov_step_result_t r = gov_sim_step(&ctx4, 0.0f, 0.0f, 0.0f, false, false, 2);
        assert(r.macro_state == MACRO_STATE_INTERSECTION);
        assert(r.steer_cmd == 0.0f);
    }

    /* Step 4: Tick 18 MUST reach timeout and return to NORMAL */
    gov_step_result_t r4_exit = gov_sim_step(&ctx4, 0.0f, 0.0f, 0.0f, false, false, 2);
    assert(r4_exit.macro_state == MACRO_STATE_NORMAL);
    printf("  [PASS] Test 4: Intersection held straight and safely returned to NORMAL in exactly 18 ticks!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 5: Line Loss Staged Recovery & Anti-Kick Reset               */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 5: Line Loss Staged Recovery (Coast -> Brake -> Stop)\n");
    gov_sim_ctx_t ctx5;
    gov_sim_init(&ctx5);

    /* Optical flicker (1-2 ticks) -> should NOT enter LINE_LOST */
    gov_step_result_t r5_1 = gov_sim_step(&ctx5, 0.0f, 0.0f, 0.0f, false, false, 0);
    assert(r5_1.macro_state == MACRO_STATE_NORMAL);
    gov_step_result_t r5_2 = gov_sim_step(&ctx5, 0.0f, 0.0f, 0.0f, false, false, 0);
    assert(r5_2.macro_state == MACRO_STATE_NORMAL);

    /* Tick 3 lost -> enters LINE_LOST */
    gov_step_result_t r5_3 = gov_sim_step(&ctx5, 0.0f, 0.0f, 0.0f, false, false, 0);
    assert(r5_3.macro_state == MACRO_STATE_LINE_LOST);
    assert(r5_3.steer_cmd == 0.0f);

    /* Re-acquire after 3 valid ticks */
    gov_sim_step(&ctx5, 0.0f, 0.0f, 0.0f, false, false, 2); // Streak 1
    assert(ctx5.macro_state == MACRO_STATE_LINE_LOST);
    gov_sim_step(&ctx5, 0.0f, 0.0f, 0.0f, false, false, 2); // Streak 2
    assert(ctx5.macro_state == MACRO_STATE_LINE_LOST);
    gov_step_result_t r5_reacq = gov_sim_step(&ctx5, 0.0f, 0.0f, 0.0f, false, false, 2); // Streak 3
    assert(r5_reacq.macro_state == MACRO_STATE_NORMAL);
    printf("  [PASS] Test 5: Line loss debounce and 3-tick re-acquisition verified!\n\n");

    printf("=================================================================\n");
    printf("   >>> ALL 5 CONTINUOUS GOVERNOR BENCHMARKS PASSED! <<<          \n");
    printf("=================================================================\n");
    return 0;
}
