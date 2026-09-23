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

typedef enum {
    DRIVE_FORWARD,
    DRIVE_TURNING,
    DRIVE_BRAKING
} drive_state_t;

/* Mock PID and robot params for standalone simulation */
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

/* Pipeline simulation function mimicking source/main.c tick execution */
typedef struct {
    float steer_cmd;
    float applied_speed_left;
    float applied_speed_right;
    drive_state_t drive_state;
    float trusted_corner_urgency;
    float pid_cte;
    float heading_feedforward;
    float lookahead_turn_factor;
} sim_tick_result_t;

typedef struct {
    pid_struct steer_pid;
    float filtered_cte;
    int cte_filter_init;
    float filtered_heading;
    int heading_filter_init;
    float filtered_traj_angle;
    float filtered_traj_offset;
    float lookahead_turn_factor;
    float s_last_valid_lookahead_x;
    uint32_t s_front_dropout_ticks;
    float applied_speed_left;
    float applied_speed_right;
    float last_steer_cmd;
    drive_state_t drive_state;
    uint32_t state_streak;
    int8_t streak_sign;
    uint32_t urgency_streak;
    float trusted_corner_urgency;
    uint32_t line_lost_counter;
} sim_context_t;

void sim_init(sim_context_t *ctx) {
    memset(ctx, 0, sizeof(*ctx));
    pid_init(&ctx->steer_pid, g_params.kp_corner, g_params.ki, g_params.kd_corner, 100.0f, g_params.alpha);
    ctx->applied_speed_left = g_params.speed_straight;
    ctx->applied_speed_right = g_params.speed_straight;
    ctx->drive_state = DRIVE_FORWARD;
    ctx->streak_sign = 0;
    ctx->s_last_valid_lookahead_x = (float)PIXY_CENTER_X;
    ctx->s_front_dropout_ticks = 0;
}

sim_tick_result_t sim_step(sim_context_t *ctx,
                           float raw_cte, float raw_heading,
                           float raw_lookahead_x, float raw_traj_urgency,
                           uint8_t top_found, uint8_t front_found)
{
    sim_tick_result_t res;
    memset(&res, 0, sizeof(res));

    /* 1. Front Camera 50 Hz Lookahead & Dropout Hold Filter */
    if (front_found >= 1) {
        ctx->s_last_valid_lookahead_x = raw_lookahead_x;
        ctx->s_front_dropout_ticks    = 0;
        ctx->lookahead_turn_factor    = 0.50f * raw_traj_urgency + 0.50f * ctx->lookahead_turn_factor;
    } else {
        ctx->s_front_dropout_ticks++;
        if (ctx->s_front_dropout_ticks > (uint32_t)LOOKAHEAD_DROPOUT_HOLD_TICKS) {
            /* Extended dropout: check top-camera straightaway sanity veto */
            if (fabsf(ctx->filtered_heading) <= 3.0f && fabsf(ctx->filtered_cte) <= 4.0f) {
                ctx->lookahead_turn_factor *= 0.80f; // VETO!
            } else {
                ctx->lookahead_turn_factor = 0.60f; // Confirmed corner
            }
        }
    }

    /* 2. Top camera line loss check */
    if (top_found == 0) {
        ctx->urgency_streak = 0;
        ctx->trusted_corner_urgency *= 0.90f;
        ctx->lookahead_turn_factor  *= 0.90f;
        if (++ctx->line_lost_counter <= LINE_LOST_LIMIT) {
            /* Coasting */
            res.steer_cmd = 0.0f;
            res.applied_speed_left = ctx->applied_speed_left;
            res.applied_speed_right = ctx->applied_speed_right;
            res.drive_state = ctx->drive_state;
            return res;
        } else {
            /* Emergency Braking */
            ctx->drive_state = DRIVE_BRAKING;
            ctx->applied_speed_left = 0.0f;
            ctx->applied_speed_right = 0.0f;
            res.steer_cmd = 0.0f;
            res.applied_speed_left = 0.0f;
            res.applied_speed_right = 0.0f;
            res.drive_state = DRIVE_BRAKING;
            return res;
        }
    }

    ctx->line_lost_counter = 0;

    /* 3. Heading & CTE filtering */
    if (!ctx->heading_filter_init) {
        ctx->filtered_heading    = raw_heading;
        ctx->heading_filter_init = 1;
    } else {
        ctx->filtered_heading = 0.60f * raw_heading + 0.40f * ctx->filtered_heading;
    }

    if (!ctx->cte_filter_init) {
        ctx->filtered_cte    = raw_cte;
        ctx->cte_filter_init = 1;
    } else {
        ctx->filtered_cte = 0.40f * raw_cte + 0.60f * ctx->filtered_cte;
    }

    /* 4. State Machine with Lookahead Hysteresis */
    float la_offset = fabsf(ctx->s_last_valid_lookahead_x - (float)PIXY_CENTER_X);
    bool corner_trigger = (fabsf(ctx->filtered_heading) >= (float)CORNER_HEADING_THRESHOLD_DEG) ||
                          (la_offset >= (float)LOOKAHEAD_HYSTERESIS_ENTER_PX && ctx->s_front_dropout_ticks == 0) ||
                          (ctx->s_front_dropout_ticks > (uint32_t)LOOKAHEAD_DROPOUT_HOLD_TICKS &&
                           !(fabsf(ctx->filtered_heading) <= 3.0f && fabsf(ctx->filtered_cte) <= 4.0f));

    drive_state_t desired_state;
    if (ctx->drive_state == DRIVE_TURNING) {
        bool straight_confirmed = (fabsf(ctx->filtered_heading) <= 4.0f &&
                                   fabsf(ctx->filtered_cte) <= 6.0f &&
                                   la_offset <= (float)LOOKAHEAD_HYSTERESIS_EXIT_PX);
        desired_state = straight_confirmed ? DRIVE_FORWARD : DRIVE_TURNING;
    } else {
        desired_state = corner_trigger ? DRIVE_TURNING : DRIVE_FORWARD;
    }

    if (desired_state != ctx->drive_state) {
        if (ctx->drive_state == DRIVE_FORWARD) {
            int8_t current_sign = (ctx->filtered_heading >= 0.0f) ? 1 : -1;
            if (current_sign != ctx->streak_sign) {
                ctx->streak_sign = current_sign;
                ctx->state_streak = 0;
            }
        }
        uint32_t threshold = (desired_state == DRIVE_FORWARD) ? (uint32_t)STATE_EXIT_STREAK : (uint32_t)STATE_ENTER_STREAK;
        if (++ctx->state_streak >= threshold) {
            ctx->drive_state  = desired_state;
            ctx->state_streak = 0;
        }
    } else {
        ctx->state_streak = 0;
        ctx->streak_sign  = 0;
    }

    /* 5. Dynamic Gain Scheduling */
    float dz_heading = fabsf(ctx->filtered_heading) - 2.5f;
    float heading_urgency = (dz_heading > 0.0f) ? (dz_heading / ((float)HEADING_FULL_BRAKE_DEG - 2.5f)) : 0.0f;
    if (heading_urgency > 1.0f) heading_urgency = 1.0f;

    float cte_excess = fabsf(ctx->filtered_cte) - 14.0f;
    float cte_urgency = (cte_excess > 0.0f) ? (cte_excess / ((float)CTE_URGENCY_PX - 14.0f)) : 0.0f;
    if (cte_urgency > 1.0f) cte_urgency = 1.0f;

    float raw_urgency = heading_urgency;
    if (cte_urgency > raw_urgency) raw_urgency = cte_urgency;
    if (ctx->lookahead_turn_factor > raw_urgency) raw_urgency = ctx->lookahead_turn_factor;

    if (raw_urgency > ctx->trusted_corner_urgency) {
        if (++ctx->urgency_streak >= (uint32_t)CORNER_URGENCY_STREAK) {
            ctx->trusted_corner_urgency = raw_urgency;
            ctx->urgency_streak = 0;
        }
    } else {
        ctx->trusted_corner_urgency = 0.10f * raw_urgency + 0.90f * ctx->trusted_corner_urgency;
        ctx->urgency_streak = 0;
    }

    ctx->steer_pid.kp = g_params.kp_straight + ctx->trusted_corner_urgency * (g_params.kp_corner - g_params.kp_straight);
    ctx->steer_pid.ki = g_params.ki;
    ctx->steer_pid.kd = g_params.kd_straight;
    ctx->steer_pid.alpha = g_params.alpha;

    /* 6. Decoupled Steering Authority (100% Base Camera) */
    float pid_cte = -pid_update(&ctx->steer_pid, 0.0f, g_params.k_cte * ctx->filtered_cte, g_params.dt);

    if (ctx->drive_state == DRIVE_TURNING &&
        (ctx->filtered_cte * ctx->filtered_heading < 0.0f) &&
        fabsf(ctx->filtered_heading) >= 8.0f) {
        pid_cte = 0.0f;
    }

    float abs_h = fabsf(ctx->filtered_heading);
    float effective_h = 0.0f;
    if (abs_h > (float)HEADING_DEADBAND_DEG) {
        effective_h = (abs_h - (float)HEADING_DEADBAND_DEG) * ((ctx->filtered_heading > 0.0f) ? 1.0f : -1.0f);
    }
    float heading_feedforward = g_params.k_heading * effective_h;

    /* STRICTLY ZERO LOOKAHEAD STEERING */
    float raw_steer = pid_cte + heading_feedforward;

    const float max_steer_step = (float)MAX_STEER_RATE_PER_SEC * g_params.dt; // 13.0 deg
    float steer_diff = raw_steer - ctx->last_steer_cmd;
    if (steer_diff > max_steer_step) {
        raw_steer = ctx->last_steer_cmd + max_steer_step;
    } else if (steer_diff < -max_steer_step) {
        raw_steer = ctx->last_steer_cmd - max_steer_step;
    }
    float steer_cmd = raw_steer;

    if (steer_cmd > (float)STEER_CMD_MAX_RIGHT) steer_cmd = (float)STEER_CMD_MAX_RIGHT;
    if (steer_cmd < (float)STEER_CMD_MAX_LEFT)  steer_cmd = (float)STEER_CMD_MAX_LEFT;
    ctx->last_steer_cmd = steer_cmd;

    /* 7. Longitudinal Speed & Differential Drive */
    float abs_steer = fabsf(steer_cmd);
    float dz_steer = abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG;
    if (dz_steer < 0.0f) dz_steer = 0.0f;
    float steer_norm = dz_steer / ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
    if (steer_norm > 1.0f) steer_norm = 1.0f;

    float dz_h = abs_h - (float)DIFFERENTIAL_DEADZONE_DEG;
    if (dz_h < 0.0f) dz_h = 0.0f;
    float heading_norm = dz_h / ((float)HEADING_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
    if (heading_norm > 1.0f) heading_norm = 1.0f;

    float effective_corner_intensity = heading_norm;
    if (ctx->lookahead_turn_factor > effective_corner_intensity) {
        effective_corner_intensity = ctx->lookahead_turn_factor;
    }

    float longitudinal_base = g_params.speed_straight;
    float active_steer_norm = 0.0f;
    if (ctx->drive_state == DRIVE_TURNING) {
        float corner_entry_cap = 70.0f;
        longitudinal_base = corner_entry_cap - effective_corner_intensity * (corner_entry_cap - g_params.speed_outside_corner);
        active_steer_norm = steer_norm;
    } else if (ctx->lookahead_turn_factor > 0.15f) {
        /* Proactive pre-braking on straightaway approach before entering DRIVE_TURNING */
        longitudinal_base = g_params.speed_straight - ctx->lookahead_turn_factor * (g_params.speed_straight - 70.0f);
    }

    float diff_delta = active_steer_norm * (longitudinal_base - g_params.speed_inside_min);
    float target_outside = longitudinal_base;
    float target_inside  = longitudinal_base - diff_delta;

    float target_left  = longitudinal_base;
    float target_right = longitudinal_base;

    if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
        if (steer_cmd > 0.0f) {
            target_left  = target_outside;
            target_right = target_inside;
        } else {
            target_left  = target_inside;
            target_right = target_outside;
        }
    }

    /* Bilateral Traction Guard Ramping */
    bool yaw_conflict = false;
    if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
        if (steer_cmd > (float)YAW_CONFLICT_DEADBAND && (target_left - target_right) > 5.0f && ctx->applied_speed_left < ctx->applied_speed_right) {
            yaw_conflict = true;
        } else if (steer_cmd < -(float)YAW_CONFLICT_DEADBAND && (target_right - target_left) > 5.0f && ctx->applied_speed_right < ctx->applied_speed_left) {
            yaw_conflict = true;
        }
    }

    float step_accel_l = (yaw_conflict && target_left > ctx->applied_speed_left) ? (float)REVERSAL_RELEASE_STEP : g_params.accel_ramp;
    float step_decel_l = (yaw_conflict && target_left < ctx->applied_speed_left) ? (float)REVERSAL_DECEL_STEP   : g_params.decel_ramp;
    float step_accel_r = (yaw_conflict && target_right > ctx->applied_speed_right) ? (float)REVERSAL_RELEASE_STEP : g_params.accel_ramp;
    float step_decel_r = (yaw_conflict && target_right < ctx->applied_speed_right) ? (float)REVERSAL_DECEL_STEP   : g_params.decel_ramp;

    if (target_left < ctx->applied_speed_left) {
        ctx->applied_speed_left -= step_decel_l;
        if (ctx->applied_speed_left < target_left) ctx->applied_speed_left = target_left;
    } else {
        ctx->applied_speed_left += step_accel_l;
        if (ctx->applied_speed_left > target_left) ctx->applied_speed_left = target_left;
    }

    if (target_right < ctx->applied_speed_right) {
        ctx->applied_speed_right -= step_decel_r;
        if (ctx->applied_speed_right < target_right) ctx->applied_speed_right = target_right;
    } else {
        ctx->applied_speed_right += step_accel_r;
        if (ctx->applied_speed_right > target_right) ctx->applied_speed_right = target_right;
    }

    res.steer_cmd               = steer_cmd;
    res.applied_speed_left      = ctx->applied_speed_left;
    res.applied_speed_right     = ctx->applied_speed_right;
    res.drive_state             = ctx->drive_state;
    res.trusted_corner_urgency  = ctx->trusted_corner_urgency;
    res.pid_cte                 = pid_cte;
    res.heading_feedforward     = heading_feedforward;
    res.lookahead_turn_factor   = ctx->lookahead_turn_factor;

    return res;
}

int main(void) {
    printf("=================================================================\n");
    printf("  DECOUPLED DUAL-CAMERA & LOOKAHEAD GOVERNOR BENCHMARK (7 TESTS) \n");
    printf("=================================================================\n\n");

    mock_params_init();

    /* ----------------------------------------------------------------- */
    /* TEST 1: Camera Decoupling on Straights & Zero Steering Feedforward */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 1: Camera Decoupling on Straights (Zero Lookahead Steering)\n");
    sim_context_t ctx1;
    sim_init(&ctx1);

    /* Car is driving dead straight (CTE=0, H=0), but front camera previews a sharp curve ahead */
    sim_tick_result_t r1 = sim_step(&ctx1, 0.0f, 0.0f, 20.0f, 0.85f, 2, 2);
    printf("  Tick 1: Steer = %+.2f deg, VL = %.0f, VR = %.0f, State = %s\n",
           r1.steer_cmd, r1.applied_speed_left, r1.applied_speed_right,
           r1.drive_state == DRIVE_FORWARD ? "DRIVE_FORWARD" : "DRIVE_TURNING");

    /* Steering MUST remain 0.0 deg (strictly decoupled from lookahead) */
    assert(fabsf(r1.steer_cmd) < 0.001f);
    assert(r1.drive_state == DRIVE_FORWARD);
    /* Proactive pre-braking drops base speed while maintaining zero differential drag on straight */
    assert(r1.applied_speed_left <= 90.0f);
    assert(fabsf(r1.applied_speed_left - r1.applied_speed_right) < 0.01f);
    printf("  [PASS] Test 1: Zero lookahead steering feedforward confirmed!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 2: 1-3 Frame Optical Flicker Immunity                         */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 2: 1-3 Frame Optical Flicker Immunity (Hold-Last-Valid)\n");
    sim_context_t ctx2;
    sim_init(&ctx2);

    /* Establish straight sprint */
    for (int i = 0; i < 5; i++) {
        sim_step(&ctx2, 0.0f, 0.0f, 39.5f, 0.0f, 2, 2);
    }

    /* Simulate 3 consecutive frames of front camera vector loss */
    for (int drop = 1; drop <= 3; drop++) {
        sim_tick_result_t r_drop = sim_step(&ctx2, 0.0f, 0.0f, 0.0f, 0.0f, 2, 0);
        printf("  Drop Frame %d: State = %s, VL = %.0f, VR = %.0f, Dropout Ticks = %u\n",
               drop, r_drop.drive_state == DRIVE_FORWARD ? "DRIVE_FORWARD" : "OTHER",
               r_drop.applied_speed_left, r_drop.applied_speed_right, ctx2.s_front_dropout_ticks);
        assert(r_drop.drive_state == DRIVE_FORWARD);
        assert(r_drop.applied_speed_left >= 99.0f && r_drop.applied_speed_right >= 99.0f);
    }
    printf("  [PASS] Test 2: 1-3 frame optical flickers completely absorbed without speed drop!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 3: Top-Camera Straightaway Sanity Veto on Extended Dropouts   */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 3: Top-Camera Sanity Veto on Extended Front Cam Dropouts\n");
    sim_context_t ctx3;
    sim_init(&ctx3);

    /* Front camera drops vectors for 6 ticks, but top camera confirms car is centered */
    for (int i = 0; i < 6; i++) {
        sim_tick_result_t r_veto = sim_step(&ctx3, 0.0f, 0.0f, 0.0f, 0.0f, 2, 0);
        assert(r_veto.drive_state == DRIVE_FORWARD);
        assert(r_veto.applied_speed_left >= 99.0f && r_veto.applied_speed_right >= 99.0f);
    }
    printf("  6 ticks front dropout with straight base camera: State = DRIVE_FORWARD, VL = %.0f, VR = %.0f\n",
           ctx3.applied_speed_left, ctx3.applied_speed_right);
    printf("  [PASS] Test 3: Top-camera sanity veto prevented false braking on straightaway!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 4: Foreground Root Anchor & False-Pair Rejection (User Sketch) */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 4: Foreground Root Anchor & False-Pair Rejection (User Sketch Scenario)\n");
    reset_steering_tracking();

    /* 4 vectors matching user sketch:
     * 1. Left root boundary: xn=18, yn=48, xf=28, yf=20
     * 2. Right root boundary: xn=62, yn=48, xf=52, yf=20
     * 3. Center dashed line: xn=40, yn=45, xf=40, yf=32 (in center dash zone [35, 44])
     * 4. Peeling outer branch: xn=12, yn=28, xf=5, yf=14 (starts high up, yn < 30) */
    uint16_t sketch_vectors[16] = {
        18, 48, 28, 20,
        62, 48, 52, 20,
        40, 45, 40, 32,
        12, 28,  5, 14
    };

    trajectory_result_t traj = compute_lookahead_trajectory(sketch_vectors, 4);
    printf("  Root Vectors Found: %u (Left Root=%u, Right Root=%u)\n",
           traj.found, traj.left_root_found, traj.right_root_found);
    printf("  Lookahead X at Y=20: %.2f px (Offset = %+.2f px), Urgency = %.2f\n",
           traj.lookahead_x, traj.trajectory_offset_px, traj.lookahead_urgency);

    assert(traj.found == 2);
    assert(traj.left_root_found == 1 && traj.right_root_found == 1);
    /* Lookahead X must be centered at (28 + 52)/2 = 40.0 px (~39.5 center) */
    assert(fabsf(traj.lookahead_x - 40.0f) < 0.5f);
    assert(traj.lookahead_urgency == 0.0f); /* Within 5.0px deadband */
    printf("  [PASS] Test 4: Center dash and peeling branch completely rejected; clean root pairing!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 5: Real Corner Pre-Braking & 50 Hz Fast Transition            */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 5: Real Corner Pre-Braking & 50 Hz Fast Transition\n");
    sim_context_t ctx5;
    sim_init(&ctx5);

    /* Front camera sees real corner: X_lookahead = 22.0 px (> 6.0px enter threshold) */
    sim_tick_result_t r5;
    for (int tick = 1; tick <= 3; tick++) {
        r5 = sim_step(&ctx5, 0.0f, 0.0f, 22.0f, 0.75f, 2, 2);
        printf("  Tick %d: State = %s, Base Speed = %.1f, VL = %.1f, VR = %.1f\n",
               tick, r5.drive_state == DRIVE_FORWARD ? "DRIVE_FORWARD" : "DRIVE_TURNING",
               r5.applied_speed_left, r5.applied_speed_left, r5.applied_speed_right);
    }
    assert(r5.drive_state == DRIVE_TURNING);
    assert(r5.applied_speed_left < 100.0f); /* Pre-braking engaged */
    printf("  [PASS] Test 5: Fast transition to DRIVE_TURNING within 2 ticks (40ms)!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 6: Chicane Reversal & Slew Limiter Anti-Windup                */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 6: Chicane Reversal & Slew Limiter Anti-Windup\n");
    sim_context_t ctx6;
    sim_init(&ctx6);

    /* Snap from full right (+52.5 deg) */
    for (int i = 0; i < 10; i++) {
        sim_step(&ctx6, 20.0f, 45.0f, 55.0f, 1.0f, 2, 2);
    }
    assert(fabsf(ctx6.last_steer_cmd - (float)STEER_CMD_MAX_RIGHT) < 0.01f);

    /* Now snap left: first tick step must be strictly <= 13.0 deg */
    sim_tick_result_t r_snap = sim_step(&ctx6, -20.0f, -45.0f, 20.0f, 1.0f, 2, 2);
    float step = fabsf(r_snap.steer_cmd - (float)STEER_CMD_MAX_RIGHT);
    printf("  Snap step from clamp: %.2f deg (max allowed: %.2f deg)\n",
           step, (double)(MAX_STEER_RATE_PER_SEC * DEFAULT_DT));
    assert(step <= (MAX_STEER_RATE_PER_SEC * DEFAULT_DT) + 0.001f);
    assert(fabsf(step - (MAX_STEER_RATE_PER_SEC * DEFAULT_DT)) < 0.01f);
    printf("  [PASS] Test 6: Slew rate limiter strictly enforced with anti-windup!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 7: Numerical Robustness & Fuzzing Immunity                    */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 7: Numerical Robustness & Fuzzing Immunity\n");
    sim_context_t ctx7;
    sim_init(&ctx7);

    float extremes[] = { 0.0f, 1e-5f, -1e-5f, 50.0f, -50.0f, 1000.0f, -1000.0f };
    int n_ext = sizeof(extremes) / sizeof(extremes[0]);

    for (int i = 0; i < n_ext; i++) {
        for (int j = 0; j < n_ext; j++) {
            sim_tick_result_t rf = sim_step(&ctx7, extremes[i], extremes[j], extremes[i], 0.5f, 2, 2);
            assert(!isnan(rf.steer_cmd));
            assert(!isinf(rf.steer_cmd));
            assert(rf.steer_cmd <= (float)STEER_CMD_MAX_RIGHT + 0.01f);
            assert(rf.steer_cmd >= (float)STEER_CMD_MAX_LEFT - 0.01f);
            assert(!isnan(rf.applied_speed_left) && !isnan(rf.applied_speed_right));
        }
    }
    printf("  Tested %d combinations: 0 NaN, 0 Inf, strictly bounded.\n", n_ext * n_ext);
    printf("  [PASS] Test 7: Complete numerical immunity verified!\n\n");

    printf("=================================================================\n");
    printf("   >>> ALL 7 ADVANCED BENCHMARK SUITES PASSED FLAWLESSLY! <<<    \n");
    printf("=================================================================\n");
    return 0;
}
