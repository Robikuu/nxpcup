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
    g_params.kp_straight         = KP_STRAIGHT;
    g_params.kp_corner           = KP_CORNER;
    g_params.ki                  = DEFAULT_KI;
    g_params.kd_straight         = KD_STRAIGHT;
    g_params.kd_corner           = KD_CORNER;
    g_params.speed_straight       = SPEED_STRAIGHT;
    g_params.speed_outside_corner = SPEED_OUTSIDE_CORNER;
    g_params.speed_inside_min     = SPEED_INSIDE_MIN;
    g_params.k_cte               = DEFAULT_K_CTE;
    g_params.k_heading           = DEFAULT_K_HEADING;
    g_params.k_lookahead         = DEFAULT_K_LOOKAHEAD;
    g_params.dt                  = DEFAULT_DT;
    g_params.alpha               = DEFAULT_ALPHA;
    g_params.steering_offset     = STEERING_OFFSET;
    g_params.accel_ramp          = ACCEL_RAMP_STEP;
    g_params.decel_ramp          = DECEL_RAMP_STEP;
    g_params.state               = STATE_RUNNING;
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
    float lookahead_feedforward;
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
}

sim_tick_result_t sim_step(sim_context_t *ctx,
                           float raw_cte, float raw_heading,
                           float raw_traj_angle, float raw_traj_urgency,
                           uint8_t top_found, uint8_t front_found)
{
    sim_tick_result_t res;
    memset(&res, 0, sizeof(res));

    /* 1. Trajectory filtering */
    if (front_found >= 1) {
        ctx->filtered_traj_angle  = 0.50f * raw_traj_angle + 0.50f * ctx->filtered_traj_angle;
        ctx->lookahead_turn_factor = 0.50f * raw_traj_urgency + 0.50f * ctx->lookahead_turn_factor;
    } else {
        ctx->filtered_traj_angle  *= 0.80f;
        ctx->lookahead_turn_factor *= 0.50f;
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

    /* 3. Heading and CTE filtering */
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

    /* 4. State Machine Transition */
    drive_state_t desired_state;
    bool corner_trigger = (fabsf(ctx->filtered_heading) >= (float)CORNER_HEADING_THRESHOLD_DEG) ||
                          (fabsf(ctx->filtered_traj_angle) >= 10.0f && ctx->lookahead_turn_factor >= 0.30f);
    if (ctx->drive_state == DRIVE_TURNING) {
        bool straight_confirmed = (fabsf(ctx->filtered_heading) <= 5.0f && fabsf(ctx->filtered_cte) <= 8.0f);
        desired_state = straight_confirmed ? DRIVE_FORWARD : DRIVE_TURNING;
    } else {
        desired_state = corner_trigger ? DRIVE_TURNING : DRIVE_FORWARD;
    }

    if (desired_state != ctx->drive_state) {
        if (ctx->drive_state == DRIVE_FORWARD) {
            /* Chicane / flick protection: ensure curve direction is sustained in ONE direction */
            int8_t current_sign = (ctx->filtered_heading >= 0.0f) ? 1 : -1;
            if (current_sign != ctx->streak_sign) {
                ctx->streak_sign = current_sign;
                ctx->state_streak = 0;
            }
        }
        uint32_t threshold = (desired_state == DRIVE_FORWARD) ? (uint32_t)STATE_EXIT_STREAK : (uint32_t)STATE_ENTER_STREAK;
        if (++ctx->state_streak >= threshold) {
            ctx->drive_state = desired_state;
            ctx->state_streak = 0;
        }
    } else {
        ctx->state_streak = 0;
        ctx->streak_sign  = 0;
    }

    /* 5. Dynamic Gain Scheduling */
    float dz_heading_urgency = fabsf(ctx->filtered_heading) - 2.5f;
    float heading_urgency = (dz_heading_urgency > 0.0f) ? (dz_heading_urgency / ((float)HEADING_FULL_BRAKE_DEG - 2.5f)) : 0.0f;
    if (heading_urgency > 1.0f) heading_urgency = 1.0f;

    float cte_excess = fabsf(ctx->filtered_cte) - 14.0f;
    float cte_urgency = (cte_excess > 0.0f) ? (cte_excess / ((float)CTE_URGENCY_PX - 14.0f)) : 0.0f;
    if (cte_urgency > 1.0f) cte_urgency = 1.0f;

    float la_urgency = 0.0f;
    float dz_traj = fabsf(ctx->filtered_traj_angle) - (float)LOOKAHEAD_DEADBAND_DEG;
    if (dz_traj > 0.0f) {
        la_urgency = dz_traj / ((float)HEADING_FULL_BRAKE_DEG - (float)LOOKAHEAD_DEADBAND_DEG);
        if (la_urgency > 1.0f) la_urgency = 1.0f;
    }
    if (ctx->lookahead_turn_factor > la_urgency) la_urgency = ctx->lookahead_turn_factor;

    float raw_corner_urgency = heading_urgency;
    if (cte_urgency > raw_corner_urgency) raw_corner_urgency = cte_urgency;
    if (la_urgency > raw_corner_urgency)  raw_corner_urgency = la_urgency;

    if (raw_corner_urgency > ctx->trusted_corner_urgency) {
        if (++ctx->urgency_streak >= (uint32_t)CORNER_URGENCY_STREAK) {
            ctx->trusted_corner_urgency = raw_corner_urgency;
            ctx->urgency_streak = 0;
        }
    } else {
        ctx->trusted_corner_urgency = 0.10f * raw_corner_urgency + 0.90f * ctx->trusted_corner_urgency;
        ctx->urgency_streak = 0;
    }

    ctx->steer_pid.kp    = g_params.kp_straight + ctx->trusted_corner_urgency * (g_params.kp_corner - g_params.kp_straight);
    ctx->steer_pid.ki    = g_params.ki;
    ctx->steer_pid.kd    = g_params.kd_straight + ctx->trusted_corner_urgency * (g_params.kd_corner - g_params.kd_straight);
    ctx->steer_pid.alpha = g_params.alpha;

    /* 6. Steering Authority */
    float pid_cte = -pid_update(&ctx->steer_pid, 0.0f, g_params.k_cte * ctx->filtered_cte, g_params.dt);

    if ((ctx->drive_state == DRIVE_TURNING || ctx->trusted_corner_urgency >= 0.20f || fabsf(ctx->filtered_heading) >= 4.0f) &&
        (ctx->filtered_cte * ctx->filtered_heading < 0.0f) &&
        fabsf(ctx->filtered_heading) >= 4.0f) {
        pid_cte = 0.0f;
    }

    float abs_h = fabsf(ctx->filtered_heading);
    float effective_h = 0.0f;
    if (abs_h > (float)HEADING_DEADBAND_DEG) {
        effective_h = (abs_h - (float)HEADING_DEADBAND_DEG) * ((ctx->filtered_heading > 0.0f) ? 1.0f : -1.0f);
    }
    float heading_feedforward = g_params.k_heading * effective_h;

    float abs_traj = fabsf(ctx->filtered_traj_angle);
    float effective_traj = 0.0f;
    if (abs_traj > (float)LOOKAHEAD_DEADBAND_DEG) {
        effective_traj = (abs_traj - (float)LOOKAHEAD_DEADBAND_DEG) * ((ctx->filtered_traj_angle > 0.0f) ? 1.0f : -1.0f);
    }
    float lookahead_feedforward = g_params.k_lookahead * effective_traj;

    float raw_steer = pid_cte + heading_feedforward + lookahead_feedforward;

    const float max_steer_step = (float)MAX_STEER_RATE_PER_SEC * g_params.dt;
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

    /* 7. Longitudinal Speeds & Differential Drag */
    float abs_steer = fabsf(steer_cmd);
    float dz_steer = abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG;
    if (dz_steer < 0.0f) dz_steer = 0.0f;
    float steer_norm = dz_steer / ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
    if (steer_norm > 1.0f) steer_norm = 1.0f;

    float abs_heading = fabsf(ctx->filtered_heading);
    float dz_heading = abs_heading - (float)DIFFERENTIAL_DEADZONE_DEG;
    if (dz_heading < 0.0f) dz_heading = 0.0f;
    float heading_norm = dz_heading / ((float)HEADING_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
    if (heading_norm > 1.0f) heading_norm = 1.0f;

    float effective_corner_intensity = heading_norm;
    if (front_found >= 1 && ctx->lookahead_turn_factor > effective_corner_intensity) {
        effective_corner_intensity = ctx->lookahead_turn_factor;
    }
    float longitudinal_base = g_params.speed_straight;
    float active_steer_norm = 0.0f;
    if (ctx->drive_state == DRIVE_TURNING) {
        longitudinal_base = g_params.speed_straight - effective_corner_intensity * (g_params.speed_straight - g_params.speed_outside_corner);
        active_steer_norm = steer_norm;
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

    res.steer_cmd = steer_cmd;
    res.applied_speed_left = ctx->applied_speed_left;
    res.applied_speed_right = ctx->applied_speed_right;
    res.drive_state = ctx->drive_state;
    res.trusted_corner_urgency = ctx->trusted_corner_urgency;
    res.pid_cte = pid_cte;
    res.heading_feedforward = heading_feedforward;
    res.lookahead_feedforward = lookahead_feedforward;
    return res;
}

int main(void)
{
    mock_params_init();
    printf("=================================================================\n");
    printf("     NXP CUP ADVANCED AUTOMATED MULTI-SCENARIO TEST SUITE        \n");
    printf("=================================================================\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 1: Actual User Crash Telemetry Replay Benchmark               */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 1: Actual Crash Telemetry Replay Benchmark\n");
    sim_context_t ctx1;
    sim_init(&ctx1);

    /* Telemetry data leading to crash:
     * Frame 13: CTE=+12.4, H=+0.5, TJ=-1.3, LA=0.0 -> straight
     * Frame 14: CTE=+13.7, H=-17.2, TJ=-24.5, LA=0.61 -> corner entry! (OLD steer was -2.5 deg)
     * Frame 15: CTE=+13.9, H=-26.8, TJ=-27.3, LA=0.65 -> in corner! (OLD steer was -10.2 deg)
     */
    float crash_data[4][4] = {
        { 12.4f,  +0.5f,  -1.3f, 0.00f },
        { 13.7f, -17.2f, -24.5f, 0.61f },
        { 13.9f, -26.8f, -27.3f, 0.65f },
        { 13.9f, -37.7f, -33.0f, 0.16f }
    };

    sim_tick_result_t r_f13;
    for (int t = 0; t < 4; t++) {
        r_f13 = sim_step(&ctx1, crash_data[0][0], crash_data[0][1], crash_data[0][2], crash_data[0][3], 2, 2);
    }
    printf("  Frame 13 (Approaching curve): Steer = %+.2f deg, VL=%.0f, VR=%.0f, State=%s\n",
           r_f13.steer_cmd, r_f13.applied_speed_left, r_f13.applied_speed_right,
           r_f13.drive_state == DRIVE_FORWARD ? "FORWARD" : "TURNING");

    sim_tick_result_t r_f14;
    for (int t = 0; t < 4; t++) {
        r_f14 = sim_step(&ctx1, crash_data[1][0], crash_data[1][1], crash_data[1][2], crash_data[1][3], 2, 2);
    }
    printf("  Frame 14 (Corner Entry):      Steer = %+.2f deg [OLD was -2.5 deg!], VL=%.0f, VR=%.0f, State=%s\n",
           r_f14.steer_cmd, r_f14.applied_speed_left, r_f14.applied_speed_right,
           r_f14.drive_state == DRIVE_FORWARD ? "FORWARD" : "TURNING");
    printf("      -> FF_heading = %+.2f deg, FF_lookahead = %+.2f deg, PID_cte = %+.2f deg\n",
           r_f14.heading_feedforward, r_f14.lookahead_feedforward, r_f14.pid_cte);
    fflush(stdout);
    /* In the old code, steer was only -2.5 deg (understeer!). In new code, must be decisive (> 15 deg)! */
    assert(r_f14.steer_cmd < -15.0f);
    assert(r_f14.drive_state == DRIVE_TURNING);

    sim_tick_result_t r_f15;
    for (int t = 0; t < 4; t++) {
        r_f15 = sim_step(&ctx1, crash_data[2][0], crash_data[2][1], crash_data[2][2], crash_data[2][3], 2, 2);
    }
    printf("  Frame 15 (Apex Authority):    Steer = %+.2f deg [OLD was -10.2 deg!], VL=%.0f, VR=%.0f\n",
           r_f15.steer_cmd, r_f15.applied_speed_left, r_f15.applied_speed_right);
    fflush(stdout);
    /* In the old code, steer was -10.2 deg. In new code, must bite hard into corner (> 25 deg)! */
    assert(r_f15.steer_cmd < -25.0f);
    /* Inside wheel must drag to pivot chassis (VR > VL for left turn) */
    assert(r_f15.applied_speed_right > r_f15.applied_speed_left);
    printf("  [PASS] Test 1: Crash telemetry replay successfully turned into corner with decisive authority!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 2: High-Speed Straightaway Stability (500 frames @ 12.6V)     */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 2: High-Speed Straightaway Stability (500 frames @ 100%% speed)\n");
    sim_context_t ctx2;
    sim_init(&ctx2);

    srand(12345);
    float max_steer_on_straight = 0.0f;
    for (int tick = 0; tick < 500; tick++) {
        /* Gaussian-like noise: CTE in [-2.5, +2.5] px, heading in [-2.0, +2.0] deg, traj in [-3.0, +3.0] deg */
        float noise_cte = ((float)(rand() % 100) / 50.0f - 1.0f) * 2.5f;
        float noise_h   = ((float)(rand() % 100) / 50.0f - 1.0f) * 2.0f;
        float noise_tj  = ((float)(rand() % 100) / 50.0f - 1.0f) * 3.0f;

        sim_tick_result_t r = sim_step(&ctx2, noise_cte, noise_h, noise_tj, 0.0f, 2, 2);

        if (fabsf(r.steer_cmd) > max_steer_on_straight) {
            max_steer_on_straight = fabsf(r.steer_cmd);
        }

        /* Verify feedforward is strictly zero within the deadbands! */
        assert(fabsf(r.heading_feedforward) < 0.01f);
        assert(fabsf(r.lookahead_feedforward) < 0.01f);
        /* Verify 100% full symmetrical motor speed push */
        assert(r.applied_speed_left >= 98.0f && r.applied_speed_right >= 98.0f);
        assert(r.drive_state == DRIVE_FORWARD);
    }
    printf("  Max steer angle over 500 straight frames: %.2f deg (strictly damped within +/- 2.5 deg)\n", max_steer_on_straight);
    assert(max_steer_on_straight < 2.5f);
    printf("  [PASS] Test 2: Straightaway stability confirmed: 0 feedforward twitch, 100%% symmetric sprint!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 3: Chicane / Rapid Direction Reversal (S-Curve)               */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 3: S-Curve Chicane (Flick in DRIVE_FORWARD @ 100%% Speed & Sustained Turn)\n");
    sim_context_t ctx3;
    sim_init(&ctx3);

    /* Part 1: Rapid chicane flick (3 ticks left, then 3 ticks right) */
    sim_tick_result_t r_flick_left;
    for (int i = 0; i < 3; i++) {
        r_flick_left = sim_step(&ctx3, -12.0f, -28.0f, -30.0f, 0.8f, 2, 2);
    }
    printf("  Chicane Part 1 (3-tick Left Flick):  Steer = %+.2f deg, VL = %.0f, VR = %.0f, State = %s\n",
           r_flick_left.steer_cmd, r_flick_left.applied_speed_left, r_flick_left.applied_speed_right,
           r_flick_left.drive_state == DRIVE_FORWARD ? "DRIVE_FORWARD" : "DRIVE_TURNING");
    assert(r_flick_left.drive_state == DRIVE_FORWARD);
    assert(r_flick_left.applied_speed_left >= 98.0f && r_flick_left.applied_speed_right >= 98.0f);
    assert(r_flick_left.steer_cmd < -35.0f);

    /* Flick right (3 ticks right): sign reverses, state_streak resets, staying in DRIVE_FORWARD */
    sim_tick_result_t r_flick_right;
    float max_slew_seen = 0.0f;
    float prev_steer = r_flick_left.steer_cmd;
    for (int i = 0; i < 3; i++) {
        r_flick_right = sim_step(&ctx3, +12.0f, +28.0f, +30.0f, 0.8f, 2, 2);
        float slew = fabsf(r_flick_right.steer_cmd - prev_steer);
        if (slew > max_slew_seen) max_slew_seen = slew;
        prev_steer = r_flick_right.steer_cmd;
    }
    printf("  Chicane Part 2 (3-tick Right Flick): Steer = %+.2f deg, VL = %.0f, VR = %.0f, State = %s\n",
           r_flick_right.steer_cmd, r_flick_right.applied_speed_left, r_flick_right.applied_speed_right,
           r_flick_right.drive_state == DRIVE_FORWARD ? "DRIVE_FORWARD" : "DRIVE_TURNING");
    assert(r_flick_right.drive_state == DRIVE_FORWARD);
    assert(r_flick_right.applied_speed_left >= 98.0f && r_flick_right.applied_speed_right >= 98.0f);
    assert(r_flick_right.steer_cmd > r_flick_left.steer_cmd); /* Reversing aggressively toward right */

    /* Part 2: Sustained corner (continue right turn for 4 more ticks to exceed STATE_ENTER_STREAK) */
    sim_tick_result_t r_sustained;
    for (int i = 0; i < 4; i++) {
        r_sustained = sim_step(&ctx3, +12.0f, +28.0f, +30.0f, 0.8f, 2, 2);
        float slew = fabsf(r_sustained.steer_cmd - prev_steer);
        if (slew > max_slew_seen) max_slew_seen = slew;
        prev_steer = r_sustained.steer_cmd;
    }
    printf("  Sustained Corner (Confirmed >4 ticks): Steer = %+.2f deg, VL = %.0f, VR = %.0f, State = %s\n",
           r_sustained.steer_cmd, r_sustained.applied_speed_left, r_sustained.applied_speed_right,
           r_sustained.drive_state == DRIVE_FORWARD ? "DRIVE_FORWARD" : "DRIVE_TURNING");
    assert(r_sustained.drive_state == DRIVE_TURNING);
    assert(r_sustained.applied_speed_right < r_sustained.applied_speed_left); /* Right inside wheel dragged */
    printf("  Max slew step seen during rapid reversal: %.2f deg/tick (max limit: %.2f deg/tick)\n",
           max_slew_seen, (double)(MAX_STEER_RATE_PER_SEC * DEFAULT_DT));
    assert(max_slew_seen <= (MAX_STEER_RATE_PER_SEC * DEFAULT_DT) + 0.001f);

    /* Part 3: Anti-windup check on sudden reversal from saturation */
    /* Saturate hard right to physical clamp (+52.5 deg) */
    for (int i = 0; i < 5; i++) {
        r_sustained = sim_step(&ctx3, +20.0f, +45.0f, +40.0f, 1.0f, 2, 2);
    }
    assert(fabsf(r_sustained.steer_cmd - (float)STEER_CMD_MAX_RIGHT) < 0.01f);
    assert(fabsf(ctx3.last_steer_cmd - (float)STEER_CMD_MAX_RIGHT) < 0.01f);
    /* Now snap left: next tick MUST step down immediately by 13.0 deg without delay */
    sim_tick_result_t r_snap = sim_step(&ctx3, -20.0f, -45.0f, -40.0f, 1.0f, 2, 2);
    float expected_steer = (float)STEER_CMD_MAX_RIGHT - (MAX_STEER_RATE_PER_SEC * DEFAULT_DT);
    printf("  Anti-Windup Reversal Check: Clamped = +%.2f deg -> Snap Tick 1 = %+.2f deg (expected %+.2f deg)\n",
           (float)STEER_CMD_MAX_RIGHT, r_snap.steer_cmd, expected_steer);
    assert(fabsf(r_snap.steer_cmd - expected_steer) < 0.01f);
    printf("  [PASS] Test 3: S-curve chicane stays in DRIVE_FORWARD at 100%% speed, anti-windup verified!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 4: Single Boundary Fallback & Curve Tracking                  */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 4: Single-Boundary Curve Tracking & Perspective Synthesis\n");
    reset_steering_tracking();
    /* Vector representing single curved left boundary: (xn=16, yn=45, xf=28, yf=15) */
    uint16_t v_single_curve[4] = { 16, 45, 28, 15 };
    steering_result_t sr_single = compute_steering_error(v_single_curve, 1);
    printf("  Single Boundary: found=%u, CTE=%.2f px, heading=%.2f deg\n",
           sr_single.found, sr_single.cte, sr_single.heading_deg);
    assert(sr_single.found == 1);
    assert(sr_single.heading_deg > 10.0f); /* Correctly identifies turn direction */
    assert(sr_single.cte > -20.0f && sr_single.cte < 20.0f);
    printf("  [PASS] Test 4: Single boundary synthesized reliable centerline error!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 5: Line Loss Coasting Window & Emergency Braking              */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 5: Line Loss Coasting Window vs Emergency Braking\n");
    sim_context_t ctx5;
    sim_init(&ctx5);

    /* 10 ticks line lost (e.g. crossing intersection gap): MUST COAST, NOT BRAKE */
    for (int i = 0; i < 10; i++) {
        sim_tick_result_t r = sim_step(&ctx5, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
        assert(r.drive_state != DRIVE_BRAKING);
        assert(r.applied_speed_left > 50.0f && r.applied_speed_right > 50.0f);
    }
    printf("  10 ticks line lost: Vehicle in coast mode (VL=%.0f, VR=%.0f), NO premature brake.\n",
           ctx5.applied_speed_left, ctx5.applied_speed_right);

    /* 15 more ticks line lost (total 25 ticks > 18 limit): MUST TRIGGER EMERGENCY BRAKE */
    for (int i = 0; i < 15; i++) {
        sim_step(&ctx5, 0.0f, 0.0f, 0.0f, 0.0f, 0, 0);
    }
    printf("  25 ticks line lost (>18 limit): State = %s (EMERGENCY BRAKE ENGAGED)\n",
           ctx5.drive_state == DRIVE_BRAKING ? "BRAKING" : "OTHER");
    assert(ctx5.drive_state == DRIVE_BRAKING);
    assert(ctx5.applied_speed_left == 0.0f && ctx5.applied_speed_right == 0.0f);
    printf("  [PASS] Test 5: Coasting and emergency braking debounce verified!\n\n");

    /* ----------------------------------------------------------------- */
    /* TEST 6: Fuzzing, Extreme Inputs & NaN / Inf Immunity               */
    /* ----------------------------------------------------------------- */
    printf(">>> TEST 6: Numerical Robustness & Fuzzing Immunity\n");
    sim_context_t ctx6;
    sim_init(&ctx6);

    float extreme_vals[] = {
        0.0f, 1e-6f, -1e-6f, 50.0f, -50.0f, 1000.0f, -1000.0f, 1e6f, -1e6f
    };
    int num_extremes = sizeof(extreme_vals) / sizeof(extreme_vals[0]);

    for (int i = 0; i < num_extremes; i++) {
        for (int j = 0; j < num_extremes; j++) {
            sim_tick_result_t r = sim_step(&ctx6, extreme_vals[i], extreme_vals[j], extreme_vals[i], 0.5f, 2, 2);
            /* Output steer must never be NaN or Inf */
            assert(!isnan(r.steer_cmd));
            assert(!isinf(r.steer_cmd));
            /* Steer command must strictly respect mechanical travel bounds */
            assert(r.steer_cmd <= (float)STEER_CMD_MAX_RIGHT + 0.01f);
            assert(r.steer_cmd >= (float)STEER_CMD_MAX_LEFT - 0.01f);
            /* Motor speeds must be valid */
            assert(!isnan(r.applied_speed_left) && !isnan(r.applied_speed_right));
            assert(r.applied_speed_left >= 0.0f && r.applied_speed_left <= 100.0f);
            assert(r.applied_speed_right >= 0.0f && r.applied_speed_right <= 100.0f);
        }
    }
    printf("  Tested %d extreme combinations: 0 NaN, 0 Inf, all commands strictly clamped to physical limits.\n",
           num_extremes * num_extremes);
    printf("  [PASS] Test 6: Complete numerical immunity verified!\n\n");

    printf("=================================================================\n");
    printf("   >>> ALL 6 ADVANCED TEST SUITES PASSED FLAWLESSLY! <<<         \n");
    printf("=================================================================\n");
    return 0;
}
