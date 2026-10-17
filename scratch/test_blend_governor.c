#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#include "Config.h"

robot_params_t g_params;

void params_init(void) {
    g_params.kp_straight         = KP_STRAIGHT;
    g_params.kp_corner           = KP_CORNER;
    g_params.ki                  = DEFAULT_KI;
    g_params.kd_straight         = KD_STRAIGHT;
    g_params.kd_corner           = KD_CORNER;
    g_params.speed_straight      = SPEED_STRAIGHT;
    g_params.speed_outside_corner= SPEED_OUTSIDE_CORNER;
    g_params.speed_inside_min    = SPEED_INSIDE_MIN;
    g_params.k_cte               = DEFAULT_K_CTE;
    g_params.k_heading           = DEFAULT_K_HEADING;
    g_params.k_cte_corner        = DEFAULT_K_CTE_CORNER;
    g_params.k_heading_corner    = DEFAULT_K_HEADING_CORNER;
    g_params.h_lo                = DEFAULT_H_LO;
    g_params.h_hi                = DEFAULT_H_HI;
    g_params.blend_attack        = DEFAULT_BLEND_ATTACK;
    g_params.blend_release       = DEFAULT_BLEND_RELEASE;
    g_params.front_angle_thresh  = DEFAULT_FRONT_ANGLE_THRESH;
    g_params.blend_enable        = DEFAULT_BLEND_ENABLE;
    g_params.k_lookahead         = DEFAULT_K_LOOKAHEAD;
    g_params.dt                  = DEFAULT_DT;
    g_params.alpha               = DEFAULT_ALPHA;
    g_params.steering_offset     = STEERING_OFFSET;
    g_params.accel_ramp          = ACCEL_RAMP_STEP;
    g_params.decel_ramp          = DECEL_RAMP_STEP;
    g_params.drive_scale         = DEFAULT_DRIVE_SCALE;
    g_params.shadow_mode_enable  = DEFAULT_SHADOW_MODE;
    g_params.motor_enable        = DEFAULT_MOTOR_ENABLE;
    g_params.state               = STATE_IDLE;
}

/* Structure to simulate vehicle control tick */
typedef struct {
    robot_params_t params;
    float filtered_cte;
    float filtered_heading;
    float filtered_front_angle;
    float steer_cmd;
    float last_steer_cmd;
    float applied_speed_left;
    float applied_speed_right;
    uint32_t s_rollout_ticks;
    uint32_t s_reacquire_ticks;
    uint8_t s_intersection_ahead;
    float s_corner_blend;
    float s_blend_target;

    /* Output tracking */
    float steer_sent;
    float target_left;
    float target_right;
    float target_speed;
    float steer_top;
    float front_t;
    float head_t;
    int steer_call_count;
} test_vehicle_t;

void Steer_mock(test_vehicle_t *v, float angle) {
    v->steer_sent = angle;
    v->steer_call_count++;
}

void test_vehicle_init(test_vehicle_t *v, uint8_t blend_mode) {
    memset(v, 0, sizeof(test_vehicle_t));
    params_init();
    v->params = g_params;
    v->params.blend_enable = blend_mode;
    v->applied_speed_left  = v->params.speed_straight;
    v->applied_speed_right = v->params.speed_straight;
    v->s_rollout_ticks     = 10;
}

void test_step(test_vehicle_t *v, uint8_t sharp_corner, uint8_t front_found, uint8_t sr_found, float cte, float head, float front_ang) {
    v->steer_call_count = 0;
    v->filtered_cte = cte;
    v->filtered_heading = head;
    v->filtered_front_angle = front_ang;

    float front_t      = 0.0f;
    float head_t       = 0.0f;
    float steer_top    = 0.0f;
    float target_speed = v->params.speed_straight;
    float target_left  = target_speed;
    float target_right = target_speed;

    if (!v->params.blend_enable) {
        /* LEGACY DISCRETE MODE BYPASS (blend 0) */
        front_t        = sharp_corner ? 1.0f : 0.0f;
        head_t         = 0.0f;
        v->s_corner_blend = front_t;
        v->s_blend_target = front_t;

        if (sr_found == 0 && !v->s_intersection_ahead) {
            v->steer_cmd = v->last_steer_cmd;
            steer_top    = v->last_steer_cmd;
        } else {
            steer_top    = (v->params.k_cte * v->filtered_cte) + (v->params.k_heading * v->filtered_heading);
            v->steer_cmd = steer_top;
            if (sharp_corner) {
                v->steer_cmd += (float)FRONT_FEEDFORWARD_GAIN * v->filtered_front_angle;
            }
        }

        if (v->s_rollout_ticks <= 5) v->steer_cmd = 0.0f;
        if (v->steer_cmd > (float)STEER_CMD_MAX_RIGHT) v->steer_cmd = (float)STEER_CMD_MAX_RIGHT;
        if (v->steer_cmd < (float)STEER_CMD_MAX_LEFT)  v->steer_cmd = (float)STEER_CMD_MAX_LEFT;
        Steer_mock(v, v->steer_cmd + v->params.steering_offset);
        v->last_steer_cmd = v->steer_cmd;

        if (sharp_corner) {
            target_speed = v->params.speed_outside_corner;
            if (fabsf(v->steer_cmd) > (float)DIFFERENTIAL_DEADZONE_DEG) {
                if (v->steer_cmd > 0.0f) {
                    target_left  = v->params.speed_outside_corner;
                    target_right = v->params.speed_inside_min;
                } else {
                    target_left  = v->params.speed_inside_min;
                    target_right = v->params.speed_outside_corner;
                }
            } else {
                target_left  = target_speed;
                target_right = target_speed;
            }
        } else {
            target_speed = v->params.speed_straight;
            float abs_steer = fabsf(v->steer_cmd);
            if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
                float steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                                   ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
                if (steer_norm > 1.0f) steer_norm = 1.0f;
                float diff_drag = steer_norm * (target_speed - v->params.speed_inside_min);
                if (v->steer_cmd > 0.0f) {
                    target_left  = target_speed;
                    target_right = target_speed - diff_drag;
                } else {
                    target_left  = target_speed - diff_drag;
                    target_right = target_speed;
                }
            } else {
                target_left  = target_speed;
                target_right = target_speed;
            }
        }
    } else {
        /* CONTINUOUS ASYMMETRIC CORNER BLEND GOVERNOR (blend 1) */
        float hold = 0.0f;

        if (v->s_intersection_ahead) {
            v->s_corner_blend += v->params.blend_release * (0.0f - v->s_corner_blend);
            v->s_blend_target = 0.0f;
        } else {
            if (front_found > 0) {
                float fa_span = (float)FRONT_CORNER_ANGLE_THRESH_DEG - v->params.front_angle_thresh;
                if (fa_span < 5.0f) fa_span = 5.0f;
                front_t = (fabsf(v->filtered_front_angle) - v->params.front_angle_thresh) / fa_span;
                if (front_t < 0.0f) front_t = 0.0f;
                if (front_t > 1.0f) front_t = 1.0f;
            }
            if (sharp_corner) {
                front_t = 1.0f;
            }

            float h_range = v->params.h_hi - v->params.h_lo;
            if (h_range > 0.1f) {
                head_t = (fabsf(v->filtered_heading) - v->params.h_lo) / h_range;
                if (head_t < 0.0f) head_t = 0.0f;
                if (head_t > 1.0f) head_t = 1.0f;
            }

            hold = fmaxf(front_t, head_t);
            v->s_blend_target = hold;

            if (front_t > v->s_corner_blend) {
                v->s_corner_blend += v->params.blend_attack * (front_t - v->s_corner_blend);
            } else if (hold < v->s_corner_blend) {
                v->s_corner_blend += v->params.blend_release * (hold - v->s_corner_blend);
            }
        }

        if (sr_found == 0 && !v->s_intersection_ahead) {
            v->steer_cmd = v->last_steer_cmd;
            steer_top    = v->last_steer_cmd;
        } else {
            float active_k_cte     = v->params.k_cte + v->s_corner_blend * (v->params.k_cte_corner - v->params.k_cte);
            float active_k_heading = v->params.k_heading + v->s_corner_blend * (v->params.k_heading_corner - v->params.k_heading);

            steer_top = (active_k_cte * v->filtered_cte) + (active_k_heading * v->filtered_heading);
            v->steer_cmd = steer_top;
            v->steer_cmd += v->s_corner_blend * (float)FRONT_FEEDFORWARD_GAIN * v->filtered_front_angle;
        }

        if (v->s_rollout_ticks <= 5) v->steer_cmd = 0.0f;
        if (v->steer_cmd > (float)STEER_CMD_MAX_RIGHT) v->steer_cmd = (float)STEER_CMD_MAX_RIGHT;
        if (v->steer_cmd < (float)STEER_CMD_MAX_LEFT)  v->steer_cmd = (float)STEER_CMD_MAX_LEFT;
        Steer_mock(v, v->steer_cmd + v->params.steering_offset);
        v->last_steer_cmd = v->steer_cmd;

        target_speed = v->params.speed_straight - v->s_corner_blend * (v->params.speed_straight - v->params.speed_outside_corner);
        target_left  = target_speed;
        target_right = target_speed;

        float abs_steer = fabsf(v->steer_cmd);
        if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
            float steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                               ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
            if (steer_norm > 1.0f) steer_norm = 1.0f;

            float diff_mult = steer_norm + v->s_corner_blend * (1.0f - steer_norm);
            float diff_drag = diff_mult * (target_speed - v->params.speed_inside_min);

            if (v->steer_cmd > 0.0f) {
                target_left  = target_speed;
                target_right = target_speed - diff_drag;
            } else {
                target_left  = target_speed - diff_drag;
                target_right = target_speed;
            }
        } else {
            target_left  = target_speed;
            target_right = target_speed;
        }
    }

    v->target_left  = target_left;
    v->target_right = target_right;
    v->target_speed = target_speed;
    v->steer_top    = steer_top;
    v->front_t      = front_t;
    v->head_t       = head_t;
}

int main(void) {
    printf("=============================================================\n");
    printf("  RUNNING CONTINUOUS ASYMMETRIC CORNER BLEND GOVERNOR TESTS  \n");
    printf("=============================================================\n");

    /* TEST 1: Exactly 1 Steer() call per tick in both modes */
    {
        test_vehicle_t v0, v1;
        test_vehicle_init(&v0, 0);
        test_vehicle_init(&v1, 1);
        test_step(&v0, 1, 1, 1, 0.0f, 15.0f, 25.0f);
        test_step(&v1, 1, 1, 1, 0.0f, 15.0f, 25.0f);
        assert(v0.steer_call_count == 1);
        assert(v1.steer_call_count == 1);
        printf("[Test 1] Steer() called exactly once per cycle (no double calls)... PASS\n");
    }

    /* TEST 2: Legacy mode (blend 0) exact parity */
    {
        test_vehicle_t v;
        test_vehicle_init(&v, 0);

        /* Straight: speed 100%, wheels centered */
        test_step(&v, 0, 1, 1, 0.0f, 0.0f, 0.0f);
        assert(v.target_speed == 100.0f);
        assert(v.target_left == 100.0f && v.target_right == 100.0f);

        /* Sharp corner: speed 50%, inside wheel pinned to 32.0% at partial steer (15 deg) */
        test_step(&v, 1, 1, 1, 0.0f, 15.0f, 20.0f);
        assert(v.target_speed == 50.0f);
        assert(v.steer_cmd > 8.0f); /* Right turn past deadzone */
        assert(fabsf(v.target_left - 50.0f) < 0.01f);
        assert(fabsf(v.target_right - 32.0f) < 0.01f);
        printf("[Test 2] Legacy mode (blend 0) parity: inside=32%%, outside=50%%... PASS\n");
    }

    /* TEST 3: Straightaway weave ratchet test (heading CANNOT pump blend) */
    {
        test_vehicle_t v;
        test_vehicle_init(&v, 1);

        /* 20 ticks of straight weaving with heading swinging +-18 deg, FS=0 */
        for (int i = 0; i < 20; i++) {
            float weave_h = (i % 2 == 0) ? 18.0f : -18.0f;
            test_step(&v, 0, 1, 1, 0.0f, weave_h, 0.0f);
            assert(v.front_t == 0.0f);
            assert(v.s_corner_blend == 0.0f); /* Ratchet diode: heading cannot raise blend! */
            assert(v.target_speed == 100.0f);
        }
        printf("[Test 3] Straightaway weave diode guard (heading cannot raise blend)... PASS\n");
    }

    /* TEST 4: Corner entry fast attack (~5 ticks to ~88%) */
    {
        test_vehicle_t v;
        test_vehicle_init(&v, 1);

        /* Front camera sees corner (FS=30.0f -> front_t=1.0) */
        for (int i = 0; i < 5; i++) {
            test_step(&v, 0, 1, 1, 0.0f, 0.0f, 30.0f);
        }
        /* 1 - (1-0.35)^5 = 1 - 0.116 = 0.884 */
        assert(v.s_corner_blend >= 0.88f);
        printf("[Test 4] Front camera attack: reached %.1f%% in 5 ticks (100ms)... PASS\n", v.s_corner_blend * 100.0f);
    }

    /* TEST 5: Mid-corner apex dropout hold (heading holds blend at 100%) */
    {
        test_vehicle_t v;
        test_vehicle_init(&v, 1);

        /* Reach full blend */
        for (int i = 0; i < 15; i++) test_step(&v, 1, 1, 1, 0.0f, 25.0f, 30.0f);
        assert(v.s_corner_blend > 0.99f);

        /* Front camera drops out (FS=0, front_found=0), heading remains 25 deg */
        for (int i = 0; i < 10; i++) {
            test_step(&v, 0, 0, 1, 0.0f, 25.0f, 0.0f);
            assert(v.head_t == 1.0f);
            assert(v.s_corner_blend > 0.99f); /* Heading floor prevents any washout! */
        }
        printf("[Test 5] Apex front-camera dropout: heading holds blend at 100%%... PASS\n");
    }

    /* TEST 6: Restored full differential bite at blend=1.0 */
    {
        test_vehicle_t v;
        test_vehicle_init(&v, 1);

        /* Pinned corner blend */
        for (int i = 0; i < 15; i++) test_step(&v, 1, 1, 1, 0.0f, 25.0f, 30.0f);
        assert(v.s_corner_blend > 0.99f);

        /* Steer at 18 deg (partial steer: abs_steer > 8.0, steer_norm ~ 0.37) */
        /* Under old buggy formula, inside was 43%. Under restored formula, must be 32%! */
        test_step(&v, 1, 1, 1, 0.0f, 15.0f, 0.0f);
        assert(fabsf(v.target_right - 32.0f) < 0.2f);
        assert(fabsf(v.target_left - 50.0f) < 0.2f);
        printf("[Test 6] Restored differential bite at partial steer (inside=%.1f%%, outside=%.1f%%)... PASS\n",
               v.target_right, v.target_left);
    }

    /* TEST 7: Corner exit smooth release (~25 ticks) */
    {
        test_vehicle_t v;
        test_vehicle_init(&v, 1);

        /* Full corner */
        for (int i = 0; i < 15; i++) test_step(&v, 1, 1, 1, 0.0f, 25.0f, 30.0f);

        /* Exit corner: front_t = 0, heading = 0 */
        int ticks_to_half = 0;
        int ticks_to_tenth = 0;
        for (int i = 1; i <= 50; i++) {
            test_step(&v, 0, 1, 1, 0.0f, 0.0f, 0.0f);
            if (v.s_corner_blend <= 0.50f && ticks_to_half == 0) ticks_to_half = i;
            if (v.s_corner_blend <= 0.10f && ticks_to_tenth == 0) ticks_to_tenth = i;
        }
        assert(ticks_to_half >= 10 && ticks_to_half <= 14);
        assert(ticks_to_tenth >= 25 && ticks_to_tenth <= 38);
        printf("[Test 7] Corner exit release: 50%% at tick %d, <10%% at tick %d... PASS\n",
               ticks_to_half, ticks_to_tenth);
    }

    /* TEST 8: Decoupled front_angle_thresh tuning */
    {
        test_vehicle_t v;
        test_vehicle_init(&v, 1);
        v.params.front_angle_thresh = 20.0f; /* Tuned to 20 deg */

        /* At 20 deg: front_t should be 0.0 */
        test_step(&v, 0, 1, 1, 0.0f, 0.0f, 20.0f);
        assert(fabsf(v.front_t - 0.0f) < 0.01f);

        /* At 30 deg (FRONT_CORNER_ANGLE_THRESH_DEG): front_t should be exactly 1.0 */
        test_step(&v, 0, 1, 1, 0.0f, 0.0f, 30.0f);
        assert(fabsf(v.front_t - 1.0f) < 0.01f);
        printf("[Test 8] Decoupled front_angle_thresh (20 deg to 30 deg span)... PASS\n");
    }

    printf("=============================================================\n");
    printf("  ALL CONTINUOUS CORNER BLEND GOVERNOR TESTS PASSED!         \n");
    printf("=============================================================\n");
    return 0;
}
