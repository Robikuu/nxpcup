#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>
#include <string.h>

#include "Config.h"
#include "vectors.h"

/* Include the actual perception implementation */
#include "../source/vectors.c"

/* Simulation Context mirroring source/main.c */
typedef struct {
    float    filtered_cte;
    float    filtered_heading;
    float    filtered_front_angle;
    float    applied_speed_left;
    float    applied_speed_right;
    float    steer_cmd;
    float    last_steer_cmd;
    uint32_t s_rollout_ticks;
    uint32_t s_off_track_ticks;

    /* Intersection Tactical State */
    uint8_t  s_intersection_ahead;
    uint32_t s_intersection_timer;
    uint32_t s_intersection_gap_ticks;
    uint8_t  s_was_in_intersection_gap;
    uint32_t s_reacquire_ticks;

    /* Actuator outputs */
    float    target_speed;
    float    target_left;
    float    target_right;
    float    servo_steer;
} sim_car_t;

void sim_car_init(sim_car_t *car) {
    memset(car, 0, sizeof(sim_car_t));
    car->applied_speed_left  = SPEED_STRAIGHT;
    car->applied_speed_right = SPEED_STRAIGHT;
    car->s_rollout_ticks     = 10; /* Past initial 5-tick rollout */
    reset_steering_tracking();
}

void sim_control_tick(sim_car_t *car, const horizon_result_t *horizon, const steering_result_t *sr) {
    /* 1. Filter updates */
    if (horizon->found > 0) {
        car->filtered_front_angle = 0.40f * horizon->horizon_angle_deg + 0.60f * car->filtered_front_angle;
    } else {
        car->filtered_front_angle *= 0.85f;
    }

    if (sr->found > 0) {
        car->filtered_cte     = 0.50f * sr->cte + 0.50f * car->filtered_cte;
        car->filtered_heading = 0.50f * sr->heading_deg + 0.50f * car->filtered_heading;
    }

    car->s_rollout_ticks++;

    /* --- Corner Priority Preemption & Tactical Intersection State --- */
    if (horizon->is_sharp_corner || sr->transverse_crosses_center) {
        /* Corner always preempts intersection immediately! */
        car->s_intersection_ahead      = 0;
        car->s_intersection_timer      = 0;
        car->s_intersection_gap_ticks  = 0;
        car->s_was_in_intersection_gap = 0;
        car->s_reacquire_ticks         = 0;
    } else if (horizon->is_intersection_ahead) {
        /* Intersection Latching & Lifetime: refresh persistence timer (~15 ticks) */
        car->s_intersection_ahead = 1;
        car->s_intersection_timer = 15;
    } else if (car->s_intersection_timer > 0) {
        car->s_intersection_timer--;
        if (car->s_intersection_timer == 0 && car->s_intersection_gap_ticks == 0) {
            car->s_intersection_ahead = 0;
        }
    }

    /* --- Blind Gap Policy (sr->found == 0) & Failsafe Guard --- */
    if (sr->found == 0) {
        car->s_off_track_ticks++;
        if (car->s_intersection_ahead) {
            /* Intersection Blind Gap Active */
            car->s_was_in_intersection_gap = 1;
            car->s_intersection_gap_ticks++;

            /* Distance-scaled window: gap_ticks = 1.5 * 25 / max(speed, 30) * 50 */
            float current_spd = fmaxf(car->applied_speed_left, car->applied_speed_right);
            uint32_t gap_window = (uint32_t)(1.5f * 25.0f / fmaxf(current_spd, 30.0f) * 50.0f);
            if (car->s_intersection_gap_ticks > gap_window) {
                car->s_intersection_ahead     = 0;
                car->s_intersection_gap_ticks = 0;
            }

            /* Decay CTE and heading across the intersection gap */
            car->filtered_cte     *= 0.70f;
            car->filtered_heading *= 0.70f;
        }
        /* If s_intersection_ahead == 0: Corner apex dropout -> zero decay! */
    } else {
        car->s_off_track_ticks = 0;
        if (car->s_was_in_intersection_gap) {
            /* Exiting intersection blind gap -> trigger Re-Acquisition Slew-Rate Limiter (3 ticks) */
            car->s_reacquire_ticks         = 3;
            car->s_was_in_intersection_gap = 0;
            car->s_intersection_gap_ticks  = 0;
            car->s_intersection_ahead      = 0;
            car->s_intersection_timer      = 0;
        }
    }

    /* --- 3. Steering Calculation --- */
    if (sr->found == 0 && !car->s_intersection_ahead) {
        /* Corner apex dropout: HOLD LAST STEER! Do NOT decay or clamp to +-3 deg */
        car->steer_cmd = car->last_steer_cmd;
    } else {
        /* Top Camera Centering between closest lines */
        float steer_top = (DEFAULT_K_CTE * car->filtered_cte) + (DEFAULT_K_HEADING * car->filtered_heading);

        car->steer_cmd = steer_top;
        if (horizon->is_sharp_corner) {
            car->steer_cmd += (float)FRONT_FEEDFORWARD_GAIN * car->filtered_front_angle;
        }

        if (car->s_intersection_ahead && sr->found == 0) {
            /* Intersection blind gap: clamp held steering to +-3.0 deg immediately after PID calculation */
            if (car->steer_cmd > 3.0f)  car->steer_cmd = 3.0f;
            if (car->steer_cmd < -3.0f) car->steer_cmd = -3.0f;
        }

        /* Re-Acquisition Slew-Rate Limiter: <= 4.0 deg/tick for 3 ticks */
        if (car->s_reacquire_ticks > 0) {
            car->s_reacquire_ticks--;
            float delta = car->steer_cmd - car->last_steer_cmd;
            if (delta > 4.0f) {
                car->steer_cmd = car->last_steer_cmd + 4.0f;
            } else if (delta < -4.0f) {
                car->steer_cmd = car->last_steer_cmd - 4.0f;
            }
        }
    }

    /* Servo Clamping & Actuation */
    if (car->steer_cmd > (float)STEER_CMD_MAX_RIGHT) car->steer_cmd = (float)STEER_CMD_MAX_RIGHT;
    if (car->steer_cmd < (float)STEER_CMD_MAX_LEFT)  car->steer_cmd = (float)STEER_CMD_MAX_LEFT;
    car->servo_steer    = car->steer_cmd + STEERING_OFFSET;
    car->last_steer_cmd = car->steer_cmd;

    /* --- 4. Speed & Differential Braking --- */
    car->target_speed = SPEED_STRAIGHT;
    car->target_left  = car->target_speed;
    car->target_right = car->target_speed;

    if (horizon->is_sharp_corner) {
        /* SHARP CORNER MODE: Drop speed to corner speed + active differential pivot */
        car->target_speed = SPEED_OUTSIDE_CORNER;
        if (car->steer_cmd > 0.0f) {
            car->target_left  = SPEED_OUTSIDE_CORNER;
            car->target_right = SPEED_INSIDE_MIN;
        } else {
            car->target_left  = SPEED_INSIDE_MIN;
            car->target_right = SPEED_OUTSIDE_CORNER;
        }
    } else {
        /* STRAIGHT / INTERSECTION / MILD CURVE MODE: 100% full throttle */
        car->target_speed = SPEED_STRAIGHT;
        float abs_steer = fabsf(car->steer_cmd);
        if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
            float steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                               ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
            if (steer_norm > 1.0f) steer_norm = 1.0f;
            float diff_drag = steer_norm * (car->target_speed - SPEED_INSIDE_MIN);

            if (car->steer_cmd > 0.0f) {
                car->target_left  = car->target_speed;
                car->target_right = car->target_speed - diff_drag;
            } else {
                car->target_left  = car->target_speed - diff_drag;
                car->target_right = car->target_speed;
            }
        } else {
            car->target_left  = car->target_speed;
            car->target_right = car->target_speed;
        }
    }

    /* Motor Slew Rate Limiting (Ramps) */
    if (car->target_left < car->applied_speed_left) {
        car->applied_speed_left -= DECEL_RAMP_STEP;
        if (car->applied_speed_left < car->target_left) car->applied_speed_left = car->target_left;
    } else {
        car->applied_speed_left += ACCEL_RAMP_STEP;
        if (car->applied_speed_left > car->target_left) car->applied_speed_left = car->target_left;
    }

    if (car->target_right < car->applied_speed_right) {
        car->applied_speed_right -= DECEL_RAMP_STEP;
        if (car->applied_speed_right < car->target_right) car->applied_speed_right = car->target_right;
    } else {
        car->applied_speed_right += ACCEL_RAMP_STEP;
        if (car->applied_speed_right > car->target_right) car->applied_speed_right = car->target_right;
    }
}

/* ------------------------------------------------------------------ */
/* Test Cases 1 through 9                                             */
/* ------------------------------------------------------------------ */

void test_case_1_normal_intersection(void) {
    printf("[Case 1] Normal 4-way intersection approach and crossing... ");
    sim_car_t car;
    sim_car_init(&car);

    /* Front camera: left crossbar x=[4,20], right crossbar x=[58,74], center gap [25,53] open.
       Forward exit pair at x=26 and x=52.
    */
    uint16_t front_vectors[16] = {
        4, 25, 20, 25,     /* Left crossbar: dx=16, dy=0, aspect ratio ok, min_x=4, max_x=20 (< 25) */
        58, 25, 74, 25,    /* Right crossbar: dx=16, dy=0, aspect ratio ok, min_x=58, max_x=74 (> 53) */
        26, 22, 26, 6,     /* Exit left line: dx=0, dy=16, heading=0 deg, x_mid=26 in [15,36], y_far=6 <= 25 */
        52, 22, 52, 6      /* Exit right line: dx=0, dy=16, heading=0 deg, x_mid=52 in [42,65], sep=26 */
    };
    horizon_result_t hz;
    compute_front_camera_horizon(front_vectors, 4, &hz);

    assert(hz.has_transverse_tape == 1U);
    assert(hz.transverse_crosses_center == 0U);
    assert(hz.is_intersection_ahead == 1U);
    assert(hz.is_sharp_corner == 0U);
    assert(fabsf(hz.horizon_angle_deg) <= 20.0f); /* Excluded cross-tape from avg_angle */

    /* Top camera approach: normal lane lines */
    uint16_t top_approach[8] = {
        14, 50, 14, 16,
        64, 50, 64, 16
    };
    steering_result_t sr;
    compute_top_camera_lane(top_approach, 2, &sr);
    assert(sr.found == 2);
    assert(sr.has_transverse_tape == 0U);

    sim_control_tick(&car, &hz, &sr);
    assert(car.s_intersection_ahead == 1);
    assert(car.target_speed == SPEED_STRAIGHT);
    assert(car.target_left == SPEED_STRAIGHT && car.target_right == SPEED_STRAIGHT);

    /* Transit through the blind gap (sr.found == 0) for 10 ticks */
    steering_result_t sr_empty;
    memset(&sr_empty, 0, sizeof(sr_empty));
    horizon_result_t hz_empty;
    memset(&hz_empty, 0, sizeof(hz_empty));

    for (int t = 0; t < 10; t++) {
        sim_control_tick(&car, &hz_empty, &sr_empty);
        assert(car.s_intersection_ahead == 1);
        assert(car.s_was_in_intersection_gap == 1);
        assert(fabsf(car.steer_cmd) <= 3.0f); /* Held steer clamped to +-3.0 deg */
        assert(car.target_speed == SPEED_STRAIGHT); /* Full speed, zero pivot drag */
        assert(car.target_left == SPEED_STRAIGHT);
        assert(car.target_right == SPEED_STRAIGHT);
    }

    /* Exit the blind gap: top camera reacquires far lines */
    uint16_t top_exit[8] = {
        14, 40, 14, 16,
        64, 40, 64, 16
    };
    compute_top_camera_lane(top_exit, 2, &sr);
    assert(sr.found == 2);

    sim_control_tick(&car, &hz_empty, &sr);
    /* Re-acquisition slew limiter active */
    assert(car.s_reacquire_ticks == 2); /* was 3, decremented by 1 during this tick */
    assert(car.s_intersection_ahead == 0);
    assert(car.s_was_in_intersection_gap == 0);

    printf("PASS\n");
}

void test_case_2_yaw_and_offcenter(void) {
    printf("[Case 2] Intersection approached at 10 deg yaw and 5 cm off-center... ");
    sim_car_t car;
    sim_car_init(&car);

    /* 10 deg yaw: tan(10) = 0.176. Aspect ratio dy <= 0.27 * dx holds (0.176 <= 0.27).
       Off-center by +6px: left crossbar [2, 19], right crossbar [58, 75].
       Exit pair with 10 deg heading:
       Left: (22, 24) to (25, 7) -> dx=3, dy=17 -> heading = 10.0 deg. x_mid = 23.5 in [15,36].
       Right: (52, 24) to (55, 7) -> dx=3, dy=17 -> heading = 10.0 deg. x_mid = 53.5 in [42,65].
       Separation = 30px in [25, 45].
       Crossbars:
       Left: (2, 27) to (19, 24) -> dx=17, dy=3 -> dy/dx = 0.176 <= 0.27. max_x = 19 < 25.
       Right: (58, 24) to (75, 21) -> dx=17, dy=3 -> dy/dx = 0.176 <= 0.27. min_x = 58 > 53.
    */
    uint16_t front_vectors[16] = {
        2, 27, 19, 24,    /* Left crossbar at 10 deg yaw */
        58, 24, 75, 21,   /* Right crossbar at 10 deg yaw */
        22, 24, 25, 7,    /* Left exit line with 10 deg heading */
        52, 24, 55, 7     /* Right exit line with 10 deg heading */
    };
    horizon_result_t hz;
    compute_front_camera_horizon(front_vectors, 4, &hz);

    assert(hz.has_transverse_tape == 1U);
    assert(hz.transverse_crosses_center == 0U);
    assert(hz.is_intersection_ahead == 1U);
    assert(hz.is_sharp_corner == 0U);
    assert(fabsf(hz.horizon_angle_deg) <= 20.0f);

    steering_result_t sr;
    memset(&sr, 0, sizeof(sr));
    sr.found = 2;
    sr.cte = 6.0f;
    sr.heading_deg = 10.0f;

    sim_control_tick(&car, &hz, &sr);
    assert(car.s_intersection_ahead == 1);
    assert(car.target_speed == SPEED_STRAIGHT);

    printf("PASS\n");
}

void test_case_3_pixy_merged_chord(void) {
    printf("[Case 3] Pixy merged chord across center (gap test fail -> corner pre-brake)... ");
    sim_car_t car;
    sim_car_init(&car);

    /* Merged chord vector across center: x from 15 to 65 at y=25 */
    uint16_t front_vectors[8] = {
        15, 25, 65, 25,   /* Single merged chord: dx=50, dy=0 -> spans [15,65], enters [25,53] */
        26, 22, 26, 6     /* Longitudinal vector */
    };
    horizon_result_t hz;
    compute_front_camera_horizon(front_vectors, 2, &hz);

    assert(hz.has_transverse_tape == 1U);
    assert(hz.transverse_crosses_center == 1U); /* Violates center gap! */
    assert(hz.is_intersection_ahead == 0U);     /* MUST NOT latch intersection! */
    assert(hz.is_sharp_corner == 1U);           /* Fail-safe default to corner! */

    steering_result_t sr;
    memset(&sr, 0, sizeof(sr));
    sr.found = 2;

    sim_control_tick(&car, &hz, &sr);
    assert(car.s_intersection_ahead == 0);
    assert(car.target_speed == SPEED_OUTSIDE_CORNER); /* Triggers pre-brake! */

    printf("PASS\n");
}

void test_case_4_sharp_corner_outer_wall(void) {
    printf("[Case 4] Sharp 90 deg corner apex (outer wall crossing center -> HOLD steer, zero decay)... ");
    sim_car_t car;
    sim_car_init(&car);

    /* Initial state: car is in middle of corner, holding steer_cmd = 32.0 deg */
    car.steer_cmd      = 32.0f;
    car.last_steer_cmd = 32.0f;
    car.filtered_cte   = 15.0f;
    car.filtered_heading = 20.0f;

    /* Top camera sees transverse outer wall crossing center: [20, 60] */
    uint16_t top_vectors[4] = {
        20, 30, 60, 30
    };
    steering_result_t sr;
    compute_top_camera_lane(top_vectors, 1, &sr);

    assert(sr.has_transverse_tape == 1U);
    assert(sr.transverse_crosses_center == 1U);
    assert(sr.found == 0); /* Discarded from left/right line selection! */

    horizon_result_t hz;
    memset(&hz, 0, sizeof(hz));
    hz.is_sharp_corner = 1U; /* Corner ahead */

    /* Run control tick with sr.found == 0 */
    float cte_before = car.filtered_cte;
    float head_before = car.filtered_heading;

    sim_control_tick(&car, &hz, &sr);

    assert(car.s_intersection_ahead == 0);
    /* MUST HOLD LAST STEER! Zero decay, zero +-3 deg clamp! */
    assert(car.steer_cmd == 32.0f);
    assert(car.filtered_cte == cte_before); /* Zero decay */
    assert(car.filtered_heading == head_before); /* Zero decay */

    printf("PASS\n");
}

void test_case_5_hairpin_return_pair(void) {
    printf("[Case 5] Hairpin with outer wall PLUS parallel return pair beyond it... ");
    sim_car_t car;
    sim_car_init(&car);

    /* Transverse outer wall crossing center [18, 62] at y=32.
       Return track beyond it: left line at x=25, right line at x=52 at y=[20, 5].
    */
    uint16_t front_vectors[12] = {
        18, 32, 62, 32,   /* Outer wall crossing center: dx=44, dy=0, min_x=18, max_x=62 */
        25, 20, 25, 5,    /* Left return line: dx=0, dy=15, heading=0 */
        52, 20, 52, 5     /* Right return line: dx=0, dy=15, heading=0 */
    };
    horizon_result_t hz;
    compute_front_camera_horizon(front_vectors, 3, &hz);

    assert(hz.has_transverse_tape == 1U);
    assert(hz.transverse_crosses_center == 1U); /* Crosses center! */
    assert(hz.is_intersection_ahead == 0U);     /* MUST reject intersection! */
    assert(hz.is_sharp_corner == 1U);           /* MUST pre-brake! */

    steering_result_t sr;
    memset(&sr, 0, sizeof(sr));
    sr.found = 2;

    sim_control_tick(&car, &hz, &sr);
    assert(car.s_intersection_ahead == 0);
    assert(car.target_speed == SPEED_OUTSIDE_CORNER); /* Confirmed pre-brake */

    printf("PASS\n");
}

void test_case_6_corner_priority_preemption(void) {
    printf("[Case 6] Corner immediately following intersection (priority preemption)... ");
    sim_car_t car;
    sim_car_init(&car);

    /* Tick 1: Intersection latch active */
    car.s_intersection_ahead = 1;
    car.s_intersection_timer = 12;
    car.s_intersection_gap_ticks = 4;
    car.s_was_in_intersection_gap = 1;

    /* Tick 2: Front camera detects sharp corner */
    horizon_result_t hz;
    memset(&hz, 0, sizeof(hz));
    hz.is_sharp_corner = 1U;

    steering_result_t sr;
    memset(&sr, 0, sizeof(sr));
    sr.found = 2;

    sim_control_tick(&car, &hz, &sr);

    /* Priority preemption: latch and gap countdown immediately killed! */
    assert(car.s_intersection_ahead == 0);
    assert(car.s_intersection_timer == 0);
    assert(car.s_intersection_gap_ticks == 0);
    assert(car.s_was_in_intersection_gap == 0);
    assert(car.target_speed == SPEED_OUTSIDE_CORNER);

    printf("PASS\n");
}

void test_case_7_straightaway_dropout(void) {
    printf("[Case 7] Straightaway with glare/line dropout (zero false latch)... ");
    sim_car_t car;
    sim_car_init(&car);

    /* Front camera sees no vectors (glare dropout), top camera sees dropout */
    horizon_result_t hz;
    memset(&hz, 0, sizeof(hz));

    steering_result_t sr;
    memset(&sr, 0, sizeof(sr));
    sr.found = 0;

    for (int t = 0; t < 5; t++) {
        sim_control_tick(&car, &hz, &sr);
        assert(car.s_intersection_ahead == 0);
        assert(car.s_was_in_intersection_gap == 0);
        assert(car.target_speed == SPEED_STRAIGHT); /* Stays in straight mode */
    }

    printf("PASS\n");
}

void test_case_8_corner_apex_dropout(void) {
    printf("[Case 8] Corner apex dropout with sr.found == 0 (must HOLD steer)... ");
    sim_car_t car;
    sim_car_init(&car);

    car.last_steer_cmd = -28.5f;
    car.steer_cmd      = -28.5f;
    car.s_intersection_ahead = 0;

    horizon_result_t hz;
    memset(&hz, 0, sizeof(hz));
    hz.is_sharp_corner = 1U;

    steering_result_t sr;
    memset(&sr, 0, sizeof(sr));
    sr.found = 0; /* Optical dropout in corner apex */

    sim_control_tick(&car, &hz, &sr);

    /* MUST HOLD LAST STEER! */
    assert(car.steer_cmd == -28.5f);
    assert(car.servo_steer == -28.5f + STEERING_OFFSET);

    printf("PASS\n");
}

void test_case_9_distance_window_scaling(void) {
    printf("[Case 9] High speed vs low speed distance window scaling... ");

    /* High speed = 100.0f */
    float spd_high = 100.0f;
    uint32_t gap_ticks_high = (uint32_t)(1.5f * 25.0f / fmaxf(spd_high, 30.0f) * 50.0f);

    /* Low speed = 30.0f */
    float spd_low = 30.0f;
    uint32_t gap_ticks_low = (uint32_t)(1.5f * 25.0f / fmaxf(spd_low, 30.0f) * 50.0f);

    /* Mid speed = 50.0f */
    float spd_mid = 50.0f;
    uint32_t gap_ticks_mid = (uint32_t)(1.5f * 25.0f / fmaxf(spd_mid, 30.0f) * 50.0f);

    assert(gap_ticks_high == 18);
    assert(gap_ticks_mid == 37);
    assert(gap_ticks_low == 62);
    assert(gap_ticks_low > gap_ticks_mid && gap_ticks_mid > gap_ticks_high);

    printf("PASS (High=%u ticks [360ms], Mid=%u ticks [740ms], Low=%u ticks [1240ms])\n",
           gap_ticks_high, gap_ticks_mid, gap_ticks_low);
}

void test_case_10_dead_end_approach_lines_only(void) {
    printf("[Case 10] Dead-end with transverse tape and approach lines ONLY (no exit lines)... ");
    sim_car_t car;
    sim_car_init(&car);

    /* Transverse crossbars at y=25 with center gap open [25, 53].
       Approach lines leading up to the transverse tape: (26, 40) to (26, 27) and (52, 40) to (52, 27).
       No lines beyond y=25.
       Must NOT be detected as an intersection! Must safely default to corner pre-braking!
    */
    uint16_t front_vectors[16] = {
        4, 25, 20, 25,     /* Left crossbar: dx=16, dy=0 */
        58, 25, 74, 25,    /* Right crossbar: dx=16, dy=0 */
        26, 40, 26, 27,    /* Left approach line: before crossbar */
        52, 40, 52, 27     /* Right approach line: before crossbar */
    };

    horizon_result_t hz;
    compute_front_camera_horizon(front_vectors, 4, &hz);

    assert(hz.has_transverse_tape == 1U);
    assert(hz.transverse_crosses_center == 0U);
    assert(hz.is_intersection_ahead == 0U); /* Must reject intersection! */
    assert(hz.is_sharp_corner == 1U);       /* Must fail-safe pre-brake! */

    steering_result_t sr;
    memset(&sr, 0, sizeof(sr));
    sr.found = 2;

    sim_control_tick(&car, &hz, &sr);
    assert(car.s_intersection_ahead == 0);
    assert(car.target_speed == SPEED_OUTSIDE_CORNER); /* Triggers pre-brake! */

    printf("PASS\n");
}

int main(void) {
    printf("=====================================================\n");
    printf("  RUNNING ADVERSARIAL INTERSECTION & CORNER TEST SUITE\n");
    printf("=====================================================\n");

    test_case_1_normal_intersection();
    test_case_2_yaw_and_offcenter();
    test_case_3_pixy_merged_chord();
    test_case_4_sharp_corner_outer_wall();
    test_case_5_hairpin_return_pair();
    test_case_6_corner_priority_preemption();
    test_case_7_straightaway_dropout();
    test_case_8_corner_apex_dropout();
    test_case_9_distance_window_scaling();
    test_case_10_dead_end_approach_lines_only();

    printf("=====================================================\n");
    printf("  ALL 10 ADVERSARIAL TEST CASES PASSED SUCCESSFULLY!\n");
    printf("=====================================================\n");

    return 0;
}
