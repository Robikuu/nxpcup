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
        /* SHARP CORNER MODE: Drop speed to corner speed + active differential pivot ONLY if steering past deadzone */
        car->target_speed = SPEED_OUTSIDE_CORNER;
        if (fabsf(car->steer_cmd) > (float)DIFFERENTIAL_DEADZONE_DEG) {
            if (car->steer_cmd > 0.0f) {
                car->target_left  = SPEED_OUTSIDE_CORNER;
                car->target_right = SPEED_INSIDE_MIN;
            } else {
                car->target_left  = SPEED_INSIDE_MIN;
                car->target_right = SPEED_OUTSIDE_CORNER;
            }
        } else {
            /* Wheels are straight (|steer| <= 10 deg): equal duty, zero yaw torque induced! */
            car->target_left  = car->target_speed;
            car->target_right = car->target_speed;
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

void test_case_11_top_camera_chord_and_chassis_rejection(void) {
    printf("[Case 11] Top camera chassis exclusion & diagonal chord rejection... ");

    /* 1. Chassis exclusion: tie rod / bumper reflection (40, 51, 40, 45) */
    uint16_t bumper_vec[4] = { 40, 51, 40, 45 }; /* x=40 in [36,44], y=51 >= 45, dy=6 <= 8 */
    steering_result_t sr_chassis;
    compute_top_camera_lane(bumper_vec, 1, &sr_chassis);
    assert(sr_chassis.found == 0); /* Bumper reflection successfully excluded! */

    /* 2. Diagonal chord rejection:
       Genuine line: (71, 51, 69, 37) -> dy=14 >= 12, heading = -8.1 deg.
       Diagonal chord: (69, 37, 78, 33) -> dx=9, dy=4, |dx| >= dy, dy < 10, angle = +66 deg.
       Left genuine line: (14, 51, 14, 30) -> dy=21.
    */
    uint16_t chord_test_vecs[12] = {
        14, 51, 14, 30,    /* Genuine left line */
        71, 51, 69, 37,    /* Genuine right line */
        69, 37, 78, 33     /* Pixy corner-merged diagonal chord */
    };
    steering_result_t sr_chord;
    compute_top_camera_lane(chord_test_vecs, 3, &sr_chord);

    assert(sr_chord.found == 2);
    assert(sr_chord.left_found == 1U);
    assert(sr_chord.right_found == 1U);
    /* Right line MUST be selected from genuine line (x=71), NOT diagonal chord (x=69)! */
    /* And heading must NOT be contaminated by the +66 deg chord */
    assert(sr_chord.heading_deg < 10.0f); /* Genuine average heading is ~ -4 deg, NOT +30 deg */

    /* 3. Chord with |angle| > 55 deg and dy < 12 */
    uint16_t steep_chord_vec[4] = { 65, 30, 78, 24 }; /* dx=13, dy=6, angle=65.2 deg, dy=6 < 12 */
    steering_result_t sr_steep;
    compute_top_camera_lane(steep_chord_vec, 1, &sr_steep);
    assert(sr_steep.found == 0); /* Steep diagonal chord rejected! */

    printf("PASS\n");
}

void test_case_12_vanishing_point_perspective_convergence(void) {
    printf("[Case 12] Real track data vanishing-point perspective convergence... ");

    /* Frame 123 from scratch/real_intersection_dump.txt:
       Left crossbar: (0, 29, 16, 26) -> dx=16, dy=3, dy <= 0.27*dx
       Right crossbar: (62, 24, 78, 24) -> dx=16, dy=0
       Left exit line: (9, 27, 18, 6) -> dx=9, dy=21
       Right exit line: (51, 22, 35, 2) -> dx=-16, dy=20
    */
    uint16_t real_frn_vectors[16] = {
        0, 29, 16, 26,     /* Left crossbar: min_x=0, max_x=16 (< 35) */
        62, 24, 78, 24,    /* Right crossbar: min_x=62, max_x=78 (> 43) */
        9, 27, 18, 6,      /* Converging left exit line: heading = +23.2 deg */
        51, 22, 35, 2      /* Converging right exit line: heading = -38.7 deg */
    };

    horizon_result_t hz;
    compute_front_camera_horizon(real_frn_vectors, 4, &hz);

    assert(hz.has_transverse_tape == 1U);
    assert(hz.transverse_crosses_center == 0U);
    assert(hz.is_intersection_ahead == 1U);
    assert(hz.is_sharp_corner == 0U);

    printf("PASS\n");
}

void test_case_13_diverging_exit_lines_rejected(void) {
    printf("[Case 13] Diverging exit lines rejected by vanishing point test... ");

    /* Exit lines diverging upward:
       Left line: (20, 22) to (15, 6) -> dx=-5, dy=16
       Right line: (50, 22) to (55, 6) -> dx=+5, dy=16
       Intersection is downwards (Y_v > 5.0) -> must be rejected!
    */
    uint16_t div_vectors[16] = {
        4, 25, 20, 25,     /* Left crossbar */
        58, 25, 74, 25,    /* Right crossbar */
        20, 22, 15, 6,     /* Diverging left line */
        50, 22, 55, 6      /* Diverging right line */
    };

    horizon_result_t hz;
    compute_front_camera_horizon(div_vectors, 4, &hz);

    assert(hz.has_transverse_tape == 1U);
    assert(hz.transverse_crosses_center == 0U);
    assert(hz.is_intersection_ahead == 0U); /* MUST REJECT diverging exit pair! */
    assert(hz.is_sharp_corner == 1U);       /* Fail-safe pre-brake! */

    printf("PASS\n");
}

void test_case_14_vanishing_point_outside_corridor_rejected(void) {
    printf("[Case 14] Exit lines converging outside track corridor [15, 60] rejected... ");

    /* Lines converging far to the right (X_v > 60) */
    uint16_t off_vp_vectors[16] = {
        4, 25, 20, 25,     /* Left crossbar */
        58, 25, 74, 25,    /* Right crossbar */
        25, 22, 45, 6,     /* Left line: dx=20, dy=16 */
        52, 22, 60, 6      /* Right line: dx=8, dy=16 */
    };

    horizon_result_t hz;
    compute_front_camera_horizon(off_vp_vectors, 4, &hz);

    assert(hz.has_transverse_tape == 1U);
    assert(hz.transverse_crosses_center == 0U);
    assert(hz.is_intersection_ahead == 0U); /* Vanishing point outside corridor -> rejected! */
    assert(hz.is_sharp_corner == 1U);

    printf("PASS\n");
}

void test_case_15_center_gap_straddle_vs_non_straddle(void) {
    printf("[Case 15] Center gap straddle condition & segmented union... ");

    /* 1. Left crossbar only reaching x=34 (min_x=5, max_x=34): does NOT straddle [35, 43] */
    uint16_t left_bar[8] = {
        5, 25, 34, 25,
        50, 22, 50, 6
    };
    horizon_result_t hz1;
    compute_front_camera_horizon(left_bar, 2, &hz1);
    assert(hz1.has_transverse_tape == 1U);
    assert(hz1.transverse_crosses_center == 0U); /* Does NOT straddle! */

    /* 2. Right crossbar only reaching x=44 (min_x=44, max_x=75): does NOT straddle [35, 43] */
    uint16_t right_bar[8] = {
        44, 25, 75, 25,
        50, 22, 50, 6
    };
    horizon_result_t hz2;
    compute_front_camera_horizon(right_bar, 2, &hz2);
    assert(hz2.has_transverse_tape == 1U);
    assert(hz2.transverse_crosses_center == 0U); /* Does NOT straddle! */

    /* 3. Broken / segmented wall union: segment 1 [15, 38], segment 2 [36, 62] at y=25 */
    uint16_t broken_wall[8] = {
        15, 25, 38, 25,   /* min=15, max=38: does not individually straddle [35, 43] */
        36, 25, 62, 25    /* min=36, max=62: does not individually straddle [35, 43] */
    };
    horizon_result_t hz3;
    compute_front_camera_horizon(broken_wall, 2, &hz3);
    assert(hz3.has_transverse_tape == 1U);
    assert(hz3.transverse_crosses_center == 1U); /* Union spans [15, 62], crosses center! */

    printf("PASS\n");
}

void test_case_16_differential_pivot_drag_decoupling(void) {
    printf("[Case 16] Decoupled differential pivot drag in sharp corner mode... ");
    sim_car_t car;
    sim_car_init(&car);

    horizon_result_t hz_corner;
    memset(&hz_corner, 0, sizeof(hz_corner));
    hz_corner.is_sharp_corner = 1U;

    steering_result_t sr_straight;
    memset(&sr_straight, 0, sizeof(sr_straight));
    sr_straight.found = 2;
    sr_straight.cte = 0.0f;
    sr_straight.heading_deg = 0.0f;

    /* A. Wheels straight (|steer| = 0.0 <= 10 deg deadzone):
       Must reduce speed to SPEED_OUTSIDE_CORNER, but left and right MUST BE EQUAL!
    */
    car.filtered_cte = 0.0f;
    car.filtered_heading = 0.0f;
    sr_straight.cte = 0.0f;
    sr_straight.heading_deg = 0.0f;
    sim_control_tick(&car, &hz_corner, &sr_straight);
    assert(car.target_speed == SPEED_OUTSIDE_CORNER);
    assert(car.target_left == SPEED_OUTSIDE_CORNER);
    assert(car.target_right == SPEED_OUTSIDE_CORNER); /* ZERO yaw torque! */

    /* B. Wheels mild steer (|steer| ~ 6.0 <= 10 deg deadzone):
       Must still have ZERO differential pivot drag!
    */
    car.filtered_heading = 5.0f;
    sr_straight.heading_deg = 5.0f;
    sim_control_tick(&car, &hz_corner, &sr_straight);
    assert(fabsf(car.steer_cmd) <= (float)DIFFERENTIAL_DEADZONE_DEG);
    assert(car.target_left == SPEED_OUTSIDE_CORNER);
    assert(car.target_right == SPEED_OUTSIDE_CORNER); /* ZERO yaw torque! */

    /* C. Wheels sharp steer right (steer > 10 deg deadzone):
       Active differential pivot drag kicks in!
    */
    steering_result_t sr_right;
    memset(&sr_right, 0, sizeof(sr_right));
    sr_right.found = 2;
    sr_right.heading_deg = 25.0f;
    car.filtered_heading = 25.0f;
    sim_control_tick(&car, &hz_corner, &sr_right);
    assert(car.steer_cmd > (float)DIFFERENTIAL_DEADZONE_DEG);
    assert(car.target_left == SPEED_OUTSIDE_CORNER);
    assert(car.target_right == SPEED_INSIDE_MIN); /* Inside wheel braked to 25%! */

    /* D. Wheels sharp steer left (steer < -10 deg deadzone):
       Active differential pivot drag on left wheel!
    */
    steering_result_t sr_left;
    memset(&sr_left, 0, sizeof(sr_left));
    sr_left.found = 2;
    sr_left.heading_deg = -25.0f;
    car.filtered_heading = -25.0f;
    sim_control_tick(&car, &hz_corner, &sr_left);
    assert(car.steer_cmd < -(float)DIFFERENTIAL_DEADZONE_DEG);
    assert(car.target_left == SPEED_INSIDE_MIN); /* Inside wheel braked to 25%! */
    assert(car.target_right == SPEED_OUTSIDE_CORNER);

    printf("PASS\n");
}

void test_case_17_near_parallel_sensor_jitter_convergence(void) {
    printf("[Case 17] Near-parallel exit lines with 1-pixel sensor jitter (dy=17 vs dy=18)... ");

    /* Realistic Pixy sensor quantization:
       Left exit line: (22, 24) to (25, 7) -> dx=3, dy=17 (angle = 10.0 deg)
       Right exit line: (52, 24) to (55, 6) -> dx=3, dy=18 (angle = 9.46 deg)
       Notice: 1-pixel difference in length! denom = s_l - s_r = 0.009804 > 1e-3.
       In unrefined vanishing point math, y_v = -3036, x_v = +562 -> rejected!
       Refined parallel corridor test recognizes this valid straight exit pair!
    */
    uint16_t jitter_vectors[16] = {
        2, 27, 19, 24,    /* Left crossbar at 10 deg yaw */
        58, 24, 75, 21,   /* Right crossbar at 10 deg yaw */
        22, 24, 25, 7,    /* Left exit line: dy=17 */
        52, 24, 55, 6     /* Right exit line: dy=18 (1-px length jitter) */
    };

    horizon_result_t hz;
    compute_front_camera_horizon(jitter_vectors, 4, &hz);

    assert(hz.has_transverse_tape == 1U);
    assert(hz.transverse_crosses_center == 0U);
    assert(hz.is_intersection_ahead == 1U); /* MUST detect intersection despite 1-px jitter! */
    assert(hz.is_sharp_corner == 0U);

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
    test_case_11_top_camera_chord_and_chassis_rejection();
    test_case_12_vanishing_point_perspective_convergence();
    test_case_13_diverging_exit_lines_rejected();
    test_case_14_vanishing_point_outside_corridor_rejected();
    test_case_15_center_gap_straddle_vs_non_straddle();
    test_case_16_differential_pivot_drag_decoupling();
    test_case_17_near_parallel_sensor_jitter_convergence();

    printf("=====================================================\n");
    printf("  ALL 17 ADVERSARIAL TEST CASES PASSED SUCCESSFULLY!\n");
    printf("=====================================================\n");

    return 0;
}
