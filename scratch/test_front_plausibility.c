#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

typedef struct {
    float trajectory_angle_deg;
    float trajectory_offset_px;
} test_traj_t;

// State variables matching main.c design
static float s_last_front_angle = 0.0f;
static float s_last_front_offset = 0.0f;
static unsigned s_consecutive_rejections = 0;
static unsigned s_front_dropout_ticks = 0;
static float s_last_valid_delta_front = 0.0f;
static unsigned s_front_hold_ticks = 0;

typedef struct {
    bool front_plausible;
    float c_front;
    float delta_front;
} front_filter_result_t;

front_filter_result_t process_front_frame(test_traj_t g_traj, unsigned num_vectors_front, float filtered_traj_angle) {
    front_filter_result_t res;
    res.front_plausible = false;
    res.c_front = 0.0f;
    res.delta_front = 0.0f;

    const float K_p_front = 1.00f;
    const float K_offset_front = 0.35f;

    if (num_vectors_front > 0) {
        if (s_front_dropout_ticks > 0) {
            /* Immediate re-seed after camera recovery from 0 vectors */
            s_last_front_angle = g_traj.trajectory_angle_deg;
            s_last_front_offset = g_traj.trajectory_offset_px;
            s_consecutive_rejections = 0;
            s_front_dropout_ticks = 0;
            res.front_plausible = true;
        } else {
            float angle_jump = fabsf(g_traj.trajectory_angle_deg - s_last_front_angle);
            float offset_jump = fabsf(g_traj.trajectory_offset_px - s_last_front_offset);

            if (fabsf(g_traj.trajectory_angle_deg) <= 75.0f &&
                angle_jump <= 25.0f &&
                offset_jump <= 18.0f) {
                res.front_plausible = true;
                s_consecutive_rejections = 0;
                s_last_front_angle = g_traj.trajectory_angle_deg;
                s_last_front_offset = g_traj.trajectory_offset_px;
            } else {
                s_consecutive_rejections++;
                if (s_consecutive_rejections >= 3) {
                    /* Self-healing reseed: prevent permanent lockout in real sharp corners */
                    res.front_plausible = true;
                    s_consecutive_rejections = 0;
                    s_last_front_angle = g_traj.trajectory_angle_deg;
                    s_last_front_offset = g_traj.trajectory_offset_px;
                }
            }
        }

        if (res.front_plausible) {
            res.c_front = (num_vectors_front >= 2) ? 1.0f : 0.60f;
        }
    } else {
        s_front_dropout_ticks++;
    }

    if (res.front_plausible) {
        res.delta_front = -(K_p_front * filtered_traj_angle + K_offset_front * g_traj.trajectory_offset_px);
        s_last_valid_delta_front = res.delta_front;
        s_front_hold_ticks = 0;
    } else {
        /* Hold last valid front command for 3 ticks (60ms), then decay gently */
        s_front_hold_ticks++;
        if (s_front_hold_ticks <= 3) {
            res.delta_front = s_last_valid_delta_front;
        } else {
            res.delta_front = s_last_valid_delta_front * 0.95f;
            s_last_valid_delta_front = res.delta_front;
        }
    }

    return res;
}

int main(void) {
    printf("Running Front Plausibility & Steering Tests...\n");

    // Case 1: Initial frame on straightaway
    test_traj_t t1 = {0.0f, 0.0f};
    front_filter_result_t r1 = process_front_frame(t1, 2, 0.0f);
    assert(r1.front_plausible == true);
    assert(r1.c_front == 1.0f);
    printf("Test 1 (Straight): PASSED (FV=1, c_front=%.1f)\n", r1.c_front);

    // Case 2: Smooth turn progression (+15 deg, +8 px)
    test_traj_t t2 = {15.0f, 8.0f};
    front_filter_result_t r2 = process_front_frame(t2, 2, 15.0f);
    assert(r2.front_plausible == true);
    assert(r2.delta_front < -15.0f);
    printf("Test 2 (Smooth Turn): PASSED (FV=1, delta_front=%.1f)\n", r2.delta_front);

    // Case 3: Glitch Jump (+60 deg jump, offset jump 25px) -> Must be rejected (FV=0)
    test_traj_t t3 = {75.0f, 35.0f};
    front_filter_result_t r3 = process_front_frame(t3, 2, 60.0f);
    assert(r3.front_plausible == false);
    assert(r3.c_front == 0.0f);
    // Should hold last valid command
    assert(fabsf(r3.delta_front - r2.delta_front) < 0.01f);
    printf("Test 3 (Glitch Jump): PASSED (FV=0, hold last valid delta_front=%.1f)\n", r3.delta_front);

    // Case 4: Sustained sharp turn: 2nd rejection, then 3rd rejection triggers self-healing reseed
    front_filter_result_t r4 = process_front_frame(t3, 2, 60.0f);
    assert(r4.front_plausible == false); // rejection #2
    front_filter_result_t r5 = process_front_frame(t3, 2, 60.0f);
    assert(r5.front_plausible == true);  // rejection #3 -> AUTO-RESEED!
    printf("Test 4 (Self-Healing Reseed): PASSED (Frame 3 accepted, FV=1, re-seeded)\n");

    // Case 5: 0 vectors dropout, then recovery
    test_traj_t t_zero = {0.0f, 0.0f};
    process_front_frame(t_zero, 0, 0.0f);
    assert(s_front_dropout_ticks == 1);
    
    test_traj_t t_recov = {-30.0f, -12.0f};
    front_filter_result_t r_recov = process_front_frame(t_recov, 2, -30.0f);
    assert(r_recov.front_plausible == true);
    assert(s_front_dropout_ticks == 0);
    printf("Test 5 (Dropout Recovery Reseed): PASSED (FV=1, s_front_dropout_ticks reset to 0)\n");

    printf("\nALL FRONT PLAUSIBILITY TESTS PASSED!\n");
    return 0;
}
