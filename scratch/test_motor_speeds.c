#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "hbridge.h"
#include "wifi_config.h"

int main(void) {
    params_init();
    motor_speeds_reset();

    float sl, sr;

    /* Test 1: Soft start ramp from 20% without torque spikes */
    // Cycle 1: Launch start speed must be 20.0f
    compute_motor_speeds(0.0f, 0.0f, 2, 0.02f, &sl, &sr);
    assert(sl >= 20.0f && sr >= 20.0f);
    assert(fabsf(sl - sr) < 1e-3f);
    assert(sl <= 20.0f + LAUNCH_RAMP_RATE * 0.02f + 1e-3f);

    // Ramp through the 1.5-second launch window (75 cycles of 0.02s = 1.50s)
    for (int i = 1; i < 75; i++) {
        float prev_spd = motor_speeds_get_current();
        compute_motor_speeds(0.0f, 0.0f, 2, 0.02f, &sl, &sr);
        float curr_spd = motor_speeds_get_current();
        float accel_rate = (curr_spd - prev_spd) / 0.02f;

        // Verify acceleration does not exceed MAX_ACCEL_PER_SEC (no torque spike)
        assert(accel_rate >= 0.0f);
        assert(accel_rate <= MAX_ACCEL_PER_SEC + 1e-3f);
        // Verify launch ramp rate is followed
        assert(accel_rate <= LAUNCH_RAMP_RATE + 1e-3f);
    }

    // Additional cycles past 1.5s to reach top speed clamp (75.0%)
    for (int i = 0; i < 20; i++) {
        compute_motor_speeds(0.0f, 0.0f, 2, 0.02f, &sl, &sr);
    }
    assert(fabsf(motor_speeds_get_current() - 75.0f) < 1e-2f);
    assert(fabsf(sl - 75.0f) < 1e-2f && fabsf(sr - 75.0f) < 1e-2f);
    printf("Test 1 (Soft Start Ramp & Anti-Wheelspin Slew Rate) PASSED\n");

    /* Test 2: Preview sharp corner heading (slope = 0.40) -> decelerates toward base_speed */
    for (int i = 0; i < 20; i++) {
        float prev_spd = motor_speeds_get_current();
        compute_motor_speeds(0.0f, 0.40f, 2, 0.02f, &sl, &sr);
        float curr_spd = motor_speeds_get_current();
        float brake_rate = (prev_spd - curr_spd) / 0.02f;

        // Verify brake rate is bounded by MAX_BRAKE_PER_SEC
        assert(brake_rate <= MAX_BRAKE_PER_SEC + 1e-3f);
    }
    // With slope=0.40, turn_intensity=0.80 -> target = 75 - 0.80*(75-38) = 45.4%
    assert(fabsf(motor_speeds_get_current() - 45.4f) < 0.1f);
    assert(motor_speeds_get_current() < 50.0f);

    // Full sharp turn preview (slope >= 0.50) -> decelerates fully to base_speed (38.0%)
    for (int i = 0; i < 20; i++) {
        compute_motor_speeds(0.0f, 0.60f, 2, 0.02f, &sl, &sr);
    }
    assert(fabsf(motor_speeds_get_current() - 38.0f) < 0.1f);
    printf("Test 2 (Heading Slope Curvature Deceleration) PASSED\n");

    /* Test 3: Ackermann differential drive split */
    // Sharp right turn (steer = +60.0f): Left wheel outside (faster), right wheel inside (slower)
    compute_motor_speeds(60.0f, 0.30f, 2, 0.02f, &sl, &sr);
    assert(sl > sr);
    assert(sl <= g_params.max_speed_left);
    assert(sr <= g_params.max_speed_right);

    // Sharp left turn (steer = -60.0f): Right wheel outside (faster), left wheel inside (slower)
    compute_motor_speeds(-60.0f, -0.30f, 2, 0.02f, &sl, &sr);
    assert(sr > sl);
    assert(sl <= g_params.max_speed_left);
    assert(sr <= g_params.max_speed_right);

    printf("Test 3 (Ackermann Differential Split) PASSED\n");

    printf("All Motor Control Tests PASSED!\n");
    return 0;
}
