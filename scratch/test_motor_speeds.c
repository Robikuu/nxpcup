#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "hbridge.h"
#include "wifi_config.h"

int main(void) {
    params_init();
    motor_speeds_reset();

    float sl, sr;
    // Test 1: Straightaway (steer=0, heading=0) -> accelerates toward max_speed (80%)
    compute_motor_speeds(0.0f, 0.0f, 2, 0.05f, &sl, &sr);
    assert(sl > 0.0f && sr > 0.0f);
    assert(fabsf(sl - sr) < 1e-3f); // Equal wheel speeds

    // Accelerate to top speed over multiple cycles
    for (int i = 0; i < 50; i++) {
        compute_motor_speeds(0.0f, 0.0f, 2, 0.02f, &sl, &sr);
    }
    assert(motor_speeds_get_current() > 75.0f);
    printf("Test 1 (Straightaway Acceleration) PASSED\n");

    // Test 2: Preview sharp corner heading (slope = 0.40) -> decelerates toward base_speed (38%)
    for (int i = 0; i < 20; i++) {
        compute_motor_speeds(0.0f, 0.40f, 2, 0.02f, &sl, &sr);
    }
    assert(motor_speeds_get_current() < 50.0f);
    printf("Test 2 (Trail-Braking Deceleration) PASSED\n");

    // Test 3: Ackermann differential drive split in sharp right turn (steer = +60.0f)
    compute_motor_speeds(60.0f, 0.30f, 2, 0.02f, &sl, &sr);
    assert(sl > sr); // Left wheel outside (faster), right wheel inside (slower)
    printf("Test 3 (Ackermann Differential Split) PASSED\n");

    printf("All Motor Control Tests PASSED!\n");
    return 0;
}
