#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// Steering control coefficients
#define STEERING_P_RIGHT      30U
#define STEERING_P_LEFT       50U

// Physical steering limits
#define STEERING_LIMIT_RIGHT  45
#define STEERING_LIMIT_LEFT  -60

// Steering angle offset
#define STEERING_OFFSET      -13

// Wheel speeds
#define SPEED_RIGHT           40
#define SPEED_LEFT            -40

#define WHEELBASE_MM        150.0
#define TRACK_WIDTH_MM      120.0
#define MAX_SERVO_ANGLE_DEG  25.0
#define MAX_LATERAL_ACCEL     3.0
#define BASE_SPEED           100.0

// ---- Robot states ----
#define STATE_IDLE     0
#define STATE_RUNNING  1

// ---- Runtime-tunable parameter struct ----
// Default values come from the #define constants above.
// Modified at runtime via Wi-Fi commands received through the ESP32 bridge.
typedef struct {
    double kp;                // PID proportional gain
    double ki;                // PID integral gain
    double kd;                // PID derivative gain
    double alpha;             // Derivative LPF smoothing factor (0.0 < alpha <= 1.0)

    double base_speed_left;   // Nominal left wheel speed (asymmetric tuning)
    double base_speed_right;  // Nominal right wheel speed (asymmetric tuning)
    double max_speed_left;    // Left motor speed clamp
    double max_speed_right;   // Right motor speed clamp

    double steering_offset;   // Servo center offset
    double k_cte;             // Cross-track error weight
    double k_heading;         // Heading angle error weight
    double dt;                // Loop timestep (seconds)

    uint8_t state;            // STATE_IDLE or STATE_RUNNING
} robot_params_t;

extern robot_params_t g_params;

void params_init(void);

#endif
