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

#define SPEED_AT_FULL_DUTY_MMPS  2000.0f
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
    float kp;                // PID proportional gain
    float ki;                // PID integral gain
    float kd;                // PID derivative gain
    float alpha;             // Derivative LPF smoothing factor (0.0 < alpha <= 1.0)

    float base_speed_left;   // Nominal left wheel speed (asymmetric tuning)
    float base_speed_right;  // Nominal right wheel speed (asymmetric tuning)
    float max_speed_left;    // Left motor speed clamp
    float max_speed_right;  // Right motor speed clamp

    float steering_offset;   // Servo center offset
    float k_cte;             // Cross-track error weight
    float k_heading;         // Heading angle error weight
    float dt;                // Loop timestep (seconds)

    uint8_t state;            // STATE_IDLE or STATE_RUNNING
} robot_params_t;

extern robot_params_t g_params;

void params_init(void);

#endif
