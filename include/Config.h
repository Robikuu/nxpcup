#ifndef CONFIG_H
#define CONFIG_H

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

#define WHEELBASE_MM        150.0  // distance between front and rear axle — MEASURE this on your chassis
#define TRACK_WIDTH_MM      120.0  // distance between left and right wheels — MEASURE this
#define MAX_SERVO_ANGLE_DEG  25.0  // real mechanical angle at Steer(±100) — CHECK/CALIBRATE this
#define MAX_LATERAL_ACCEL     3.0  // tuning knob: how hard you let the car corner before it risks sliding
#define BASE_SPEED           60.0

#endif
