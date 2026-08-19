#ifndef CONTROL_THEORY_H
#define CONTROL_THEORY_H

#include <stdint.h> 
typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_measurement;
    float d_filtered;      // Derivative low-pass filter state

    float output_limit;
    float alpha;           // Filter coefficient (0.0 < alpha <= 1.0)

    float prev_output;     // Last output, returned when dt <= 0
    uint8_t initialized;   // Guards first-call derivative spike
} pid_struct;

/**
 * @brief Initializes the PID controller struct.
 * @param PID Pointer to the PID struct
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 * @param output_limit Max magnitude for the control output
 * @param alpha Derivative LPF smoothing factor (typically 0.05 to 0.3)
 */
void  pid_init(pid_struct *PID, float kp, float ki, float kd,
               float output_limit, float alpha);

/**
 * @brief Computes the PID output step.
 * @param PID Pointer to the PID struct
 * @param setpoint Target value (e.g., center line position or target speed)
 * @param measurement Current sensor feedback
 * @param dt Time step in seconds
 * @return Clamped control output
 */
float pid_update(pid_struct *PID, float setpoint, float measurement, float dt);

/**
 * @brief Computes adaptive steering gain based on speed, CTE, and heading error.
 * @param current_speed Current vehicle speed
 * @param base_speed Base speed threshold below which gain is 1.0
 * @param max_speed Maximum speed threshold
 * @param min_scale Minimum attenuation scale at maximum speed on straight
 * @param abs_cte Absolute cross-track error
 * @param abs_heading Absolute heading error
 * @return Adaptive steer scale factor (between min_scale and 1.0)
 */
float compute_speed_steer_gain_adaptive(float current_speed, float base_speed, float max_speed,
                                       float min_scale, float abs_cte, float abs_heading);

#endif // CONTROL_THEORY_H