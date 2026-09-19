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
 * @brief Clears PID state to prevent derivative kick on re-entry.
 * @param PID Pointer to the PID struct
 */
static inline void reset_pid_state(pid_struct *PID) {
    PID->integral = 0.0f;
    PID->d_filtered = 0.0f;
    PID->prev_measurement = 0.0f;
    PID->prev_output = 0.0f;
    PID->initialized = 0;
}

#endif // CONTROL_THEORY_H