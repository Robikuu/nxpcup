#ifndef CONTROL_THEORY_H
#define CONTROL_THEORY_H

typedef struct {
    double kp;
    double ki;
    double kd;
    
    double integral;
    double prev_measurement;
    double d_filtered;       // Stores state for derivative low-pass filter
    
    double output_limit;
    double alpha;            // Filter coefficient (0.0 < alpha <= 1.0)
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
void pid_init(pid_struct *PID, double kp, double ki, double kd,
              double output_limit, double alpha);

/**
 * @brief Computes the PID output step.
 * @param PID Pointer to the PID struct
 * @param setpoint Target value (e.g., center line position or target speed)
 * @param measurement Current sensor feedback
 * @param dt Time step in seconds
 * @return Clamped control output
 */
double pid_update(pid_struct *PID, double setpoint, double measurement, double dt);

#endif // CONTROL_THEORY_H