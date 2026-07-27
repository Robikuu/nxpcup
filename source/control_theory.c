#include <math.h>
#include "control_theory.h"

void pid_init(pid_struct *PID, double kp, double ki, double kd,
              double output_limit, double alpha) {
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->integral = 0.0;
    PID->prev_measurement = 0.0;
    PID->d_filtered = 0.0;
    PID->output_limit = output_limit;
    PID->alpha = alpha; // LPF smoothing factor (e.g., 0.1 to 0.3)
}

double pid_update(pid_struct *PID, double setpoint, double measurement, double dt) {
    if (dt <= 0.0) return 0.0;

    double error = setpoint - measurement;

    // 1. Proportional Term
    double p_term = PID->kp * error;

    // 2. Derivative on Measurement (Prevents Derivative Kick) + Low-Pass Filter
    double d_raw = -(measurement - PID->prev_measurement) / dt;
    PID->d_filtered = (PID->alpha * d_raw) + ((1.0 - PID->alpha) * PID->d_filtered);
    double d_term = PID->kd * PID->d_filtered;
    PID->prev_measurement = measurement;

    // Unclamped candidate output before integral
    double raw_output = p_term + (PID->ki * PID->integral) + d_term;

    // 3. Conditional Integration (Anti-Windup)
    // Only accumulate integral if we are NOT at maximum actuator saturation
    if (fabs(raw_output) < PID->output_limit) {
        PID->integral += error * dt;
    }

    // Recalculate output with updated integral
    double output = p_term + (PID->ki * PID->integral) + d_term;

    // 4. Output Clamping
    if (output > PID->output_limit)  output = PID->output_limit;
    if (output < -PID->output_limit) output = -PID->output_limit;

    return output;
}