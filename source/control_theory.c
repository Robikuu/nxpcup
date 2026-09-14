#include <math.h>
#include "control_theory.h"

void pid_init(pid_struct *PID, float kp, float ki, float kd, float output_limit, float alpha) {
    PID->kp = kp;
    PID->ki = ki;
    PID->kd = kd;
    PID->integral = 0.0f;
    PID->prev_measurement = 0.0f;
    PID->d_filtered = 0.0f;
    PID->output_limit = output_limit;
    PID->alpha = alpha;
    PID->prev_output = 0.0f;
    PID->initialized = 0;
}

float pid_update(pid_struct *PID, float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return PID->prev_output;

    float error = setpoint - measurement;
    float p_term = PID->kp * error;
    float d_raw;

    if (!PID->initialized) {
        d_raw = 0.0f;
        PID->initialized = 1;
    } else {
        d_raw = -(measurement - PID->prev_measurement) / dt;
    }

    PID->d_filtered = (PID->alpha * d_raw) + ((1.0f - PID->alpha) * PID->d_filtered);
    float d_term = PID->kd * PID->d_filtered;
    PID->prev_measurement = measurement;

    float raw_output = p_term + (PID->ki * PID->integral) + d_term;

    if (fabsf(raw_output) < PID->output_limit || (error * raw_output) < 0.0f) {
        PID->integral += error * dt;
    }

    float output = p_term + (PID->ki * PID->integral) + d_term;

    if (output >  PID->output_limit) output =  PID->output_limit;
    if (output < -PID->output_limit) output = -PID->output_limit;

    PID->prev_output = output;
    return output;
}