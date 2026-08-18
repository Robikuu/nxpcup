#include <math.h>
#include "control_theory.h"

void pid_init(pid_struct *PID, float kp, float ki, float kd,
              float output_limit, float alpha)
{
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

float pid_update(pid_struct *PID, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) return PID->prev_output;

    float error = setpoint - measurement;

    // 1. Proportional
    float p_term = PID->kp * error;

    // 2. Derivative on measurement + low-pass filter
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

    // Candidate output before integral update
    float raw_output = p_term + (PID->ki * PID->integral) + d_term;

    // 3. Conditional integration: skip when saturated, unless error
    //    would help unwind the saturation
    if (fabsf(raw_output) < PID->output_limit || (error * raw_output) < 0.0f) {
        PID->integral += error * dt;
    }

    float output = p_term + (PID->ki * PID->integral) + d_term;

    // 4. Clamp
    if (output >  PID->output_limit) output =  PID->output_limit;
    if (output < -PID->output_limit) output = -PID->output_limit;

    PID->prev_output = output;
    return output;
}

float compute_speed_steer_gain(float current_speed, float base_speed, float max_speed, float min_scale)
{
    if (max_speed <= base_speed) {
        return 1.0f;
    }
    float speed_ratio = (current_speed - base_speed) / (max_speed - base_speed);
    if (speed_ratio < 0.0f) speed_ratio = 0.0f;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;
    return 1.0f - ((1.0f - min_scale) * speed_ratio);
}

float compute_speed_steer_gain_adaptive(float current_speed, float base_speed, float max_speed,
                                       float min_scale, float abs_cte, float abs_heading)
{
    if (max_speed <= base_speed) return 1.0f;

    float speed_ratio = (current_speed - base_speed) / (max_speed - base_speed);
    if (speed_ratio < 0.0f) speed_ratio = 0.0f;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;

    float straight_attenuation = 1.0f - ((1.0f - min_scale) * speed_ratio);

    // Curvature demand [0.0 = straight line, 1.0 = sharp turn]
    float cte_urgency     = abs_cte / 16.0f;
    float heading_urgency = abs_heading / 0.35f;
    float urgency         = (cte_urgency > heading_urgency) ? cte_urgency : heading_urgency;
    if (urgency > 1.0f) urgency = 1.0f;

    // Blend: when cornering urgency is high, restore full 100% steering authority
    return straight_attenuation + (1.0f - straight_attenuation) * urgency;
}