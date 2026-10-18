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

#include "Config.h"

static float clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

/* Linear ramp 0 at lo -> 1 at hi; returns 0 if the range is degenerate (never NaN). */
static float ramp01(float value, float lo, float hi)
{
    float range = hi - lo;
    if (range < 0.1f) return 0.0f;
    return clamp01((value - lo) / range);
}

float corner_severity_update(corner_severity_t *s,
                             float filtered_front_angle_deg,
                             float filtered_heading_deg,
                             uint8_t front_corner_wall,
                             uint8_t top_wall_crosses_center,
                             uint8_t intersection_latched,
                             float fa_th, float h_lo, float h_hi)
{
    /* Latched intersection: drive straight through at full speed and forget any corner peak */
    if (intersection_latched) {
        corner_severity_reset(s);
        return 0.0f;
    }

    /* Front camera lookahead. Uses the filtered angle every tick (it decays on dropped frames),
     * so a missed front frame cannot cause a 1-tick speed spike. */
    float front_sev = front_corner_wall ? 1.0f
                    : ramp01(fabsf(filtered_front_angle_deg), fa_th, (float)FRONT_SEVERITY_FULL_DEG);

    /* Top camera: keeps speed down while the car is actually turning through the corner */
    float top_sev = top_wall_crosses_center ? 1.0f
                  : ramp01(fabsf(filtered_heading_deg), h_lo, h_hi);

    float raw = fmaxf(front_sev, top_sev);

    /* Peak hold: rising severity passes through immediately; a drop is delayed by
     * CORNER_SEVERITY_HOLD_TICKS (covers a corner wall flickering out for a few frames). */
    if (raw >= s->held_severity) {
        s->held_severity = raw;
        s->hold_ticks    = (uint8_t)CORNER_SEVERITY_HOLD_TICKS;
    } else if (s->hold_ticks > 0U) {
        s->hold_ticks--;
    } else {
        s->held_severity = raw;
    }
    return s->held_severity;
}

float corner_severity_speed(float severity, float spd_s, float spd_out)
{
    return spd_s - clamp01(severity) * (spd_s - spd_out);
}
