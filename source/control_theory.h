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

/* ---- Severity-Scaled Corner Speed ----
 * severity = max(front_sev, top_sev), 0.0 = straight, 1.0 = full corner braking.
 *   front_sev: 0 at fa_th -> 1 at FRONT_SEVERITY_FULL_DEG, or 1 on a front-camera corner wall.
 *   top_sev:   0 at h_lo  -> 1 at h_hi, or 1 when a top-camera wall crosses center.
 * The peak is held for CORNER_SEVERITY_HOLD_TICKS after it drops (wall flicker guard),
 * and forced to 0 (hold cleared) while an intersection is latched.
 */
typedef struct {
    float   held_severity;
    uint8_t hold_ticks;
} corner_severity_t;

static inline void corner_severity_reset(corner_severity_t *s) {
    s->held_severity = 0.0f;
    s->hold_ticks    = 0U;
}

float corner_severity_update(corner_severity_t *s,
                             float filtered_front_angle_deg,
                             float filtered_heading_deg,
                             uint8_t front_corner_wall,
                             uint8_t top_wall_crosses_center,
                             uint8_t intersection_latched,
                             float fa_th, float h_lo, float h_hi);

/* target_speed = spd_s - severity * (spd_s - spd_out) */
float corner_severity_speed(float severity, float spd_s, float spd_out);

#endif // CONTROL_THEORY_H