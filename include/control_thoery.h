#ifndef CONTROL_THEORY_H
#define CONTROL_THEORY_H

typedef struct {
    float kp,kd,ki;
    float integral;
    float prev_error;
    float integral_limit;
    float output_limit;
}pid_struct;

void pid_init(pid_struct *PID, float kp, float ki, float kd,
              float integral_limit, float output_limit);

float pid_update(pid_struct *PID, float error, float dt);

#endif