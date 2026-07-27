#ifndef CONTROL_THEORY_H
#define CONTROL_THEORY_H

typedef struct {
    double kp,kd,ki;
    double integral;
    double prev_error;
    double integral_limit;
    double output_limit;
}pid_struct;

void pid_init(pid_struct *PID, double kp, double ki, double kd,
              double integral_limit, double output_limit);

double pid_update(pid_struct *PID, double error, double dt);

#endif