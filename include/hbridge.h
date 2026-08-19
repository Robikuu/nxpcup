#ifndef HBRIDGE_H
#define HBRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <stdint.h>

#ifdef __arm__
#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#else
typedef void CTIMER_Type;
typedef void GPIO_Type;
typedef uint32_t ctimer_match_t;
#endif

#define LAUNCH_DURATION_SEC   1.5f
#define LAUNCH_START_SPEED    20.0f
#define LAUNCH_RAMP_RATE      40.0f
#define MAX_ACCEL_PER_SEC     80.0f
#define MAX_BRAKE_PER_SEC     400.0f


typedef struct {
    CTIMER_Type   *pwmPeripheral;
    ctimer_match_t periodChannel;
    ctimer_match_t pwm1Channel;
    ctimer_match_t pwm2Channel;
    GPIO_Type     *motor1DirPort;
    uint32_t       motor1DirPin;
    GPIO_Type     *motor2DirPort;
    uint32_t       motor2DirPin;
} Hbridge;

extern Hbridge g_hbridge;

void HbridgeInit(Hbridge *h,
                 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1DirPort, uint32_t m1DirPin,
                 GPIO_Type *m2DirPort, uint32_t m2DirPin);

void  HbridgeSpeed(Hbridge *h, int16_t speed_right, int16_t speed_left);
void  HbridgeBrake(Hbridge *h);
void  motor_speeds_reset(void);
float motor_speeds_get_current(void);
void  compute_motor_speeds(float steer_cmd, float heading_err, uint8_t lines_found, float dt, float *speed_left, float *speed_right);

#ifdef __cplusplus
}
#endif

#endif
