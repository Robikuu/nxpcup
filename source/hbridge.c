#include "hbridge.h"
#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "Config.h"
#include <math.h>

Hbridge g_hbridge;
static uint32_t s_srcClockHz;

void HbridgeInit(Hbridge *h,
                 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1DirPort, uint32_t m1DirPin,
                 GPIO_Type *m2DirPort, uint32_t m2DirPin)
{
    h->periodChannel  = periodCh;
    h->pwm1Channel    = pwm1Ch;
    h->pwm2Channel    = pwm2Ch;
    h->motor1DirPort  = m1DirPort;
    h->motor1DirPin   = m1DirPin;
    h->motor2DirPort  = m2DirPort;
    h->motor2DirPin   = m2DirPin;
    h->pwmPeripheral  = pwmPeriph;
}

static inline int16_t clamp100(int16_t v)
{
    if (v >  100) return  100;
    if (v < -100) return -100;
    return v;
}

void HbridgeSpeed(Hbridge *h, int16_t speed_left, int16_t speed_right)
{
    speed_left  = clamp100(speed_left);
    speed_right = clamp100(speed_right);

    uint8_t duty_left  = (uint8_t)(speed_left  >= 0 ? speed_left  : -speed_left);
    uint8_t duty_right = (uint8_t)(speed_right >= 0 ? speed_right : -speed_right);

    /* LEFT motor: forward = DIR HIGH */
    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, speed_left >= 0 ? 1U : 0U);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel,
                              h->pwm2Channel, duty_left);

    /* RIGHT motor: forward = DIR LOW */
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, speed_right >= 0 ? 0U : 1U);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel,
                              h->pwm1Channel, duty_right);
}

void HbridgeBrake(Hbridge *h)
{
    /* NOTE: with one direction pin per motor, duty=0 COASTS (freewheel).
       True short-brake on an L298N needs both IN pins at the same level,
       which this wiring can't do. If stopping distance is too long,
       implement a brief reverse-pulse brake in software instead. */
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel,
                              h->pwm1Channel, 0U);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel,
                              h->pwm2Channel, 0U);
}

/*
 * steering_output: -100..+100, positive = RIGHT turn (must match Steer()
 * sign convention — verify on the bench).
 * speed_left/right outputs are in duty %.
 */
void compute_motor_speeds(float steering_output, float *speed_left, float *speed_right)
{
    float steer_rad = (steering_output / 100.0f)
                    * (MAX_SERVO_ANGLE_DEG * (float)M_PI / 180.0f);

    float radius = (fabsf(steer_rad) > 0.001f)
                 ? (WHEELBASE_MM / tanf(fabsf(steer_rad)))
                 : 1e6f;

    /* Physical cornering speed limit (mm/s) */
    float max_corner_speed = sqrtf(MAX_LATERAL_ACCEL * radius);

    /* Convert base duty % to approx mm/s via measured top speed.
       SPEED_AT_FULL_DUTY_MMPS must be measured once: full throttle,
       measure distance over time. Until then the limiter is approximate. */
    float left_base  = g_params.base_speed_left;
    float right_base = g_params.base_speed_right;
    float avg_mmps   = 0.5f * (left_base + right_base) * 0.01f
                     * SPEED_AT_FULL_DUTY_MMPS;

    float speed_scale = 1.0f;
    if (avg_mmps > max_corner_speed && avg_mmps > 0.0f) {
        speed_scale = max_corner_speed / avg_mmps;
    }

    /* Ackermann inside/outside wheel split */
    float half_track    = TRACK_WIDTH_MM * 0.5f;
    float inside_scale  = 1.0f - half_track / radius;
    float outside_scale = 1.0f + half_track / radius;
    if (inside_scale < 0.0f) inside_scale = 0.0f;  /* extreme turn guard */

    if (steering_output > 0.0f) {          /* right turn: right wheel inside */
        *speed_right = right_base * inside_scale  * speed_scale;
        *speed_left  = left_base  * outside_scale * speed_scale;
    } else if (steering_output < 0.0f) {   /* left turn: left wheel inside */
        *speed_left  = left_base  * inside_scale  * speed_scale;
        *speed_right = right_base * outside_scale * speed_scale;
    } else {
        *speed_left  = left_base  * speed_scale;
        *speed_right = right_base * speed_scale;
    }

    if (*speed_left  >  g_params.max_speed_left)  *speed_left  =  g_params.max_speed_left;
    if (*speed_left  < -g_params.max_speed_left)  *speed_left  = -g_params.max_speed_left;
    if (*speed_right >  g_params.max_speed_right) *speed_right =  g_params.max_speed_right;
    if (*speed_right < -g_params.max_speed_right) *speed_right = -g_params.max_speed_right;
}