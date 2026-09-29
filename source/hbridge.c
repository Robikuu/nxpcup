#include "hbridge.h"
#include "fsl_ctimer.h"
#include "fsl_gpio.h"
#include "fsl_clock.h"
#include "Config.h"
#include <math.h>

Hbridge g_hbridge;
static uint32_t s_srcClockHz;

//TODO: incearca sa intelegi cum functioneaza compute motor speeds si vezi daca e worth it sa o mentimen sau nu (eventual doar schimbata)

void HbridgeInit(Hbridge *h,
                 CTIMER_Type *pwmPeriph,
                 ctimer_match_t periodCh,
                 ctimer_match_t pwm1Ch,
                 ctimer_match_t pwm2Ch,
                 GPIO_Type *m1DirPort, uint32_t m1DirPin,
                 GPIO_Type *m2DirPort, uint32_t m2DirPin)
{
    h->periodChannel    = periodCh;
    h->pwm1Channel      = pwm1Ch;
    h->pwm2Channel      = pwm2Ch;
    h->motor1DirPort    = m1DirPort;
    h->motor1DirPin     = m1DirPin;
    h->motor2DirPort    = m2DirPort;
    h->motor2DirPin     = m2DirPin;
    h->pwmPeripheral    = pwmPeriph;
    g_hbridge           = *h;
}

static inline void set_pwm_duty_float(CTIMER_Type *base, ctimer_match_t periodCh, ctimer_match_t matchCh, float duty_percent)
{
    if (duty_percent <= 0.0f) {
        base->MR[matchCh] = base->MR[periodCh] + 1U;
    } else if (duty_percent >= 100.0f) {
        base->MR[matchCh] = 0U;
    } else {
        uint32_t period = base->MR[periodCh];
        base->MR[matchCh] = (uint32_t)((float)period * (100.0f - duty_percent) / 100.0f + 0.5f);
    }
}

void HbridgeSpeed(Hbridge *h, float speed_left, float speed_right)
{
    /* Clamp to -100.0f..+100.0f */
    if (speed_left >  100.0f) speed_left =  100.0f;
    if (speed_left < -100.0f) speed_left = -100.0f;
    if (speed_right >  100.0f) speed_right =  100.0f;
    if (speed_right < -100.0f) speed_right = -100.0f;

    float duty_left, duty_right;

    /* Left Motor (Motor 2: IN3 on pwm2Channel/P0_26, IN4 on motor2DirPin/P0_27) */
    if (speed_left > 0.0f) {
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 1U);
        duty_left = 100.0f - speed_left;
    } else if (speed_left < 0.0f) {
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);
        /* Quadratic compensation for DRV8871 Fast Decay (Coast) freewheeling */
        float mag = -speed_left;
        duty_left = (mag * mag) / 100.0f;
    } else {
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);
        duty_left = 0.0f;
    }

    /* Right Motor (Motor 1: IN1 on pwm1Channel/P0_25, IN2 on motor1DirPin/P0_24) */
    if (speed_right > 0.0f) {
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 1U);
        duty_right = 100.0f - speed_right;
    } else if (speed_right < 0.0f) {
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
        /* Quadratic compensation for DRV8871 Fast Decay (Coast) freewheeling */
        float mag = -speed_right;
        duty_right = (mag * mag) / 100.0f;
    } else {
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
        duty_right = 0.0f;
    }

    /* Update PWM duty cycles independently to eliminate cross-motor coupling (float tick resolution) */
    set_pwm_duty_float(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, duty_left);
    set_pwm_duty_float(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, duty_right);
}

void HbridgeBrake(Hbridge *h)
{
    /* DRV8871 Active Low-Side Dynamic Brake: IN1=1, IN2=1 */
    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 1U);
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 1U);

    set_pwm_duty_float(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, 100.0f);
    set_pwm_duty_float(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, 100.0f);
}

