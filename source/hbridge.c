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

void HbridgeSpeed(Hbridge *h, int16_t speed_left, int16_t speed_right)
{
    /* Clamp to -100..+100 */
    if (speed_left >  100) speed_left =  100;
    if (speed_left < -100) speed_left = -100;
    if (speed_right >  100) speed_right =  100;
    if (speed_right < -100) speed_right = -100;

    uint8_t duty_left, duty_right;

    /* Left Motor (Motor 2: IN3 on pwm2Channel/P0_26, IN4 on motor2DirPin/P0_27):
     * The motors are physically mounted facing opposite directions on the chassis.
     * To spin forward when speed_left >= 0, Motor 2 polarity is inverted relative to Motor 1:
     * - Forward (speed_left >= 0): IN4 = 0U, IN3 = PWM (active-high duty = speed_left)
     * - Reverse (speed_left < 0):  IN4 = 1U, IN3 = PWM (active-low duty = 100 + speed_left) */
    if (speed_left >= 0) {
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);
        duty_left = (uint8_t)speed_left;
    } else {
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 1U);
        duty_left = (uint8_t)(100 + speed_left);
    }

    /* Right Motor (Motor 1: IN1 on motor1DirPin/P0_24, IN2 on pwm1Channel/P0_25):
     * - Forward (speed_right >= 0): IN1 = 1U, IN2 = PWM (active-low duty = 100 - speed)
     * - Reverse (speed_right < 0):  IN1 = 0U, IN2 = PWM (active-high duty = -speed) */
    if (speed_right >= 0) {
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 1U);
        duty_right = (uint8_t)(100 - speed_right);
    } else {
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
        duty_right = (uint8_t)(-speed_right);
    }

    /* Update PWM duty cycles independently to eliminate cross-motor coupling */
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, duty_left);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, duty_right);
}

void HbridgeBrake(Hbridge *h)
{
    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, 0U);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, 0U);
}
