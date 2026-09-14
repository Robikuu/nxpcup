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
    ctimer_config_t config;

    g_hbridge = *h;
    h->periodChannel    = periodCh;
    h->pwm1Channel      = pwm1Ch;
    h->pwm2Channel      = pwm2Ch;
    h->motor1DirPort    = m1DirPort;
    h->motor1DirPin     = m1DirPin;
    h->motor2DirPort    = m2DirPort;
    h->motor2DirPin     = m2DirPin;
    h->pwmPeripheral    = pwmPeriph;
}

void HbridgeSpeed(Hbridge *h, int16_t speed1, int16_t speed2)
{
    /* Clamp to -100..+100 */
    if (speed1 >  100) speed1 =  100;
    if (speed1 < -100) speed1 = -100;
    if (speed2 >  100) speed2 =  100;
    if (speed2 < -100) speed2 = -100;

    uint8_t duty1, duty2;

    /* Speed1 (Right Motor): controls motor1DirPort & pwm2Channel */
    if (speed1 >= 0) {
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 1U);
        duty1 = (uint8_t)(100 - speed1);
    } else {
        GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
        duty1 = (uint8_t)(-speed1);
    }

    /* Speed2 (Left Motor): controls motor2DirPort & pwm1Channel */
    if (speed2 >= 0) {
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 1U);
        duty2 = (uint8_t)(100 - speed2);
    } else {
        GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);
        duty2 = (uint8_t)(-speed2);
    }

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, duty1);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, duty2);
}

void HbridgeBrake(Hbridge *h)
{
    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm1Channel, 0U);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral, h->periodChannel, h->pwm2Channel, 0U);
}
