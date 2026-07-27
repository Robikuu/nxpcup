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
	uint8_t duty1;
	if (speed1 >= 0) {
	    duty1 = (uint8_t)speed1;
	} else {
	    duty1 = (uint8_t)(100 + speed1);
	}

	uint8_t duty2;
	if (speed2 >= 0) {
	    duty2 = (uint8_t)speed2;
	} else {
	    duty2 = (uint8_t)(100 + speed2);
	}

    if (duty1 > 100U) duty1 = 100U;
    if (duty2 > 100U) duty2 = 100U;

    if(speed2 < 0) {
    	 GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 1U);
    } else {
    	 GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
    }

    if(speed1 < 0){
		 GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 1U);
	} else {
		 GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);
	}

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral,h->periodChannel , h->pwm1Channel, duty1);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral,h->periodChannel , h->pwm2Channel, duty2);
}

void HbridgeBrake(Hbridge *h)
{
    GPIO_PinWrite(h->motor1DirPort, h->motor1DirPin, 0U);
    GPIO_PinWrite(h->motor2DirPort, h->motor2DirPin, 0U);

    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral,h->periodChannel , h->pwm1Channel, 0U);
    CTIMER_UpdatePwmDutycycle(h->pwmPeripheral,h->periodChannel , h->pwm2Channel, 0U);
}

void compute_motor_speeds(double steering_output, double *speed_left, double *speed_right)
{
    // 1. Convert abstract PID output (-100..100) into a real steering angle
    double steer_angle_deg = (steering_output / 100.0) * MAX_SERVO_ANGLE_DEG;
    double steer_angle_rad = steer_angle_deg * (M_PI / 180.0);

    // 2. Bicycle-model turn radius from steering angle
    //    (near-zero angle -> near-infinite radius -> straight line)
    double radius;
    if (fabs(steer_angle_rad) > 0.001) {
        radius = WHEELBASE_MM / tan(fabs(steer_angle_rad));
    } else {
        radius = 1e6; // effectively straight
    }

    // 3. Max safe cornering speed for this radius (classic v = sqrt(a_lat * R))
    double max_corner_speed = sqrt(MAX_LATERAL_ACCEL * radius);
    double target_speed = BASE_SPEED;
    if (target_speed > max_corner_speed) target_speed = max_corner_speed;

    // 4. Differential speed from the SAME radius — inside wheel travels a smaller arc
    double half_track = TRACK_WIDTH_MM / 2.0;
    double inside_speed  = target_speed * (1.0 - half_track / radius);
    double outside_speed = target_speed * (1.0 + half_track / radius);

    if (steering_output > 0) { // turning right -> right wheel is inside
        *speed_right = inside_speed;
        *speed_left  = outside_speed;
    } else {
        *speed_left  = inside_speed;
        *speed_right = outside_speed;
    }

    if (*speed_left  > 100.0) *speed_left  = 100.0;
    if (*speed_right > 100.0) *speed_right = 100.0;
    if (*speed_left  < -100.0) *speed_left = -100.0;
    if (*speed_right < -100.0) *speed_right = -100.0;
}
