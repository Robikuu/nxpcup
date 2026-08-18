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

    h->periodChannel    = periodCh;
    h->pwm1Channel      = pwm1Ch;
    h->pwm2Channel      = pwm2Ch;
    h->motor1DirPort    = m1DirPort;
    h->motor1DirPin     = m1DirPin;
    h->motor2DirPort    = m2DirPort;
    h->motor2DirPin     = m2DirPin;
    h->pwmPeripheral    = pwmPeriph;
    g_hbridge = *h;
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

static float s_current_speed = 0.0f;
static float s_filtered_diff  = 0.0f;

void motor_speeds_reset(void)
{
    s_current_speed = 0.0f;
    s_filtered_diff = 0.0f;
}

float motor_speeds_get_current(void)
{
    return s_current_speed;
}

/*
 * steer_cmd: -100..+100, positive = RIGHT turn
 * heading_err: slope (lateral px / forward px), positive = RIGHT curve
 * lines_found: 0 = lost, 1 = single line, 2 = dual line
 * dt: loop duration in seconds
 * speed_left/right: duty % outputs
 */
void compute_motor_speeds(float steer_cmd, float heading_err, uint8_t lines_found, float dt, float *speed_left, float *speed_right)
{
    /* 1. Combined Curvature Demand [0.0 to 1.0] from steering + heading preview
     * HEADING_CURVATURE_GAIN = 2.0f triggers early trail-braking before corner entry. */
    const float HEADING_CURVATURE_GAIN = 2.0f;
    float steer_factor   = fabsf(steer_cmd) / 100.0f;
    float heading_factor = fabsf(heading_err) * HEADING_CURVATURE_GAIN;
    float turn_intensity = (steer_factor > heading_factor) ? steer_factor : heading_factor;
    if (turn_intensity > 1.0f) turn_intensity = 1.0f;

    /* 2. Target Speed Interpolation between max_speed (straights) and base_speed (corners) */
    float target_left  = g_params.max_speed_left  - turn_intensity * (g_params.max_speed_left  - g_params.base_speed_left);
    float target_right = g_params.max_speed_right - turn_intensity * (g_params.max_speed_right - g_params.base_speed_right);
    float target_base  = 0.5f * (target_left + target_right);

    /* 3. Safety Derating when vision confidence is degraded */
    if (lines_found < 2) {
        float safe_cap = (g_params.base_speed_left + g_params.base_speed_right) * 0.5f;
        if (target_base > safe_cap) {
            target_base  = safe_cap;
            target_left  = g_params.base_speed_left;
            target_right = g_params.base_speed_right;
        }
    }

    /* 4. Slew-Rate Limiter (Asymmetric Accel vs Braking) */
    const float MAX_ACCEL_PER_SEC = 200.0f; /* max +200% duty/sec */
    const float MAX_BRAKE_PER_SEC = 500.0f; /* max -500% duty/sec */

    float max_pos_step = MAX_ACCEL_PER_SEC * dt;
    float max_neg_step = MAX_BRAKE_PER_SEC * dt;

    if (target_base > s_current_speed) {
        float diff = target_base - s_current_speed;
        s_current_speed += (diff > max_pos_step) ? max_pos_step : diff;
    } else {
        float diff = s_current_speed - target_base;
        s_current_speed -= (diff > max_neg_step) ? max_neg_step : diff;
    }

    /* 5. Kinematic Ackermann Differential Drive Split */
    float steer_rad = (steer_cmd / 100.0f)
                    * (MAX_SERVO_ANGLE_DEG * (float)M_PI / 180.0f);

    float radius = (fabsf(steer_rad) > 0.001f)
                 ? (WHEELBASE_MM / tanf(fabsf(steer_rad)))
                 : 1e6f;

    float half_track    = TRACK_WIDTH_MM * 0.5f;
    float raw_diff      = (radius > 1.0f) ? (half_track / radius) : 0.0f;
    if (raw_diff > 0.25f) raw_diff = 0.25f; /* clamp maximum 25% wheel speed divergence */

    /* Low-pass filter the differential split (alpha = 0.20) to suppress chassis yaw resonance */
    s_filtered_diff = 0.80f * s_filtered_diff + 0.20f * raw_diff;

    float inside_scale  = 1.0f - s_filtered_diff;
    float outside_scale = 1.0f + s_filtered_diff;
    if (inside_scale < 0.0f) inside_scale = 0.0f;

    float base_l = (target_base > 0.0f) ? (s_current_speed * (target_left / target_base)) : s_current_speed;
    float base_r = (target_base > 0.0f) ? (s_current_speed * (target_right / target_base)) : s_current_speed;

    if (steer_cmd > 0.0f) {          /* Right turn: right wheel inside, left wheel outside */
        *speed_right = base_r * inside_scale;
        *speed_left  = base_l * outside_scale;
    } else if (steer_cmd < 0.0f) {   /* Left turn: left wheel inside, right wheel outside */
        *speed_left  = base_l * inside_scale;
        *speed_right = base_r * outside_scale;
    } else {                         /* Straightaway */
        *speed_left  = base_l;
        *speed_right = base_r;
    }

    /* 6. Hardware & Parameter Clamping */
    if (*speed_left  >  g_params.max_speed_left)  *speed_left  =  g_params.max_speed_left;
    if (*speed_left  < -g_params.max_speed_left)  *speed_left  = -g_params.max_speed_left;
    if (*speed_right >  g_params.max_speed_right) *speed_right =  g_params.max_speed_right;
    if (*speed_right < -g_params.max_speed_right) *speed_right = -g_params.max_speed_right;
}
