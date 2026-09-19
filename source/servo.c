#include "fsl_ctimer.h"
#include "peripherals.h"
#include "fsl_debug_console.h"
#include "utils.h"
#include "Config.h"
#include "servo.h"
#include <math.h>

void Steer(float angle)
{
    /* Clamp strictly to physical mechanical limits of this car: [-70.0f, +87.0f] */
    if (angle > STEERING_LIMIT_RIGHT) angle = STEERING_LIMIT_RIGHT;
    if (angle < STEERING_LIMIT_LEFT)  angle = STEERING_LIMIT_LEFT;

    /*
     * Standard 50 Hz RC servo mapping:
     * -100 -> 5% duty (1.0 ms)
     *    0 -> 7.5% duty (1.5 ms)
     * +100 -> 10% duty (2.0 ms)
     * (Trim offset is applied by caller in main.c)
     */
    float duty = 5.0f + ((angle + 100.0f) / 200.0f) * 5.0f;

    uint32_t periodTicks = CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_PERIOD_CH];
    uint32_t pulseTicks  = (uint32_t)((periodTicks * (100.0f - duty)) / 100.0f);

    CTIMER2_PERIPHERAL->MR[2] = pulseTicks;
}

void TestServo(void)
{
    int SteerStrength;
    while (1) {
        for (SteerStrength = (int)STEERING_LIMIT_LEFT; SteerStrength <= (int)STEERING_LIMIT_RIGHT; SteerStrength++) {
            volatile int delay = 200000;
            while (delay) { delay--; }
            PRINTF("Steer: %d\n", SteerStrength);
            Steer((float)SteerStrength);

            if (button_pressed_edge()) {
                return;
            }
        }
        for (SteerStrength = (int)STEERING_LIMIT_RIGHT; SteerStrength >= (int)STEERING_LIMIT_LEFT; SteerStrength--) {
            volatile int delay = 200000;
            while (delay) { delay--; }
            PRINTF("Steer: %d\n", SteerStrength);
            Steer((float)SteerStrength);

            if (button_pressed_edge()) {
                return;
            }
        }
    }
}
