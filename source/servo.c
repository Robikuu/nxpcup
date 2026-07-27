#include "fsl_ctimer.h"
#include "peripherals.h"
#include "fsl_debug_console.h"
#include "utils.h"
#include <math.h>

void Steer(double angle)
{
    if (angle > 100.0)  angle = 100.0;
    if (angle < -100.0) angle = -100.0;

    double duty = 5.0 + ((angle + 100.0) / 200.0) * 5.0;

    uint32_t periodTicks = CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_PERIOD_CH];
    uint32_t pulseTicks = (uint32_t)((periodTicks * (100.0 - duty)) / 100.0);

    CTIMER2_PERIPHERAL->MR[2] = pulseTicks;
}

void TestServo(void)
{
    int SteerStrength;
    while (1) {
        for (SteerStrength = -100; SteerStrength <= 100; SteerStrength++) {
            volatile int delay = 200000;
            while (delay) { delay--; }
            PRINTF("Steer: %d\n", SteerStrength);
            Steer(SteerStrength);

            if (button_pressed_edge()) {
                return;
            }
        }
		for (SteerStrength = 100; SteerStrength >= -100; SteerStrength--) {
            volatile int delay = 200000;
            while (delay) { delay--; }
            PRINTF("Steer: %d\n", SteerStrength);
            Steer(SteerStrength);

            if (button_pressed_edge()) {
                return;
            }
        }
    }
}
