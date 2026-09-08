#include "fsl_ctimer.h"
#include "peripherals.h"
#include "fsl_debug_console.h"
#include "utils.h"
#include <math.h>

void Steer(float angle)
{
	// physical limit on the chassis
    if (angle > 70.0f)  angle = 70.0f;
    if (angle < -100.0f) angle = -100.0f;

    float duty = 5.0f + ((angle + 100.0f) / 200.0f) * 5.0f;

    uint32_t periodTicks = CTIMER2_PERIPHERAL->MR[CTIMER2_PWM_PERIOD_CH];
    uint32_t pulseTicks = (uint32_t)((periodTicks * (100.0f - duty)) / 100.0f);

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
