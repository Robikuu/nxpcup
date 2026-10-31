#include "utils.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include <string.h>
#include <stdlib.h>
#include "servo.h"
#include "fsl_lpuart.h"
#include "peripherals.h"
#include <math.h>
#include "hbridge.h"
#include "control_theory.h"

#include "Config.h"

//nume peripheral = LP_FLEXCOMM9_PERIPHERAL

static int button_prev_state = 1;
static int button_debounce_cnt = 0;
static uint32_t button_hold_cnt = 0;
static uint8_t button_hold_fired = 0;
static uint8_t button_press_consumed = 0;

void receive_message(void) {
    uint8_t character;
    
    // Check if data has arrived in the hardware buffer
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LP_FLEXCOMM9_PERIPHERAL)) {
        // Read the character from the ESP
        character = LPUART_ReadByte(LP_FLEXCOMM9_PERIPHERAL);
        
        // Print it to the NXP debug console
        PUTCHAR(character); 
    }
}

void buttons_init(void)
{
    button_prev_state     = 1; // idles HIGH due to pull-up
    button_debounce_cnt   = 0;
    button_hold_cnt       = 0;
    button_hold_fired     = 0;
    button_press_consumed = 0;
}

button_event_t button_update(void)
{
    button_event_t event = BUTTON_EVT_NONE;
    int button_raw = GPIO_PinRead(GPIO0, 23U);

    if (button_raw != button_prev_state) {
        button_debounce_cnt++;
        if (button_debounce_cnt >= 3) {
            int was_released = (button_prev_state == 1);
            button_prev_state = button_raw;
            button_debounce_cnt = 0;

            if (was_released && button_raw == 0) {
                /* Falling edge: Button just pressed down */
                button_hold_cnt = 0;
                button_hold_fired = 0;
                button_press_consumed = 0;
                event = BUTTON_EVT_PRESS;
            } else if (!was_released && button_raw == 1) {
                /* Rising edge: Button just released */
                if (!button_hold_fired && !button_press_consumed) {
                    event = BUTTON_EVT_SHORT_CLICK;
                }
                button_hold_cnt = 0;
                button_hold_fired = 0;
                button_press_consumed = 0;
            }
        }
    } else {
        button_debounce_cnt = 0;

        /* While stably held in pressed state (0) */
        if (button_prev_state == 0) {
            button_hold_cnt++;
            if (!button_hold_fired && !button_press_consumed && (button_hold_cnt >= BUTTON_HOLD_TICKS)) {
                button_hold_fired = 1;
                event = BUTTON_EVT_HOLD;
            }
        }
    }

    return event;
}

void button_consume_press(void)
{
    button_press_consumed = 1;
}

int button_pressed_edge(void)
{
    button_event_t evt = button_update();
    return (evt == BUTTON_EVT_PRESS) ? 1 : 0;
}

int button_is_pressed(void)
{
    return (button_prev_state == 0);
}

void print_double(const char *label, float val)
{
    int whole = (int)val;
    float frac_part = val - whole;
    if (frac_part < 0) frac_part = -frac_part;
    int frac = (int)(frac_part * 100.0f);
    PRINTF("%s%d.%02d\r\n", label, whole, frac);
}

void TestHbridge(void)
{
    PRINTF("=== H-Bridge Test ===\r\n");

    // Phase 1: Both motors forward
    PRINTF("Both motors forward...\r\n");
    HbridgeSpeed(&g_hbridge, 50, 50);
    for (int i = 0; i < 20; i++) {
        volatile int delay = 1000000;
        while (delay) { delay--; }
        if (button_pressed_edge()) {
            HbridgeBrake(&g_hbridge);
            return;
        }
    }
    HbridgeBrake(&g_hbridge);

    // Brief pause between phases
    PRINTF("Pause...\r\n");
    for (int i = 0; i < 10; i++) {
        volatile int delay = 1000000;
        while (delay) { delay--; }
    }

    // Phase 2: Left motor only
    PRINTF("Left motor only...\r\n");
    HbridgeSpeed(&g_hbridge, 50, 0);
    for (int i = 0; i < 20; i++) {
        volatile int delay = 1000000;
        while (delay) { delay--; }
        if (button_pressed_edge()) {
            HbridgeBrake(&g_hbridge);
            return;
        }
    }
    HbridgeBrake(&g_hbridge);

    // Brief pause
    PRINTF("Pause...\r\n");
    for (int i = 0; i < 10; i++) {
        volatile int delay = 1000000;
        while (delay) { delay--; }
    }

    // Phase 3: Right motor only
    PRINTF("Right motor only...\r\n");
    HbridgeSpeed(&g_hbridge, 0, 50);
    for (int i = 0; i < 20; i++) {
        volatile int delay = 1000000;
        while (delay) { delay--; }
        if (button_pressed_edge()) {
            HbridgeBrake(&g_hbridge);
            return;
        }
    }
    HbridgeBrake(&g_hbridge);

    PRINTF("=== H-Bridge Test Done ===\r\n");
}

void TestPidSteer(void)
{
    pid_struct steer_pid;

    // Updated init matching: (PID, kp, ki, kd, output_limit, alpha)
    // alpha = 0.2 provides a solid balance for filtering derivative noise
    pid_init(&steer_pid, 2.0, 0.1, 0.5, /*output_limit*/ 100.0, /*alpha*/ 0.2);

    float setpoint = 0.0f;            // Target center position/angle
    float fake_measurement = -80.0f;  // Initial measurement (gives error = +80.0)
    float dt = 0.02f;

    while (1) {
        // Updated pid_update signature: (PID, setpoint, measurement, dt)
        float output = pid_update(&steer_pid, setpoint, fake_measurement, dt);

        print_double("measurement=", fake_measurement);
        print_double("output=", output);
        Steer(output);

        volatile int delay = 2000000;
        while (delay) { delay--; }

        // Decay measurement towards setpoint (0.0)
        fake_measurement *= 0.9;

        // Step response toggle: when car aligns with line, jump error to opposite side
        if (fabs(fake_measurement) < 1.0) {
            fake_measurement = (fake_measurement < 0) ? 80.0 : -80.0;
        }

        if (button_pressed_edge()) {
            return;
        }
    }
}

void TestHBridge(void)
{
    TestHbridge();
}
