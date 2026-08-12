#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "app.h"
#include "fsl_pwm.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "hbridge.h"
#include "pixy.h"
#include "fsl_common.h"
#include "Config.h"
#include "servo.h"
#include "esc.h"
#include <math.h>
#include "fsl_lpuart.h"
#include "control_theory.h"
#include "fsl_gpio.h"
#include "utils.h"
#include "vectors.h"
#include "wifi_config.h"

#define MAX_VECTORS 10
#define LINE_LOST_LIMIT    5

void TestHbridge(void)
{
    PRINTF("=== H-Bridge Test ===\r\n");

    // Phase 1: Both motors forward
    PRINTF("Both motors forward...\r\n");
    HbridgeSpeed(&g_hbridge, 50, -50);
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
    HbridgeSpeed(&g_hbridge, 0, -50);
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

    PRINTF("=== H-Bridge Test Done ===\r\n");
}

void TestPidSteer(void)
{
    pid_struct steer_pid;
    
    // Updated init matching: (PID, kp, ki, kd, output_limit, alpha)
    // alpha = 0.2 provides a solid balance for filtering derivative noise
    pid_init(&steer_pid, 2.0, 0.1, 0.5, /*output_limit*/ 100.0, /*alpha*/ 0.2);

    double setpoint = 0.0;            // Target center position/angle
    double fake_measurement = -80.0;  // Initial measurement (gives error = +80.0)
    double dt = 0.02;

    while (1) {
        // Updated pid_update signature: (PID, setpoint, measurement, dt)
        double output = pid_update(&steer_pid, setpoint, fake_measurement, dt);

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

#define PIXY_GRID_SCALE    4.0f

/* Convert ground-space errors to cm / degrees so the existing gains
   (tuned for pixel/degree-ish scales) stay meaningful. */
#define CTE_TO_CM          100.0f
#define HEADING_TO_DEG     (180.0f / (float)M_PI)

int main(void)
{
    /* ---------- 1. Hardware ---------- */
    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    /* ---------- 2. Params FIRST (everything below reads g_params) ---------- */
    params_init();

    /* ---------- 3. Actuators: safe state immediately ---------- */
    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U,
                GPIO0, 27U);
    HbridgeBrake(&g_hbridge);
    Steer(g_params.steering_offset);

    //TestHbridge();

    /* ---------- 4. Steering controller ---------- */
    static pid_struct steer_pid;
    pid_init(&steer_pid,
             g_params.kp, g_params.ki, g_params.kd,
             100.0f, g_params.alpha);

    SDK_DelayAtLeastUs(3000000, SystemCoreClock);
    /* ---------- 5. Camera ---------- */
    static pixy_t cam1;
    status_t ps = pixy_init(&cam1, LPI2C2, 0x54U,
                            &LP_FLEXCOMM2_RX_Handle,
                            &LP_FLEXCOMM2_TX_Handle);
    if (ps == kStatus_Success) {
        pixy_set_led(&cam1, 0, 255, 0);      /* green = camera OK */
    } else {
        PRINTF("Pixy init FAILED\r\n");      /* car stays IDLE */
    }

    uint32_t line_lost_counter = 0;
    uint16_t vectors[MAX_VECTORS * 4];
    size_t   num_vectors;

    PRINTF("Booted. Send 'start;' to run, 'stop;' to halt.\r\n");

    /* ---------- 6. Main loop ---------- */
    while (1) {
        command_poll();

        /* live-update gains from remote tuning (no pid_init re-call!) */
        steer_pid.kp    = g_params.kp;
        steer_pid.ki    = g_params.ki;
        steer_pid.kd    = g_params.kd;
        steer_pid.alpha = g_params.alpha;

        if (g_params.state == STATE_RUNNING) {

            status_t s = pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors);

            if (s != kStatus_Success || num_vectors == 0) {
                /* camera failure and "no line" get identical treatment */
                if (++line_lost_counter > LINE_LOST_LIMIT) {
                    HbridgeBrake(&g_hbridge);
                }
            } else {
                line_lost_counter = 0;

                /* pick the vector whose tail is closest to the car (max y0) */
                size_t best = 0;
                for (size_t i = 1; i < num_vectors; i++) {
                    if (vectors[4*i + 1] > vectors[4*best + 1]) best = i;
                }

                /* scale 79x52 grid -> 316x208 calibration frame */
                float u0 = vectors[4*best + 0] * PIXY_GRID_SCALE;
                float v0 = vectors[4*best + 1] * PIXY_GRID_SCALE;
                float u1 = vectors[4*best + 2] * PIXY_GRID_SCALE;
                float v1 = vectors[4*best + 3] * PIXY_GRID_SCALE;

                /* pixel -> ground (meters, X right, Y forward) */
                float X0, Y0, X1, Y1;
                pixel_to_ground(u0, v0, &X0, &Y0);
                pixel_to_ground(u1, v1, &X1, &Y1);

                /* errors: lateral offset (cm) + line heading (deg) */
                float e_cte     = X0 * CTE_TO_CM;
                float e_heading = atan2f(X1 - X0, Y1 - Y0) * HEADING_TO_DEG;

                /* combined error: positive = line is to the RIGHT / veering right */
                float combined = g_params.k_cte * e_cte
                               + g_params.k_heading * e_heading;

                /* PID error = 0 - combined = -combined, so negate the output:
                   steer_cmd = +kp*combined + ... -> steer TOWARD the line */
                float steer_cmd = -pid_update(&steer_pid, 0.0f, combined, g_params.dt);

                /* sign convention: positive steer_cmd = RIGHT turn.
                   If your servo is mirrored, flip here instead. */
                Steer(steer_cmd + g_params.steering_offset);

                float sl, sr;
                compute_motor_speeds(steer_cmd, &sl, &sr);
                HbridgeSpeed(&g_hbridge, (int16_t)sl, (int16_t)sr);
            }

        } else {
            /* IDLE: actually stopped */
            HbridgeBrake(&g_hbridge);
        }

        if (button_pressed_edge()) {
            g_params.state = STATE_IDLE;
            HbridgeBrake(&g_hbridge);
            PRINTF("Button: STOPPED\r\n");
        }
    }
}
