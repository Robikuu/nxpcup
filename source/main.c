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

                lane_result_t lane = compute_lane_center(vectors, num_vectors);

                if (lane.lines_found == 0) {
                    /* all vectors filtered out — treat as line lost */
                    if (++line_lost_counter > LINE_LOST_LIMIT) {
                        HbridgeBrake(&g_hbridge);
                    }
                } else {
                    line_lost_counter = 0;

                    /* errors: lateral offset (cm) + lane heading (deg) */
                    float e_cte     = lane.cte * CTE_TO_CM;
                    float e_heading = lane.heading * HEADING_TO_DEG;

                    /* combined error: positive = lane center is to the RIGHT */
                    float combined = g_params.k_cte * e_cte
                                   + g_params.k_heading * e_heading;

                    float steer_cmd = -pid_update(&steer_pid, 0.0f, combined, g_params.dt);

                    /* debug: L=lines_found  C=cte*10  H=heading*10  S=steer*10 */
                    esp_printf("L:%u C:%d H:%d S:%d\r\n",
                               (unsigned)lane.lines_found,
                               (int)(e_cte * 10.0f),
                               (int)(e_heading * 10.0f),
                               (int)(steer_cmd * 10.0f));

                    Steer(steer_cmd + g_params.steering_offset);

                    HbridgeSpeed(&g_hbridge, (int16_t)g_params.base_speed_left, (int16_t)g_params.base_speed_right);
                }
            }

        } else {
            /* IDLE: clear controller state so next run starts clean */
            steer_pid.integral         = 0.0f;
            steer_pid.prev_measurement = 0.0f;
            steer_pid.d_filtered       = 0.0f;
            steer_pid.prev_output      = 0.0f;
            steer_pid.initialized      = 0;
            line_lost_counter          = 0;
            HbridgeBrake(&g_hbridge);
        }

        if (button_pressed_edge()) {
            g_params.state = STATE_IDLE;
            HbridgeBrake(&g_hbridge);
            PRINTF("Button: STOPPED\r\n");
        }
    }
}
