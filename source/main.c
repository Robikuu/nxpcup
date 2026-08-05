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

int main(void)
{
    uint16_t vectors[MAX_VECTORS * 4];
    size_t   num_vectors;

    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U,
                GPIO0, 27U);

    params_init();

    // pixy_t cam1;
    // pixy_init(&cam1, LPI2C2, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    // pixy_set_led(&cam1, 255, 0, 0);

    pid_struct steer_pid;
    pid_init(&steer_pid, g_params.kp, g_params.ki, g_params.kd,
             100.0, g_params.alpha);

    // const double PIXY_CENTER_X = 39.0;
    uint32_t line_lost_counter = 0;

    PRINTF("NXP Board Booted. Send commands via ESP32 Wi-Fi (port 3333).\r\n");

    while (1)
    {
        command_poll();

        steer_pid.kp    = g_params.kp;
        steer_pid.ki    = g_params.ki;
        steer_pid.kd    = g_params.kd;
        steer_pid.alpha = g_params.alpha;

        if (g_params.state == STATE_RUNNING)
        {
        //     if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success)
        //     {
        //         if (num_vectors > 0)
        //         {
        //             line_lost_counter = 0;

        //             size_t primary_idx = 0;
        //             uint16_t max_y0 = 0;

        //             for (size_t i = 0; i < num_vectors; i++) {
        //                 uint16_t y0 = vectors[4 * i + 1];
        //                 if (y0 >= max_y0) {
        //                     max_y0 = y0;
        //                     primary_idx = i;
        //                 }
        //             }

        //             uint16_t x0 = vectors[4 * primary_idx + 0];
        //             uint16_t y0 = vectors[4 * primary_idx + 1];
        //             uint16_t x1 = vectors[4 * primary_idx + 2];
        //             uint16_t y1 = vectors[4 * primary_idx + 3];

        //             double cte = (double)x0 - PIXY_CENTER_X;

        //             double dx = (double)x1 - (double)x0;
        //             double dy = (double)y0 - (double)y1;
        //             double heading_deg = 0.0;
        //             if (dy != 0.0) {
        //                 heading_deg = atan2(dx, dy) * (180.0 / M_PI);
        //             } else {
        //                 heading_deg = (dx >= 0) ? 90.0 : -90.0;
        //             }

        //             double measurement = (g_params.k_cte * cte) + (g_params.k_heading * heading_deg);
        //             double setpoint = 0.0;

        //             double steering_output = pid_update(&steer_pid, setpoint, measurement, g_params.dt);

        //             Steer(steering_output + g_params.steering_offset);

        //             double speed_left, speed_right;
        //             compute_motor_speeds(steering_output, &speed_left, &speed_right);
        //             HbridgeSpeed(&g_hbridge, (int16_t)speed_left, (int16_t)speed_right);
        //         }
        //         else
        //         {
        //             line_lost_counter++;
        //             if (line_lost_counter > 5) {
        //                 HbridgeBrake(&g_hbridge);
        //             }
        //         }
        //     }
        // }
        //else
        //{
            HbridgeSpeed(&g_hbridge,g_params.base_speed_left,g_params.base_speed_right);
            
        }
        

        if (button_pressed_edge()) {
            g_params.state = STATE_IDLE;
            HbridgeBrake(&g_hbridge);
            PRINTF("Button pressed: STOPPED\r\n");
        }
    }
    

}
