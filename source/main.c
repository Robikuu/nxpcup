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

    char comanda[50];
    volatile int INDEXXXXX;
    INDEXXXXX = 0;
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
    PRINTF("NXP Board Booted. Waiting for ESP32 Wi-Fi...\r\n");

    while (1) {
            uint8_t received_char;

            // Check the LPUART status flags to see if a new byte has arrived
            uint32_t flags = LPUART_GetStatusFlags(LP_FLEXCOMM9_PERIPHERAL);

            if (flags & kLPUART_RxDataRegFullFlag) {
                // Read the incoming character from the ESP
                received_char = LPUART_ReadByte(LP_FLEXCOMM9_PERIPHERAL);

                if (received_char == ';') {
                // Only process if the string isn't empty
                    if (INDEXXXXX > 0) {
                        PRINTF("\r\nCommand received: %s\r\n", comanda);
                        
                        if(!strcmp(comanda, "steer")) {
                            Steer(100);
                        }
                        if(!strcmp(comanda, "steer1")) {
							Steer(-100);
						}
                        INDEXXXXX = 0;
                        comanda[0] = '\0'; 
                    }
                } 
                // Ignore hidden enter key presses from the terminal
                else if (received_char != '\n' && received_char != '\r') {
                    
                    // Prevent buffer overflow by checking the length
                    if (INDEXXXXX < (50 - 1)) {
                        
                        // Put the character in the array and increment the index
                        comanda[INDEXXXXX] = received_char;
                        INDEXXXXX++;
                        
                        // Always cap the string with a null terminator so PRINTF and strcmp work safely
                        comanda[INDEXXXXX] = '\0';
                    }
                }
            }
        }

    pixy_t cam1;
    pixy_init(&cam1, LPI2C2, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam1, 255, 0, 0);

    pid_struct steer_pid;
    // pid_init signature: (PID, kp, ki, kd, output_limit, alpha)
    pid_init(&steer_pid, /*kp*/ 0.8, /*ki*/ 0.0, /*kd*/ 0.1,
             /*output_limit*/ 100.0, /*alpha*/ 0.2);

    const double PIXY_CENTER_X = 39.0; // Pixy2 Line Tracking frame X center (0..78)
    const double K_CTE         = 1.2;  // Weight for Cross-Track (position) Error
    const double K_HEADING     = 0.8;  // Weight for Heading Angle Error

    uint32_t line_lost_counter = 0;

    while (1) 
    {
        if (pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors) == kStatus_Success) {
            if (num_vectors > 0) {
                line_lost_counter = 0;

                // 1. Primary Vector Selection (find the vector starting closest to the bottom of camera frame)
                size_t primary_idx = 0;
                uint16_t max_y0 = 0;

                for (size_t i = 0; i < num_vectors; i++) {
                    uint16_t y0 = vectors[4 * i + 1];
                    if (y0 >= max_y0) {
                        max_y0 = y0;
                        primary_idx = i;
                    }
                }

                uint16_t x0 = vectors[4 * primary_idx + 0];
                uint16_t y0 = vectors[4 * primary_idx + 1];
                uint16_t x1 = vectors[4 * primary_idx + 2];
                uint16_t y1 = vectors[4 * primary_idx + 3];

                // 2. Cross-Track Error (CTE): distance from screen center (positive = line is to the right)
                double cte = (double)x0 - PIXY_CENTER_X;

                // 3. Heading Angle (deg): direction of vector relative to vertical axis
                double dx = (double)x1 - (double)x0;
                double dy = (double)y0 - (double)y1; // positive going up
                double heading_deg = 0.0;
                if (dy != 0.0) {
                    heading_deg = atan2(dx, dy) * (180.0 / M_PI);
                } else {
                    heading_deg = (dx >= 0) ? 90.0 : -90.0;
                }

                // 4. Enhanced Combined Error Metric
                // Positive measurement means car needs to steer right to compensate
                double measurement = (K_CTE * cte) + (K_HEADING * heading_deg);
                double setpoint = 0.0;
                double dt = 0.02; // ~50Hz nominal loop

                double steering_output = pid_update(&steer_pid, setpoint, measurement, dt);

                // 5. Apply Steering and Motor Differential Speeds
                Steer(steering_output + STEERING_OFFSET);

                double speed_left, speed_right;
                compute_motor_speeds(steering_output, &speed_left, &speed_right);
                HbridgeSpeed(&g_hbridge, speed_left, speed_right);

            } else {
                // Line lost handling
                line_lost_counter++;
                if (line_lost_counter > 5) {
                    HbridgeBrake(&g_hbridge); // Stop if line lost for > 5 consecutive frames
                }
            }
        }
    }
}
