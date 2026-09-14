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
#include "fsl_ctimer.h"
#include "fsl_mrt.h"

#define MAX_VECTORS 10

volatile bool g_tickFlag = false;
robot_params_t g_params;

typedef enum {
    DRIVE_FORWARD,
    DRIVE_TURNING,
    DRIVE_BRAKING
} drive_state_t;

void SysTick_Handler(void) {
    g_tickFlag = true;
}

static void reset_pid_state(pid_struct *pid) {
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filtered       = 0.0f;
    pid->prev_output      = 0.0f;
    pid->initialized      = 0;
}

int main(void) {
    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    params_init();
    buttons_init();

    HbridgeInit(&g_hbridge, CTIMER0_PERIPHERAL, CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL, CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U, GPIO0, 27U);
    HbridgeBrake(&g_hbridge);
    Steer(0.0f);

    static pid_struct steer_pid;
    // Initial PID init, gains get dynamically overwritten in the loop below
    pid_init(&steer_pid, g_params.kp_corner, g_params.ki, g_params.kd_corner, 100.0f, g_params.alpha);

    SDK_DelayAtLeastUs(2000000, SystemCoreClock);
    pixy_t cam1;
    pixy_init(&cam1, LPI2C2, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam1, 0, 255, 0);   

    SysTick_Config(SystemCoreClock / 50U);

    uint16_t vectors[MAX_VECTORS * 4];
    size_t   num_vectors;
    uint32_t line_lost_counter = 0;
    float    filtered_cte      = 0.0f;
    int      cte_filter_init   = 0;
    
    // Start at corner speed
    float    applied_speed     = g_params.speed_corner;
    float    steer_cmd         = 0.0f;
    drive_state_t drive_state  = DRIVE_FORWARD;
    uint32_t      state_streak = 0;

    PRINTF("Running. Waiting for Start.\r\n");

    while (1) {
        command_poll();

        if (g_tickFlag) {
            g_tickFlag = false;

            if (button_pressed_edge()) {
                if (g_params.state == STATE_IDLE) {
                    g_params.state = STATE_RUNNING;
                    PRINTF("Button: STARTED\r\n");
                } else {
                    g_params.state = STATE_IDLE;
                    HbridgeBrake(&g_hbridge);
                    PRINTF("Button: STOPPED\r\n");
                }
            }

            if (g_params.state != STATE_RUNNING) {
                reset_pid_state(&steer_pid);
                line_lost_counter = 0;
                filtered_cte      = 0.0f;
                cte_filter_init   = 0;
                applied_speed     = g_params.speed_corner;
                steer_cmd         = 0.0f;
                drive_state       = DRIVE_FORWARD;
                state_streak      = 0;
                HbridgeBrake(&g_hbridge);
                continue;
            }

            status_t s = pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors);
            if (s != kStatus_Success || num_vectors == 0) {
                if (++line_lost_counter > LINE_LOST_LIMIT) {
                    HbridgeBrake(&g_hbridge);
                }
                continue;
            }
            line_lost_counter = 0;

            vector_filtering(vectors, &num_vectors);
            steering_result_t sr = compute_steering_error(vectors, num_vectors);

            drive_state_t desired_state = DRIVE_BRAKING;
            if (sr.found >= 2) {
                desired_state = (fabsf(sr.heading_deg) < CORNER_HEADING_THRESHOLD_DEG) ? DRIVE_FORWARD : DRIVE_TURNING;
            } else if (sr.found == 1) {
                desired_state = DRIVE_TURNING;
            }

            if (desired_state == drive_state) {
                state_streak = 0;
            } else {
                state_streak++;
                uint32_t required = (desired_state == DRIVE_FORWARD) ? STATE_EXIT_STREAK : STATE_ENTER_STREAK;
                if (state_streak >= required) {
                    if (desired_state == DRIVE_BRAKING && drive_state != DRIVE_BRAKING) {
                        reset_pid_state(&steer_pid);
                        filtered_cte    = 0.0f;
                        cte_filter_init = 0;
                        applied_speed   = g_params.speed_corner;
                        steer_cmd       = 0.0f;
                    }
                    drive_state  = desired_state;
                    state_streak = 0;
                }
            }

            if (sr.found > 0 && drive_state != DRIVE_BRAKING) {
                if (!cte_filter_init) {
                    filtered_cte    = sr.cte;
                    cte_filter_init = 1;
                } else {
                    filtered_cte = 0.4f * sr.cte + 0.6f * filtered_cte;
                }

                float heading_ratio = fabsf(sr.heading_deg) / HEADING_FULL_BRAKE_DEG;
                if (heading_ratio > 1.0f) heading_ratio = 1.0f;
                
                float target_speed = g_params.speed_straight - (heading_ratio * (g_params.speed_straight - g_params.speed_corner));
                
                if (target_speed < applied_speed) {
                    applied_speed = target_speed;
                } else {
                    applied_speed += g_params.accel_ramp; // Replaced ACCEL_RAMP_STEP
                    if (applied_speed > target_speed) {
                        applied_speed = target_speed;
                    }
                }

                // Protect against division by zero in case speed_straight == speed_corner
                float speed_diff = g_params.speed_straight - g_params.speed_corner;
                float speed_ratio = 0.0f;
                if (speed_diff > 0.001f) {
                    speed_ratio = (applied_speed - g_params.speed_corner) / speed_diff;
                }
                
                if (speed_ratio < 0.0f) speed_ratio = 0.0f;
                if (speed_ratio > 1.0f) speed_ratio = 1.0f;
                
                // Write the dynamic gains back to the PID struct
                steer_pid.kp = g_params.kp_corner - speed_ratio * (g_params.kp_corner - g_params.kp_straight);
                steer_pid.ki = g_params.ki;
                steer_pid.kd = g_params.kd_corner + speed_ratio * (g_params.kd_straight - g_params.kd_corner);
                steer_pid.alpha = g_params.alpha; 

                // Calculate maximum symmetric travel so left and right turns behave identically
                float max_right_travel = MAX_STEER - g_params.steering_offset;
                float max_left_travel  = MAX_STEER + g_params.steering_offset;
                float sym_limit = (max_right_travel < max_left_travel) ? max_right_travel : max_left_travel;

                float combined = g_params.k_cte * filtered_cte + g_params.k_heading * sr.heading_deg;
                steer_cmd = -pid_update(&steer_pid, 0.0f, combined, g_params.dt);

                // Clamp the algorithm to the safe symmetric limit
                if (steer_cmd > sym_limit) steer_cmd = sym_limit;
                if (steer_cmd < -sym_limit) steer_cmd = -sym_limit;
                
                Steer(steer_cmd + g_params.steering_offset);

                // Use the symmetric limit for ratio calculations instead of MAX_STEER
                float steer_ratio = fabsf(steer_cmd) / sym_limit;
                if (steer_ratio > 1.0f) steer_ratio = 1.0f;
                if (steer_ratio > heading_ratio) {
                    float steer_target = g_params.speed_straight - (steer_ratio * (g_params.speed_straight - g_params.speed_corner));
                    if (steer_target < applied_speed) {
                        applied_speed = steer_target;
                    }
                }

                // Balance the electronic differential using the symmetric limit
                // Balance the electronic differential using the symmetric limit
                float diff_offset = (steer_cmd / sym_limit) * (applied_speed * g_params.differential_gain); // Replaced DIFFERENTIAL_GAIN
                int16_t speed_left  = (int16_t)(applied_speed + diff_offset);
                int16_t speed_right = (int16_t)(applied_speed - diff_offset);
                
                HbridgeSpeed(&g_hbridge, speed_left, speed_right);

                if (drive_state == DRIVE_FORWARD) {
                    esp_printf("L:%u R:%d F:%d S:%d ST:F\r\n", (unsigned)sr.found, (int)(sr.cte * 10.0f), (int)(filtered_cte * 10.0f), (int)(steer_cmd * 10.0f));
                } else {
                    esp_printf("L:%u R:%d F:%d S:%d ST:T (corner)\r\n", (unsigned)sr.found, (int)(sr.cte * 10.0f), (int)(filtered_cte * 10.0f), (int)(steer_cmd * 10.0f));
                }

            } else if (drive_state == DRIVE_BRAKING) {
                esp_printf("L:%u ST:B (braking)\r\n", (unsigned)sr.found);
                Steer(g_params.steering_offset);
                HbridgeBrake(&g_hbridge);
                applied_speed = g_params.speed_corner;
                steer_cmd     = 0.0f;
            }
        }
    }
}