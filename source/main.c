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

void SysTick_Handler(void) {
    g_tickFlag = true;
}

int main(void) {
    /* 1. Hardware initializations */
    BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    /* Ensure all DMA & Flexcomm interrupts for LP_FLEXCOMM3 are enabled in NVIC */
    NVIC_SetPriority(EDMA_0_CH2_IRQn, 3);
    EnableIRQ(EDMA_0_CH2_IRQn);
    NVIC_SetPriority(EDMA_0_CH3_IRQn, 3);
    EnableIRQ(EDMA_0_CH3_IRQn);
    NVIC_SetPriority(LP_FLEXCOMM3_IRQn, 3);
    EnableIRQ(LP_FLEXCOMM3_IRQn);

    params_init();
    buttons_init();

    /* 2. Actuators safe state */
    HbridgeInit(&g_hbridge, CTIMER0_PERIPHERAL, CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL, CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U, GPIO0, 27U);
    HbridgeBrake(&g_hbridge);
    Steer(g_params.steering_offset);

    /* 3. Camera initialization */
    SDK_DelayAtLeastUs(2000000, SystemCoreClock);

    /* PID Camera: LP_FLEXCOMM3 (Header J5) - Top/Homography view */
    pixy_t cam_pid;
    pixy_init(&cam_pid, LP_FLEXCOMM3_PERIPHERAL, 0x54U, &LP_FLEXCOMM3_RX_Handle, &LP_FLEXCOMM3_TX_Handle);
    pixy_set_led(&cam_pid, 0, 255, 0); /* Green LED */

    /* Speed / Lookahead Camera: LP_FLEXCOMM2 - Forward view */
    pixy_t cam_front;
    pixy_init(&cam_front, LP_FLEXCOMM2_PERIPHERAL, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam_front, 0, 0, 255); /* Blue LED */

    /* 4. 50 Hz control loop (20ms) */
    SysTick_Config(SystemCoreClock / 50U);

    /* Control & State Variables */
    uint16_t vectors_pid[MAX_VECTORS * 4];
    size_t   num_vectors_pid          = 0;
    uint16_t vectors_front[MAX_VECTORS * 4];
    size_t   num_vectors_front        = 0;

    float    filtered_cte             = 0.0f;
    float    filtered_heading         = 0.0f;
    float    filtered_front_angle     = 0.0f;

    float    applied_speed_left       = g_params.speed_straight;
    float    applied_speed_right      = g_params.speed_straight;
    float    steer_cmd                = 0.0f;
    float    last_steer_cmd           = 0.0f;
    uint32_t s_rollout_ticks          = 0;
    uint32_t s_off_track_ticks        = 0;

    /* Intersection Tactical State */
    uint8_t  s_intersection_ahead      = 0;
    uint32_t s_intersection_timer      = 0; /* Persistence timer (~15 ticks = 300ms) */
    uint32_t s_intersection_gap_ticks  = 0; /* Elapsed ticks in blind gap */
    uint8_t  s_was_in_intersection_gap = 0; /* Flag set during intersection blind gap */
    uint32_t s_reacquire_ticks         = 0; /* Slew-rate limiter ticks after blind gap */
    uint8_t  s_telemetry_div          = 0; /* 12.5 Hz telemetry divider */

    PRINTF("Running. Waiting for Start.\r\n");

    while (1) {
        command_poll();

        if (g_tickFlag) {
            g_tickFlag = false;

            /* Hardware push button toggle */
            if (button_pressed_edge()) {
                motor_test_stop();
                if (g_params.state == STATE_IDLE) {
                    g_params.state = STATE_RUNNING;
                    PRINTF("Button: STARTED\r\n");
                } else {
                    g_params.state = STATE_IDLE;
                    HbridgeBrake(&g_hbridge);
                    PRINTF("Button: STOPPED\r\n");
                }
            }

            /* Idle state handling */
            if (g_params.state != STATE_RUNNING) {
                reset_steering_tracking();
                filtered_cte              = 0.0f;
                filtered_heading          = 0.0f;
                filtered_front_angle      = 0.0f;
                applied_speed_left        = g_params.speed_straight;
                applied_speed_right       = g_params.speed_straight;
                steer_cmd                 = 0.0f;
                last_steer_cmd            = 0.0f;
                s_rollout_ticks           = 0;
                s_off_track_ticks         = 0;
                s_intersection_ahead      = 0;
                s_intersection_timer      = 0;
                s_intersection_gap_ticks  = 0;
                s_was_in_intersection_gap = 0;
                s_reacquire_ticks         = 0;

                if (motor_test_is_active()) {
                    motor_test_tick();
                } else {
                    HbridgeBrake(&g_hbridge);
                }
                continue;
            }

            /* --- 1. Forward Horizon View Camera (LP_FLEXCOMM2) --- */
            status_t s_front = pixy_get_vectors(&cam_front, vectors_front, MAX_VECTORS, &num_vectors_front);
            horizon_result_t horizon;
            memset(&horizon, 0, sizeof(horizon));
            if (s_front == kStatus_Success && num_vectors_front > 0) {
                compute_front_camera_horizon(vectors_front, num_vectors_front, &horizon);
                if (horizon.found > 0) {
                    filtered_front_angle = 0.40f * horizon.horizon_angle_deg + 0.60f * filtered_front_angle;
                } else {
                    filtered_front_angle *= 0.85f;
                }
            } else {
                num_vectors_front = 0;
                filtered_front_angle *= 0.85f;
            }

            /* --- 2. Top / Homography View Camera (LP_FLEXCOMM3) --- */
            status_t s_pid = pixy_get_vectors(&cam_pid, vectors_pid, MAX_VECTORS, &num_vectors_pid);
            steering_result_t sr;
            memset(&sr, 0, sizeof(sr));
            if (s_pid == kStatus_Success && num_vectors_pid > 0) {
                compute_top_camera_lane(vectors_pid, num_vectors_pid, &sr);
                if (sr.found > 0) {
                    filtered_cte     = 0.50f * sr.cte + 0.50f * filtered_cte;
                    filtered_heading = 0.50f * sr.heading_deg + 0.50f * filtered_heading;
                }
            } else {
                num_vectors_pid = 0;
            }

            s_rollout_ticks++;

            /* --- Raw Vector Debug Logging over Serial/WiFi --- */
            if (g_dump_vectors && (s_telemetry_div == 0 || g_dump_vectors == 2)) {
                char dump_buf[256];
                int dn = snprintf(dump_buf, sizeof(dump_buf), "RAW_TOP[%u]:", (unsigned)num_vectors_pid);
                for (size_t vi = 0; vi < num_vectors_pid && dn < (int)sizeof(dump_buf) - 20; vi++) {
                    dn += snprintf(dump_buf + dn, sizeof(dump_buf) - (size_t)dn, " (%u,%u,%u,%u)",
                                   vectors_pid[4*vi], vectors_pid[4*vi+1], vectors_pid[4*vi+2], vectors_pid[4*vi+3]);
                }
                dn += snprintf(dump_buf + dn, sizeof(dump_buf) - (size_t)dn, "\r\nRAW_FRN[%u]:", (unsigned)num_vectors_front);
                for (size_t vi = 0; vi < num_vectors_front && dn < (int)sizeof(dump_buf) - 20; vi++) {
                    dn += snprintf(dump_buf + dn, sizeof(dump_buf) - (size_t)dn, " (%u,%u,%u,%u)",
                                   vectors_front[4*vi], vectors_front[4*vi+1], vectors_front[4*vi+2], vectors_front[4*vi+3]);
                }
                snprintf(dump_buf + dn, sizeof(dump_buf) - (size_t)dn, "\r\n");
                esp_printf("%s", dump_buf);
                if (g_dump_vectors == 2) {
                    g_dump_vectors = 0; /* Auto-clear one-shot */
                }
            }

            /* --- Corner Priority Preemption & Tactical Intersection State --- */
            if (horizon.is_sharp_corner || sr.transverse_crosses_center) {
                /* Corner always preempts intersection immediately! */
                s_intersection_ahead      = 0;
                s_intersection_timer      = 0;
                s_intersection_gap_ticks  = 0;
                s_was_in_intersection_gap = 0;
                s_reacquire_ticks         = 0; /* Clear reacquisition slew limit for immediate cornering response */
            } else if (horizon.is_intersection_ahead) {
                /* Intersection Latching & Lifetime: refresh persistence timer (~15 ticks) */
                s_intersection_ahead = 1;
                s_intersection_timer = 15;
            } else if (s_intersection_timer > 0) {
                s_intersection_timer--;
                if (s_intersection_timer == 0 && s_intersection_gap_ticks == 0) {
                    s_intersection_ahead = 0;
                }
            }

            /* --- Blind Gap Policy (sr.found == 0) & Failsafe Guard --- */
            if (sr.found == 0) {
                s_off_track_ticks++;
                if (s_off_track_ticks >= (uint32_t)OFF_TRACK_FAILSAFE_TICKS) {
                    g_params.state      = STATE_IDLE;
                    HbridgeBrake(&g_hbridge);
                    Steer(g_params.steering_offset);
                    applied_speed_left  = 0.0f;
                    applied_speed_right = 0.0f;
                    steer_cmd           = 0.0f;
                    last_steer_cmd      = 0.0f;
                    s_off_track_ticks   = 0;
                    PRINTF("FAILSAFE: Off track for 2.5s -> STOPPED\r\n");
                    esp_printf("ALERT: FAILSAFE OFF TRACK STOP\r\n");
                    continue;
                }

                if (s_intersection_ahead) {
                    /* Intersection Blind Gap Active */
                    s_was_in_intersection_gap = 1;
                    s_intersection_gap_ticks++;

                    /* Distance-scaled window: gap_ticks = 1.5 * 25 / max(speed, 30) * 50 */
                    float current_spd = fmaxf(applied_speed_left, applied_speed_right);
                    uint32_t gap_window = (uint32_t)(1.5f * 25.0f / fmaxf(current_spd, 30.0f) * 50.0f);
                    if (s_intersection_gap_ticks > gap_window) {
                        s_intersection_ahead     = 0;
                        s_intersection_gap_ticks = 0;
                    }

                    /* Decay CTE and heading across the intersection gap */
                    filtered_cte     *= 0.70f;
                    filtered_heading *= 0.70f;
                }
                /* If s_intersection_ahead == 0: Corner apex dropout -> zero decay! */
            } else {
                s_off_track_ticks = 0;
                if (s_was_in_intersection_gap) {
                    /* Exiting intersection blind gap -> trigger Re-Acquisition Slew-Rate Limiter (3 ticks) */
                    s_reacquire_ticks         = 3;
                    s_was_in_intersection_gap = 0;
                    s_intersection_gap_ticks  = 0;
                    s_intersection_ahead      = 0;
                    s_intersection_timer      = 0;
                }
            }

            /* --- 3. Steering Calculation --- */
            float steer_top = 0.0f;
            if (sr.found == 0 && !s_intersection_ahead) {
                /* Corner apex dropout: HOLD LAST STEER! Do NOT decay or clamp to +-3 deg */
                steer_cmd = last_steer_cmd;
                steer_top = last_steer_cmd;
            } else {
                /* Top Camera Centering between closest lines */
                steer_top = (g_params.k_cte * filtered_cte) + (g_params.k_heading * filtered_heading);

                steer_cmd = steer_top;
                if (horizon.is_sharp_corner) {
                    steer_cmd += (float)FRONT_FEEDFORWARD_GAIN * filtered_front_angle;
                }

                if (s_intersection_ahead && sr.found == 0) {
                    /* Intersection blind gap: clamp held steering to +-3.0 deg immediately after PID calculation */
                    if (steer_cmd > 3.0f)  steer_cmd = 3.0f;
                    if (steer_cmd < -3.0f) steer_cmd = -3.0f;
                }

                /* Re-Acquisition Slew-Rate Limiter: <= 4.0 deg/tick for 3 ticks */
                if (s_reacquire_ticks > 0) {
                    s_reacquire_ticks--;
                    float delta = steer_cmd - last_steer_cmd;
                    if (delta > 4.0f) {
                        steer_cmd = last_steer_cmd + 4.0f;
                    } else if (delta < -4.0f) {
                        steer_cmd = last_steer_cmd - 4.0f;
                    }
                }
            }

            /* Rollout protection for first 5 ticks */
            if (s_rollout_ticks <= 5) {
                steer_cmd = 0.0f;
            }

            /* Servo Clamping & Actuation */
            if (steer_cmd > (float)STEER_CMD_MAX_RIGHT) steer_cmd = (float)STEER_CMD_MAX_RIGHT;
            if (steer_cmd < (float)STEER_CMD_MAX_LEFT)  steer_cmd = (float)STEER_CMD_MAX_LEFT;
            Steer(steer_cmd + g_params.steering_offset);
            last_steer_cmd = steer_cmd;

            /* --- 4. Speed & Differential Braking --- */
            float target_speed = g_params.speed_straight;
            float target_left  = target_speed;
            float target_right = target_speed;

            if (horizon.is_sharp_corner) {
                /* SHARP CORNER MODE: Drop speed to corner speed + active differential pivot ONLY if steering past deadzone */
                target_speed = g_params.speed_outside_corner;
                if (fabsf(steer_cmd) > (float)DIFFERENTIAL_DEADZONE_DEG) {
                    if (steer_cmd > 0.0f) {
                        /* Turning Right: Left is outside, Right is inside pivot */
                        target_left  = g_params.speed_outside_corner;
                        target_right = g_params.speed_inside_min;
                    } else {
                        /* Turning Left: Right is outside, Left is inside pivot */
                        target_left  = g_params.speed_inside_min;
                        target_right = g_params.speed_outside_corner;
                    }
                } else {
                    /* Wheels are straight (|steer| <= 10 deg): equal duty, zero yaw torque induced! */
                    target_left  = target_speed;
                    target_right = target_speed;
                }
            } else {
                /* STRAIGHT / INTERSECTION / MILD CURVE MODE: 100% full throttle */
                target_speed = g_params.speed_straight;
                float abs_steer = fabsf(steer_cmd);
                if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
                    float steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                                       ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
                    if (steer_norm > 1.0f) steer_norm = 1.0f;
                    float diff_drag = steer_norm * (target_speed - g_params.speed_inside_min);

                    if (steer_cmd > 0.0f) {
                        target_left  = target_speed;
                        target_right = target_speed - diff_drag;
                    } else {
                        target_left  = target_speed - diff_drag;
                        target_right = target_speed;
                    }
                } else {
                    target_left  = target_speed;
                    target_right = target_speed;
                }
            }

            /* --- 5. Motor Slew Rate Limiting (Ramps) --- */
            /* Left Motor Ramp */
            if (target_left < applied_speed_left) {
                applied_speed_left -= g_params.decel_ramp;
                if (applied_speed_left < target_left) applied_speed_left = target_left;
            } else {
                applied_speed_left += g_params.accel_ramp;
                if (applied_speed_left > target_left) applied_speed_left = target_left;
            }

            /* Right Motor Ramp */
            if (target_right < applied_speed_right) {
                applied_speed_right -= g_params.decel_ramp;
                if (applied_speed_right < target_right) applied_speed_right = target_right;
            } else {
                applied_speed_right += g_params.accel_ramp;
                if (applied_speed_right > target_right) applied_speed_right = target_right;
            }

            /* --- 6. Motor Actuation --- */
            if (!g_params.motor_enable) {
                applied_speed_left  = 0.0f;
                applied_speed_right = 0.0f;
                HbridgeBrake(&g_hbridge);
            } else {
                HbridgeSpeed(&g_hbridge, applied_speed_left, applied_speed_right);
            }

            /* --- 7. Telemetry Output (12.5 Hz = Every 4th Tick) --- */
            if (++s_telemetry_div >= 4) {
                s_telemetry_div = 0;
                esp_printf("L:%u/%u R:%d F:%d H:%d TS:%d FS:%d S:%d BW:%d DOM:%c FV:%u IX:%u ST:%s VL:%d VR:%d\r\n",
                           (unsigned)sr.found, (unsigned)horizon.found,
                           (int)(sr.cte * 10.0f), (int)(filtered_cte * 10.0f),
                           (int)(filtered_heading * 10.0f),
                           (int)(steer_top * 10.0f),
                           (int)(filtered_front_angle * 10.0f),
                           (int)(steer_cmd * 10.0f),
                           (int)(horizon.is_sharp_corner ? 0 : 100),
                           horizon.is_sharp_corner ? 'F' : 'T',
                           horizon.found > 0 ? 1U : 0U,
                           (unsigned)s_intersection_ahead,
                           s_intersection_ahead ? "I" : (horizon.is_sharp_corner ? "C" : "S"),
                           (int)applied_speed_left, (int)applied_speed_right);
            }
        }
    }
}
