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

    /* 3. PID Controller initialization */
    static pid_struct steer_pid;
    pid_init(&steer_pid, g_params.kp_corner, g_params.ki, g_params.kd_corner, 100.0f, g_params.alpha);

    /* 4. Camera initialization */
    SDK_DelayAtLeastUs(2000000, SystemCoreClock);

    /* PID Camera: LP_FLEXCOMM3 (Header J5) - Top/Homography view */
    pixy_t cam_pid;
    pixy_init(&cam_pid, LP_FLEXCOMM3_PERIPHERAL, 0x54U, &LP_FLEXCOMM3_RX_Handle, &LP_FLEXCOMM3_TX_Handle);
    pixy_set_led(&cam_pid, 0, 255, 0); /* Green LED */

    /* Speed / Lookahead Camera: LP_FLEXCOMM2 - Forward view */
    pixy_t cam_front;
    pixy_init(&cam_front, LP_FLEXCOMM2_PERIPHERAL, 0x54U, &LP_FLEXCOMM2_RX_Handle, &LP_FLEXCOMM2_TX_Handle);
    pixy_set_led(&cam_front, 0, 0, 255); /* Blue LED */

    /* 5. 50 Hz control loop (20ms) */
    SysTick_Config(SystemCoreClock / 50U);

    /* 6. State variables */
    uint16_t vectors_pid[MAX_VECTORS * 4];
    size_t   num_vectors_pid         = 0;
    uint16_t vectors_front[MAX_VECTORS * 4];
    size_t   num_vectors_front       = 0;
    trajectory_result_t g_traj;
    memset(&g_traj, 0, sizeof(g_traj));
    g_traj.lookahead_x       = (float)PIXY_CENTER_X;
    float    filtered_traj_angle     = 0.0f;
    float    filtered_traj_offset    = 0.0f;
    float    lookahead_turn_factor   = 0.0f;
    float    s_last_valid_lookahead_x = (float)PIXY_CENTER_X;
    uint32_t s_front_dropout_ticks    = 0;
    uint32_t line_lost_counter       = 0;
    float    filtered_cte            = 0.0f;
    int      cte_filter_init         = 0;
    float    filtered_heading        = 0.0f;
    int      heading_filter_init     = 0;
    float    applied_speed_left      = g_params.speed_straight;
    float    applied_speed_right     = g_params.speed_straight;
    float         steer_cmd               = 0.0f;
    float         last_steer_cmd          = 0.0f;
    drive_state_t drive_state             = DRIVE_FORWARD;
    uint32_t      state_streak            = 0;
    int8_t        streak_sign             = 0;
    uint32_t      urgency_streak          = 0;
    float         trusted_corner_urgency  = 0.0f;

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
                reset_pid_state(&steer_pid);
                reset_steering_tracking();
                line_lost_counter         = 0;
                filtered_cte              = 0.0f;
                cte_filter_init           = 0;
                filtered_heading          = 0.0f;
                heading_filter_init       = 0;
                filtered_traj_angle       = 0.0f;
                filtered_traj_offset      = 0.0f;
                lookahead_turn_factor     = 0.0f;
                s_last_valid_lookahead_x  = (float)PIXY_CENTER_X;
                s_front_dropout_ticks     = 0;
                memset(&g_traj, 0, sizeof(g_traj));
                g_traj.lookahead_x        = (float)PIXY_CENTER_X;
                applied_speed_left        = g_params.speed_straight;
                applied_speed_right       = g_params.speed_straight;
                steer_cmd                 = 0.0f;
                last_steer_cmd            = 0.0f;
                drive_state               = DRIVE_FORWARD;
                state_streak              = 0;
                streak_sign               = 0;
                urgency_streak            = 0;
                trusted_corner_urgency    = 0.0f;
                if (motor_test_is_active()) {
                    motor_test_tick();
                } else {
                    HbridgeBrake(&g_hbridge);
                }
                continue;
            }

            /* Trajectory Camera polling (LP_FLEXCOMM2 - 50 Hz every tick) */
            status_t s_front = pixy_get_vectors(&cam_front, vectors_front, MAX_VECTORS, &num_vectors_front);
            if (s_front == kStatus_Success && num_vectors_front > 0) {
                g_traj = compute_lookahead_trajectory(vectors_front, num_vectors_front);
            } else {
                memset(&g_traj, 0, sizeof(g_traj));
                g_traj.lookahead_x = (float)PIXY_CENTER_X;
            }

            /* 50 Hz Foreground Lookahead & Dropout Hold Filter */
            if (g_traj.found >= 1) {
                s_last_valid_lookahead_x = g_traj.lookahead_x;
                s_front_dropout_ticks    = 0;
                filtered_traj_angle      = 0.50f * g_traj.trajectory_angle_deg + 0.50f * filtered_traj_angle;
                filtered_traj_offset     = 0.50f * g_traj.trajectory_offset_px  + 0.50f * filtered_traj_offset;
                lookahead_turn_factor    = 0.50f * g_traj.lookahead_urgency     + 0.50f * lookahead_turn_factor;
            } else {
                /* Zero root vectors detected: 1-3 tick hold-last-valid filter */
                s_front_dropout_ticks++;
                filtered_traj_angle  *= 0.80f;
                filtered_traj_offset *= 0.80f;
                if (s_front_dropout_ticks > (uint32_t)LOOKAHEAD_DROPOUT_HOLD_TICKS) {
                    /* Extended dropout: check top-camera sanity guard */
                    if (fabsf(filtered_heading) <= 3.0f && fabsf(filtered_cte) <= 4.0f) {
                        /* Straightaway sanity veto: vehicle is dead-center on a straight! */
                        lookahead_turn_factor *= 0.80f;
                    } else {
                        /* Curve confirmed: line lost in corner */
                        lookahead_turn_factor = 0.60f;
                    }
                }
            }

            /* PID Camera line tracking (Top/Homography view) */
            status_t s = pixy_get_vectors(&cam_pid, vectors_pid, MAX_VECTORS, &num_vectors_pid);
            if (s != kStatus_Success || num_vectors_pid == 0) {
                urgency_streak         = 0;
                trusted_corner_urgency *= 0.90f;
                lookahead_turn_factor  *= 0.90f;
                if (++line_lost_counter <= LINE_LOST_LIMIT) {
                    /* Dead-reckoning coast through intersection gap without braking */
                    Steer(g_params.steering_offset);
                    HbridgeSpeed(&g_hbridge, (int16_t)applied_speed_left, (int16_t)applied_speed_right);
                } else {
                    drive_state = DRIVE_BRAKING;
                    Steer(g_params.steering_offset);
                    HbridgeBrake(&g_hbridge);
                    reset_steering_tracking();
                    reset_pid_state(&steer_pid);
                    static uint8_t s_brake_div = 0;
                    if (++s_brake_div >= 5) {
                        s_brake_div = 0;
                        esp_printf("L:0 ST:B (braking)\r\n");
                    }
                }
                continue;
            }

            /* Geometric pair validation & filtering on PID camera (Pass 2 rejects crossing lines) */
            vector_filtering(vectors_pid, &num_vectors_pid);

            /* Error computation with perspective synthesis */
            steering_result_t sr = compute_steering_error(vectors_pid, num_vectors_pid);

            if (sr.found == 0) {
                urgency_streak         = 0;
                trusted_corner_urgency *= 0.90f;
                lookahead_turn_factor  *= 0.90f;
                if (++line_lost_counter <= LINE_LOST_LIMIT) {
                    /* Dead-reckoning coast through intersection gap without braking */
                    Steer(g_params.steering_offset);
                    HbridgeSpeed(&g_hbridge, (int16_t)applied_speed_left, (int16_t)applied_speed_right);
                } else {
                    drive_state = DRIVE_BRAKING;
                    Steer(g_params.steering_offset);
                    HbridgeBrake(&g_hbridge);
                    reset_steering_tracking();
                    reset_pid_state(&steer_pid);
                    static uint8_t s_brake_div = 0;
                    if (++s_brake_div >= 5) {
                        s_brake_div = 0;
                        esp_printf("L:0 ST:B (braking)\r\n");
                    }
                }
                continue;
            }

            /* Valid lane lines confirmed: reset line loss tracking */
            line_lost_counter = 0;

            if (!heading_filter_init) {
                filtered_heading    = sr.heading_deg;
                heading_filter_init = 1;
            } else {
                filtered_heading = 0.60f * sr.heading_deg + 0.40f * filtered_heading;
            }

            /* State machine transitions:
             * - DRIVE_TURNING (Corner):
             *   Triggered by local top camera curvature (|filtered_heading| >= CORNER_HEADING_THRESHOLD_DEG),
             *   or lookahead point exceeding enter hysteresis (> 6.0px),
             *   or confirmed sustained line loss outside straight sanity veto.
             * - DRIVE_FORWARD (Straight):
             *   Strictly confirmed when both top camera and front lookahead confirm centering:
             *   |filtered_heading| <= 4.0 deg, |filtered_cte| <= 6.0 px, and lookahead offset <= 4.0 px.
             *   Hysteresis debounce of STATE_EXIT_STREAK (2 ticks = 40ms) guarantees chatter-free transitions. */
            float la_offset = fabsf(s_last_valid_lookahead_x - (float)PIXY_CENTER_X);
            bool corner_trigger = (fabsf(filtered_heading) >= (float)CORNER_HEADING_THRESHOLD_DEG) ||
                                  (la_offset >= (float)LOOKAHEAD_HYSTERESIS_ENTER_PX && s_front_dropout_ticks == 0) ||
                                  (s_front_dropout_ticks > (uint32_t)LOOKAHEAD_DROPOUT_HOLD_TICKS &&
                                   !(fabsf(filtered_heading) <= 3.0f && fabsf(filtered_cte) <= 4.0f));

            drive_state_t desired_state;
            if (drive_state == DRIVE_TURNING) {
                bool straight_confirmed = (fabsf(filtered_heading) <= 4.0f &&
                                           fabsf(filtered_cte) <= 6.0f &&
                                           la_offset <= (float)LOOKAHEAD_HYSTERESIS_EXIT_PX);
                desired_state = straight_confirmed ? DRIVE_FORWARD : DRIVE_TURNING;
            } else {
                desired_state = corner_trigger ? DRIVE_TURNING : DRIVE_FORWARD;
            }

            if (desired_state != drive_state) {
                if (drive_state == DRIVE_FORWARD) {
                    /* Chicane / flick protection: ensure curve direction is sustained in ONE direction */
                    int8_t current_sign = (filtered_heading >= 0.0f) ? 1 : -1;
                    if (current_sign != streak_sign) {
                        streak_sign = current_sign;
                        state_streak = 0;
                    }
                }
                uint32_t threshold = (desired_state == DRIVE_FORWARD) ? (uint32_t)STATE_EXIT_STREAK : (uint32_t)STATE_ENTER_STREAK;
                if (++state_streak >= threshold) {
                    drive_state  = desired_state;
                    state_streak = 0;
                }
            } else {
                state_streak = 0;
                streak_sign  = 0;
            }

            /* Cross-track error low-pass filtering */
                if (!cte_filter_init) {
                    filtered_cte    = sr.cte;
                    cte_filter_init = 1;
                } else {
                    filtered_cte = 0.4f * sr.cte + 0.6f * filtered_cte;
                }

                /* 1. Dynamic PID Gain Scheduling Based on Corner Urgency
                 * Continuous deadband of 2.5 deg on heading: straightaway yaw jitter is strictly ignored. */
                float dz_heading_urgency = fabsf(filtered_heading) - 2.5f;
                float heading_urgency = (dz_heading_urgency > 0.0f) ? (dz_heading_urgency / ((float)HEADING_FULL_BRAKE_DEG - 2.5f)) : 0.0f;
                if (heading_urgency > 1.0f) heading_urgency = 1.0f;

                /* Only let CTE drive corner urgency if CTE enters the outer lane band (>14px) */
                float cte_excess = fabsf(filtered_cte) - 14.0f;
                float cte_urgency = (cte_excess > 0.0f) ? (cte_excess / ((float)CTE_URGENCY_PX - 14.0f)) : 0.0f;
                if (cte_urgency > 1.0f) cte_urgency = 1.0f;

                float raw_corner_urgency = heading_urgency;
                if (cte_urgency > raw_corner_urgency) raw_corner_urgency = cte_urgency;
                if (lookahead_turn_factor > raw_corner_urgency) raw_corner_urgency = lookahead_turn_factor;

                if (raw_corner_urgency > trusted_corner_urgency) {
                    if (++urgency_streak >= (uint32_t)CORNER_URGENCY_STREAK) {
                        trusted_corner_urgency = raw_corner_urgency;
                        urgency_streak = 0;
                    }
                } else {
                    /* Smooth urgency decay: prevents mid-corner dropouts and throttle surges */
                    trusted_corner_urgency = 0.10f * raw_corner_urgency + 0.90f * trusted_corner_urgency;
                    urgency_streak = 0;
                }

                steer_pid.kp    = g_params.kp_straight + trusted_corner_urgency * (g_params.kp_corner - g_params.kp_straight);
                steer_pid.ki    = g_params.ki;
                steer_pid.kd    = g_params.kd_straight + trusted_corner_urgency * (g_params.kd_corner - g_params.kd_straight);
                steer_pid.alpha = g_params.alpha;

                /* 2. Decoupled Steering Authority:
                 * - CTE error is actively damped and regulated by the PID controller.
                 * - Heading error acts as direct, instantaneous feedforward anticipation into curves.
                 * - Lookahead feedforward is STRICTLY ZERO (100% decoupled from servo).
                 * - Slew rate is capped by MAX_STEER_RATE_PER_SEC to prevent sudden chassis snapping and tire scrub. */
                float pid_cte = -pid_update(&steer_pid, 0.0f, g_params.k_cte * filtered_cte, g_params.dt);

                /* Decisive Turn-In & Apex Protection:
                 * When actively negotiating a corner (DRIVE_TURNING) with significant curvature
                 * (|filtered_heading| >= 8.0 deg), if CTE opposes heading (filtered_cte * filtered_heading < 0.0f):
                 * Suppress pid_cte so the vehicle bites cleanly through the apex without understeer!
                 * On straightaways (DRIVE_FORWARD), pid_cte is NEVER suppressed. */
                if (drive_state == DRIVE_TURNING &&
                    (filtered_cte * filtered_heading < 0.0f) &&
                    fabsf(filtered_heading) >= 8.0f) {
                    pid_cte = 0.0f;
                }

                /* Continuous Deadband Heading Feedforward:
                 * On straightaways (|filtered_heading| <= HEADING_DEADBAND_DEG), track curvature is zero;
                 * feedforward is strictly 0.0f, locking the car to centerline via PID derivative damping.
                 * As soon as a curve begins (|filtered_heading| > HEADING_DEADBAND_DEG), full k_heading gain
                 * applies immediately without fractional urgency division or lag! */
                float abs_h = fabsf(filtered_heading);
                float effective_h = 0.0f;
                if (abs_h > (float)HEADING_DEADBAND_DEG) {
                    effective_h = (abs_h - (float)HEADING_DEADBAND_DEG) * ((filtered_heading > 0.0f) ? 1.0f : -1.0f);
                }
                float heading_feedforward = g_params.k_heading * effective_h;

                /* STRICTLY DECOUPLED: Front camera does NOT inject steering angle */
                float raw_steer = pid_cte + heading_feedforward;

                const float max_steer_step = (float)MAX_STEER_RATE_PER_SEC * g_params.dt;
                float steer_diff = raw_steer - last_steer_cmd;
                if (steer_diff > max_steer_step) {
                    raw_steer = last_steer_cmd + max_steer_step;
                } else if (steer_diff < -max_steer_step) {
                    raw_steer = last_steer_cmd - max_steer_step;
                }
                steer_cmd = raw_steer;

                /* Clamp to relative command travel: [-77.5f, +52.5f] */
                if (steer_cmd > (float)STEER_CMD_MAX_RIGHT) {
                    steer_cmd = (float)STEER_CMD_MAX_RIGHT;
                }
                if (steer_cmd < (float)STEER_CMD_MAX_LEFT) {
                    steer_cmd = (float)STEER_CMD_MAX_LEFT;
                }

                /* Slew tracker updated AFTER clamping to prevent windup beyond physical servo travel */
                last_steer_cmd = steer_cmd;

                /* Output to physical servo (offset added exactly once) */
                Steer(steer_cmd + g_params.steering_offset);

                /* 3. Direction-Aware Steering Normalization & Predictive Corner Intensity */
                float abs_steer = fabsf(steer_cmd);
                float dz_steer = abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG;
                if (dz_steer < 0.0f) dz_steer = 0.0f;

                float steer_norm = dz_steer / ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
                if (steer_norm > 1.0f) steer_norm = 1.0f;

                float abs_heading = fabsf(filtered_heading);
                float dz_heading = abs_heading - (float)DIFFERENTIAL_DEADZONE_DEG;
                if (dz_heading < 0.0f) dz_heading = 0.0f;
                float heading_norm = dz_heading / ((float)HEADING_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
                if (heading_norm > 1.0f) heading_norm = 1.0f;

                /* 4. Longitudinal Base Speed & Differential Assignment
                 * In DRIVE_FORWARD (straightaways & rapid chicane flicks):
                 * Maintain full 100% straight sprint speed with zero differential drag.
                 * Proactively pre-brake if front lookahead senses an upcoming corner (> 0.15 urgency).
                 * In DRIVE_TURNING (sustained curves after STATE_ENTER_STREAK confirmation):
                 * Immediately clamp base speed to corner envelope (70% down to 58%) and apply inside wheel pivot drag. */
                float effective_corner_intensity = heading_norm;
                if (lookahead_turn_factor > effective_corner_intensity) {
                    effective_corner_intensity = lookahead_turn_factor;
                }
                float longitudinal_base = g_params.speed_straight;
                float active_steer_norm = 0.0f;
                if (drive_state == DRIVE_TURNING) {
                    float corner_entry_cap = 70.0f;
                    longitudinal_base = corner_entry_cap - effective_corner_intensity * (corner_entry_cap - g_params.speed_outside_corner);
                    active_steer_norm = steer_norm;
                } else if (lookahead_turn_factor > 0.15f) {
                    /* Proactive pre-braking on straightaway approach before entering DRIVE_TURNING */
                    longitudinal_base = g_params.speed_straight - lookahead_turn_factor * (g_params.speed_straight - 70.0f);
                }

                /* Lateral Differential Drag: strictly driven by active physical steering in DRIVE_TURNING */
                float diff_delta = active_steer_norm * (longitudinal_base - g_params.speed_inside_min);
                float target_outside = longitudinal_base;
                float target_inside  = longitudinal_base - diff_delta;

                float target_left    = longitudinal_base;
                float target_right   = longitudinal_base;

                if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
                    if (steer_cmd > 0.0f) {
                        /* Right turn: Left is outside, Right is inside */
                        target_left  = target_outside;
                        target_right = target_inside;
                    } else {
                        /* Left turn: Right is outside, Left is inside */
                        target_left  = target_inside;
                        target_right = target_outside;
                    }
                }

                /* 5. Bilateral Anti-Spin Traction Guard with Dynamic Reversal Override
                 * Chicane / S-curve reversal override:
                 * Only trigger when target speeds actively demand an OPPOSITE turn:
                 * - Steering right, demanding left wheel faster than right (target_left > target_right + 5%),
                 *   while applied speeds are still configured for a left turn (applied_left < applied_right).
                 * - Steering left, demanding right wheel faster than left (target_right > target_left + 5%),
                 *   while applied speeds are still configured for a right turn (applied_right < applied_left).
                 * CRITICAL: On corner exit onto a straight, target_left == target_right, so yaw_conflict
                 * is strictly FALSE. This completely prevents dumping a +35% torque spike into the
                 * unweighted inside rear wheel on exit (eliminates corner-exit wheelspin)! */
                bool yaw_conflict = false;
                if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
                    if (steer_cmd > (float)YAW_CONFLICT_DEADBAND && (target_left - target_right) > 5.0f && applied_speed_left < applied_speed_right) {
                        yaw_conflict = true;
                    } else if (steer_cmd < -(float)YAW_CONFLICT_DEADBAND && (target_right - target_left) > 5.0f && applied_speed_right < applied_speed_left) {
                        yaw_conflict = true;
                    }
                }

                float step_accel_l = (yaw_conflict && target_left > applied_speed_left) ? (float)REVERSAL_RELEASE_STEP : g_params.accel_ramp;
                float step_decel_l = (yaw_conflict && target_left < applied_speed_left) ? (float)REVERSAL_DECEL_STEP   : g_params.decel_ramp;
                float step_accel_r = (yaw_conflict && target_right > applied_speed_right) ? (float)REVERSAL_RELEASE_STEP : g_params.accel_ramp;
                float step_decel_r = (yaw_conflict && target_right < applied_speed_right) ? (float)REVERSAL_DECEL_STEP   : g_params.decel_ramp;

                /* Left wheel rate limiting */
                if (target_left < applied_speed_left) {
                    applied_speed_left -= step_decel_l;
                    if (applied_speed_left < target_left) applied_speed_left = target_left;
                } else {
                    applied_speed_left += step_accel_l;
                    if (applied_speed_left > target_left) applied_speed_left = target_left;
                }

                /* Right wheel rate limiting */
                if (target_right < applied_speed_right) {
                    applied_speed_right -= step_decel_r;
                    if (applied_speed_right < target_right) applied_speed_right = target_right;
                } else {
                    applied_speed_right += step_accel_r;
                    if (applied_speed_right > target_right) applied_speed_right = target_right;
                }

                /* Actuate dual motors */
                HbridgeSpeed(&g_hbridge, (int16_t)applied_speed_left, (int16_t)applied_speed_right);

                /* Telemetry output with heading (H), trajectory (TJ), and lookahead (LA)
                 * Throttled to 12.5 Hz (every 4th tick = 80ms) to prevent UART buffer flooding & save CPU time */
                static uint8_t s_telemetry_div = 0;
                if (++s_telemetry_div >= 4) {
                    s_telemetry_div = 0;
                    esp_printf("L:%u/%u R:%d F:%d H:%d TJ:%d S:%d LA:%d ST:%s VL:%d VR:%d\r\n",
                               (unsigned)sr.found, (unsigned)num_vectors_front,
                               (int)(sr.cte * 10.0f), (int)(filtered_cte * 10.0f),
                               (int)(filtered_heading * 10.0f),
                               (int)(filtered_traj_angle * 10.0f),
                               (int)(steer_cmd * 10.0f),
                               (int)(lookahead_turn_factor * 100.0f),
                               (drive_state == DRIVE_FORWARD) ? "F" : "T",
                               (int)applied_speed_left, (int)applied_speed_right);
                }
        }
    }
}
