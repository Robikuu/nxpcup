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
    uint32_t front_cam_phase         = 0;
    trajectory_result_t g_traj;
    memset(&g_traj, 0, sizeof(g_traj));
    float    filtered_traj_angle     = 0.0f;
    float    filtered_traj_offset    = 0.0f;
    float    lookahead_turn_factor   = 0.0f;
    uint32_t line_lost_counter       = 0;
    float    filtered_cte            = 0.0f;
    int      cte_filter_init         = 0;
    float    filtered_heading        = 0.0f;
    int      heading_filter_init     = 0;
    float    applied_speed_left      = g_params.speed_straight;
    float    applied_speed_right     = g_params.speed_straight;
    float    steer_cmd               = 0.0f;
    drive_state_t drive_state        = DRIVE_FORWARD;
    uint32_t      state_streak       = 0;
    uint32_t      urgency_streak     = 0;
    float         trusted_corner_urgency = 0.0f;

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
                memset(&g_traj, 0, sizeof(g_traj));
                applied_speed_left        = g_params.speed_straight;
                applied_speed_right       = g_params.speed_straight;
                steer_cmd                 = 0.0f;
                drive_state               = DRIVE_FORWARD;
                state_streak              = 0;
                urgency_streak            = 0;
                trusted_corner_urgency    = 0.0f;
                if (motor_test_is_active()) {
                    motor_test_tick();
                } else {
                    HbridgeBrake(&g_hbridge);
                }
                continue;
            }

            /* Trajectory Camera polling (LP_FLEXCOMM2 - alternating 25 Hz) */
            if ((front_cam_phase++ & 1) == 0) {
                status_t s_front = pixy_get_vectors(&cam_front, vectors_front, MAX_VECTORS, &num_vectors_front);
                if (s_front == kStatus_Success && num_vectors_front > 0) {
                    /* Calculate trajectory of the car ahead from front camera */
                    g_traj = compute_lookahead_trajectory(vectors_front, num_vectors_front);

                    filtered_traj_angle  = 0.50f * g_traj.trajectory_angle_deg + 0.50f * filtered_traj_angle;
                    filtered_traj_offset = 0.50f * g_traj.trajectory_offset_px  + 0.50f * filtered_traj_offset;

                    /* Lookahead urgency for corner speed modulation:
                     * Fast attack (0.70x) to immediately initiate pre-braking on the straight;
                     * Smooth decay (0.25x) to maintain stable deceleration through preview jitter. */
                    if (g_traj.lookahead_urgency > lookahead_turn_factor) {
                        lookahead_turn_factor = 0.70f * g_traj.lookahead_urgency + 0.30f * lookahead_turn_factor;
                    } else {
                        lookahead_turn_factor = 0.25f * g_traj.lookahead_urgency + 0.75f * lookahead_turn_factor;
                    }
                } else {
                    filtered_traj_angle   *= 0.80f;
                    filtered_traj_offset  *= 0.80f;
                    lookahead_turn_factor *= 0.85f;
                    g_traj.found = 0;
                }
            }

            /* PID Camera line tracking (Top/Homography view) */
            status_t s = pixy_get_vectors(&cam_pid, vectors_pid, MAX_VECTORS, &num_vectors_pid);
            if (s != kStatus_Success || num_vectors_pid == 0) {
                urgency_streak         = 0;
                trusted_corner_urgency = 0.0f;
                lookahead_turn_factor  = 0.0f;
                if (++line_lost_counter > LINE_LOST_LIMIT) {
                    HbridgeBrake(&g_hbridge);
                    reset_steering_tracking();
                }
                continue;
            }
            line_lost_counter = 0;

            /* Geometric pair validation & filtering on PID camera */
            vector_filtering(vectors_pid, &num_vectors_pid);

            /* Error computation with perspective synthesis */
            steering_result_t sr = compute_steering_error(vectors_pid, num_vectors_pid);

            if (sr.found > 0) {
                if (!heading_filter_init) {
                    filtered_heading    = sr.heading_deg;
                    heading_filter_init = 1;
                } else {
                    filtered_heading = 0.60f * sr.heading_deg + 0.40f * filtered_heading;
                }
            }

            /* State machine transitions with dual-camera consensus:
             * - DRIVE_TURNING (Corner):
             *   Triggered either by local top camera curvature (|filtered_heading| >= CORNER_HEADING_THRESHOLD_DEG)
             *   OR by forward lookahead preview of upcoming turn (lookahead_turn_factor >= 0.50f).
             * - DRIVE_FORWARD (Straight):
             *   Strictly confirmed by TOP CAMERA: local lane is centered (|filtered_heading| <= 6.0 deg
             *   and |filtered_cte| <= 8.0 px) AND lookahead confirms no immediate curve (lookahead_turn_factor < 0.30f).
             *   Hysteresis debounce of STATE_EXIT_STREAK (5 ticks = 100ms) guarantees chatter-free transitions. */
            drive_state_t desired_state;
            if (sr.found == 0) {
                desired_state = DRIVE_BRAKING;
            } else {
                bool corner_trigger = (fabsf(filtered_heading) >= CORNER_HEADING_THRESHOLD_DEG || lookahead_turn_factor >= 0.50f);
                if (drive_state == DRIVE_TURNING) {
                    bool straight_confirmed = (fabsf(filtered_heading) <= 6.0f && fabsf(filtered_cte) <= 8.0f && lookahead_turn_factor < 0.30f);
                    desired_state = straight_confirmed ? DRIVE_FORWARD : DRIVE_TURNING;
                } else {
                    desired_state = corner_trigger ? DRIVE_TURNING : DRIVE_FORWARD;
                }
            }

            if (desired_state != drive_state) {
                uint32_t threshold = (desired_state == DRIVE_FORWARD) ? (uint32_t)STATE_EXIT_STREAK :
                                     ((desired_state == DRIVE_BRAKING) ? (uint32_t)LINE_LOST_BRAKE_TICKS : 1U);
                if (++state_streak >= threshold) {
                    if (desired_state == DRIVE_BRAKING) {
                        reset_pid_state(&steer_pid);
                        reset_steering_tracking();
                        filtered_cte            = 0.0f;
                        cte_filter_init         = 0;
                        filtered_heading        = 0.0f;
                        heading_filter_init     = 0;
                        filtered_traj_angle     = 0.0f;
                        filtered_traj_offset    = 0.0f;
                        applied_speed_left      = g_params.speed_straight;
                        applied_speed_right     = g_params.speed_straight;
                        urgency_streak          = 0;
                        trusted_corner_urgency  = 0.0f;
                        lookahead_turn_factor   = 0.0f;
                        memset(&g_traj, 0, sizeof(g_traj));
                    }
                    drive_state  = desired_state;
                    state_streak = 0;
                }
            } else {
                state_streak = 0;
            }

            if (sr.found > 0 && drive_state != DRIVE_BRAKING) {
                /* Cross-track error low-pass filtering */
                if (!cte_filter_init) {
                    filtered_cte    = sr.cte;
                    cte_filter_init = 1;
                } else {
                    filtered_cte = 0.4f * sr.cte + 0.6f * filtered_cte;
                }

                /* 1. Dynamic PID Gain Scheduling Based on Corner Urgency (Decoupled from Wheel Drag)
                 * Deadband of 8.0 deg on heading: normal straightaway chassis yaw flutter (<= 8 deg)
                 * MUST NOT elevate urgency or trigger corner gains. */
                float dz_heading_urgency = fabsf(filtered_heading) - 8.0f;
                float heading_urgency = (dz_heading_urgency > 0.0f) ? (dz_heading_urgency / ((float)HEADING_FULL_BRAKE_DEG - 8.0f)) : 0.0f;
                if (heading_urgency > 1.0f) heading_urgency = 1.0f;

                /* Only let CTE drive corner urgency if heading is turning or CTE enters the outer lane band (>14px) */
                float cte_excess = fabsf(filtered_cte) - 14.0f;
                float cte_urgency = (cte_excess > 0.0f) ? (cte_excess / ((float)CTE_URGENCY_PX - 14.0f)) : 0.0f;
                if (cte_urgency > 1.0f) cte_urgency = 1.0f;

                float raw_corner_urgency = (heading_urgency > cte_urgency) ? heading_urgency : cte_urgency;

                /* Front camera predictive urgency pre-engagement:
                 * Only allow lookahead to elevate urgency if a genuine upcoming turn is detected (> 0.35f) */
                if (lookahead_turn_factor > 0.35f && lookahead_turn_factor > raw_corner_urgency) {
                    raw_corner_urgency = lookahead_turn_factor;
                }

                if (raw_corner_urgency > trusted_corner_urgency) {
                    if (++urgency_streak >= (uint32_t)CORNER_URGENCY_STREAK) {
                        trusted_corner_urgency = raw_corner_urgency;
                        urgency_streak = 0;
                    }
                } else {
                    trusted_corner_urgency = raw_corner_urgency;
                    urgency_streak = 0;
                }

                steer_pid.kp    = g_params.kp_straight + trusted_corner_urgency * (g_params.kp_corner - g_params.kp_straight);
                steer_pid.ki    = g_params.ki;
                steer_pid.kd    = g_params.kd_straight + trusted_corner_urgency * (g_params.kd_corner - g_params.kd_straight);
                steer_pid.alpha = g_params.alpha;

                /* 2. Decoupled Steering Authority:
                 * Steering servo is 100% driven by Top Camera local PID feedback.
                 * Flexcomm 2 Front Camera operates strictly for predictive speed profiling & state transitions. */
                float effective_cte = filtered_cte;
                /* Decisive Turn-In & Apex Protection:
                 * When the vehicle is negotiating a turn (|filtered_heading| >= 3.5 deg)
                 * and CTE opposes heading (filtered_cte * filtered_heading < 0), the car is
                 * approaching the turn from the outside or clipping the inside apex.
                 * Suppress the opposing CTE (effective_cte = 0.0f) so heading error immediately
                 * commands authoritative steering into the curve without being cancelled out! */
                if (filtered_cte * filtered_heading < 0.0f && fabsf(filtered_heading) >= 3.5f) {
                    effective_cte = 0.0f;
                }
                float combined = g_params.k_cte * effective_cte + g_params.k_heading * filtered_heading;
                float feedback_steer = -pid_update(&steer_pid, 0.0f, combined, g_params.dt);

                steer_cmd = feedback_steer;

                /* Clamp to relative command travel: [-77.5f, +52.5f] */
                if (steer_cmd > (float)STEER_CMD_MAX_RIGHT) {
                    steer_cmd = (float)STEER_CMD_MAX_RIGHT;
                }
                if (steer_cmd < (float)STEER_CMD_MAX_LEFT) {
                    steer_cmd = (float)STEER_CMD_MAX_LEFT;
                }

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

                /* 4. Asymmetric Target Wheel Speed Assignment
                 * A. Longitudinal Pre-Braking: slows down vehicle base speed into upcoming corners.
                 *    Both wheels decelerate symmetrically so there is ZERO yaw kick on straights.
                 *    Proactive braking: combines local heading angle, lateral displacement (CTE),
                 *    and lookahead front camera preview.
                 *    Crucial for corner exit: if the vehicle has swung wide (|CTE| > 8px), it HOLDS
                 *    controlled cornering speed (80%) until centered, preventing 100% sprint wheelspin! */
                float abs_cte = fabsf(filtered_cte);
                float dz_cte = abs_cte - 8.0f;
                if (dz_cte < 0.0f) dz_cte = 0.0f;
                float cte_norm = dz_cte / (20.0f - 8.0f);
                if (cte_norm > 1.0f) cte_norm = 1.0f;

                float effective_corner_intensity = heading_norm;
                if (cte_norm > effective_corner_intensity) {
                    effective_corner_intensity = cte_norm;
                }
                if (lookahead_turn_factor > effective_corner_intensity) {
                    effective_corner_intensity = lookahead_turn_factor;
                }
                float longitudinal_base = g_params.speed_straight - effective_corner_intensity * (g_params.speed_straight - g_params.speed_outside_corner);

                /* Continuous differential drag authority:
                 * On straights (trusted_corner_urgency <= 0.20f), differential drag is strictly 0.0f
                 * ensuring both rear wheels provide 100% symmetric, straight-line drive with zero yaw kicks;
                 * Only as corner urgency rises (> 0.20f), differential drag smoothly engages (up to 1.0x)
                 * to pivot the vehicle sharply through the turn. */
                float diff_scale = 0.0f;
                if (trusted_corner_urgency > 0.20f) {
                    diff_scale = (trusted_corner_urgency - 0.20f) / 0.80f;
                }
                float active_steer_norm = steer_norm * diff_scale;

                /* B. Lateral Differential Drag: strictly driven by active physical steering */
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

            } else if (drive_state == DRIVE_BRAKING) {
                static uint8_t s_brake_div = 0;
                if (++s_brake_div >= 5) {
                    s_brake_div = 0;
                    esp_printf("L:%u ST:B (braking)\r\n", (unsigned)sr.found);
                }
                Steer(g_params.steering_offset);
                HbridgeBrake(&g_hbridge);
                applied_speed_left  = g_params.speed_inside_min;
                applied_speed_right = g_params.speed_inside_min;
                steer_cmd           = 0.0f;
            }
        }
    }
}
