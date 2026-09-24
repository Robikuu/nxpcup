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

    /* 6. Continuous Curvature Governor & 3-Macro-State Variables */
    uint16_t vectors_pid[MAX_VECTORS * 4];
    size_t   num_vectors_pid          = 0;
    uint16_t vectors_front[MAX_VECTORS * 4];
    size_t   num_vectors_front        = 0;

    trajectory_result_t g_traj;
    memset(&g_traj, 0, sizeof(g_traj));
    g_traj.lookahead_x        = (float)PIXY_CENTER_X;

    float    filtered_traj_angle      = 0.0f;
    float    lookahead_turn_factor    = 0.0f;
    uint32_t s_front_dropout_ticks    = 0;
    float    s_last_front_angle       = 0.0f;
    float    s_last_front_offset      = 0.0f;
    uint32_t s_consecutive_rejections = 0;
    float    s_last_valid_delta_front = 0.0f;
    uint32_t s_front_hold_ticks       = 0;
    float    s_blend_weight           = 1.0f;
    float    s_last_c_top             = 1.0f;
    float    s_last_valid_delta_top   = 0.0f;
    float    delta_front              = 0.0f;
    float    delta_top                = 0.0f;
    bool     front_plausible          = false;
    bool     s_has_acquired_lane      = false;
    uint32_t s_startup_rollout_ticks  = 0;
    uint32_t s_curved_intersection_approaches = 0;

    uint8_t  macro_state              = MACRO_STATE_NORMAL;
    uint32_t intersection_ticks       = 0;
    uint32_t intersection_detect_streak = 0;
    uint32_t line_lost_ticks          = 0;
    uint32_t line_reacquire_streak    = 0;

    float    filtered_cte             = 0.0f;
    int      cte_filter_init          = 0;
    float    filtered_heading         = 0.0f;
    int      heading_filter_init      = 0;

    float    applied_speed_left       = g_params.speed_straight;
    float    applied_speed_right      = g_params.speed_straight;
    float    steer_cmd                = 0.0f;
    float    last_steer_cmd           = 0.0f;
    float    kappa                    = 0.0f;

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
                macro_state              = MACRO_STATE_NORMAL;
                intersection_ticks       = 0;
                intersection_detect_streak = 0;
                line_lost_ticks          = 0;
                line_reacquire_streak    = 0;
                filtered_cte             = 0.0f;
                cte_filter_init          = 0;
                filtered_heading         = 0.0f;
                heading_filter_init      = 0;
                filtered_traj_angle      = 0.0f;
                lookahead_turn_factor    = 0.0f;
                s_front_dropout_ticks    = 0;
                s_has_acquired_lane      = false;
                s_startup_rollout_ticks  = 0;
                s_blend_weight           = 1.0f;
                memset(&g_traj, 0, sizeof(g_traj));
                g_traj.lookahead_x       = (float)PIXY_CENTER_X;
                applied_speed_left       = g_params.speed_straight;
                applied_speed_right      = g_params.speed_straight;
                steer_cmd                = 0.0f;
                last_steer_cmd           = 0.0f;
                kappa                    = 0.0f;

                if (motor_test_is_active()) {
                    motor_test_tick();
                } else {
                    HbridgeBrake(&g_hbridge);
                }
                continue;
            }

            /* --- 1. Trajectory Camera Polling (LP_FLEXCOMM2 - Forward Horizon View) --- */
            status_t s_front = pixy_get_vectors(&cam_front, vectors_front, MAX_VECTORS, &num_vectors_front);
            if (s_front == kStatus_Success && num_vectors_front > 0) {
                g_traj = compute_lookahead_curvature(vectors_front, num_vectors_front);
                filtered_traj_angle = 0.40f * g_traj.trajectory_angle_deg + 0.60f * filtered_traj_angle;

                /* Temporal Plausibility Check with Self-Healing Reseed */
                if (s_front_dropout_ticks > 0) {
                    /* Immediate re-seed after recovery from 0 vectors */
                    s_last_front_angle = g_traj.trajectory_angle_deg;
                    s_last_front_offset = g_traj.trajectory_offset_px;
                    s_consecutive_rejections = 0;
                    s_front_dropout_ticks = 0;
                    front_plausible = true;
                } else {
                    float angle_jump  = fabsf(g_traj.trajectory_angle_deg - s_last_front_angle);
                    float offset_jump = fabsf(g_traj.trajectory_offset_px - s_last_front_offset);

                    if (fabsf(g_traj.trajectory_angle_deg) <= 75.0f &&
                        angle_jump <= 25.0f &&
                        offset_jump <= 18.0f) {
                        front_plausible = true;
                        s_consecutive_rejections = 0;
                        s_last_front_angle  = g_traj.trajectory_angle_deg;
                        s_last_front_offset = g_traj.trajectory_offset_px;
                    } else {
                        s_consecutive_rejections++;
                        if (s_consecutive_rejections >= 3) {
                            /* Self-healing reseed: prevent permanent lockout in real sharp corners */
                            front_plausible = true;
                            s_consecutive_rejections = 0;
                            s_last_front_angle  = g_traj.trajectory_angle_deg;
                            s_last_front_offset = g_traj.trajectory_offset_px;
                        } else {
                            front_plausible = false;
                        }
                    }
                }

                /* Front steering calculation (calibrated from shadow test, sign inversion removed) */
                const float K_p_front = (float)FRONT_KP;
                const float K_offset_front = (float)FRONT_K_OFFSET;
                if (front_plausible) {
                    delta_front = (K_p_front * filtered_traj_angle + K_offset_front * g_traj.trajectory_offset_px);
                    s_last_valid_delta_front = delta_front;
                    s_front_hold_ticks = 0;
                } else {
                    s_front_hold_ticks++;
                    if (s_front_hold_ticks <= 3) {
                        delta_front = s_last_valid_delta_front;
                    } else {
                        delta_front = s_last_valid_delta_front * 0.95f;
                        s_last_valid_delta_front = delta_front;
                    }
                }

                /* Asymmetric urgency filter: fast attack into corners, slow decay to prevent optical flicker acceleration surges */
                if (g_traj.lookahead_urgency >= lookahead_turn_factor) {
                    lookahead_turn_factor = 0.80f * g_traj.lookahead_urgency + 0.20f * lookahead_turn_factor;
                } else {
                    lookahead_turn_factor = 0.15f * g_traj.lookahead_urgency + 0.85f * lookahead_turn_factor;
                }
            } else {
                s_front_dropout_ticks++;
                front_plausible = false;
                filtered_traj_angle   *= 0.80f;
                lookahead_turn_factor *= 0.85f;
                s_front_hold_ticks++;
                if (s_front_hold_ticks <= 3) {
                    delta_front = s_last_valid_delta_front;
                } else {
                    delta_front = s_last_valid_delta_front * 0.95f;
                    s_last_valid_delta_front = delta_front;
                }
                memset(&g_traj, 0, sizeof(g_traj));
                g_traj.lookahead_x = (float)PIXY_CENTER_X;
            }

            /* --- 2. PID Camera Polling & Geometry Validation (Top/Homography View) --- */
            status_t s_pid = pixy_get_vectors(&cam_pid, vectors_pid, MAX_VECTORS, &num_vectors_pid);
            bool intersection_trigger = false;

            if (s_pid == kStatus_Success && num_vectors_pid > 0) {
                /* Detect crossing lines before Pass 2 horizontal-vector rejection.
                 * CRITICAL: An intersection can ONLY be detected if the vehicle is travelling
                 * strictly straight along the lane (|H| < 6.0 deg, |steer| < 8.0 deg, |TJ| < 12.0 deg,
                 * lookahead_urgency < 0.20, kappa < 0.20, |CTE| < 12.0 px). In sharp curves,
                 * transverse tape boundaries appear horizontal in perspective projection. */
                if (fabsf(filtered_heading) < 6.0f &&
                    fabsf(steer_cmd) < 8.0f &&
                    fabsf(filtered_traj_angle) < 12.0f &&
                    lookahead_turn_factor < 0.20f &&
                    kappa < 0.20f &&
                    fabsf(filtered_cte) < 12.0f) {
                    for (size_t i = 0; i < num_vectors_pid; i++) {
                        int dx = (int)vectors_pid[4 * i + 2] - (int)vectors_pid[4 * i + 0];
                        int dy = (int)vectors_pid[4 * i + 3] - (int)vectors_pid[4 * i + 1];
                        if (dx < 0) dx = -dx;
                        if (dy < 0) dy = -dy;
                        if (dx >= 18 && (dx * 20 > dy * 17)) {
                            intersection_trigger = true;
                            break;
                        }
                    }
                }
                vector_filtering(vectors_pid, &num_vectors_pid);
            } else {
                num_vectors_pid = 0;
            }

            steering_result_t sr;
            if (s_pid == kStatus_Success && num_vectors_pid > 0) {
                sr = compute_steering_error(vectors_pid, num_vectors_pid);
            } else {
                sr.found       = 0;
                sr.cte         = 0.0f;
                sr.heading_deg = 0.0f;
            }

            /* Dual-camera lane acquisition & startup rollout counter */
            s_startup_rollout_ticks++;
            if (sr.found > 0 || (front_plausible && num_vectors_front >= 1) || s_startup_rollout_ticks >= 10) {
                s_has_acquired_lane = true;
            }

            /* --- 3. Macro State Transitions --- */
            /* Dual-blind safety: ONLY enter LINE_LOST if BOTH cameras have lost the track */
            bool dual_blind = (sr.found == 0) && (!front_plausible || num_vectors_front == 0);

            if (dual_blind) {
                line_reacquire_streak = 0;
                line_lost_ticks++;
                if (line_lost_ticks >= (uint32_t)LINE_LOST_DEBOUNCE_TICKS) {
                    macro_state = MACRO_STATE_LINE_LOST;
                }
            } else {
                if (macro_state == MACRO_STATE_LINE_LOST) {
                    if (++line_reacquire_streak >= (uint32_t)LINE_REACQUIRE_TICKS) {
                        /* Exit LINE_LOST -> NORMAL: Reset filters to prevent steering kick */
                        macro_state           = MACRO_STATE_NORMAL;
                        line_lost_ticks       = 0;
                        line_reacquire_streak = 0;
                        cte_filter_init       = 0;
                        heading_filter_init   = 0;
                        reset_pid_state(&steer_pid);
                    }
                } else {
                    line_lost_ticks       = 0;
                    line_reacquire_streak = 0;
                }
            }

            if (macro_state == MACRO_STATE_NORMAL) {
                if (intersection_trigger) {
                    if (++intersection_detect_streak >= 4) {
                        macro_state                = MACRO_STATE_INTERSECTION;
                        intersection_ticks         = 0;
                        intersection_detect_streak = 0;
                        reset_pid_state(&steer_pid);
                    }
                } else {
                    intersection_detect_streak = 0;
                }
            } else {
                intersection_detect_streak = 0;
            }

            /* --- 4. Dynamic Dual-Camera Blend Weight & Line Filtering --- */
            /* Instantaneous Top camera confidence based on vector count */
            float c_top = 0.0f;
            if (sr.found > 0) {
                c_top = (num_vectors_pid >= 2) ? 1.0f : 0.50f;
            } else {
                c_top = 0.0f;
            }

            /* Target blend weight: if front is invalid and top has any signal, rely entirely on top */
            float target_alpha = c_top;
            if (!front_plausible && c_top > 0.0f) {
                target_alpha = 1.0f;
            }

            /* Asymmetric blend slew rate: fast transfer to front (40ms), smooth return to top (240ms) */
            float delta_alpha = target_alpha - s_blend_weight;
            if (delta_alpha > (float)BLEND_RISE_SLEW_MAX) {
                delta_alpha = (float)BLEND_RISE_SLEW_MAX;
            } else if (delta_alpha < -(float)BLEND_FALL_SLEW_MAX) {
                delta_alpha = -(float)BLEND_FALL_SLEW_MAX;
            }
            s_blend_weight += delta_alpha;
            if (s_blend_weight > 1.0f) s_blend_weight = 1.0f;
            if (s_blend_weight < 0.0f) s_blend_weight = 0.0f;

            if (sr.found > 0) {
                if (!heading_filter_init || s_last_c_top == 0.0f) {
                    filtered_heading    = sr.heading_deg;
                    heading_filter_init = 1;
                } else {
                    filtered_heading = 0.60f * sr.heading_deg + 0.40f * filtered_heading;
                }

                if (!cte_filter_init || s_last_c_top == 0.0f) {
                    filtered_cte    = sr.cte;
                    cte_filter_init = 1;
                } else {
                    filtered_cte = 0.40f * sr.cte + 0.60f * filtered_cte;
                }
            }
            s_last_c_top = c_top;

            /* --- 5. State-Specific Control Computation --- */
            float target_speed = g_params.speed_straight;
            float raw_steer    = 0.0f;

            if (macro_state == MACRO_STATE_NORMAL) {
                /* A. Preview Curvature Metric (kappa) with Deadbands */
                float abs_h = fabsf(filtered_heading);
                float k_base = 0.0f;
                if (abs_h > (float)GOVERNOR_HEADING_DEADBAND_DEG) {
                    k_base = (abs_h - (float)GOVERNOR_HEADING_DEADBAND_DEG) /
                             ((float)GOVERNOR_HEADING_MAX_DEG - (float)GOVERNOR_HEADING_DEADBAND_DEG);
                    if (k_base > 1.0f) k_base = 1.0f;
                }

                float k_front = lookahead_turn_factor * 1.25f;
                if (k_front > 1.0f) k_front = 1.0f;

                kappa = (k_base > k_front) ? k_base : k_front;
                if (kappa > 1.0f) kappa = 1.0f;

                /* B. Continuous Target Speed */
                target_speed = g_params.speed_straight - kappa * (g_params.speed_straight - g_params.speed_outside_corner);
                if (s_blend_weight < 0.30f && target_speed > (float)BLEND_FALLBACK_SPEED_CAP) {
                    target_speed = (float)BLEND_FALLBACK_SPEED_CAP;
                }

                /* C. Dynamic PID Gain Scheduling (Straightaway vs Corner Regimes) */
                steer_pid.kp = g_params.kp_straight + kappa * (g_params.kp_corner - g_params.kp_straight);
                steer_pid.kd = g_params.kd_straight + kappa * (g_params.kd_corner - g_params.kd_straight);

                /* D. Decoupled Steering Authority */
                float effective_h = 0.0f;
                if (abs_h > (float)HEADING_DEADBAND_DEG) {
                    effective_h = (abs_h - (float)HEADING_DEADBAND_DEG) * ((filtered_heading > 0.0f) ? 1.0f : -1.0f);
                }
                float heading_ff = g_params.k_heading * effective_h;

                /* E. Predictive Turn-in Feedforward from Front Camera */
                float effective_traj = 0.0f;
                float abs_traj = fabsf(filtered_traj_angle);
                if (abs_traj > 3.0f) {
                    effective_traj = (abs_traj - 3.0f) * ((filtered_traj_angle > 0.0f) ? 1.0f : -1.0f);
                }
                float lookahead_ff = g_params.k_lookahead * effective_traj;
                if (lookahead_ff > 15.0f) {
                    lookahead_ff = 15.0f;
                } else if (lookahead_ff < -15.0f) {
                    lookahead_ff = -15.0f;
                }

                /* F. PID CTE Centering Authority & Reseed */
                if (c_top > 0.0f) {
                    float pid_cte = -pid_update(&steer_pid, 0.0f, g_params.k_cte * filtered_cte, g_params.dt);

                    /* G. Apex Protection: Suppress Counter-Steering CTE into Corners
                     * When approaching or inside a curve (kappa > 0.15), if PID CTE opposes
                     * the overall turn direction, attenuate CTE so the car bites into the apex
                     * rather than pushing wide into the outside boundary. */
                    float turn_dir = 0.0f;
                    if (abs_h > 3.0f) {
                        turn_dir = filtered_heading;
                    } else if (abs_traj > 5.0f) {
                        turn_dir = filtered_traj_angle;
                    }

                    if (turn_dir != 0.0f && (pid_cte * turn_dir < 0.0f)) {
                        float atten = 1.0f - 0.75f * kappa;
                        if (atten < 0.20f) atten = 0.20f;
                        pid_cte *= atten;
                    }

                    delta_top = pid_cte + heading_ff + lookahead_ff;
                    s_last_valid_delta_top = delta_top;
                } else {
                    /* Freeze Top PID integrator to prevent windup during blindness */
                    steer_pid.initialized = 0; /* Clears derivative kick when c_top returns */
                    delta_top = s_last_valid_delta_top * 0.95f; /* Smooth decay while authority transfers to front */
                    s_last_valid_delta_top = delta_top;
                }

                if (s_startup_rollout_ticks <= 5) {
                    /* First 100ms (5 ticks): roll straight forward to clear start line cleanly */
                    raw_steer = 0.0f;
                } else if (g_params.shadow_mode_enable) {
                    raw_steer = delta_top;
                } else {
                    raw_steer = s_blend_weight * delta_top + (1.0f - s_blend_weight) * delta_front;
                }
            }
            else if (macro_state == MACRO_STATE_INTERSECTION) {
                kappa        = 0.0f;
                target_speed = 65.0f;
                raw_steer    = last_steer_cmd * 0.85f; /* Smoothly decay toward straight, never violent snap */

                /* Immediate curve abort: If vehicle heading swings into a curve (|H| >= 10.0 deg)
                 * or front camera detects upcoming curvature, we are clearly navigating a turn,
                 * NOT crossing an intersection. Revert to NORMAL immediately so steering is not frozen! */
                if (fabsf(filtered_heading) >= 10.0f || lookahead_turn_factor >= 0.35f || fabsf(filtered_traj_angle) >= 18.0f) {
                    macro_state         = MACRO_STATE_NORMAL;
                    intersection_ticks  = 0;
                    cte_filter_init     = 0;
                    heading_filter_init = 0;
                    reset_pid_state(&steer_pid);
                } else if (++intersection_ticks >= (uint32_t)INTERSECTION_TIMEOUT_TICKS) {
                    /* Guaranteed Timeout Exit -> NORMAL */
                    macro_state         = MACRO_STATE_NORMAL;
                    intersection_ticks  = 0;
                    cte_filter_init     = 0;
                    heading_filter_init = 0;
                    reset_pid_state(&steer_pid);
                }
            }
            else if (macro_state == MACRO_STATE_LINE_LOST) {
                kappa = 0.0f;

                if (line_lost_ticks <= (uint32_t)LINE_LOST_BRAKE_TICKS) {
                    /* Phase 1: Coast holding last good steering angle to reacquire in curves */
                    raw_steer    = last_steer_cmd;
                    target_speed = applied_speed_left;
                } else if (line_lost_ticks <= (uint32_t)LINE_LOST_STOP_TICKS) {
                    /* Phase 2: Active Dynamic Braking while centering steering */
                    raw_steer    = last_steer_cmd * 0.90f;
                    target_speed = 0.0f;
                } else {
                    /* Phase 3: Emergency Stop */
                    raw_steer      = 0.0f;
                    target_speed   = 0.0f;
                    g_params.state = STATE_IDLE;
                    HbridgeBrake(&g_hbridge);
                }
            }

            /* --- 6. Global Steering Slew Rate Limiter --- */
            const float max_steer_step = (float)MAX_STEER_RATE_PER_SEC * g_params.dt;
            float steer_diff = raw_steer - last_steer_cmd;
            if (steer_diff > max_steer_step) {
                raw_steer = last_steer_cmd + max_steer_step;
            } else if (steer_diff < -max_steer_step) {
                raw_steer = last_steer_cmd - max_steer_step;
            }

            if (raw_steer > (float)STEER_CMD_MAX_RIGHT) raw_steer = (float)STEER_CMD_MAX_RIGHT;
            if (raw_steer < (float)STEER_CMD_MAX_LEFT)  raw_steer = (float)STEER_CMD_MAX_LEFT;
            steer_cmd      = raw_steer;
            last_steer_cmd = steer_cmd;

            Steer(steer_cmd + g_params.steering_offset);

            /* --- 7. Dynamic Differential Drag (Tank Steering Bite) --- */
            float abs_steer = fabsf(steer_cmd);
            float steer_norm = 0.0f;
            if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
                steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                             ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
                if (steer_norm > 1.0f) steer_norm = 1.0f;
            }

            float diff_delta = steer_norm * (target_speed - g_params.speed_inside_min);
            if (diff_delta < 0.0f) diff_delta = 0.0f;

            float target_left  = target_speed;
            float target_right = target_speed;

            if (steer_cmd > (float)DIFFERENTIAL_DEADZONE_DEG) {
                /* Turning Right: Left outside, Right inside */
                target_left  = target_speed;
                target_right = target_speed - diff_delta;
            } else if (steer_cmd < -(float)DIFFERENTIAL_DEADZONE_DEG) {
                /* Turning Left: Right outside, Left inside */
                target_left  = target_speed - diff_delta;
                target_right = target_speed;
            }

            /* --- 8. Bilateral Anti-Spin Traction Guard & Speed Slew Limiter --- */
            bool yaw_conflict = false;
            if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
                if (steer_cmd > (float)YAW_CONFLICT_DEADBAND &&
                    (target_left - target_right) > 5.0f &&
                    applied_speed_left < applied_speed_right) {
                    yaw_conflict = true;
                } else if (steer_cmd < -(float)YAW_CONFLICT_DEADBAND &&
                           (target_right - target_left) > 5.0f &&
                           applied_speed_right < applied_speed_left) {
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
            if (!g_params.motor_enable) {
                applied_speed_left  = 0.0f;
                applied_speed_right = 0.0f;
                HbridgeBrake(&g_hbridge);
            } else if (macro_state == MACRO_STATE_LINE_LOST && line_lost_ticks > (uint32_t)LINE_LOST_BRAKE_TICKS) {
                HbridgeBrake(&g_hbridge);
            } else {
                HbridgeSpeed(&g_hbridge, (int16_t)applied_speed_left, (int16_t)applied_speed_right);
            }

            /* --- 9. Telemetry Output (Throttled to 12.5 Hz = Every 4th Tick) --- */
            static uint8_t s_telemetry_div = 0;
            if (++s_telemetry_div >= 4) {
                s_telemetry_div = 0;
                const char *st_str = (macro_state == MACRO_STATE_NORMAL) ? "N" :
                                     ((macro_state == MACRO_STATE_INTERSECTION) ? "I" : "L");
                char dom_char = g_params.shadow_mode_enable ? 'T' :
                                ((s_blend_weight >= 0.70f) ? 'T' : ((s_blend_weight <= 0.30f) ? 'F' : 'B'));
                bool is_roughly_straight = (fabsf(filtered_heading) <= 15.0f) && (fabsf(steer_cmd) <= 18.0f);
                bool has_transverse_tape = (g_traj.has_transverse_tape || intersection_trigger);
                uint8_t ix_flag = (has_transverse_tape && is_roughly_straight) ? 1 : 0;
                if (has_transverse_tape && !is_roughly_straight) {
                    s_curved_intersection_approaches++;
                }

                esp_printf("L:%u/%u R:%d F:%d H:%d TS:%d FS:%d S:%d BW:%d DOM:%c FV:%u IX:%u ST:%s VL:%d VR:%d\r\n",
                           (unsigned)sr.found, (unsigned)num_vectors_front,
                           (int)(sr.cte * 10.0f), (int)(filtered_cte * 10.0f),
                           (int)(filtered_heading * 10.0f),
                           (int)(delta_top * 10.0f),
                           (int)(delta_front * 10.0f),
                           (int)(steer_cmd * 10.0f),
                           (int)(s_blend_weight * 100.0f),
                           dom_char,
                           front_plausible ? 1U : 0U,
                           (unsigned)ix_flag,
                           st_str,
                           (int)applied_speed_left, (int)applied_speed_right);
            }
        }
    }
}
