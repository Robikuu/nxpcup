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

    /* Ensure all DMA & Flexcomm interrupts for both cameras (LP_FLEXCOMM2 & LP_FLEXCOMM3) are enabled in NVIC */
    NVIC_SetPriority(EDMA_0_CH0_IRQn, 3);
    EnableIRQ(EDMA_0_CH0_IRQn);
    NVIC_SetPriority(EDMA_0_CH1_IRQn, 3);
    EnableIRQ(EDMA_0_CH1_IRQn);
    NVIC_SetPriority(LP_FLEXCOMM2_IRQn, 3);
    EnableIRQ(LP_FLEXCOMM2_IRQn);

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
    float    prev_filtered_cte        = 0.0f;
    float    filtered_d_cte           = 0.0f;
    float    filtered_heading         = 0.0f;
    float    filtered_front_angle     = 0.0f;

    float    applied_speed_left       = g_params.speed_straight;
    float    applied_speed_right      = g_params.speed_straight;
    float    steer_cmd                = 0.0f;
    float    last_steer_cmd           = 0.0f;
    float    last_valid_steer_top     = 0.0f;
    uint32_t s_rollout_ticks          = 0;
    uint32_t s_off_track_ticks        = 0;

    /* Intersection Tactical State */
    uint8_t  s_intersection_ahead      = 0;
    uint32_t s_intersection_timer      = 0; /* Persistence timer (~15 ticks = 300ms) */
    uint32_t s_intersection_gap_ticks  = 0; /* Elapsed ticks in blind gap */
    uint8_t  s_was_in_intersection_gap = 0; /* Flag set during intersection blind gap */
    uint32_t s_reacquire_ticks         = 0; /* Slew-rate limiter ticks after blind gap */
    uint8_t  s_sharp_cancel_count      = 0; /* 3-tick hysteresis before canceling intersection latch */
    uint8_t  s_telemetry_div           = 0; /* 12.5 Hz telemetry divider */
    float    s_corner_blend            = 0.0f; /* 0.0 = straight, 1.0 = sharp corner */
    float    s_blend_target            = 0.0f; /* Debug target holding blend */

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
                prev_filtered_cte         = 0.0f;
                filtered_d_cte            = 0.0f;
                filtered_heading          = 0.0f;
                filtered_front_angle      = 0.0f;
                applied_speed_left        = g_params.speed_straight;
                applied_speed_right       = g_params.speed_straight;
                steer_cmd                 = 0.0f;
                last_steer_cmd            = 0.0f;
                last_valid_steer_top      = 0.0f;
                s_rollout_ticks           = 0;
                s_off_track_ticks         = 0;
                s_intersection_ahead      = 0;
                s_intersection_timer      = 0;
                s_intersection_gap_ticks  = 0;
                s_was_in_intersection_gap = 0;
                s_reacquire_ticks         = 0;
                s_sharp_cancel_count      = 0;
                s_corner_blend            = 0.0f;
                s_blend_target            = 0.0f;

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
                if (horizon.found > 0 && (!horizon.has_transverse_tape || fabsf(horizon.horizon_angle_deg) > 0.01f)) {
                    filtered_front_angle = 0.40f * horizon.horizon_angle_deg + 0.60f * filtered_front_angle;
                } else if (!horizon.is_sharp_corner) {
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
                    filtered_cte      = 0.75f * sr.cte + 0.25f * filtered_cte;
                    filtered_heading  = 0.75f * sr.heading_deg + 0.25f * filtered_heading;
                    float dt_safe     = (g_params.dt > 0.005f) ? g_params.dt : 0.02f;
                    float raw_d_cte   = (filtered_cte - prev_filtered_cte) / dt_safe;
                    prev_filtered_cte = filtered_cte;
                    filtered_d_cte    = g_params.alpha * raw_d_cte + (1.0f - g_params.alpha) * filtered_d_cte;
                } else {
                    filtered_d_cte *= 0.50f;
                }
            } else {
                num_vectors_pid = 0;
                filtered_d_cte *= 0.50f;
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
            if (sr.transverse_crosses_center) {
                /* Top camera confirms wall across center right in front of car -> immediate corner preempt */
                s_intersection_ahead      = 0;
                s_intersection_timer      = 0;
                s_intersection_gap_ticks  = 0;
                s_was_in_intersection_gap = 0;
                s_reacquire_ticks         = 0;
                s_sharp_cancel_count      = 0;
            } else if (horizon.is_intersection_ahead) {
                /* Intersection Latching & Lifetime: refresh persistence timer (~15 ticks) */
                s_intersection_ahead = 1;
                s_intersection_timer = 15;
                s_sharp_cancel_count = 0;
            } else if (horizon.is_sharp_corner) {
                if (s_intersection_ahead) {
                    /* Require 3 consecutive ticks of sharp corner to cancel an active intersection latch */
                    s_sharp_cancel_count++;
                    if (s_sharp_cancel_count >= 3U) {
                        s_intersection_ahead      = 0;
                        s_intersection_timer      = 0;
                        s_intersection_gap_ticks  = 0;
                        s_was_in_intersection_gap = 0;
                        s_reacquire_ticks         = 0;
                        s_sharp_cancel_count      = 0;
                    }
                } else {
                    s_reacquire_ticks    = 0;
                    s_sharp_cancel_count = 0;
                }
            } else {
                s_sharp_cancel_count = 0;
                if (s_intersection_timer > 0) {
                    s_intersection_timer--;
                    if (s_intersection_timer == 0 && s_intersection_gap_ticks == 0) {
                        s_intersection_ahead = 0;
                    }
                }
            }

            /* --- Blind Gap Policy (sr.found == 0) & Failsafe Guard --- */
            if (sr.found == 0) {
                s_off_track_ticks++;
                if (s_off_track_ticks >= (uint32_t)OFF_TRACK_FAILSAFE_TICKS) {
                    g_params.state       = STATE_IDLE;
                    HbridgeBrake(&g_hbridge);
                    Steer(g_params.steering_offset);
                    applied_speed_left   = 0.0f;
                    applied_speed_right  = 0.0f;
                    steer_cmd            = 0.0f;
                    last_steer_cmd       = 0.0f;
                    last_valid_steer_top = 0.0f;
                    filtered_d_cte       = 0.0f;
                    s_off_track_ticks    = 0;
                    PRINTF("FAILSAFE: Off track for 2.5s -> STOPPED\r\n");
                    esp_printf("ALERT: FAILSAFE OFF TRACK STOP\r\n");
                    continue;
                }

                if (s_intersection_ahead) {
                    /* Intersection Blind Gap Active */
                    s_was_in_intersection_gap = 1;
                    s_intersection_gap_ticks++;

                    /* Distance-scaled window: gap_ticks = 1.5 * 25 / max(speed * drive_scale, 30) * 50 */
                    float current_spd = fmaxf(applied_speed_left, applied_speed_right) * g_params.drive_scale;
                    uint32_t gap_window = (uint32_t)(1.5f * 25.0f / fmaxf(current_spd, 30.0f) * 50.0f);
                    if (s_intersection_gap_ticks > gap_window) {
                        s_intersection_ahead     = 0;
                        s_intersection_gap_ticks = 0;
                    }

                    /* Decay CTE and heading across the intersection gap */
                    filtered_cte     *= 0.70f;
                    prev_filtered_cte = filtered_cte;
                    filtered_d_cte    = 0.0f;
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

            float steer_top       = 0.0f;
            float target_speed    = g_params.speed_straight;
            float target_left     = target_speed;
            float target_right    = target_speed;
            float max_right_delta = (float)STEERING_LIMIT_RIGHT - g_params.steering_offset;
            float max_left_delta  = (float)STEERING_LIMIT_LEFT  - g_params.steering_offset;

            if (!g_params.blend_enable) {
                /* =========================================================================
                 * LEGACY DISCRETE MODE BYPASS (blend 0)
                 * ========================================================================= */
                s_corner_blend = horizon.is_sharp_corner ? 1.0f : 0.0f;
                s_blend_target = s_corner_blend;

                /* Steering Calculation */
                if (sr.found == 0 && !s_intersection_ahead) {
                    float active_ff = horizon.is_sharp_corner ? ((float)FRONT_FEEDFORWARD_GAIN * filtered_front_angle) : 0.0f;
                    float candidate_steer = last_valid_steer_top + active_ff;
                    if (fabsf(candidate_steer) > fabsf(last_steer_cmd) &&
                        (candidate_steer * last_steer_cmd >= 0.0f || fabsf(last_steer_cmd) < 5.0f)) {
                        steer_cmd = candidate_steer;
                    } else {
                        steer_cmd = last_steer_cmd;
                    }
                    steer_top = last_valid_steer_top;
                } else {
                    float eff_heading = filtered_heading;
                    if (!horizon.is_sharp_corner && fabsf(eff_heading) < (float)HEADING_DEADBAND_DEG) {
                        eff_heading = 0.0f;
                    }
                    float active_kd = horizon.is_sharp_corner ? g_params.kd_corner : g_params.kd_straight;
                    steer_top = (g_params.k_cte * filtered_cte) +
                                (g_params.k_heading * eff_heading) +
                                (active_kd * filtered_d_cte);
                    if (sr.found > 0) {
                        last_valid_steer_top = steer_top;
                    }

                    steer_cmd = steer_top;
                    if (horizon.is_sharp_corner) {
                        steer_cmd += (float)FRONT_FEEDFORWARD_GAIN * filtered_front_angle;
                    }

                    if (s_intersection_ahead && sr.found == 0) {
                        if (steer_cmd > 3.0f)  steer_cmd = 3.0f;
                        if (steer_cmd < -3.0f) steer_cmd = -3.0f;
                    }

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

                if (s_rollout_ticks <= 5) {
                    steer_cmd = 0.0f;
                }
                if (steer_cmd > max_right_delta) steer_cmd = max_right_delta;
                if (steer_cmd < max_left_delta)  steer_cmd = max_left_delta;
                Steer(steer_cmd + g_params.steering_offset);
                last_steer_cmd = steer_cmd;

                /* Speed & Differential Braking */
                if (horizon.is_sharp_corner) {
                    target_speed = g_params.speed_outside_corner;
                    if (fabsf(steer_cmd) > (float)DIFFERENTIAL_DEADZONE_DEG) {
                        if (steer_cmd > 0.0f) {
                            target_left  = g_params.speed_outside_corner;
                            target_right = g_params.speed_inside_min;
                        } else {
                            target_left  = g_params.speed_inside_min;
                            target_right = g_params.speed_outside_corner;
                        }
                    } else {
                        target_left  = target_speed;
                        target_right = target_speed;
                    }
                } else {
                    target_speed = g_params.speed_straight;
                    float abs_steer = fabsf(steer_cmd);
                    if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
                        float steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                                           ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
                        if (steer_norm > 1.0f) steer_norm = 1.0f;
                        float straight_scale = (abs_steer < 22.0f) ? 0.35f : 1.0f;
                        float diff_drag = straight_scale * steer_norm * (target_speed - g_params.speed_inside_min);

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
            } else {
                /* =========================================================================
                 * CONTINUOUS ASYMMETRIC CORNER BLEND GOVERNOR (blend 1)
                 * ========================================================================= */
                /* 1. Calculate Target & Diode Hold */
                float front_t = 0.0f;
                float head_t  = 0.0f;
                float hold    = 0.0f;

                if (s_intersection_ahead) {
                    /* Suppress corner blend during intersection transit with smooth release */
                    s_corner_blend += g_params.blend_release * (0.0f - s_corner_blend);
                    s_blend_target = 0.0f;
                } else {
                    /* Front Camera Preview Target */
                    if (horizon.found > 0) {
                        front_t = (fabsf(filtered_front_angle) - g_params.front_angle_thresh) / g_params.front_angle_thresh;
                        if (front_t < 0.0f) front_t = 0.0f;
                        if (front_t > 1.0f) front_t = 1.0f;
                    }
                    if (horizon.is_sharp_corner) {
                        front_t = 1.0f; /* Lock to 1.0 on transverse wall or sharp corner flag */
                    }

                    /* Top Camera Heading Floor */
                    float h_range = g_params.h_hi - g_params.h_lo;
                    if (h_range > 0.1f) {
                        head_t = (fabsf(filtered_heading) - g_params.h_lo) / h_range;
                        if (head_t < 0.0f) head_t = 0.0f;
                        if (head_t > 1.0f) head_t = 1.0f;
                    }

                    /* Diode / Ratchet Asymmetric Leaky Integrator */
                    hold = fmaxf(front_t, head_t);
                    s_blend_target = hold;

                    if (front_t > s_corner_blend) {
                        s_corner_blend += g_params.blend_attack * (front_t - s_corner_blend);
                    } else if (hold < s_corner_blend) {
                        s_corner_blend += g_params.blend_release * (hold - s_corner_blend);
                    }
                }

                /* 2. Steering Calculation */
                if (sr.found == 0 && !s_intersection_ahead) {
                    /* Corner apex dropout: preserve or increase turn angle if front camera sees tighter corner */
                    float active_ff = s_corner_blend * (float)FRONT_FEEDFORWARD_GAIN * filtered_front_angle;
                    float candidate_steer = last_valid_steer_top + active_ff;
                    if (fabsf(candidate_steer) > fabsf(last_steer_cmd) &&
                        (candidate_steer * last_steer_cmd >= 0.0f || fabsf(last_steer_cmd) < 5.0f)) {
                        steer_cmd = candidate_steer;
                    } else {
                        steer_cmd = last_steer_cmd;
                    }
                    steer_top = last_valid_steer_top;
                } else {
                    /* Continuous Gain Scheduling between straight and corner gains */
                    float active_k_cte     = g_params.k_cte + s_corner_blend * (g_params.k_cte_corner - g_params.k_cte);
                    float active_k_heading = g_params.k_heading + s_corner_blend * (g_params.k_heading_corner - g_params.k_heading);
                    float active_kd        = g_params.kd_straight + s_corner_blend * (g_params.kd_corner - g_params.kd_straight);

                    float eff_heading = filtered_heading;
                    if (!horizon.is_sharp_corner && s_corner_blend < 0.20f && fabsf(eff_heading) < (float)HEADING_DEADBAND_DEG) {
                        eff_heading = 0.0f;
                    }

                    /* Top Camera Centering + Derivative Damping */
                    steer_top = (active_k_cte * filtered_cte) +
                                (active_k_heading * eff_heading) +
                                (active_kd * filtered_d_cte);
                    if (sr.found > 0) {
                        last_valid_steer_top = steer_top;
                    }

                    steer_cmd = steer_top;
                    /* Front feedforward pre-steer scaled smoothly by corner blend */
                    steer_cmd += s_corner_blend * (float)FRONT_FEEDFORWARD_GAIN * filtered_front_angle;

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
                if (steer_cmd > max_right_delta) steer_cmd = max_right_delta;
                if (steer_cmd < max_left_delta)  steer_cmd = max_left_delta;
                Steer(steer_cmd + g_params.steering_offset);
                last_steer_cmd = steer_cmd;

                /* 3. Speed & Differential Braking */
                /* Continuous blend smoothly lowers speed target from straight to outside corner */
                target_speed = g_params.speed_straight - s_corner_blend * (g_params.speed_straight - g_params.speed_outside_corner);
                target_left  = target_speed;
                target_right = target_speed;

                float abs_steer = fabsf(steer_cmd);
                if (abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
                    float steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                                       ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
                    if (steer_norm > 1.0f) steer_norm = 1.0f;

                    /* Proportional differential drag scaled by corner blend (no premature lockup at small steer angles) */
                    float corner_authority = 0.25f + 0.75f * s_corner_blend;
                    float diff_mult = steer_norm * corner_authority;
                    float diff_drag = diff_mult * (target_speed - g_params.speed_inside_min);

                    if (steer_cmd > 0.0f) {
                        /* Turning Right: Left is outside, Right is inside pivot */
                        target_left  = target_speed;
                        target_right = target_speed - diff_drag;
                    } else {
                        /* Turning Left: Right is outside, Left is inside pivot */
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
                esp_printf("L:%u/%u R:%d F:%d H:%d TS:%d FS:%d S:%d BL:%d BT:%d DOM:%c FV:%u IX:%u ST:%s VL:%d VR:%d\r\n",
                           (unsigned)sr.found, (unsigned)horizon.found,
                           (int)(sr.cte * 10.0f), (int)(filtered_cte * 10.0f),
                           (int)(filtered_heading * 10.0f),
                           (int)(steer_top * 10.0f),
                           (int)(filtered_front_angle * 10.0f),
                           (int)(steer_cmd * 10.0f),
                           (int)(s_corner_blend * 100.0f),
                           (int)(s_blend_target * 100.0f),
                           s_corner_blend > 0.30f ? 'F' : 'T',
                           horizon.found > 0 ? 1U : 0U,
                           (unsigned)s_intersection_ahead,
                           s_intersection_ahead ? "I" : (s_corner_blend > 0.30f ? "C" : "S"),
                           (int)applied_speed_left, (int)applied_speed_right);
            }
        }
    }
}
