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
    uint8_t  s_corner_hold_ticks       = 0; /* Release hold for corner latch */
    uint8_t  s_front_wall_ticks        = 0; /* 2-tick consecutive filter for front corner wall */
    uint8_t  s_straight_ticks          = 0; /* Consecutive straight/centered 2-line ticks with blank front cam */
    uint8_t  s_lost_hold_ticks         = 0; /* Ticks of off-center gentle-decay hold while sr.found == 0 */
    float    s_corner_blend            = 0.0f; /* 0.0 = straight, 1.0 = sharp corner */
    float    s_blend_target            = 0.0f; /* Debug target holding blend */
    corner_severity_t s_corner_sev;            /* Severity-scaled corner speed state (peak hold) */
    corner_severity_reset(&s_corner_sev);

    /* Lookahead Gate & Trigger State Variables */
    float    s_fs_ff                   = 0.0f; /* Lightly filtered front angle EMA(alpha=0.6) */
    float    s_h_ff                    = 0.0f; /* Lightly filtered heading EMA(alpha=0.6) */
    uint8_t  s_cv_sign_ticks           = 0;    /* Consecutive ticks with same sign(cv_ff) */
    int      s_prev_cv_sign            = 0;    /* Previous sign of cv_ff (-1, 0, +1) */
    uint8_t  s_early_brake_trig_ticks  = 0;    /* Consecutive ticks for early brake trigger */
    uint8_t  s_early_brake_rel_ticks   = 0;    /* Consecutive ticks for early brake release */
    bool     s_early_brake_active      = false;/* Early brake active latch */
    uint8_t  s_cv_corner_ticks         = 0;    /* Consecutive ticks of |cv| >= 11 for ST:C */
    uint8_t  s_fv0_ticks               = 0;    /* Consecutive blind front ticks (horizon.found == 0) */
    uint8_t  s_cte_reject_count        = 0;    /* Consecutive single-boundary CTE jump rejects */
    float    s_prev_valid_cte          = 0.0f; /* Previous accepted CTE for jump rejection */
    uint8_t  s_has_prev_valid_cte      = 0;

    PRINTF("Running. Waiting for Start.\r\n");

    while (1) {
        command_poll();

        if (g_tickFlag) {
            g_tickFlag = false;

            /* Hardware push button toggle */
            if (button_pressed_edge()) {
                SDK_DelayAtLeastUs(3000000, SystemCoreClock);
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
                last_valid_steer_top      = 0.0f;
                s_rollout_ticks           = 0;
                s_off_track_ticks         = 0;
                s_intersection_ahead      = 0;
                s_intersection_timer      = 0;
                s_intersection_gap_ticks  = 0;
                s_was_in_intersection_gap = 0;
                s_reacquire_ticks         = 0;
                s_sharp_cancel_count      = 0;
                s_corner_hold_ticks       = 0;
                s_front_wall_ticks        = 0;
                s_straight_ticks          = 0;
                s_lost_hold_ticks         = 0;
                corner_severity_reset(&s_corner_sev);
                s_corner_blend            = 0.0f;
                s_blend_target            = 0.0f;
                s_fs_ff                   = 0.0f;
                s_h_ff                    = 0.0f;
                s_cv_sign_ticks           = 0;
                s_prev_cv_sign            = 0;
                s_early_brake_trig_ticks  = 0;
                s_early_brake_rel_ticks   = 0;
                s_early_brake_active      = false;
                s_cv_corner_ticks         = 0;
                s_fv0_ticks               = 0;
                s_cte_reject_count        = 0;
                s_has_prev_valid_cte      = 0;
                s_prev_valid_cte          = 0.0f;

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
            uint8_t frej_code = 0U;
            if (s_front != kStatus_Success) {
                frej_code = 3U; /* Comm error */
            } else if (num_vectors_front > 0) {
                compute_front_camera_horizon(vectors_front, num_vectors_front, &horizon);
                if (horizon.found == 0) {
                    frej_code = horizon.has_transverse_tape ? 1U : 2U;
                }
                if (horizon.found > 0 &&
                    (!horizon.has_transverse_tape || fabsf(horizon.horizon_angle_deg) > 0.1f || horizon.is_intersection_ahead)) {
                    filtered_front_angle = 0.40f * horizon.horizon_angle_deg + 0.60f * filtered_front_angle;
                } else {
                    filtered_front_angle *= 0.85f;
                }
            } else {
                num_vectors_front = 0;
                filtered_front_angle *= 0.85f;
            }

            /* Blind-camera speed cap tick counter (Item B) */
            if (horizon.found == 0) {
                if (s_fv0_ticks < 255U) s_fv0_ticks++;
            } else {
                s_fv0_ticks = 0U;
            }

            /* Lightly filtered front angle for feedforward: EMA(alpha=0.6) (Item 2) */
            if (horizon.found > 0) {
                s_fs_ff = 0.60f * horizon.horizon_angle_deg + 0.40f * s_fs_ff;
            } else {
                s_fs_ff *= 0.85f;
            }

            /* --- 2. Top / Homography View Camera (LP_FLEXCOMM3) --- */
            status_t s_pid = pixy_get_vectors(&cam_pid, vectors_pid, MAX_VECTORS, &num_vectors_pid);
            steering_result_t sr;
            memset(&sr, 0, sizeof(sr));
            if (s_pid == kStatus_Success && num_vectors_pid > 0) {
                compute_top_camera_lane(vectors_pid, num_vectors_pid, &sr);

                /* Single-boundary CTE jump rejection (Item D) */
                if (sr.found == 1) {
                    float cte_new = sr.cte;
                    if (s_has_prev_valid_cte && fabsf(cte_new - s_prev_valid_cte) > g_params.cte_jump_thresh_px) {
                        if (s_cte_reject_count < g_params.cte_jump_max_rejects) {
                            s_cte_reject_count++;
                            sr.cte = s_prev_valid_cte; /* Discard cte_new, hold cte_prev */
                        } else {
                            /* Accept after 3 consecutive rejects */
                            s_cte_reject_count = 0U;
                            s_prev_valid_cte = cte_new;
                        }
                    } else {
                        s_cte_reject_count = 0U;
                        s_prev_valid_cte = cte_new;
                        s_has_prev_valid_cte = 1U;
                    }
                } else if (sr.found == 2) {
                    s_cte_reject_count = 0U;
                    s_prev_valid_cte = sr.cte;
                    s_has_prev_valid_cte = 1U;
                }

                if (sr.found > 0) {
                    filtered_cte     = 0.60f * sr.cte + 0.40f * filtered_cte;
                    filtered_heading = 0.60f * sr.heading_deg + 0.40f * filtered_heading;
                    s_h_ff           = 0.60f * sr.heading_deg + 0.40f * s_h_ff;
                } else {
                    s_h_ff *= 0.85f;
                }
            } else {
                num_vectors_pid = 0;
                s_h_ff *= 0.85f;
            }

            /* --- Stale Front-Lookahead Release at Corner Exit (50 Hz tick math) ---
             * filtered_front_angle lags the car's rotation: after a corner the top camera already sees a
             * straight, centered lane while FS still reads >= fa_th, which keeps ST:C feed-forward and
             * inside-wheel drag active and over-rotates the car into the outer boundary.
             * Guarded so it never fires at a corner APEX (there the top cam heading is large). */
            if (num_vectors_front == 0 && sr.found == 2 &&
                fabsf(sr.heading_deg) < STALE_FRONT_MAX_HEADING_DEG &&
                fabsf(sr.cte) < STALE_FRONT_MAX_CTE_PX) {
                /* The no-front-vector branch above already applied *0.85 this tick; scale so net = 0.65. */
                filtered_front_angle *= (FRONT_STALE_DECAY_NET / 0.85f);
            }

            /* 6-tick (120 ms) straight-lane confirmation before wiping stale corner state. Requires the
             * front camera to be blank, so a genuine corner entry (front cam sees the bend while the top
             * cam is still straight) is never released. */
            bool top_lane_straight = (sr.found == 2) &&
                                     (fabsf(sr.heading_deg) < STALE_FRONT_MAX_HEADING_DEG) &&
                                     (sr.lane_width_px >= CORNER_RELEASE_LANE_MIN_PX) &&
                                     (sr.lane_width_px <= CORNER_RELEASE_LANE_MAX_PX) &&
                                     (sr.transverse_crosses_center == 0U) &&
                                     (horizon.found == 0U);
            if (top_lane_straight) {
                if (s_straight_ticks < 255U) s_straight_ticks++;
            } else {
                s_straight_ticks = 0U;
            }
            if (s_straight_ticks >= CORNER_RELEASE_STRAIGHT_TICKS) {
                corner_severity_reset(&s_corner_sev);
                filtered_front_angle = 0.0f;
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

            /* --- Corner Priority Preemption & Tactical Intersection State ---
             * If the front camera sees a transverse crossbar that did not yet match Pass 2 exit rules,
             * but ALSO sees straight longitudinal lane lines OR top camera sees both boundaries (sr.found >= 2)
             * while heading and relative front curvature are small, treat the transverse tape as an
             * unconfirmed straight-corridor crossbar rather than a 90-deg corner wall. */
            /* Bug 3 Fix: When horizon.found == 0 (s_fv0_ticks > 0), set curv = 0.0f */
            float curv = 0.0f;
            if (horizon.found > 0 && s_fv0_ticks == 0U) {
                curv = filtered_front_angle - filtered_heading;
            }
            float front_curve_rel = curv;
            bool straight_corridor_crossbar = horizon.has_transverse_tape &&
                                              !horizon.is_intersection_ahead &&
                                              ((horizon.valid_longitudinal_count > 0U) || (sr.found >= 2)) &&
                                              ((fabsf(horizon.horizon_angle_deg) < 20.0f) ||
                                               (fabsf(horizon.horizon_angle_deg - filtered_heading) < 25.0f)) &&
                                              (fabsf(curv) < 25.0f) &&
                                              (!sr.transverse_crosses_center) &&
                                              (fabsf(filtered_heading) < 20.0f);

            /* Require 2 consecutive ticks of raw front_corner_wall so 1-tick pre-intersection
             * crossbar sightings before IX:1 latches never trigger a BT:100 brake check to 65%. */
            uint8_t raw_front_corner_wall = (horizon.has_transverse_tape &&
                                             !horizon.is_intersection_ahead &&
                                             !straight_corridor_crossbar) ? 1U : 0U;
            if (raw_front_corner_wall && !horizon.is_intersection_ahead) {
                if (s_front_wall_ticks < 255U) {
                    s_front_wall_ticks++;
                }
            } else {
                s_front_wall_ticks = 0U;
            }
            uint8_t front_corner_wall = (s_front_wall_ticks >= 2U) ? 1U : 0U;

            float eff_front_angle = filtered_front_angle;

            bool effective_sharp_corner = (front_corner_wall != 0U) ||
                                          (!straight_corridor_crossbar &&
                                           (fabsf(curv) >= g_params.front_angle_thresh ||
                                            (horizon.is_sharp_corner && !horizon.has_transverse_tape)));

            if (sr.transverse_crosses_center) {
                /* Top camera confirms wall across center right in front of car -> immediate corner preempt */
                s_intersection_ahead      = 0;
                s_intersection_timer      = 0;
                s_intersection_gap_ticks  = 0;
                s_was_in_intersection_gap = 0;
                s_reacquire_ticks         = 0;
                s_sharp_cancel_count      = 0;
            } else if (horizon.is_intersection_ahead &&
                       fabsf(filtered_cte) <= IX_BLOCK_CTE_PX &&
                       fabsf(filtered_heading) <= IX_BLOCK_HEADING_DEG) {
                /* Intersection Latching & Lifetime: refresh persistence timer (~15 ticks).
                 * Not latched when far off-center / steeply angled: a "horizontal" front-cam line is then
                 * almost certainly a track boundary crossed diagonally, not an intersection. */
                s_intersection_ahead = 1;
                s_intersection_timer = 15;
                s_sharp_cancel_count = 0;
                s_front_wall_ticks   = 0;
                front_corner_wall    = 0U;
            } else if (effective_sharp_corner || raw_front_corner_wall) {
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

            if (s_intersection_ahead) {
                front_corner_wall = 0U;
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

                    /* Decay CTE and heading across the intersection gap.
                     * Far off-center (|cte| > 10 px): gentle per-tick decay (0.975/0.96 = 0.90/0.85 per
                     * telemetry line) for up to 24 ticks so steering keeps pulling the car back to the
                     * lane; otherwise the original fast 0.70 decay. */
                    if (fabsf(filtered_cte) > OFFCENTER_HOLD_CTE_PX &&
                        s_lost_hold_ticks < OFFCENTER_HOLD_MAX_TICKS) {
                        s_lost_hold_ticks++;
                        filtered_cte     *= OFFCENTER_HOLD_CTE_DECAY;
                        filtered_heading *= OFFCENTER_HOLD_HEAD_DECAY;
                    } else {
                        filtered_cte     *= 0.70f;
                        filtered_heading *= 0.70f;
                    }
                }
                /* If s_intersection_ahead == 0: Corner apex dropout -> zero decay! */
            } else {
                s_off_track_ticks = 0;
                s_lost_hold_ticks = 0;
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
#if STEER_SYMMETRIC_CLAMP
            /* Equal relative throw both ways around the trim so CW and CCW corners get identical steering
             * authority (trim 10 + limits -67/+77 otherwise gives -77 left but only +67 right). */
            {
                float sym = fminf(max_right_delta, -max_left_delta);
                max_right_delta =  sym;
                max_left_delta  = -sym;
            }
#endif

            /* Item 1, 2, 3: cv = FS - H, feedforward cv_ff, confirm w */
            /* Bug 3 Fix: When horizon.found == 0 (s_fv0_ticks > 0), set cv = 0.0f and cv_ff = 0.0f */
            float cv = 0.0f;
            float cv_ff = 0.0f;
            if (horizon.found > 0 && s_fv0_ticks == 0U) {
                cv = filtered_front_angle - filtered_heading;
                cv_ff = s_fs_ff - s_h_ff;
            }

            /* Bug 2(a) Fix: cv_ff must only be active in the direction of fs_ff:
             * if cv_ff * fs_ff <= 0.0f or |cv_ff| > |fs_ff| (meaning FS is closer to 0 deg than H is,
             * i.e., the road ahead is straightening out relative to the car), clamp cv_ff = 0.0f */
            if ((cv_ff * s_fs_ff <= 0.0f) || (fabsf(cv_ff) > fabsf(s_fs_ff))) {
                cv_ff = 0.0f;
            }

            /* Bug 2(b) Fix: When the car is off-center (|filtered_cte| > 8.0f) and filtered_cte * cv_ff < 0.0f
             * (front curvature wants to steer further away from the track center while |H| < 15.0f),
             * gate w = 0.0f and do not allow CV alone to trigger ST:C */
            bool off_center_cv_conflict = (fabsf(filtered_cte) > 8.0f) &&
                                          (filtered_cte * (s_fs_ff - s_h_ff) < 0.0f) &&
                                          (fabsf(filtered_heading) < 15.0f);
            if (off_center_cv_conflict) {
                cv_ff = 0.0f;
            }

            /* Confirm: w = 0 unless horizon.found > 0 AND sign(cv_ff) has been the same for >= 2 ticks */
            int cur_cv_sign = (cv_ff > 0.1f) ? 1 : ((cv_ff < -0.1f) ? -1 : 0);
            if (cur_cv_sign != 0 && cur_cv_sign == s_prev_cv_sign) {
                if (s_cv_sign_ticks < 255U) s_cv_sign_ticks++;
            } else {
                s_cv_sign_ticks = (cur_cv_sign != 0) ? 1U : 0U;
                s_prev_cv_sign = cur_cv_sign;
            }

            float w = 0.0f;
            if (horizon.found > 0 && s_cv_sign_ticks >= 2U && !off_center_cv_conflict) {
                float abs_cv_ff = fabsf(cv_ff);
                w = (abs_cv_ff - g_params.ff_w_lo) / (g_params.ff_w_hi - g_params.ff_w_lo);
                if (w < 0.0f) w = 0.0f;
                if (w > 1.0f) w = 1.0f;
            }
            float top_turn_gate = w;

            /* Item 5: Early brake */
            bool fv_valid = (horizon.found > 0 && num_vectors_front > 0);
            if (fv_valid && fabsf(cv) >= g_params.early_brake_cv_th && fabsf(filtered_heading) < g_params.early_brake_h_max && !off_center_cv_conflict) {
                if (s_early_brake_trig_ticks < 255U) s_early_brake_trig_ticks++;
            } else {
                s_early_brake_trig_ticks = 0U;
            }

            if (s_early_brake_trig_ticks >= 3U) {
                s_early_brake_active = true;
            }

            if (s_early_brake_active) {
                if (fabsf(cv) < g_params.early_brake_release_cv) {
                    if (s_early_brake_rel_ticks < 255U) s_early_brake_rel_ticks++;
                } else {
                    s_early_brake_rel_ticks = 0U;
                }
                if (s_early_brake_rel_ticks >= 3U) {
                    s_early_brake_active = false;
                    s_early_brake_rel_ticks = 0U;
                    s_early_brake_trig_ticks = 0U;
                }
            } else {
                s_early_brake_rel_ticks = 0U;
            }

            /* Item 6: Corner state entry: also enter ST:C when |cv| >= 11 for 2 ticks (suppressed by off-center conflict) */
            if (fabsf(cv) >= g_params.cv_corner_th && !off_center_cv_conflict) {
                if (s_cv_corner_ticks < 255U) s_cv_corner_ticks++;
            } else {
                s_cv_corner_ticks = 0U;
            }
            bool cv_corner_entry = (s_cv_corner_ticks >= g_params.cv_corner_ticks);

            bool top_in_corner = (sr.found == 0) ||
                                 (sr.transverse_crosses_center != 0U) ||
                                 (fabsf(cv) >= g_params.front_angle_thresh) ||
                                 (fabsf(filtered_heading) >= 2.0f);

            /* Bug 1 Fix: Inside ST:C (or upon entering), start front_sev ramping from cv_th */
            float eff_fa_th = (s_corner_hold_ticks > 0U || cv_corner_entry) ?
                              fminf(g_params.front_angle_thresh, g_params.cv_corner_th) :
                              g_params.front_angle_thresh;

            /* Severity-scaled corner speed (both modes): max(front lookahead curv, top apex), 6-tick peak hold,
             * zero while an intersection is latched. */
            float corner_severity = corner_severity_update(&s_corner_sev,
                                                           cv, filtered_heading,
                                                           front_corner_wall,
                                                           sr.transverse_crosses_center ? 1U : 0U,
                                                           s_intersection_ahead ? 1U : 0U,
                                                           eff_fa_th,
                                                           g_params.h_lo, g_params.h_hi);
            float severity_speed = corner_severity_speed(corner_severity,
                                                         g_params.speed_straight,
                                                         g_params.speed_outside_corner);

            if (!g_params.blend_enable) {
                /* =========================================================================
                 * LEGACY DISCRETE MODE BYPASS (blend 0)
                 * is_corner controls feedforward + differential mode only; speed comes from severity.
                 * Latched corner state: exits only when |H| < 8 deg AND |cv| < 8 deg AND hold >= 10 ticks (200 ms).
                 * ========================================================================= */
                bool raw_corner = (!s_intersection_ahead) &&
                                  (cv_corner_entry ||
                                   (top_in_corner &&
                                    (effective_sharp_corner ||
                                     sr.transverse_crosses_center ||
                                     fabsf(filtered_heading) >= 24.0f)));
                if (s_intersection_ahead) {
                    s_corner_hold_ticks = 0U;
                    s_early_brake_active = false;
                } else if (s_corner_hold_ticks > 0U) {
                    if (s_corner_hold_ticks < 255U) {
                        s_corner_hold_ticks++;
                    }
                    if (s_corner_hold_ticks >= 10U &&
                        fabsf(filtered_heading) < 8.0f &&
                        fabsf(cv) < 8.0f &&
                        !raw_corner) {
                        s_corner_hold_ticks = 0U;
                    }
                } else if (raw_corner) {
                    s_corner_hold_ticks = 1U;
                }
                bool is_corner = (s_corner_hold_ticks > 0U);
                if (is_corner) {
                    s_early_brake_active = false; /* ST:C takes over from early brake */
                }
                s_corner_blend = is_corner ? 1.0f : 0.0f;
                s_blend_target = corner_severity; /* BT: in telemetry = speed severity (0..100) */

                /* Steering Calculation */
                if (sr.found == 0 && !s_intersection_ahead) {
                    steer_top = last_valid_steer_top;
                } else {
                    float eff_heading = filtered_heading;
                    if (!is_corner && fabsf(eff_heading) < (float)HEADING_DEADBAND_DEG) {
                        eff_heading = 0.0f;
                    }
                    steer_top = (g_params.k_cte * filtered_cte) + (g_params.k_heading * eff_heading);
                    if (sr.found > 0) {
                        last_valid_steer_top = steer_top;
                    }
                }

                /* S_target = TS + 0.8 * cv_ff * w */
                float s_target = steer_top + (g_params.ff_gain * cv_ff * w);

                if (s_intersection_ahead && sr.found == 0 &&
                    fabsf(filtered_cte) <= OFFCENTER_HOLD_CTE_PX) {
                    if (s_target > 3.0f)  s_target = 3.0f;
                    if (s_target < -3.0f) s_target = -3.0f;
                }

                /* S = prev_S + clamp(S_target - prev_S, -90, +90) (deg10 per tick = 9.0 deg/tick) */
                float delta_s = s_target - last_steer_cmd;
                if (delta_s > g_params.steer_slew_max)  delta_s = g_params.steer_slew_max;
                if (delta_s < -g_params.steer_slew_max) delta_s = -g_params.steer_slew_max;
                steer_cmd = last_steer_cmd + delta_s;

                if (s_rollout_ticks <= 5) {
                    steer_cmd = 0.0f;
                }
                if (steer_cmd > max_right_delta) steer_cmd = max_right_delta;
                if (steer_cmd < max_left_delta)  steer_cmd = max_left_delta;
                Steer(steer_cmd + g_params.steering_offset);
                last_steer_cmd = steer_cmd;

                /* Speed & Differential Braking */
                if (is_corner) {
                    target_speed = severity_speed;
                    /* Bug 1 Fix: never allow target_speed in ST:C to exceed eb_spd (70.0%) during first 12 ticks (HT <= 12) */
                    if (s_corner_hold_ticks <= 12U) {
                        target_speed = fminf(target_speed, g_params.early_brake_speed);
                    }
                } else {
                    target_speed = g_params.speed_straight;
                    /* Item 5: Early brake speed cap */
                    if (s_early_brake_active) {
                        target_speed = fminf(target_speed, g_params.early_brake_speed);
                    }
                }
                /* Bug 3 Fix: enforce blind_spd (70%) cap across both ST:S and ST:C whenever s_fv0_ticks > 3 */
                if (s_fv0_ticks > g_params.blind_speed_ticks) {
                    target_speed = fminf(target_speed, g_params.blind_speed_cap);
                }

                float abs_steer = fabsf(steer_cmd);
                bool exit_straightening = ((filtered_cte * steer_cmd < 0.0f) &&
                                           (fabsf(filtered_cte) > 3.0f) &&
                                           (fabsf(filtered_heading) < 18.0f) &&
                                           (fabsf(cv) < 15.0f) &&
                                           !front_corner_wall &&
                                           !sr.transverse_crosses_center) ||
                                          (fabsf(cv) < g_params.front_angle_thresh &&
                                           fabsf(filtered_heading) < 12.0f &&
                                           !front_corner_wall &&
                                           !sr.transverse_crosses_center &&
                                           !is_corner) ||
                                          ((filtered_front_angle * steer_cmd < 0.0f) &&
                                           (fabsf(filtered_front_angle) > 10.0f) &&
                                           (fabsf(filtered_heading) < 35.0f)) ||
                                          (((filtered_cte * filtered_heading > 0.0f) ||
                                            (filtered_heading * filtered_front_angle > 0.0f)) &&
                                           (fabsf(filtered_cte) > 4.0f) &&
                                           (fabsf(filtered_heading) < 24.0f) &&
                                           (fabsf(cv) < 15.0f) &&
                                           !front_corner_wall &&
                                           !sr.transverse_crosses_center);
                if (!exit_straightening && abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG && is_corner) {
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
            } else {
                /* =========================================================================
                 * CONTINUOUS ASYMMETRIC CORNER BLEND GOVERNOR (blend 1)
                 * ========================================================================= */
                /* 1. Calculate Target & Diode Hold from unified corner_severity */
                if (s_intersection_ahead) {
                    /* Suppress corner blend during intersection transit with smooth release */
                    s_corner_blend += g_params.blend_release * (0.0f - s_corner_blend);
                    s_blend_target = 0.0f;
                } else {
                    float hold = corner_severity;
                    s_blend_target = hold;

                    if (hold > s_corner_blend) {
                        s_corner_blend += g_params.blend_attack * (hold - s_corner_blend);
                    } else if (hold < s_corner_blend) {
                        s_corner_blend += g_params.blend_release * (hold - s_corner_blend);
                    }
                }

                /* 2. Steering Calculation */
                if (sr.found == 0 && !s_intersection_ahead) {
                    steer_top = last_valid_steer_top;
                } else {
                    /* Continuous Gain Scheduling between straight and corner gains */
                    float active_k_cte     = g_params.k_cte + s_corner_blend * (g_params.k_cte_corner - g_params.k_cte);
                    float active_k_heading = g_params.k_heading + s_corner_blend * (g_params.k_heading_corner - g_params.k_heading);

                    float eff_heading = filtered_heading;
                    if (!effective_sharp_corner && s_corner_blend < 0.20f && fabsf(eff_heading) < (float)HEADING_DEADBAND_DEG) {
                        eff_heading = 0.0f;
                    }

                    /* Top Camera Centering + Heading Damping */
                    steer_top = (active_k_cte * filtered_cte) + (active_k_heading * eff_heading);
                    if (sr.found > 0) {
                        last_valid_steer_top = steer_top;
                    }
                }

                /* S_target = TS + 0.8 * cv_ff * w */
                float s_target = steer_top + (g_params.ff_gain * cv_ff * w);

                if (s_intersection_ahead && sr.found == 0 &&
                    fabsf(filtered_cte) <= OFFCENTER_HOLD_CTE_PX) {
                    /* Intersection blind gap: clamp held steering to +-3.0 deg (only when near lane center) */
                    if (s_target > 3.0f)  s_target = 3.0f;
                    if (s_target < -3.0f) s_target = -3.0f;
                }

                /* S = prev_S + clamp(S_target - prev_S, -90, +90) (deg10 per tick = 9.0 deg/tick) */
                float delta_s = s_target - last_steer_cmd;
                if (delta_s > g_params.steer_slew_max)  delta_s = g_params.steer_slew_max;
                if (delta_s < -g_params.steer_slew_max) delta_s = -g_params.steer_slew_max;
                steer_cmd = last_steer_cmd + delta_s;

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
                float base_speed = g_params.speed_straight - s_corner_blend * (g_params.speed_straight - g_params.speed_outside_corner);
                /* Bug 3 Fix: enforce blind_spd across both straight and corner whenever s_fv0_ticks > 3 */
                if (s_fv0_ticks > g_params.blind_speed_ticks) {
                    base_speed = fminf(base_speed, g_params.blind_speed_cap);
                }
                if (s_early_brake_active && s_corner_blend < 0.20f) {
                    base_speed = fminf(base_speed, g_params.early_brake_speed);
                }
                /* Bug 1 Fix: never allow target_speed in corner to exceed eb_spd during first 12 ticks */
                if (s_corner_hold_ticks <= 12U && s_corner_blend > 0.20f) {
                    base_speed = fminf(base_speed, g_params.early_brake_speed);
                }
                target_speed = base_speed;
                target_left  = target_speed;
                target_right = target_speed;

                float abs_steer = fabsf(steer_cmd);
                bool exit_straightening = ((filtered_cte * steer_cmd < 0.0f) &&
                                           (fabsf(filtered_cte) > 3.0f) &&
                                           (fabsf(filtered_heading) < 18.0f) &&
                                           (fabsf(cv) < 15.0f) &&
                                           !front_corner_wall &&
                                           !sr.transverse_crosses_center) ||
                                          (fabsf(cv) < g_params.front_angle_thresh &&
                                           fabsf(filtered_heading) < 12.0f &&
                                           !front_corner_wall &&
                                           !sr.transverse_crosses_center &&
                                           (s_corner_blend < 0.25f)) ||
                                          ((filtered_front_angle * steer_cmd < 0.0f) &&
                                           (fabsf(filtered_front_angle) > 10.0f) &&
                                           (fabsf(filtered_heading) < 35.0f)) ||
                                          (((filtered_cte * filtered_heading > 0.0f) ||
                                            (filtered_heading * filtered_front_angle > 0.0f)) &&
                                           (fabsf(filtered_cte) > 4.0f) &&
                                           (fabsf(filtered_heading) < 24.0f) &&
                                           (fabsf(cv) < 15.0f) &&
                                           !front_corner_wall &&
                                           !sr.transverse_crosses_center);
                if (!exit_straightening && abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
                    float steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                                       ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
                    if (steer_norm > 1.0f) steer_norm = 1.0f;

                    /* Proportional differential drag scaled by corner blend (gated by top_in_corner to avoid early pivot) */
                    float corner_authority = 0.25f + 0.75f * (top_in_corner ? s_corner_blend : 0.0f);
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

            /* --- 5. Motor Slew Rate Limiting (Ramps) & Fast Inside-Wheel Exit Equalization --- */
            bool equal_exit_ramp = (target_left >= applied_speed_left) &&
                                   (target_right >= applied_speed_right) &&
                                   (fabsf(target_left - target_right) < 1.0f);
            bool left_lagging    = equal_exit_ramp && (applied_speed_left < applied_speed_right);
            bool right_lagging   = equal_exit_ramp && (applied_speed_right < applied_speed_left);

            /* Left Motor Ramp */
            if (target_left < applied_speed_left) {
                applied_speed_left -= g_params.decel_ramp;
                if (applied_speed_left < target_left) applied_speed_left = target_left;
            } else {
                float step_l = left_lagging ? (2.0f * g_params.accel_ramp) : g_params.accel_ramp;
                applied_speed_left += step_l;
                if (applied_speed_left > target_left) applied_speed_left = target_left;
            }

            /* Right Motor Ramp */
            if (target_right < applied_speed_right) {
                applied_speed_right -= g_params.decel_ramp;
                if (applied_speed_right < target_right) applied_speed_right = target_right;
            } else {
                float step_r = right_lagging ? (2.0f * g_params.accel_ramp) : g_params.accel_ramp;
                applied_speed_right += step_r;
                if (applied_speed_right > target_right) applied_speed_right = target_right;
            }

            if (left_lagging && applied_speed_left > applied_speed_right) {
                applied_speed_left = applied_speed_right;
            }
            if (right_lagging && applied_speed_right > applied_speed_left) {
                applied_speed_right = applied_speed_left;
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
                esp_printf("L:%u/%u R:%d F:%d H:%d TS:%d FS:%d S:%d CV:%d TG:%d HT:%u BL:%d BT:%d DOM:%c FV:%u IX:%u ST:%s VL:%d VR:%d CR:%u FRAW:%u FREJ:%u FV0:%u\r\n",
                           (unsigned)sr.found, (unsigned)horizon.found,
                           (int)(sr.cte * 10.0f), (int)(filtered_cte * 10.0f),
                           (int)(filtered_heading * 10.0f),
                           (int)(steer_top * 10.0f),
                           (int)(filtered_front_angle * 10.0f),
                           (int)(steer_cmd * 10.0f),
                           (int)(curv * 10.0f),
                           (int)(top_turn_gate * 100.0f),
                           (unsigned)s_corner_hold_ticks,
                           (int)(s_corner_blend * 100.0f),
                           (int)(s_blend_target * 100.0f),
                           s_corner_blend > 0.30f ? 'F' : 'T',
                           horizon.found > 0 ? 1U : 0U,
                           (unsigned)s_intersection_ahead,
                           s_intersection_ahead ? "I" : (s_corner_blend > 0.30f ? "C" : "S"),
                           (int)applied_speed_left, (int)applied_speed_right,
                           (unsigned)s_cte_reject_count,
                           (unsigned)num_vectors_front,
                           (unsigned)frej_code,
                           (unsigned)s_fv0_ticks);
            }
        }
    }
}
