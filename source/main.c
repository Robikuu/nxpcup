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
    /* Initialize on-board Red LED as preset indicator (active-low, start OFF = Preset 1) */
    LED_RED_INIT(LOGIC_LED_OFF);

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
    uint8_t  s_corner_hold_ticks       = 0; /* 3-tick release hold for blend 0 corner latch */
    uint8_t  s_front_wall_ticks        = 0; /* 2-tick consecutive filter for front corner wall */
    uint8_t  s_straight_ticks          = 0; /* Consecutive straight/centered 2-line ticks with blank front cam */
    uint8_t  s_lost_hold_ticks         = 0; /* Ticks of off-center gentle-decay hold while sr.found == 0 */
    float    s_corner_blend            = 0.0f; /* 0.0 = straight, 1.0 = sharp corner */
    float    s_blend_target            = 0.0f; /* Debug target holding blend */
    corner_severity_t s_corner_sev;            /* Severity-scaled corner speed state (peak hold) */
    corner_severity_reset(&s_corner_sev);

    PRINTF("Running. Waiting for Start.\r\n");

    while (1) {
        command_poll();

        if (g_tickFlag) {
            g_tickFlag = false;

            /* Hardware push button handling:
             * - In RUNNING: Any press immediately halts the vehicle (fail-safe emergency stop).
             * - In IDLE: Short tap & release starts the vehicle;
             *            Holding for >= BUTTON_HOLD_TICKS (1.0s) switches between Preset 1 & Preset 2.
             */
            button_event_t btn_evt = button_update();

            if (g_params.state == STATE_RUNNING) {
                if (btn_evt == BUTTON_EVT_PRESS) {
                    motor_test_stop();
                    g_params.state = STATE_IDLE;
                    HbridgeBrake(&g_hbridge);
                    button_consume_press();
                }
            } else {
                if (btn_evt == BUTTON_EVT_SHORT_CLICK) {
                    motor_test_stop();
                    g_params.state = STATE_RUNNING;
                } else if (btn_evt == BUTTON_EVT_HOLD) {
                    params_switch_preset();
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

            /* --- 2. Top / Homography View Camera (LP_FLEXCOMM3) --- */
            status_t s_pid = pixy_get_vectors(&cam_pid, vectors_pid, MAX_VECTORS, &num_vectors_pid);
            steering_result_t sr;
            memset(&sr, 0, sizeof(sr));
            if (s_pid == kStatus_Success && num_vectors_pid > 0) {
                compute_top_camera_lane(vectors_pid, num_vectors_pid, &sr);
                if (sr.found > 0) {
                    filtered_cte     = 0.60f * sr.cte + 0.40f * filtered_cte;
                    filtered_heading = 0.60f * sr.heading_deg + 0.40f * filtered_heading;
                }
            } else {
                num_vectors_pid = 0;
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
                s_corner_hold_ticks  = 0U;
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
            float front_curve_rel = filtered_front_angle - filtered_heading;
            bool straight_corridor_crossbar = horizon.has_transverse_tape &&
                                              !horizon.is_intersection_ahead &&
                                              ((horizon.valid_longitudinal_count > 0U) || (sr.found >= 2)) &&
                                              ((fabsf(horizon.horizon_angle_deg) < 20.0f) ||
                                               (fabsf(horizon.horizon_angle_deg - filtered_heading) < 25.0f)) &&
                                              (fabsf(front_curve_rel) < 25.0f) &&
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

            /* True road curvature guard against straightaway chassis yaw:
             * When off-center on a straightaway without a corner wall and front camera simply reflects
             * chassis yaw (H * FS > 0, |H| < 24 deg, and |FS - H| < fa_th), use (FS - H) as the effective front angle. */
            bool straight_yaw_artifact = (fabsf(filtered_cte) > 4.0f) &&
                                         (fabsf(filtered_heading) < 24.0f) &&
                                         (!front_corner_wall) &&
                                         (!sr.transverse_crosses_center) &&
                                         (filtered_heading * filtered_front_angle > 0.0f) &&
                                         (fabsf(front_curve_rel) < g_params.front_angle_thresh);
            float eff_front_angle = straight_yaw_artifact ? front_curve_rel : filtered_front_angle;

            bool effective_sharp_corner = (front_corner_wall != 0U) ||
                                          (!straight_corridor_crossbar &&
                                           (fabsf(eff_front_angle) >= g_params.front_angle_thresh ||
                                            (horizon.is_sharp_corner && !horizon.has_transverse_tape && !straight_yaw_artifact)));

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

            /* Delayed, sign-aware corner-entry gate for front-camera steering feedforward:
             * - 0.0 while the top camera still sees a straight/approach lane (|heading| < 12 deg, no top wall),
             *   so the car brakes early via corner_severity without turning in before reaching the curve.
             * - Ramps smoothly from 0.0 to 1.0 across |heading| = 12.0 deg..28.0 deg in the turn direction.
             * - Capped at 0.35f while both lane boundaries are visible in the top camera (sr.found == 2),
             *   because steer_top already tracks 2-line CTE + heading; full 1.0f at single-line/blind apex.
             * - Scaled down by inside_scale whenever the car is already on the inside of the turn
             *   (filtered_cte * eff_front_angle < 0) so feedforward never pulls across the inside curb.
             */
            bool top_in_corner = (sr.found == 0) ||
                                 (sr.transverse_crosses_center != 0U) ||
                                 (fabsf(filtered_heading) >= 12.0f &&
                                  (filtered_heading * eff_front_angle > 0.0f || fabsf(filtered_heading) >= 24.0f));
            float top_turn_gate = 0.0f;
            if (sr.found == 0 || sr.transverse_crosses_center != 0U) {
                top_turn_gate = 1.0f;
            } else if ((fabsf(eff_front_angle) >= g_params.front_angle_thresh || front_corner_wall != 0U) &&
                       (filtered_heading * eff_front_angle > 0.0f) &&
                       (fabsf(filtered_heading) >= 12.0f)) {
                float h_norm = (fabsf(filtered_heading) - 12.0f) / (28.0f - 12.0f);
                if (h_norm > 1.0f) h_norm = 1.0f;
                top_turn_gate = h_norm;
                if (sr.found == 2 && top_turn_gate > 0.35f) {
                    top_turn_gate = 0.35f;
                }
            }

            if ((sr.found > 0) && (filtered_cte * eff_front_angle < 0.0f) && !sr.transverse_crosses_center) {
                float inside_scale = (12.0f - fabsf(filtered_cte)) / (12.0f - 3.0f);
                if (inside_scale < 0.0f) inside_scale = 0.0f;
                if (inside_scale > 1.0f) inside_scale = 1.0f;
                top_turn_gate *= inside_scale;
            }

            /* Severity-scaled corner speed (both modes): max(front lookahead, top apex), 6-tick peak hold,
             * zero while an intersection is latched. */
            float corner_severity = corner_severity_update(&s_corner_sev,
                                                           eff_front_angle, filtered_heading,
                                                           front_corner_wall,
                                                           sr.transverse_crosses_center ? 1U : 0U,
                                                           s_intersection_ahead ? 1U : 0U,
                                                           g_params.front_angle_thresh,
                                                           g_params.h_lo, g_params.h_hi);
            float severity_speed = corner_severity_speed(corner_severity,
                                                         g_params.speed_straight,
                                                         g_params.speed_outside_corner);

            if (!g_params.blend_enable) {
                /* =========================================================================
                 * LEGACY DISCRETE MODE BYPASS (blend 0)
                 * is_corner controls feedforward + differential mode only; speed comes from severity.
                 * Gated by top_in_corner so ST:C and full inside-wheel drag wait until the car
                 * actually reaches the curve entrance in the top camera.
                 * ========================================================================= */
                bool raw_corner = (!s_intersection_ahead) &&
                                  top_in_corner &&
                                  (effective_sharp_corner ||
                                   sr.transverse_crosses_center ||
                                   fabsf(filtered_heading) >= 24.0f);
                if (s_intersection_ahead) {
                    s_corner_hold_ticks = 0U;
                } else if (raw_corner) {
                    s_corner_hold_ticks = 3U; /* 3-tick release hold prevents 1-tick ST:C <-> ST:S chattering */
                } else if (s_corner_hold_ticks > 0U) {
                    s_corner_hold_ticks--;
                }
                bool is_corner = (s_corner_hold_ticks > 0U);
                s_corner_blend = is_corner ? 1.0f : 0.0f;
                s_blend_target = corner_severity; /* BT: in telemetry = speed severity (0..100) */

                /* Steering Calculation */
                if (sr.found == 0 && !s_intersection_ahead) {
                    float active_ff = is_corner ? ((float)FRONT_FEEDFORWARD_GAIN * filtered_front_angle) : 0.0f;
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
                    if (!is_corner && fabsf(eff_heading) < (float)HEADING_DEADBAND_DEG) {
                        eff_heading = 0.0f;
                    }
                    steer_top = (g_params.k_cte * filtered_cte) + (g_params.k_heading * eff_heading);
                    if (sr.found > 0) {
                        last_valid_steer_top = steer_top;
                    }

                    steer_cmd = steer_top;
                    if (is_corner) {
                        steer_cmd += top_turn_gate * (float)FRONT_FEEDFORWARD_GAIN * eff_front_angle;
                    }

                    if (s_intersection_ahead && sr.found == 0 &&
                        fabsf(filtered_cte) <= OFFCENTER_HOLD_CTE_PX) {
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

                /* Speed & Differential Braking (disabled during exit counter-steer / straight recovery) */
                target_speed = severity_speed;
                float abs_steer = fabsf(steer_cmd);
                bool exit_straightening = ((filtered_cte * steer_cmd < 0.0f) &&
                                           (fabsf(filtered_cte) > 3.0f) &&
                                           (fabsf(filtered_heading) < 18.0f) &&
                                           (fabsf(front_curve_rel) < 15.0f) &&
                                           !front_corner_wall &&
                                           !sr.transverse_crosses_center) ||
                                          (fabsf(eff_front_angle) < g_params.front_angle_thresh &&
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
                                           (fabsf(front_curve_rel) < 15.0f) &&
                                           !front_corner_wall &&
                                           !sr.transverse_crosses_center);
                if (!exit_straightening && abs_steer > (float)DIFFERENTIAL_DEADZONE_DEG) {
                    float steer_norm = (abs_steer - (float)DIFFERENTIAL_DEADZONE_DEG) /
                                       ((float)STEER_FULL_BRAKE_DEG - (float)DIFFERENTIAL_DEADZONE_DEG);
                    if (steer_norm > 1.0f) steer_norm = 1.0f;
                    float mult = is_corner ? 1.0f : 0.25f;
                    float diff_drag = mult * steer_norm * (target_speed - g_params.speed_inside_min);

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

                    float eff_heading = filtered_heading;
                    if (!effective_sharp_corner && s_corner_blend < 0.20f && fabsf(eff_heading) < (float)HEADING_DEADBAND_DEG) {
                        eff_heading = 0.0f;
                    }

                    /* Top Camera Centering + Heading Damping */
                    steer_top = (active_k_cte * filtered_cte) + (active_k_heading * eff_heading);
                    if (sr.found > 0) {
                        last_valid_steer_top = steer_top;
                    }

                    steer_cmd = steer_top;
                    /* Front feedforward pre-steer gated by top-camera turn entry (prevents early turn-in on straights) */
                    steer_cmd += s_corner_blend * top_turn_gate * (float)FRONT_FEEDFORWARD_GAIN * eff_front_angle;

                    if (s_intersection_ahead && sr.found == 0 &&
                        fabsf(filtered_cte) <= OFFCENTER_HOLD_CTE_PX) {
                        /* Intersection blind gap: clamp held steering to +-3.0 deg (only when near lane center) */
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
                bool exit_straightening = ((filtered_cte * steer_cmd < 0.0f) &&
                                           (fabsf(filtered_cte) > 3.0f) &&
                                           (fabsf(filtered_heading) < 18.0f) &&
                                           (fabsf(front_curve_rel) < 15.0f) &&
                                           !front_corner_wall &&
                                           !sr.transverse_crosses_center) ||
                                          (fabsf(eff_front_angle) < g_params.front_angle_thresh &&
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
                                           (fabsf(front_curve_rel) < 15.0f) &&
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
