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
#define LINE_LOST_LIMIT    5

/* heading is already in radians from compute_center_line(); convert to
   degrees since k_heading/CORNER_HEADING_THRESHOLD_DEG are tuned in degrees.
   cte from compute_center_line() is already in real cm -- no CTE_TO_CM
   conversion needed (the old *100.0f here was a leftover double-scale bug). */
#define HEADING_TO_DEG     (180.0f / (float)M_PI)

/* ---- corner-detection tuning -- these are starting guesses, not
   measured values. Tune against your actual track using print_frame_debug
   / live_vectors.py to see what heading your real corners produce, the
   same way everything else in this pipeline got tuned. ---- */
//#define CORNER_HEADING_THRESHOLD_DEG   20.0f
//#define CORNER_ENTER_STREAK            3   /* consecutive corner-like ticks before braking */
//#define CORNER_EXIT_STREAK             5   /* consecutive straight ticks before resuming -- higher than
//                                               ENTER on purpose: quick to get cautious, slower to fully commit again */

/* single-vector fallback staging: 0 = log the estimate only, never steer
   on it. 1 = let it drive. Flip only after reviewing logged single-vector
   vs 2-line comparisons per the testing order in the design doc. */
#define SINGLE_VECTOR_FALLBACK_LIVE 1

#define TICK_CHANNEL kMRT_Channel_0
#define TIMER_CLOCK_HZ 150000000U

volatile bool g_tickFlag = false;

typedef enum {
    DRIVE_FORWARD,
    DRIVE_BRAKING
} drive_state_t;

void MRT0_IRQHandler(void) {
	MRT_ClearStatusFlags(MRT0, TICK_CHANNEL, kMRT_TimerInterruptFlag);
	g_tickFlag = true;
	SDK_ISR_EXIT_BARRIER;
}

static void reset_pid_state(pid_struct *pid)
{
    pid->integral         = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->d_filtered       = 0.0f;
    pid->prev_output      = 0.0f;
    pid->initialized      = 0;
}

int main(void)
{
    /* ---------- 1. Hardware ---------- */
	BOARD_InitHardware();
    BOARD_InitBootPins();
    BOARD_InitBootPeripherals();

    /* ---------- 2. Params FIRST (everything below reads g_params) ---------- */
    params_init();

    /* ---------- 3. Actuators: safe state immediately ---------- */
    HbridgeInit(&g_hbridge,
                CTIMER0_PERIPHERAL,
                CTIMER0_PWM_PERIOD_CH,
                CTIMER0_PWM_1_CHANNEL,
                CTIMER0_PWM_2_CHANNEL,
                GPIO0, 24U,
                GPIO0, 27U);
    HbridgeBrake(&g_hbridge);
    Steer(g_params.steering_offset);

    //TestHbridge();

    /* ---------- 4. Steering controller ---------- */
    static pid_struct steer_pid;
    pid_init(&steer_pid,
             g_params.kp, g_params.ki, g_params.kd,
             100.0f, g_params.alpha);

    SDK_DelayAtLeastUs(3000000, SystemCoreClock);
    /* ---------- 5. Camera ---------- */
    static pixy_t cam1;
    status_t ps = pixy_init(&cam1, LPI2C2, 0x54U,
                            &LP_FLEXCOMM2_RX_Handle,
                            &LP_FLEXCOMM2_TX_Handle);
    if (ps == kStatus_Success) {
        pixy_set_led(&cam1, 0, 255, 0);      /* green = camera OK */
    } else {
        PRINTF("Pixy init FAILED\r\n");      /* car stays IDLE */
    }

    uint32_t line_lost_counter = 0;
    uint16_t vectors[MAX_VECTORS * 6];
    size_t   num_vectors;
    float    filtered_cte    = 0.0f;
    int      cte_filter_init = 0;

    /* single-vector fallback tracking state -- see vectors.h for
       tracked_center_t and the tuning constants used below. */
    static tracked_center_t track_ref = {0};
    side_t single_side;

    PRINTF("Booted. Send 'start;' to run, 'stop;' to halt.\r\n");

    CLOCK_EnableClock(kCLOCK_Mrt);           // ungate MRT0's clock

	mrt_config_t mrtConfig;
	MRT_GetDefaultConfig(&mrtConfig);
	MRT_Init(MRT0, &mrtConfig);

	MRT_SetupChannelMode(MRT0, TICK_CHANNEL, kMRT_RepeatMode);
	MRT_EnableInterrupts(MRT0, TICK_CHANNEL, kMRT_TimerInterruptEnable);
	EnableIRQ(MRT0_IRQn);

	uint32_t mrtClockHz = CLOCK_GetFreq(kCLOCK_BusClk); // see note below
	MRT_StartTimer(MRT0, TICK_CHANNEL, MSEC_TO_COUNT(20, mrtClockHz));

	float average = 0.0f;
	uint32_t count = 0;

    /* ---------- 6. Main loop ---------- */
    while (1) {

        command_poll();

        if (g_tickFlag) {
        	g_tickFlag = false;

            /* live-update gains from remote tuning (no pid_init re-call!) */
            steer_pid.kp    = g_params.kp;
            steer_pid.ki    = g_params.ki;
            steer_pid.kd    = g_params.kd;
            steer_pid.alpha = g_params.alpha;

            if (g_params.state == STATE_RUNNING) {

                /* age the single-vector reference BEFORE this tick's
                   selection/estimation, and before it's possibly reset
                   to 0 below by a fresh 2-line update -- see age_ticks
                   discussion: this ordering is what keeps a frame that
                   updates the reference reading back as age_ticks==0
                   immediately, instead of 1. */
                age_tracked_center(&track_ref);

                status_t s = pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors);

                if (s != kStatus_Success || num_vectors == 0) {
                    /* camera failure and "no line at all" get identical treatment,
                       unchanged from before -- this is a harder failure than a
                       corner, so it's kept separate from the drive/brake state
                       machine below rather than folded into it. */
                    if (++line_lost_counter > LINE_LOST_LIMIT) {
                        HbridgeBrake(&g_hbridge);
                    }
                }
                else {
                    line_lost_counter = 0;

//                    /* --- raw vector dump: what Pixy2 actually returned, before selection --- */
//					esp_printf("n=%u\n", (unsigned)num_vectors);
//					for (size_t i = 0; i < num_vectors; i++) {
//						uint16_t x0    = vectors[i*6 + 0];
//						uint16_t y0    = vectors[i*6 + 1];
//						uint16_t x1    = vectors[i*6 + 2];
//						uint16_t y1    = vectors[i*6 + 3];
//						uint16_t index = vectors[i*6 + 4];
//						uint16_t flags = vectors[i*6 + 5];
//						esp_printf("  v%u: x0=%u y0=%u x1=%u y1=%u idx=%u flags=%u\n",
//								   (unsigned)i, x0, y0, x1, y1, index, flags);
//					}

                    ground_vector_t left, right;
                    uint8_t found = select_boundary_vectors(vectors, num_vectors,
                                                             &track_ref, &left, &right,
                                                             &single_side);

                    float   e_cte     = 0.0f;
                    float   e_heading = 0.0f;
                    uint16_t valid_error = 0;

                    if (found == 2) {
						center_line_t c = compute_center_line(&left, &right);

						// Tightened the limit to 1.0x! Do not update reference if CTE implies we are off-track.
						if (c.valid && fabsf(c.cte) <= TRACK_HALF_WIDTH_CM) {
							e_cte     = c.cte;
							e_heading = c.heading * HEADING_TO_DEG;
							valid_error = 1;
							update_tracked_center(&track_ref, e_cte);
						} else {
							// 2-line frame failed sanity checks. We saw a phantom line.
							// Downgrade to 1-line mode to try and survive!
							found = 1;

							float nearX_left = (left.Y0 < left.Y1) ? left.X0 : left.X1;
							float nearX_right = (right.Y0 < right.Y1) ? right.X0 : right.X1;

							float d_left = fabsf(nearX_left - track_ref.cte_est);
							float d_right = fabsf(nearX_right - track_ref.cte_est);

							if (d_left < d_right) {
							    single_side = classify_side(nearX_left, &track_ref);
							    // 'left' vector already holds the correct line
							} else {
							    single_side = classify_side(nearX_right, &track_ref);
							    left = right; // move the surviving vector into 'left' for the 1vec logic
							}
						}
					}

                    if (found == 1) {
                        float est_cte;

                        svest_reason_t reason = estimate_single_vector_cte_debug(&left, single_side, &track_ref, &est_cte);
                        uint8_t est_ok = (reason == SVEST_OK);

                        if (est_ok) {
                            esp_printf("1vec side=%d ok=1 est=%d ref=%d age=%u\n",
                                       (int)single_side, (int)(est_cte * 100),
                                       (int)(track_ref.cte_est * 100), (unsigned)track_ref.age_ticks);
                        } else {
                        	esp_printf("1vec side=%d ok=0 reason=%d ref=%d age=%u\n",
                        	               (int)single_side, (int)reason,
                        	               (int)(track_ref.cte_est * 100), (unsigned)track_ref.age_ticks);
                        }

#if SINGLE_VECTOR_FALLBACK_LIVE
                        if (est_ok) {
							e_cte       = est_cte;
							float dx = left.X1 - left.X0;
							float dy = left.Y1 - left.Y0;
							e_heading = atan2f(dx, dy) * HEADING_TO_DEG;
							valid_error = 1;

							// SAFE 1-VECTOR UPDATE: Roll the reference slowly (90% old, 10% new estimate)
							// This prevents sudden poisoning while allowing it to survive long turns.
							if (track_ref.initialized) {
								track_ref.cte_est = 0.90f * track_ref.cte_est + 0.10f * est_cte;
								track_ref.age_ticks = 0; // Reset age so it doesn't time out
							} else {
								update_tracked_center(&track_ref, est_cte);
							}
						}
#endif
                    }
                    else {
                    	if (found != 2)
                    		esp_printf("found0----------\n");
                    }

                    if (valid_error) {
                        /* EMA filter on CTE to smooth frame-to-frame noise */
                        if (!cte_filter_init) {
                            filtered_cte    = e_cte;
                            cte_filter_init = 1;
                        }
                        else {
                            filtered_cte = g_params.cte_alpha * e_cte + (1 - g_params.cte_alpha) * filtered_cte;
                        }

                        float combined = g_params.k_cte * filtered_cte
                                       + g_params.k_heading * e_heading;

                        float steer_cmd = -pid_update(&steer_pid, 0.0f, combined, g_params.dt);

                        /* debug: L=boundaries found  R=raw_cte*10  F=filtered_cte*10  S=steer*10  ST=drive state */
//                        esp_printf("L:%u R:%d F:%d S:%d ST:F\r\n",
//                                   (unsigned)found,
//                                   (int)(e_cte * 10.0f),
//                                   (int)(filtered_cte * 10.0f),
//                                   (int)(steer_cmd * 10.0f));

                        Steer(steer_cmd + g_params.steering_offset);

                        /* g_params.base_speed_left/right is your existing tunable
                           drive speed -- set it to whatever value corresponds to
                           30% in HbridgeSpeed's own units. Not hardcoding a "30"
                           literal here since I don't know HbridgeSpeed's actual
                           scale (could be raw PWM ticks, not a 0-100 percentage). */
                        HbridgeSpeed(&g_hbridge, (int16_t)g_params.base_speed_left, (int16_t)g_params.base_speed_right);
                    }
                    else {
                        /* no trustworthy frame: brake, center wheels, don't touch steer_pid */
                        Steer(g_params.steering_offset);
                        HbridgeBrake(&g_hbridge);
                    }
                }

            }
            else {
                /* IDLE: clear controller + drive-state so next run starts clean */
                reset_pid_state(&steer_pid);
                line_lost_counter  = 0;
                filtered_cte       = 0.0f;
                cte_filter_init    = 0;
                track_ref.initialized = 0;
                track_ref.age_ticks    = 0;
                HbridgeBrake(&g_hbridge);
            }

            if (button_pressed_edge()) {
                g_params.state = STATE_IDLE;
                HbridgeBrake(&g_hbridge);
                PRINTF("Button: STOPPED\r\n");
            }
        }
	}
}
