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
   degrees since k_heading and the turn thresholds are tuned in degrees.
   cte from compute_center_line() is already in real cm -- no CTE_TO_CM
   conversion needed (the old *100.0f here was a leftover double-scale bug). */
#define HEADING_TO_DEG     (180.0f / (float)M_PI)

/* ---- Turn-state tuning -------------------------------------------------
 * These angles are measured AFTER the homography, where +X is right and
 * +heading is therefore a right turn.  Tune these from frame logs on the
 * real track; do not use pixel-space angles here. */
#define TURN_ENTER_HEADING_DEG         20.0f
#define TURN_EXIT_HEADING_DEG          12.0f
#define TURN_ENTER_STREAK              2U
#define TURN_EXIT_STREAK               2U
#define TURN_MAX_OBSERVED_HEADING_DEG  85.0f

/* Steering commands use the same -100 (left) .. +100 (right) coordinate
 * system as Steer().  The slew limiter makes the turn progressive instead
 * of jumping from the PID output to full lock. */
#define TURN_MIN_STEER_CMD             30.0f
#define TURN_FULL_STEER_HEADING_DEG    60.0f
#define TURN_STEER_SLEW_PER_TICK        4.0f

#define TICK_CHANNEL kMRT_Channel_0
#define TIMER_CLOCK_HZ 150000000U

volatile bool g_tickFlag = false;

typedef enum {
    DRIVE_FORWARD,
    DRIVE_TURN_LEFT,
    DRIVE_TURN_RIGHT
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

static float clampf(float value, float low, float high)
{
    if (value < low)  return low;
    if (value > high) return high;
    return value;
}

static float slew_toward(float current, float target, float maximum_step)
{
    if (target > current + maximum_step) return current + maximum_step;
    if (target < current - maximum_step) return current - maximum_step;
    return target;
}

/* Convert one raw Pixy vector to a heading in ground coordinates.  Pixy
 * may reverse endpoint order, so sort by ground Y before calculating the
 * heading.  That guarantees a negative result means "the line goes left as
 * it goes away from the car", and positive means right. */
static uint8_t raw_vector_ground_heading_deg(const uint16_t *raw,
                                             float *heading_deg)
{
    float x0, y0, x1, y1;
    if (!pixel_to_ground((float)raw[0], (float)raw[1], &x0, &y0) ||
        !pixel_to_ground((float)raw[2], (float)raw[3], &x1, &y1)) {
        return 0;
    }

    if (y0 > y1) {
        float temp;
        temp = x0; x0 = x1; x1 = temp;
        temp = y0; y0 = y1; y1 = temp;
    }

    if ((y1 - y0) < 1.0f) {
        return 0;
    }

    *heading_deg = atan2f(x1 - x0, y1 - y0) * HEADING_TO_DEG;
    return (fabsf(*heading_deg) <= TURN_MAX_OBSERVED_HEADING_DEG);
}

/* Find the strongest usable ground-space line.  While already turning, only
 * accept a line curving in the commanded direction.  This is deliberately
 * independent from select_boundary_vectors(), whose normal lane filter
 * rejects the steep line that is expected during a corner. */
static uint8_t find_turn_heading(const uint16_t *vectors, size_t num_vectors,
                                 int direction, float *heading_deg)
{
    float best_abs = 0.0f;
    uint8_t found = 0;

    for (size_t i = 0; i < num_vectors; i++) {
        float candidate;
        if (!raw_vector_ground_heading_deg(&vectors[6U * i], &candidate)) {
            continue;
        }
        if (direction != 0 && candidate * (float)direction <= 0.0f) {
            continue;
        }
        if (fabsf(candidate) > best_abs) {
            best_abs = fabsf(candidate);
            *heading_deg = candidate;
            found = 1;
        }
    }

    return found;
}

static float turn_target_from_heading(float heading_deg)
{
    float magnitude = fabsf(heading_deg) *
                      (100.0f / TURN_FULL_STEER_HEADING_DEG);
    magnitude = clampf(magnitude, TURN_MIN_STEER_CMD, 100.0f);
    return (heading_deg < 0.0f) ? -magnitude : magnitude;
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
        esp_printf("Pixy init FAILED\r\n");      /* car stays IDLE */
    }

    uint32_t line_lost_counter = 0;
    uint16_t vectors[MAX_VECTORS * 6];
    size_t   num_vectors;
    float    filtered_cte    = 0.0f;
    int      cte_filter_init = 0;

    drive_state_t drive_state       = DRIVE_FORWARD;
    uint32_t      turn_entry_streak = 0;
    uint32_t      straight_streak   = 0;
    float         applied_steer_cmd = 0.0f;
    float         turn_target_cmd   = 0.0f;
    uint8_t       recentering       = 0;

    esp_printf("Booted. Send 'start;' to run, 'stop;' to halt.\r\n");

    //TODO: refa main loopul si try to make it a state machine (straight, intersection, turn left, turn right, stop)
    //also go over cum sunt vectori parsati si sa incercam cumva sa le normalizam directia lor

    CLOCK_EnableClock(kCLOCK_Mrt);           // ungate MRT0's clock

	mrt_config_t mrtConfig;
	MRT_GetDefaultConfig(&mrtConfig);
	MRT_Init(MRT0, &mrtConfig);

	MRT_SetupChannelMode(MRT0, TICK_CHANNEL, kMRT_RepeatMode);
	MRT_EnableInterrupts(MRT0, TICK_CHANNEL, kMRT_TimerInterruptEnable);
	EnableIRQ(MRT0_IRQn);

	uint32_t mrtClockHz = CLOCK_GetFreq(kCLOCK_BusClk); // see note below
	MRT_StartTimer(MRT0, TICK_CHANNEL, MSEC_TO_COUNT(20, mrtClockHz));

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

                status_t s = pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors);

                print_frame_debug(vectors, num_vectors);

                if (s != kStatus_Success || num_vectors == 0) {
                    /* camera failure and "no line at all" get identical treatment,
                       unchanged from before -- this is a harder failure than a
                       corner, so it's kept separate from the drive/brake state
                       machine below rather than folded into it. */
                    if (++line_lost_counter > LINE_LOST_LIMIT) {
                        HbridgeBrake(&g_hbridge);
                    }
                } else {
                    line_lost_counter = 0;

                    ground_vector_t left, right;
                    uint8_t found = select_boundary_vectors(vectors, num_vectors, &left, &right);

                    uint8_t center_valid = 0;
                    float   e_cte     = 0.0f;
                    float   e_heading = 0.0f;

                    if (found == 2) {
                        center_line_t c = compute_center_line(&left, &right);
                        if (c.valid) {
                            e_cte     = c.cte;                       /* already cm */
                            e_heading = c.heading * HEADING_TO_DEG;  /* rad -> deg */
                            center_valid = 1;
                        }
                    }

                    if (drive_state == DRIVE_FORWARD) {
                        float turn_heading = 0.0f;
                        uint8_t have_turn_heading = 0;

                        /* With both boundaries, the centerline is the most stable
                         * turn detector.  Once one boundary disappears, inspect
                         * the individual transformed lines instead: that steep
                         * remaining boundary is expected in a turn. */
                        if (center_valid &&
                            fabsf(e_heading) >= TURN_ENTER_HEADING_DEG) {
                            turn_heading = e_heading;
                            have_turn_heading = 1;
                        } else if (!center_valid) {
                            have_turn_heading = find_turn_heading(vectors, num_vectors,
                                                                  0, &turn_heading);
                            if (have_turn_heading &&
                                fabsf(turn_heading) < TURN_ENTER_HEADING_DEG) {
                                have_turn_heading = 0;
                            }
                        }

                        if (have_turn_heading) {
                            if (turn_entry_streak < TURN_ENTER_STREAK) {
                                turn_entry_streak++;
                            }

                            if (turn_entry_streak >= TURN_ENTER_STREAK) {
                                drive_state = (turn_heading < 0.0f) ?
                                              DRIVE_TURN_LEFT : DRIVE_TURN_RIGHT;
                                turn_target_cmd = turn_target_from_heading(turn_heading);
                                straight_streak = 0;
                                recentering = 0;
                                reset_pid_state(&steer_pid);
                                filtered_cte = 0.0f;
                                cte_filter_init = 0;
                            }
                        } else {
                            turn_entry_streak = 0;
                        }

                        if (drive_state == DRIVE_FORWARD && center_valid) {
                        /* EMA filter on CTE to smooth frame-to-frame noise */
                            if (!cte_filter_init) {
                                filtered_cte    = e_cte;
                                cte_filter_init = 1;
                            } else {
                                filtered_cte = 0.4f * e_cte + 0.6f * filtered_cte;
                            }

                            float combined = g_params.k_cte * filtered_cte
                                           + g_params.k_heading * e_heading;
                            float pid_steer_cmd = -pid_update(&steer_pid, 0.0f,
                                                               combined, g_params.dt);

                            /* The first frames after a turn return the wheels to
                             * the PID target progressively.  Normal straight-line
                             * PID operation remains unchanged. */
                            if (recentering) {
                                applied_steer_cmd = slew_toward(applied_steer_cmd,
                                                                pid_steer_cmd,
                                                                TURN_STEER_SLEW_PER_TICK);
                                if (fabsf(applied_steer_cmd - pid_steer_cmd) <
                                    TURN_STEER_SLEW_PER_TICK) {
                                    recentering = 0;
                                }
                            } else {
                                applied_steer_cmd = pid_steer_cmd;
                            }
                        }
                    }

                    if (drive_state == DRIVE_TURN_LEFT ||
                        drive_state == DRIVE_TURN_RIGHT) {
                        int direction = (drive_state == DRIVE_TURN_LEFT) ? -1 : 1;
                        float turn_heading;

                        if (find_turn_heading(vectors, num_vectors, direction,
                                              &turn_heading)) {
                            turn_target_cmd = turn_target_from_heading(turn_heading);
                        }

                        applied_steer_cmd = slew_toward(applied_steer_cmd,
                                                        turn_target_cmd,
                                                        TURN_STEER_SLEW_PER_TICK);

                        if (center_valid &&
                            fabsf(e_heading) <= TURN_EXIT_HEADING_DEG) {
                            if (straight_streak < TURN_EXIT_STREAK) {
                                straight_streak++;
                            }
                        } else {
                            straight_streak = 0;
                        }

                        if (straight_streak >= TURN_EXIT_STREAK) {
                            drive_state = DRIVE_FORWARD;
                            turn_entry_streak = 0;
                            straight_streak = 0;
                            recentering = 1;
                            reset_pid_state(&steer_pid);
                            filtered_cte = 0.0f;
                            cte_filter_init = 0;
                        }
                    }

                    /* A single boundary is allowed to persist in either turn
                     * state.  Keep driving using the last valid steering target;
                     * only a complete camera/vector loss reaches the separate
                     * LINE_LOST_LIMIT safety path above. */
                    Steer(applied_steer_cmd + g_params.steering_offset);
                    HbridgeSpeed(&g_hbridge,
                                  (int16_t)g_params.base_speed_left,
                                  (int16_t)g_params.base_speed_right);
                }

            } else {
                /* IDLE: clear controller + drive-state so next run starts clean */
                reset_pid_state(&steer_pid);
                line_lost_counter  = 0;
                filtered_cte       = 0.0f;
                cte_filter_init    = 0;
                drive_state        = DRIVE_FORWARD;
                turn_entry_streak  = 0;
                straight_streak    = 0;
                applied_steer_cmd  = 0.0f;
                turn_target_cmd    = 0.0f;
                recentering        = 0;
                HbridgeBrake(&g_hbridge);
            }

            if (button_pressed_edge()) {
                g_params.state = STATE_IDLE;
                HbridgeBrake(&g_hbridge);
                esp_printf("Button: STOPPED\r\n");
            }
        }
	}
}
