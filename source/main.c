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

#define CORNER_HEADING_THRESHOLD_DEG   12.0f
#define CORNER_ENTER_STREAK            2   /* consecutive corner-like ticks before braking */
#define CORNER_EXIT_STREAK             5   /* consecutive straight ticks before resuming -- higher than
                                               ENTER on purpose: quick to get cautious, slower to fully commit again */

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
    float    filtered_heading = 0.0f;
    int      cte_filter_init = 0;

    drive_state_t drive_state    = DRIVE_FORWARD;
    uint32_t      corner_streak  = 0;
    uint32_t      straight_streak = 0;
    /* Historical blind failsafe variables */
    static float historical_steer = 0.0f;
    uint32_t     blind_streak     = 0;

    PRINTF("Booted. Send 'start;' to run, 'stop;' to halt.\r\n");

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

                    uint8_t confident_straight = 0;
                    float   e_cte     = 0.0f;
                    float   e_heading = 0.0f;
                    uint8_t computed  = 0;

                    if (found == 2) {
                    center_line_t c = compute_center_line(&left, &right);
                    if (c.valid) {
                        e_cte     = c.cte;                       
                        e_heading = c.heading * HEADING_TO_DEG;  
                        computed  = 1;
                    } 
                } else if (found == 1) {
                    center_line_t c = compute_center_line_single(&left);
                    if (c.valid) {
                        e_cte     = c.cte;
                        e_heading = c.heading * HEADING_TO_DEG;
                        computed  = 1;
                    }
                }

                if (computed) {
                    blind_streak = 0;

                    /* EMA filter on CTE */
                    if (!cte_filter_init) {
                        filtered_cte     = e_cte;
                        filtered_heading = e_heading;
                        cte_filter_init  = 1;
                    } else {
                        /* Clamp CTE to the pixel screen bounds */
                        const float MAX_CTE_PX = 39.5f; 
                        if (e_cte >  MAX_CTE_PX) e_cte =  MAX_CTE_PX;
                        if (e_cte < -MAX_CTE_PX) e_cte = -MAX_CTE_PX;

                        /* Pixel rate limiter */
                        const float MAX_CTE_JUMP = 10.0f;
                        float delta = e_cte - filtered_cte;
                        if (delta >  MAX_CTE_JUMP) e_cte = filtered_cte + MAX_CTE_JUMP;
                        if (delta < -MAX_CTE_JUMP) e_cte = filtered_cte - MAX_CTE_JUMP;
                        
                        filtered_cte     = 0.4f * e_cte + 0.6f * filtered_cte;
                        filtered_heading = 0.4f * e_heading + 0.6f * filtered_heading;
                    }

                    if (fabsf(filtered_heading) < CORNER_HEADING_THRESHOLD_DEG) {
                        confident_straight = 1;
                    }

                    if (confident_straight) {
                        straight_streak = (straight_streak < CORNER_EXIT_STREAK) ? straight_streak + 1 : straight_streak;
                        corner_streak   = 0;
                    } else {
                        corner_streak   = (corner_streak < CORNER_ENTER_STREAK) ? corner_streak + 1 : corner_streak;
                        straight_streak = 0;
                    }

                    if (drive_state == DRIVE_FORWARD && corner_streak >= CORNER_ENTER_STREAK) {
                        drive_state = DRIVE_BRAKING;
                        reset_pid_state(&steer_pid);
                        filtered_cte    = 0.0f;
                        cte_filter_init = 0;
                    } else if (drive_state == DRIVE_BRAKING && straight_streak >= CORNER_EXIT_STREAK) {
                        drive_state = DRIVE_FORWARD;
                    }

                    float combined = g_params.k_cte * filtered_cte + g_params.k_heading * filtered_heading;
                    float steer_cmd = -pid_update(&steer_pid, 0.0f, combined, g_params.dt);

                    /* Update the historical path memory */
                    historical_steer = 0.90f * historical_steer + 0.10f * steer_cmd;

                    if (drive_state == DRIVE_FORWARD) {
                        esp_printf("L:%u R:%d F:%d S:%d ST:F\r\n", (unsigned)found, (int)(e_cte * 10.0f), (int)(filtered_cte * 10.0f), (int)(steer_cmd * 10.0f));
                        Steer(steer_cmd + g_params.steering_offset);
                        HbridgeSpeed(&g_hbridge, (int16_t)g_params.base_speed_left, (int16_t)g_params.base_speed_right);
                    } else { 
                        esp_printf("L:%u R:%d F:%d S:%d ST:B (corner)\r\n", (unsigned)found, (int)(e_cte * 10.0f), (int)(filtered_cte * 10.0f), (int)(steer_cmd * 10.0f));
                        Steer(steer_cmd + g_params.steering_offset);
                        HbridgeSpeed(&g_hbridge, (int16_t)(g_params.base_speed_left * 1.3f), (int16_t)(g_params.base_speed_right * 1.3f));
                    }
                } else {
                    /* BLIND CORNER FAILSAFE (Uses Temporal Historical Path Memory) */
                    blind_streak++;
                    historical_steer *= 0.95f; // Mimic the L_max bound decay limit 
                    
                    if (blind_streak > 50) { // Lost for 1 full second
                        HbridgeBrake(&g_hbridge);
                        esp_printf("L:%u ST: HALT (Lost track)\r\n", (unsigned)found);
                    } else {
                        if (drive_state == DRIVE_FORWARD) {
                            esp_printf("L:%u ST:F (blind, using history)\r\n", (unsigned)found);
                            Steer(historical_steer + g_params.steering_offset);
                            HbridgeSpeed(&g_hbridge, (int16_t)g_params.base_speed_left, (int16_t)g_params.base_speed_right);
                        } else {
                            esp_printf("L:%u ST:B (corner, blind, using history)\r\n", (unsigned)found);
                            Steer(historical_steer + g_params.steering_offset);
                            HbridgeSpeed(&g_hbridge, (int16_t)(g_params.base_speed_left * 1.3f), (int16_t)(g_params.base_speed_right * 1.3f));
                        }
                    }
                }
            }

            } else {
                /* IDLE: clear controller + drive-state so next run starts clean */
                reset_pid_state(&steer_pid);
                line_lost_counter  = 0;
                filtered_cte       = 0.0f;
                cte_filter_init    = 0;
                drive_state        = DRIVE_FORWARD;
                corner_streak      = 0;
                straight_streak    = 0;
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