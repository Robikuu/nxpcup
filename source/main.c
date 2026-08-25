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
#include <stdio.h>
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

/* Convert ground-space errors to cm / degrees so the existing gains
   (tuned for pixel/degree-ish scales) stay meaningful. */
#define CTE_TO_CM          100.0f
#define HEADING_TO_DEG     (180.0f / (float)M_PI)
#define TICK_CHANNEL kMRT_Channel_0
#define TIMER_CLOCK_HZ 150000000U

volatile bool g_tickFlag = false;

void MRT0_IRQHandler(void) {
	MRT_ClearStatusFlags(MRT0, TICK_CHANNEL, kMRT_TimerInterruptFlag);
	g_tickFlag = true;
	SDK_ISR_EXIT_BARRIER;
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

    PRINTF("Booted. Send 'start;' to run, 'stop;' to halt.\r\n");

    //TODO: refa main loopul si try to make it a state machine (straight, intersection, turn left, turn right, stop)
    //also go over cum sunt vectori parsati si sa incercam cumva sa le normalizam directia lor

    /* ---------- 6. Main loop ---------- */
//    ctimer_config_t timer_config;
//    CTIMER_GetDefaultConfig(&timer_config);
//    timer_config.mode     = kCTIMER_TimerMode;
//    timer_config.prescale = 0;                 // no prescale = full 150MHz resolution
//
//    CTIMER_Init(CTIMER1, &timer_config);
//    CTIMER_StartTimer(CTIMER1);

    CLOCK_EnableClock(kCLOCK_Mrt);           // ungate MRT0's clock

	mrt_config_t mrtConfig;
	MRT_GetDefaultConfig(&mrtConfig);
	MRT_Init(MRT0, &mrtConfig);

	MRT_SetupChannelMode(MRT0, TICK_CHANNEL, kMRT_RepeatMode);
	MRT_EnableInterrupts(MRT0, TICK_CHANNEL, kMRT_TimerInterruptEnable);
	EnableIRQ(MRT0_IRQn);

	uint32_t mrtClockHz = CLOCK_GetFreq(kCLOCK_BusClk); // see note below
	MRT_StartTimer(MRT0, TICK_CHANNEL, MSEC_TO_COUNT(20, mrtClockHz));

    uint32_t max_time = 0;
    uint32_t tick_count = 0;

    while (1) {
command_poll();
    	if (g_tickFlag) {
    		g_tickFlag = false;
    		tick_count ++;

    		// if(tick_count % 50 == 0) {
    		// 	printf("ticks: %u\n", tick_count);
    		// }

        	//uint32_t start = CTIMER_GetTimerCountValue(CTIMER1);
            

            /* live-update gains from remote tuning (no pid_init re-call!) */
            steer_pid.kp    = g_params.kp;
            steer_pid.ki    = g_params.ki;
            steer_pid.kd    = g_params.kd;
            steer_pid.alpha = g_params.alpha;

            if (g_params.state == STATE_RUNNING) {

                status_t s = pixy_get_vectors(&cam1, vectors, MAX_VECTORS, &num_vectors);

                if (s != kStatus_Success || num_vectors == 0) {
                    /* camera failure and "no line" get identical treatment */
                    if (++line_lost_counter > LINE_LOST_LIMIT) {
                        steer_pid.integral = 0.0f;
                        steer_pid.prev_measurement = 0.0f;
                        steer_pid.d_filtered = 0.0f;
                        steer_pid.prev_output = 0.0f;
                        steer_pid.initialized = 0;
                        filtered_cte = 0.0f;
                        cte_filter_init = 0;
                        HbridgeBrake(&g_hbridge);
                    }
                } else {

                    lane_result_t lane = compute_lane_center(vectors, num_vectors);

                    if (lane.lines_found == 0) {
                        /* all vectors filtered out — treat as line lost */
                        if (++line_lost_counter > LINE_LOST_LIMIT) {
                            steer_pid.integral = 0.0f;
                            steer_pid.prev_measurement = 0.0f;
                            steer_pid.d_filtered = 0.0f;
                            steer_pid.prev_output = 0.0f;
                            steer_pid.initialized = 0;
                            filtered_cte = 0.0f;
                            cte_filter_init = 0;
                            HbridgeBrake(&g_hbridge);
                        }
                    } else {
                        line_lost_counter = 0;

                        /* raw errors: lateral offset (cm) + heading (deg) */
                        float e_cte     = lane.cte * CTE_TO_CM;
                        float e_heading = lane.heading * HEADING_TO_DEG;

                        /* EMA filter on CTE to smooth frame-to-frame noise */
                        if (!cte_filter_init) {
                            filtered_cte    = e_cte;
                            cte_filter_init = 1;
                        } else {
                            filtered_cte = 0.4f * e_cte + 0.6f * filtered_cte;
                        }

                        /* Combine lateral and heading errors before PID control. */
                        float combined = g_params.k_cte * filtered_cte
                                       + g_params.k_heading * e_heading;

                        float steer_cmd = -pid_update(&steer_pid, 0.0f, combined, g_params.dt);

                        esp_printf("L:%u CTE:%d F:%d H:%d E:%d OUT:%d\r\n",
                               (unsigned)lane.lines_found,
                               (int)(e_cte * 10.0f),
                               (int)(filtered_cte * 10.0f),
                               (int)(e_heading * 10.0f),
                               (int)(combined * 10.0f),
                               (int)(steer_cmd * 10.0f));

                        Steer(steer_cmd + g_params.steering_offset);

                        HbridgeSpeed(&g_hbridge, (int16_t)g_params.base_speed_left, (int16_t)g_params.base_speed_right);
                    }
                }

            } else {
                /* IDLE: clear controller state so next run starts clean */
                steer_pid.integral         = 0.0f;
                steer_pid.prev_measurement = 0.0f;
                steer_pid.d_filtered       = 0.0f;
                steer_pid.prev_output      = 0.0f;
                steer_pid.initialized      = 0;
                line_lost_counter          = 0;
                filtered_cte               = 0.0f;
                cte_filter_init            = 0;
                HbridgeBrake(&g_hbridge);
            }

            if (button_pressed_edge()) {
                g_params.state = STATE_IDLE;
                HbridgeBrake(&g_hbridge);
                PRINTF("Button: STOPPED\r\n");
            }
    //        uint32_t end = CTIMER_GetTimerCountValue(CTIMER1);
    //        uint32_t ticks = end - start;   // unsigned subtraction handles wraparound safely
    //
    //        uint32_t elapsed_us = (uint32_t)(((uint64_t)ticks * 1000000u) / TIMER_CLOCK_HZ);
    //        if (elapsed_us > (uint32_t)max_time) {
    //            max_time = elapsed_us;
    //        }
    //
    //        printf("us = %u\nmax = %u\n", elapsed_us, max_time);
    	}


    }
}
