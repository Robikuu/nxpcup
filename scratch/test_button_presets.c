#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include "Config.h"
#include "utils.h"

/* Mock global params and hardware state for testing */
robot_params_t g_params;
static int s_mock_gpio_pin = 1; /* 1 = idle high (unpressed), 0 = low (pressed) */

/* Mock hardware functions */
int GPIO_PinRead(void *base, uint32_t pin) {
    (void)base;
    (void)pin;
    return s_mock_gpio_pin;
}

void Steer(float angle) {
    (void)angle;
}

/* Implementation under test from wifi_config.c logic */
static uint8_t s_k_cte_corner_overridden = 0;
static uint8_t s_k_heading_corner_overridden = 0;

void params_load_preset(uint8_t preset_id) {
    if (preset_id == 2U) {
        g_params.active_preset        = 2U;
        g_params.speed_straight       = PRESET2_SPEED_STRAIGHT;
        g_params.speed_outside_corner = PRESET2_SPEED_OUTSIDE_CORNER;
        g_params.speed_inside_min     = PRESET2_SPEED_INSIDE_MIN;
        g_params.k_cte                = PRESET2_K_CTE;
        g_params.k_heading            = PRESET2_K_HEADING;
        g_params.k_cte_corner         = PRESET2_K_CTE_CORNER;
        g_params.k_heading_corner     = PRESET2_K_HEADING_CORNER;
        g_params.h_lo                 = PRESET2_H_LO;
        g_params.h_hi                 = PRESET2_H_HI;
        g_params.blend_attack         = PRESET2_BLEND_ATTACK;
        g_params.blend_release        = PRESET2_BLEND_RELEASE;
        g_params.front_angle_thresh   = PRESET2_FRONT_ANGLE_THRESH;
        g_params.blend_enable         = PRESET2_BLEND_ENABLE;
        g_params.steering_offset      = PRESET2_STEERING_OFFSET;
        g_params.accel_ramp           = PRESET2_ACCEL_RAMP;
        g_params.decel_ramp           = PRESET2_DECEL_RAMP;
        g_params.drive_scale          = PRESET2_DRIVE_SCALE;
        g_params.motor_enable         = PRESET2_MOTOR_ENABLE;
    } else {
        g_params.active_preset        = 1U;
        g_params.speed_straight       = PRESET1_SPEED_STRAIGHT;
        g_params.speed_outside_corner = PRESET1_SPEED_OUTSIDE_CORNER;
        g_params.speed_inside_min     = PRESET1_SPEED_INSIDE_MIN;
        g_params.k_cte                = PRESET1_K_CTE;
        g_params.k_heading            = PRESET1_K_HEADING;
        g_params.k_cte_corner         = PRESET1_K_CTE_CORNER;
        g_params.k_heading_corner     = PRESET1_K_HEADING_CORNER;
        g_params.h_lo                 = PRESET1_H_LO;
        g_params.h_hi                 = PRESET1_H_HI;
        g_params.blend_attack         = PRESET1_BLEND_ATTACK;
        g_params.blend_release        = PRESET1_BLEND_RELEASE;
        g_params.front_angle_thresh   = PRESET1_FRONT_ANGLE_THRESH;
        g_params.blend_enable         = PRESET1_BLEND_ENABLE;
        g_params.steering_offset      = PRESET1_STEERING_OFFSET;
        g_params.accel_ramp           = PRESET1_ACCEL_RAMP;
        g_params.decel_ramp           = PRESET1_DECEL_RAMP;
        g_params.drive_scale          = PRESET1_DRIVE_SCALE;
        g_params.motor_enable         = PRESET1_MOTOR_ENABLE;
    }
    s_k_cte_corner_overridden = 0;
    s_k_heading_corner_overridden = 0;
    Steer(g_params.steering_offset);
}

void params_switch_preset(void) {
    if (g_params.active_preset == 1U) {
        params_load_preset(2U);
    } else {
        params_load_preset(1U);
    }
}

void params_init(void) {
    g_params.state = STATE_IDLE;
    params_load_preset(1U);
}

/* Include utils.c button implementation directly for white-box unit testing */
static int s_button_prev_state = 1;
static int s_button_debounce_cnt = 0;
static uint32_t s_button_hold_cnt = 0;
static uint8_t s_button_hold_fired = 0;
static uint8_t s_button_press_consumed = 0;

void test_buttons_init(void) {
    s_button_prev_state     = 1;
    s_button_debounce_cnt   = 0;
    s_button_hold_cnt       = 0;
    s_button_hold_fired     = 0;
    s_button_press_consumed = 0;
}

button_event_t test_button_update(void) {
    button_event_t event = BUTTON_EVT_NONE;
    int button_raw = s_mock_gpio_pin;

    if (button_raw != s_button_prev_state) {
        s_button_debounce_cnt++;
        if (s_button_debounce_cnt >= 3) {
            int was_released = (s_button_prev_state == 1);
            s_button_prev_state = button_raw;
            s_button_debounce_cnt = 0;

            if (was_released && button_raw == 0) {
                s_button_hold_cnt = 0;
                s_button_hold_fired = 0;
                s_button_press_consumed = 0;
                event = BUTTON_EVT_PRESS;
            } else if (!was_released && button_raw == 1) {
                if (!s_button_hold_fired && !s_button_press_consumed) {
                    event = BUTTON_EVT_SHORT_CLICK;
                }
                s_button_hold_cnt = 0;
                s_button_hold_fired = 0;
                s_button_press_consumed = 0;
            }
        }
    } else {
        s_button_debounce_cnt = 0;

        if (s_button_prev_state == 0) {
            s_button_hold_cnt++;
            if (!s_button_hold_fired && !s_button_press_consumed && (s_button_hold_cnt >= BUTTON_HOLD_TICKS)) {
                s_button_hold_fired = 1;
                event = BUTTON_EVT_HOLD;
            }
        }
    }

    return event;
}

void test_button_consume_press(void) {
    s_button_press_consumed = 1;
}

int main(void) {
    printf("============================================================\n");
    printf("   BUTTON HOLD & PRESET SWITCHING UNIT TEST SUITE\n");
    printf("============================================================\n");

    /* TEST 1: Preset initialization */
    printf("\n[TEST 1] Testing params_init() and Preset 1 defaults...\n");
    params_init();
    assert(g_params.active_preset == 1);
    assert(g_params.speed_straight == PRESET1_SPEED_STRAIGHT);
    assert(g_params.speed_outside_corner == PRESET1_SPEED_OUTSIDE_CORNER);
    assert(g_params.speed_inside_min == PRESET1_SPEED_INSIDE_MIN);
    assert(g_params.accel_ramp == PRESET1_ACCEL_RAMP);
    assert(g_params.decel_ramp == PRESET1_DECEL_RAMP);
    assert(g_params.state == STATE_IDLE);
    printf("  [PASS] Preset 1 initialized correctly (spd_s=%.1f, spd_out=%.1f, spd_in=%.1f)\n",
           g_params.speed_straight, g_params.speed_outside_corner, g_params.speed_inside_min);

    /* TEST 2: Preset 2 loading */
    printf("\n[TEST 2] Testing params_load_preset(2)...\n");
    params_load_preset(2);
    assert(g_params.active_preset == 2);
    assert(g_params.speed_straight == PRESET2_SPEED_STRAIGHT);
    assert(g_params.speed_outside_corner == PRESET2_SPEED_OUTSIDE_CORNER);
    assert(g_params.speed_inside_min == PRESET2_SPEED_INSIDE_MIN);
    assert(g_params.accel_ramp == PRESET2_ACCEL_RAMP);
    assert(g_params.decel_ramp == PRESET2_DECEL_RAMP);
    printf("  [PASS] Preset 2 loaded correctly (spd_s=%.1f, spd_out=%.1f, spd_in=%.1f)\n",
           g_params.speed_straight, g_params.speed_outside_corner, g_params.speed_inside_min);

    /* TEST 3: params_switch_preset() toggling */
    printf("\n[TEST 3] Testing params_switch_preset() toggle...\n");
    params_switch_preset();
    assert(g_params.active_preset == 1);
    assert(g_params.speed_straight == PRESET1_SPEED_STRAIGHT);
    params_switch_preset();
    assert(g_params.active_preset == 2);
    assert(g_params.speed_straight == PRESET2_SPEED_STRAIGHT);
    params_switch_preset();
    assert(g_params.active_preset == 1);
    printf("  [PASS] params_switch_preset() toggles 2 -> 1 -> 2 -> 1 cleanly\n");

    /* TEST 4: Button Debounce Verification */
    printf("\n[TEST 4] Button debounce filter (3 ticks requirement)...\n");
    test_buttons_init();
    s_mock_gpio_pin = 1; /* unpressed */
    button_event_t evt = test_button_update();
    assert(evt == BUTTON_EVT_NONE);

    /* Glitch: 2 ticks low, then back high */
    s_mock_gpio_pin = 0;
    evt = test_button_update(); /* tick 1 */
    assert(evt == BUTTON_EVT_NONE);
    evt = test_button_update(); /* tick 2 */
    assert(evt == BUTTON_EVT_NONE);
    s_mock_gpio_pin = 1;
    evt = test_button_update(); /* back to high */
    assert(evt == BUTTON_EVT_NONE);
    printf("  [PASS] 2-tick bounce glitch rejected with zero events\n");

    /* TEST 5: Short Click (Tap & Release in IDLE) */
    printf("\n[TEST 5] Short click in IDLE (tap to start)...\n");
    test_buttons_init();
    s_mock_gpio_pin = 0; /* User presses button */
    evt = test_button_update(); assert(evt == BUTTON_EVT_NONE); /* Debounce 1 */
    evt = test_button_update(); assert(evt == BUTTON_EVT_NONE); /* Debounce 2 */
    evt = test_button_update(); assert(evt == BUTTON_EVT_PRESS); /* Debounce 3: PRESS emitted */

    /* Hold for 5 ticks (~100 ms total) */
    for (int i = 0; i < 5; i++) {
        evt = test_button_update();
        assert(evt == BUTTON_EVT_NONE);
    }

    /* User releases button */
    s_mock_gpio_pin = 1;
    evt = test_button_update(); assert(evt == BUTTON_EVT_NONE); /* Release debounce 1 */
    evt = test_button_update(); assert(evt == BUTTON_EVT_NONE); /* Release debounce 2 */
    evt = test_button_update(); assert(evt == BUTTON_EVT_SHORT_CLICK); /* Release debounce 3: CLICK emitted */

    /* Follow-up ticks while released */
    evt = test_button_update();
    assert(evt == BUTTON_EVT_NONE);
    printf("  [PASS] Short tap emits PRESS on edge and SHORT_CLICK on release\n");

    /* TEST 6: Long Hold in IDLE (Hold >= BUTTON_HOLD_TICKS to switch preset) */
    printf("\n[TEST 6] Long hold in IDLE (Hold for %u ticks = 1.0s)...\n", BUTTON_HOLD_TICKS);
    test_buttons_init();
    s_mock_gpio_pin = 0; /* User presses and holds */
    test_button_update(); test_button_update();
    evt = test_button_update();
    assert(evt == BUTTON_EVT_PRESS);

    /* Hold until tick 49 */
    for (uint32_t t = 1; t < BUTTON_HOLD_TICKS; t++) {
        evt = test_button_update();
        assert(evt == BUTTON_EVT_NONE);
    }

    /* Tick 50: Exact hold threshold reached! */
    evt = test_button_update();
    assert(evt == BUTTON_EVT_HOLD);
    printf("  [PASS] BUTTON_EVT_HOLD triggered at exactly tick %u\n", BUTTON_HOLD_TICKS);

    /* Keep holding for 20 more ticks: should NOT trigger repeat hold events */
    for (int t = 0; t < 20; t++) {
        evt = test_button_update();
        assert(evt == BUTTON_EVT_NONE);
    }
    printf("  [PASS] Continuous hold does NOT emit duplicate hold events\n");

    /* User releases button after long hold: should NOT emit click/start! */
    s_mock_gpio_pin = 1;
    test_button_update(); test_button_update();
    evt = test_button_update();
    assert(evt == BUTTON_EVT_NONE); /* Suppressed! */
    printf("  [PASS] Release after hold does NOT emit SHORT_CLICK (vehicle remains safely stopped)\n");

    /* TEST 7: Emergency Stop while RUNNING */
    printf("\n[TEST 7] Immediate stop while STATE_RUNNING...\n");
    test_buttons_init();
    g_params.state = STATE_RUNNING;

    /* User presses button while car is running */
    s_mock_gpio_pin = 0;
    test_button_update(); test_button_update();
    evt = test_button_update();
    assert(evt == BUTTON_EVT_PRESS);

    /* Simulated main.c logic on BUTTON_EVT_PRESS while RUNNING: */
    if (g_params.state == STATE_RUNNING && evt == BUTTON_EVT_PRESS) {
        g_params.state = STATE_IDLE;
        test_button_consume_press();
    }
    assert(g_params.state == STATE_IDLE);

    /* User holds finger down for 200ms while grabbing car, then releases */
    for (int i = 0; i < 10; i++) {
        evt = test_button_update();
        assert(evt == BUTTON_EVT_NONE);
    }
    s_mock_gpio_pin = 1;
    test_button_update(); test_button_update();
    evt = test_button_update();
    assert(evt == BUTTON_EVT_NONE); /* Click suppressed by test_button_consume_press() */
    assert(g_params.state == STATE_IDLE);
    printf("  [PASS] Immediate stop on PRESS down, and release does NOT restart car\n");

    /* TEST 8: Full Lifecycle Simulation */
    printf("\n[TEST 8] Full end-to-end user scenario simulation...\n");
    test_buttons_init();
    params_init(); /* Boot state */
    assert(g_params.state == STATE_IDLE && g_params.active_preset == 1);

    /* Step A: User holds button to switch to Preset 2 */
    s_mock_gpio_pin = 0;
    for (uint32_t t = 0; t < 3 + BUTTON_HOLD_TICKS; t++) {
        button_event_t e = test_button_update();
        if (e == BUTTON_EVT_HOLD) {
            params_switch_preset();
        }
    }
    assert(g_params.active_preset == 2);
    assert(g_params.state == STATE_IDLE);
    s_mock_gpio_pin = 1;
    test_button_update(); test_button_update(); test_button_update();
    assert(g_params.state == STATE_IDLE && g_params.active_preset == 2);

    /* Step B: User taps button to START the car with Preset 2 */
    s_mock_gpio_pin = 0;
    test_button_update(); test_button_update(); test_button_update();
    s_mock_gpio_pin = 1;
    test_button_update(); test_button_update();
    evt = test_button_update();
    assert(evt == BUTTON_EVT_SHORT_CLICK);
    if (g_params.state == STATE_IDLE && evt == BUTTON_EVT_SHORT_CLICK) {
        g_params.state = STATE_RUNNING;
    }
    assert(g_params.state == STATE_RUNNING && g_params.active_preset == 2);

    /* Step C: Car is racing. User taps button to STOP the car */
    s_mock_gpio_pin = 0;
    test_button_update(); test_button_update();
    evt = test_button_update();
    assert(evt == BUTTON_EVT_PRESS);
    if (g_params.state == STATE_RUNNING && evt == BUTTON_EVT_PRESS) {
        g_params.state = STATE_IDLE;
        test_button_consume_press();
    }
    assert(g_params.state == STATE_IDLE);
    s_mock_gpio_pin = 1;
    test_button_update(); test_button_update(); test_button_update();
    assert(g_params.state == STATE_IDLE);

    /* Step D: User holds button to switch back to Preset 1 */
    s_mock_gpio_pin = 0;
    for (uint32_t t = 0; t < 3 + BUTTON_HOLD_TICKS; t++) {
        button_event_t e = test_button_update();
        if (e == BUTTON_EVT_HOLD) {
            params_switch_preset();
        }
    }
    assert(g_params.active_preset == 1);
    s_mock_gpio_pin = 1;
    test_button_update(); test_button_update(); test_button_update();
    assert(g_params.state == STATE_IDLE && g_params.active_preset == 1);

    printf("  [PASS] Full lifecycle executed with 100%% precision!\n");

    printf("\n============================================================\n");
    printf("   ALL BUTTON & PRESET UNIT TESTS PASSED SUCCESSFULLY! (8/8)\n");
    printf("============================================================\n");
    return 0;
}
