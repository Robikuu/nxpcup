#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

typedef enum {
    BUTTON_EVT_NONE = 0,
    BUTTON_EVT_PRESS,        /* Falling edge: button pressed down */
    BUTTON_EVT_SHORT_CLICK,  /* Button released after short press without hold trigger */
    BUTTON_EVT_HOLD          /* Button held continuously for >= BUTTON_HOLD_TICKS */
} button_event_t;

void receive_message(void);
void buttons_init(void);
int button_pressed_edge(void);
int button_is_pressed(void);
button_event_t button_update(void);
void button_consume_press(void);
void print_double(const char *label, float val);
void TestPidSteer(void);
void TestHBridge(void);
void TestHbridge(void);

#endif
