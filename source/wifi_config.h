#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include "Config.h"
#include <stdbool.h>

void params_init(void);
void command_poll(void);
void esp_printf(const char *fmt, ...);

void motor_test_start(int16_t speed_l, int16_t speed_r, uint32_t ticks);
void motor_test_stop(void);
bool motor_test_is_active(void);
void motor_test_tick(void);

extern volatile uint8_t g_dump_vectors;

#endif
