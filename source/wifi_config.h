#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include "Config.h"

void params_init(void);
void command_poll(void);
void esp_printf(const char *fmt, ...);

#endif
