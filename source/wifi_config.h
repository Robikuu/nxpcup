#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include "Config.h"

/**
 * @brief Non-blocking UART command poll.
 * Call this from the main loop. Reads one byte at a time from
 * LP_FLEXCOMM9 (ESP32 UART), accumulates into a buffer, and
 * executes when a ';' terminator is received.
 */
void command_poll(void);

#endif
