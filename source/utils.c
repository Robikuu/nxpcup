#include "utils.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include <string.h>
#include <stdlib.h>
#include "servo.h"
#include "fsl_lpuart.h"
#include "peripherals.h"

#define MAX_ESP_MSG_LEN 64

#define ESP_UART_BASE LPUART9 // The instance from your pin routing

static int button_prev_state = 1;
static int s_ip_received = 0;
int esp_msg_index = 0;
char esp_msg_buffer[MAX_ESP_MSG_LEN];

void esp_steer_test_loop(void) {
    uint8_t rx_char;
    uint32_t send_timer = 0;

    PRINTF("Listening to ESP board on LPUART9...\r\n");

    // Clear any startup overrun faults silently
    if (LPUART_GetStatusFlags(LP_FLEXCOMM9_PERIPHERAL) & kLPUART_RxOverrunFlag) {
        LPUART_ClearStatusFlags(LP_FLEXCOMM9_PERIPHERAL, kLPUART_RxOverrunFlag);
    }

    // Send the first request NOW, right before dropping straight into the clean listen loop!
    request_esp_ip();

    while (1) {
        // Automatically re-send GET_IP only if we haven't received the IP answer yet!
        if (!s_ip_received) {
            send_timer++;
            if (send_timer >= 20000000UL) {
                send_timer = 0;
                request_esp_ip();
            }
        }

        // Clear overrun error silently (no slow PRINTF to cause more overruns)
        if (LPUART_GetStatusFlags(LP_FLEXCOMM9_PERIPHERAL) & kLPUART_RxOverrunFlag) {
            LPUART_ClearStatusFlags(LP_FLEXCOMM9_PERIPHERAL, kLPUART_RxOverrunFlag);
        }

        // Drain ALL waiting bytes at maximum speed using colleague's while loop
        while (LPUART_GetStatusFlags(LP_FLEXCOMM9_PERIPHERAL) & kLPUART_RxDataRegFullFlag) {
            rx_char = LPUART_ReadByte(LP_FLEXCOMM9_PERIPHERAL);
            handle_incoming_uart_byte((char)rx_char);
        }
    }
}

// 1. The Parser (from earlier)
void parse_esp_command(const char *message) {
    // Existing steering command (supporting both upper and lower case!)
    if (strncmp(message, "STEER ", 6) == 0 || strncmp(message, "steer ", 6) == 0) {
        int steering_val = atoi(message + 6);
        Steer((double)steering_val);
        PRINTF(">>> STEER command executed with value: %d <<<\r\n", steering_val);
    } 
    // NEW: Catch the IP address response
    else if (strncmp(message, "IP ", 3) == 0) {
        s_ip_received = 1; // Stop recurring requests!
        PRINTF("\r\n>>> ESP Board IP Address: %s <<<\r\n", message + 3);
        PRINTF("IP recorded! Bridge ready for steering commands over Wi-Fi.\r\n");
    }
    else {
        PRINTF("Unknown ESP command received: '%s'\r\n", message);
    }
}

// 2. The Accumulator with Backspace Support (Running silently at maximum hardware speed!)
void handle_incoming_uart_byte(char c) {
    // Handle Backspace / Delete keys (0x08 or 0x7F) from PuTTY so typos don't corrupt the string!
    if (c == 0x08 || c == 0x7F) {
        if (esp_msg_index > 0) {
            esp_msg_index--;
        }
        return;
    }

    // If we hit a newline, carriage return, OR semicolon (;), the message is complete!
    if (c == '\n' || c == '\r' || c == ';') {
        if (esp_msg_index > 0) {
            esp_msg_buffer[esp_msg_index] = '\0'; // Null-terminate the string
            
            // THIS passes the finished string to your perfect parser
            parse_esp_command(esp_msg_buffer); 
            
            esp_msg_index = 0; // Reset for the next message
        }
    } 
    else {
        // Skip leading control characters or noise when starting a fresh message
        if (esp_msg_index == 0 && (c < 32 || c > 126)) {
            return;
        }
        // Keep adding valid characters to the buffer
        if (esp_msg_index < MAX_ESP_MSG_LEN - 1) {
            esp_msg_buffer[esp_msg_index++] = c;
        }
    }
}

void buttons_init(void)
{
    button_prev_state = 1; // idles HIGH due to pull-up
}

int button_pressed_edge(void)
{
    int button_now = GPIO_PinRead(GPIO0, 23U);
    int edge = (button_prev_state == 1 && button_now == 0);
    button_prev_state = button_now;
    return edge;
}

void send_to_esp_string(const char *str) {
    while (*str != '\0') {
        volatile uint32_t timeout = 50000;
        // Wait safely for TX buffer to be empty without freezing the board
        while (!(LPUART_GetStatusFlags(ESP_UART_BASE) & kLPUART_TxDataRegEmptyFlag) && --timeout) {
        }
        LPUART_WriteByte(ESP_UART_BASE, (uint8_t)(*str));
        str++;
    }
}

void request_esp_ip(void) {
    const char *command = "GET_IP\n";
    PRINTF("Requested IP from ESP board...\r\n");
    LPUART_WriteBlocking(LP_FLEXCOMM9_PERIPHERAL, (const uint8_t *)command, strlen(command));
}

// void check_uart_commands(void)
// {
//     if (/* UART has data available - use your board's UART receive check */) {
//         char cmd[32];
//         // read a line of text into cmd, e.g. "SPEED 60" or "PROFILE 2" or "TEST SERVO"

//         if (strncmp(cmd, "PROFILE", 7) == 0) {
//             int idx = atoi(cmd + 8);
//             if (idx >= 0 && idx < NUM_PROFILES) {
//                 current_profile = idx;
//                 pid_init(&steer_pid, profiles[idx].kp, profiles[idx].ki,
//                           profiles[idx].kd, profiles[idx].integral_limit,
//                           profiles[idx].output_limit);
//                 PRINTF("Switched to profile %d\r\n", idx);
//             }
//         }
//         else if (strncmp(cmd, "SPEED", 5) == 0) {
//             BASE_SPEED_runtime = atof(cmd + 6);  // if you make BASE_SPEED a variable, not a #define
//             PRINTF("Base speed set to %.2f\r\n", BASE_SPEED_runtime);
//         }
//         else if (strncmp(cmd, "TEST SERVO", 10) == 0) {
//             TestServo();
//         }
//         else if (strncmp(cmd, "TEST PID", 8) == 0) {
//             TestPidSteer();
//         }
//         // add more commands as you need them
//     }
// }

void print_double(const char *label, double val)
{
    int whole = (int)val;
    double frac_part = val - whole;
    if (frac_part < 0) frac_part = -frac_part;
    int frac = (int)(frac_part * 100.0);
    PRINTF("%s%d.%02d\r\n", label, whole, frac);
}
