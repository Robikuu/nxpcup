#include "utils.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include <string.h>
#include <stdlib.h>
#include "servo.h"
#include "fsl_lpuart.h"

#define MAX_ESP_MSG_LEN 64

#define ESP_UART_BASE LPUART9 // The instance from your pin routing

static int button_prev_state = 1;
int esp_msg_index = 0;
char esp_msg_buffer[MAX_ESP_MSG_LEN];

void esp_steer_test_loop(void) {
    uint8_t rx_char;

    PRINTF("Listening to ESP board on USART9...\r\n");

    while (1) {
        // 1. Check if the FLEXCOMM USART receiver has a new byte waiting
        // (Note: Check fsl_usart.h in your project if this exact flag name differs slightly)
    	uint32_t flags = LPUART_GetStatusFlags(ESP_UART_BASE);
        
    	if (flags & kLPUART_RxDataRegFullFlag) {
            // 2. Grab the byte directly from the hardware register
        	rx_char = LPUART_ReadByte(ESP_UART_BASE);
            
            // 3. Feed it to the accumulator
        	handle_incoming_uart_byte((char)rx_char);
        }
    }
}

// 1. The Parser (from earlier)
void parse_esp_command(const char *message) {
    // Existing steering command
    if (strncmp(message, "STEER ", 6) == 0) {
        char *endptr;
        double steering_val = strtod(message + 6, &endptr);

        if (endptr != (message + 6)) {
            Steer(steering_val);
        }
    } 
    // NEW: Catch the IP address response
    else if (strncmp(message, "IP ", 3) == 0) {
        PRINTF("\r\n>>> ESP Board IP Address: %s <<<\r\n", message + 3);
    }
    else {
        PRINTF("Unknown ESP command: %s\r\n", message);
    }
}

// 2. The Accumulator with Backspace Support
void handle_incoming_uart_byte(char c) {
    // If we hit a newline or carriage return, the message is complete
    if (c == '\n' || c == '\r') {
        if (esp_msg_index > 0) {
            esp_msg_buffer[esp_msg_index] = '\0'; // Null-terminate the string
            
            // THIS is what passes the finished string to your perfect parser
            parse_esp_command(esp_msg_buffer); 
            
            esp_msg_index = 0; // Reset for the next message
        }
    } 
    else {
        // Keep adding characters to the buffer
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

void request_esp_ip(void) {
    const char *command = "GET_IP\n";
    LPUART_WriteBlocking(ESP_UART_BASE, (const uint8_t *)command, strlen(command));
    PRINTF("Requested IP from ESP board...\r\n");
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
