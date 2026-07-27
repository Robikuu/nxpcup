#ifndef UTILS_H
#define UTILS_H

void request_esp_ip(void);
void esp_steer_test_loop(void);
void buttons_init(void);
int button_pressed_edge(void);
void print_double(const char *label, double val);
void putty_steer_test_loop(void);
void handle_incoming_uart_byte(char c);
void parse_esp_command(const char *message);


#endif