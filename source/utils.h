#ifndef UTILS_H
#define UTILS_H

void receive_message();
void buttons_init(void);
int button_pressed_edge(void);
void print_double(const char *label, float val);
void TestPidSteer(void);
void TestHBridge(void);

#endif
