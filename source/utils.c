#include "utils.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"
#include <string.h>
#include <stdlib.h>
#include "servo.h"
#include "fsl_lpuart.h"
#include "peripherals.h"

//nume peripheral = LP_FLEXCOMM9_PERIPHERAL

static int button_prev_state = 1;

void receive_message() {
    uint8_t character;
    
    // Check if data has arrived in the hardware buffer
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LP_FLEXCOMM9_PERIPHERAL)) {
        // Read the character from the ESP
        character = LPUART_ReadByte(LP_FLEXCOMM9_PERIPHERAL);
        
        // Print it to the NXP debug console
        PUTCHAR(character); 
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

void print_double(const char *label, double val)
{
    int whole = (int)val;
    double frac_part = val - whole;
    if (frac_part < 0) frac_part = -frac_part;
    int frac = (int)(frac_part * 100.0);
    PRINTF("%s%d.%02d\r\n", label, whole, frac);
}
