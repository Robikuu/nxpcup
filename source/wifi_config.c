#include "wifi_config.h"
#include "peripherals.h"
#include "fsl_lpuart.h"
#include "fsl_debug_console.h"
#include "hbridge.h"
#include "servo.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "Config.h"

#define CMD_BUF_SIZE 64

static char   s_cmd_buf[CMD_BUF_SIZE];
static size_t s_cmd_idx = 0;

void params_init(void) {
    g_params.kp_straight         = KP_STRAIGHT;
    g_params.kp_corner           = KP_CORNER;
    g_params.ki                  = DEFAULT_KI; 
    g_params.kd_straight         = KD_STRAIGHT;
    g_params.kd_corner           = KD_CORNER;
    g_params.speed_straight       = SPEED_STRAIGHT;
    g_params.speed_outside_corner = SPEED_OUTSIDE_CORNER;
    g_params.speed_inside_min     = SPEED_INSIDE_MIN;
    g_params.speed_outside_sharp  = SPEED_OUTSIDE_SHARP;
    g_params.speed_inside_sharp   = SPEED_INSIDE_SHARP;
    g_params.k_cte               = DEFAULT_K_CTE;
    g_params.k_heading           = DEFAULT_K_HEADING;
    g_params.dt                  = DEFAULT_DT;
    g_params.alpha               = DEFAULT_ALPHA;
    g_params.steering_offset     = STEERING_OFFSET;
    g_params.accel_ramp          = ACCEL_RAMP_STEP;
    g_params.decel_ramp          = DECEL_RAMP_STEP;
    g_params.state               = STATE_IDLE;
}

static int parse_float(const char *s, float *out) {
    while (*s == ' ' || *s == '\t') s++;
    int sign = 1;
    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }
    if ((*s < '0' || *s > '9') && *s != '.') return 0;
    
    float whole = 0.0f;
    int has_digits = 0;
    while (*s >= '0' && *s <= '9') {
        whole = whole * 10.0f + (float)(*s - '0');
        s++;
        has_digits = 1;
    }
    
    float frac = 0.0f;
    if (*s == '.') {
        s++;
        float scale = 0.1f;
        while (*s >= '0' && *s <= '9') {
            frac += (float)(*s - '0') * scale;
            scale *= 0.1f;
            s++;
            has_digits = 1;
        }
    }
    if (!has_digits) return 0;
    *out = (float)sign * (whole + frac);
    return 1;
}

static void send_ack(const char *msg) {
    LPUART_WriteBlocking(LP_FLEXCOMM9_PERIPHERAL, (const uint8_t *)msg, strlen(msg));
}

static void send_ack_dbl(const char *prefix, float val) {
    char buf[48];
    int  whole  = (int)val;
    float frac = val - (float)whole;
    if (frac < 0) frac = -frac;
    int  f100   = (int)(frac * 100.0f);
    snprintf(buf, sizeof(buf), "OK %s%d.%02d\r\n", prefix, whole, f100);
    send_ack(buf);
}

static int append_flt(char *buf, int off, size_t cap, const char *label, float val) {
    int whole = (int)val;
    float frac = val - (float)whole;
    if (frac < 0) frac = -frac;
    int f100 = (int)(frac * 100.0f);
    return off + snprintf(buf + off, cap - (size_t)off, "%s%d.%02d\r\n", label, whole, f100);
}

static void command_execute(char *cmd) {
    char keyword[32];
    float val = 0.0f;
    int has_val = 0;
    char *space = strchr(cmd, ' ');

    if (space) {
        size_t klen = (size_t)(space - cmd);
        if (klen >= sizeof(keyword)) klen = sizeof(keyword) - 1;
        memcpy(keyword, cmd, klen);
        keyword[klen] = '\0';
        if (parse_float(space + 1, &val)) has_val = 1;
    } else {
        strncpy(keyword, cmd, sizeof(keyword) - 1);
        keyword[sizeof(keyword) - 1] = '\0';
    }

    if (!strcmp(keyword, "start")) {
        g_params.state = STATE_RUNNING;
        send_ack("OK state=RUNNING\r\n");
    }
    else if (!strcmp(keyword, "stop") || !strcmp(keyword, "brake")) {
        g_params.state = STATE_IDLE;
        HbridgeBrake(&g_hbridge);
        send_ack("OK state=IDLE\r\n");
    }
    else if (!strcmp(keyword, "kp_straight") && has_val) { g_params.kp_straight = val; send_ack_dbl("kp_straight=", val); }
    else if (!strcmp(keyword, "kp_corner") && has_val) { g_params.kp_corner = val; send_ack_dbl("kp_corner=", val); }
    else if (!strcmp(keyword, "kd_straight") && has_val) { g_params.kd_straight = val; send_ack_dbl("kd_straight=", val); }
    else if (!strcmp(keyword, "kd_corner") && has_val) { g_params.kd_corner = val; send_ack_dbl("kd_corner=", val); }
    else if (!strcmp(keyword, "ki") && has_val) { g_params.ki = val; send_ack_dbl("ki=", val); }
    else if (!strcmp(keyword, "speed_straight") && has_val) { g_params.speed_straight = val; send_ack_dbl("speed_straight=", val); }
    else if ((!strcmp(keyword, "speed_outside") || !strcmp(keyword, "speed_out")) && has_val) { g_params.speed_outside_corner = val; send_ack_dbl("speed_outside=", val); }
    else if ((!strcmp(keyword, "speed_inside") || !strcmp(keyword, "speed_in") || !strcmp(keyword, "speed_corner")) && has_val) { g_params.speed_inside_min = val; send_ack_dbl("speed_inside=", val); }
    else if ((!strcmp(keyword, "speed_out_sharp") || !strcmp(keyword, "speed_outside_sharp")) && has_val) { g_params.speed_outside_sharp = val; send_ack_dbl("speed_out_sharp=", val); }
    else if ((!strcmp(keyword, "speed_in_sharp") || !strcmp(keyword, "speed_inside_sharp")) && has_val) { g_params.speed_inside_sharp = val; send_ack_dbl("speed_in_sharp=", val); }
    else if (!strcmp(keyword, "alpha") && has_val) { g_params.alpha = val; send_ack_dbl("alpha=", val); }
    else if (!strcmp(keyword, "k_cte") && has_val) { g_params.k_cte = val; send_ack_dbl("k_cte=", val); }
    else if (!strcmp(keyword, "k_heading") && has_val) { g_params.k_heading = val; send_ack_dbl("k_heading=", val); }
    else if (!strcmp(keyword, "steering_offset") && has_val) { g_params.steering_offset = val; send_ack_dbl("steering_offset=", val); }
    else if (!strcmp(keyword, "accel_ramp") && has_val) { g_params.accel_ramp = val; send_ack_dbl("accel_ramp=", val); }
    else if (!strcmp(keyword, "decel_ramp") && has_val) { g_params.decel_ramp = val; send_ack_dbl("decel_ramp=", val); }
    else if (!strcmp(keyword, "steer") && has_val) {
        if (g_params.state == STATE_RUNNING) {
            send_ack("ERR steer only in IDLE. Send 'stop;' first.\r\n");
        } else {
            Steer(val + g_params.steering_offset);
            send_ack_dbl("steer=", val);
        }
    }
    else if (!strcmp(keyword, "status")) {
        char buf[512];
        int n = 0;
        n = append_flt(buf, n, sizeof(buf), "kp_s=", g_params.kp_straight);
        n = append_flt(buf, n, sizeof(buf), "kp_c=", g_params.kp_corner);
        n = append_flt(buf, n, sizeof(buf), "ki=", g_params.ki);
        n = append_flt(buf, n, sizeof(buf), "kd_s=", g_params.kd_straight);
        n = append_flt(buf, n, sizeof(buf), "kd_c=", g_params.kd_corner);
        n = append_flt(buf, n, sizeof(buf), "spd_s=", g_params.speed_straight);
        n = append_flt(buf, n, sizeof(buf), "spd_out=", g_params.speed_outside_corner);
        n = append_flt(buf, n, sizeof(buf), "spd_in=", g_params.speed_inside_min);
        n = append_flt(buf, n, sizeof(buf), "spd_out_sh=", g_params.speed_outside_sharp);
        n = append_flt(buf, n, sizeof(buf), "spd_in_sh=", g_params.speed_inside_sharp);
        n = append_flt(buf, n, sizeof(buf), "offset=", g_params.steering_offset);
        n = append_flt(buf, n, sizeof(buf), "k_cte=", g_params.k_cte);
        n = append_flt(buf, n, sizeof(buf), "k_head=", g_params.k_heading);
        n = append_flt(buf, n, sizeof(buf), "dt=", g_params.dt);
        n = append_flt(buf, n, sizeof(buf), "alpha=", g_params.alpha);
        n = append_flt(buf, n, sizeof(buf), "ramp_a=", g_params.accel_ramp);
        n = append_flt(buf, n, sizeof(buf), "ramp_d=", g_params.decel_ramp);
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "state=%s\r\n", g_params.state ? "RUNNING" : "IDLE");
        send_ack(buf);
    }
    else if (!strcmp(keyword, "help")) {
        send_ack("Commands: start, stop, kp_straight, kp_corner, kd_straight, kd_corner, speed_straight, speed_out, speed_in, speed_out_sharp, speed_in_sharp, k_cte, k_heading, alpha, steer, accel_ramp, decel_ramp, status\r\n");
    }
    else {
        send_ack("ERR unknown command\r\n");
    }
}

void command_poll(void) {
    while (LPUART_GetStatusFlags(LP_FLEXCOMM9_PERIPHERAL) & kLPUART_RxDataRegFullFlag) {
        uint8_t c = LPUART_ReadByte(LP_FLEXCOMM9_PERIPHERAL);
        if (c == ';') {
            if (s_cmd_idx > 0) {
                s_cmd_buf[s_cmd_idx] = '\0';
                command_execute(s_cmd_buf);
                s_cmd_idx = 0;
            }
        } else if (c != '\n' && c != '\r') {
            if (s_cmd_idx < CMD_BUF_SIZE - 1) {
                s_cmd_buf[s_cmd_idx] = (char)c;
                s_cmd_idx++;
            }
        }
    }
}

void esp_printf(const char *fmt, ...) {
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        size_t actual_len = (len < (int)sizeof(buf)) ? (size_t)len : sizeof(buf) - 1;
        LPUART_WriteBlocking(LP_FLEXCOMM9_PERIPHERAL, (const uint8_t *)buf, actual_len);
    }
}