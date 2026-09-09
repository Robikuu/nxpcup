#include "wifi_config.h"
#include "peripherals.h"
#include "fsl_lpuart.h"
#include "fsl_debug_console.h"
#include "hbridge.h"
#include "servo.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define CMD_BUF_SIZE 64

robot_params_t g_params;

static char   s_cmd_buf[CMD_BUF_SIZE];
static size_t s_cmd_idx = 0;

//TODO: de vezut si eventual de refacut tot fisierul asta

void params_init(void)
{
    g_params.kp = 3.0f;
    g_params.ki = 0.0f; 
    g_params.kd = 0.5f;
    g_params.k_cte     = 1.0f;
    g_params.k_heading = 0.0f;

    g_params.dt    = 0.02f;   /* CRITICAL — must be nonzero */
    g_params.alpha = 0.3f;    /* CRITICAL — must be in (0, 1] */

    g_params.base_speed_left  = 33.0f;   /* replaces base_speed */
    g_params.base_speed_right = 33.0f;

    g_params.max_speed_left  = 50.0f;
    g_params.max_speed_right = 50.0f;

    g_params.steering_offset = 23.0f;
    g_params.state           = STATE_IDLE;
}

static int parse_float(const char *s, float *out)
{
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

static void send_ack(const char *msg)
{
    LPUART_WriteBlocking(LP_FLEXCOMM9_PERIPHERAL,
                         (const uint8_t *)msg, strlen(msg));
}

static void send_ack_dbl(const char *prefix, float val)
{
    char buf[48];
    int  whole  = (int)val;
    float frac = val - whole;
    if (frac < 0) frac = -frac;
    int  f100   = (int)(frac * 100.0f);
    snprintf(buf, sizeof(buf), "OK %s%d.%02d\r\n", prefix, whole, f100);
    send_ack(buf);
}

static int append_flt(char *buf, int off, size_t cap, const char *label, float val)
{
    int whole = (int)val;
    float frac = val - whole;
    if (frac < 0) frac = -frac;
    int f100 = (int)(frac * 100.0f);
    return off + snprintf(buf + off, cap - (size_t)off,
                          "%s%d.%02d\r\n", label, whole, f100);
}

static void command_execute(char *cmd)
{
    char   keyword[32];
    float val     = 0.0f;
    int    has_val = 0;

    char *space = strchr(cmd, ' ');
    if (space) {
        send_ack("GOT SPACE\r\n");
        size_t klen = (size_t)(space - cmd);
        if (klen >= sizeof(keyword)) klen = sizeof(keyword) - 1;
        memcpy(keyword, cmd, klen);
        keyword[klen] = '\0';
        if (parse_float(space + 1, &val))
            has_val = 1;
        send_ack("PARSED\r\n");
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
    else if (!strcmp(keyword, "kp") && has_val) {
        g_params.kp = val;
        send_ack_dbl("kp=", val);
    }
    else if (!strcmp(keyword, "ki") && has_val) {
        g_params.ki = val;
        send_ack_dbl("ki=", val);
    }
    else if (!strcmp(keyword, "kd") && has_val) {
        g_params.kd = val;
        send_ack_dbl("kd=", val);
    }
    else if (!strcmp(keyword, "alpha") && has_val) {
        g_params.alpha = val;
        send_ack_dbl("alpha=", val);
    }
    else if (!strcmp(keyword, "base_speed") && has_val) {
        g_params.base_speed_left  = val;
        g_params.base_speed_right = val;
        send_ack_dbl("base_speed=", val);
    }
    else if (!strcmp(keyword, "speed_left") && has_val) {
        g_params.base_speed_left = val;
        send_ack_dbl("speed_left=", val);
    }
    else if (!strcmp(keyword, "speed_right") && has_val) {
        g_params.base_speed_right = val;
        send_ack_dbl("speed_right=", val);
    }
    else if (!strcmp(keyword, "max_speed_left") && has_val) {
        g_params.max_speed_left = val;
        send_ack_dbl("max_speed_left=", val);
    }
    else if (!strcmp(keyword, "max_speed_right") && has_val) {
        g_params.max_speed_right = val;
        send_ack_dbl("max_speed_right=", val);
    }
    else if (!strcmp(keyword, "steer") && has_val) {
        if (g_params.state == STATE_RUNNING) {
            send_ack("ERR steer only in IDLE. Send 'stop;' first.\r\n");
        } else {
            Steer(val);
            send_ack_dbl("steer=", val);
        }
    }
    else if (!strcmp(keyword, "steering_offset") && has_val) {
        g_params.steering_offset = val;
        send_ack_dbl("steering_offset=", val);
    }
    else if (!strcmp(keyword, "k_cte") && has_val) {
        g_params.k_cte = val;
        send_ack_dbl("k_cte=", val);
    }
    else if (!strcmp(keyword, "k_heading") && has_val) {
        g_params.k_heading = val;
        send_ack_dbl("k_heading=", val);
    }
    else if (!strcmp(keyword, "status")) {
        char buf[300];
        int  n = 0;
        n = append_flt(buf, n, sizeof(buf), "kp=",              g_params.kp);
        n = append_flt(buf, n, sizeof(buf), "ki=",              g_params.ki);
        n = append_flt(buf, n, sizeof(buf), "kd=",              g_params.kd);
        n = append_flt(buf, n, sizeof(buf), "alpha=",           g_params.alpha);
        n = append_flt(buf, n, sizeof(buf), "base_speed_left=",  g_params.base_speed_left);
        n = append_flt(buf, n, sizeof(buf), "base_speed_right=", g_params.base_speed_right);
        n = append_flt(buf, n, sizeof(buf), "max_speed_left=",   g_params.max_speed_left);
        n = append_flt(buf, n, sizeof(buf), "max_speed_right=",  g_params.max_speed_right);
        n = append_flt(buf, n, sizeof(buf), "steering_offset=", g_params.steering_offset);
        n = append_flt(buf, n, sizeof(buf), "k_cte=",           g_params.k_cte);
        n = append_flt(buf, n, sizeof(buf), "k_heading=",       g_params.k_heading);
        n = append_flt(buf, n, sizeof(buf), "dt=",              g_params.dt);
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "state=%s\r\n",
                      g_params.state ? "RUNNING" : "IDLE");
        send_ack(buf);
    }
    else if (!strcmp(keyword, "help")) {
        send_ack(
            "Commands (end with ;):\r\n"
            "  start              Begin lane following\r\n"
            "  stop               Stop and brake\r\n"
            "  kp <val>           PID proportional\r\n"
            "  ki <val>           PID integral\r\n"
            "  kd <val>           PID derivative\r\n"
            "  alpha <val>        Derivative LPF factor\r\n"
            "  base_speed <val>   Set both wheel base speeds\r\n");
            // send_ack(
            // "  speed_left <val>   Left wheel base speed\r\n"
            // "  speed_right <val>  Right wheel base speed\r\n"
            // "  max_speed_left <val>   Left speed clamp\r\n"
            // "  max_speed_right <val>  Right speed clamp\r\n"
            // "  steer <val>        Manual steer (-100..100, IDLE only)\r\n"
            // "  steering_offset <val>  Servo center offset\r\n"
            // "  k_cte <val>        Cross-track error weight\r\n"
            // "  k_heading <val>    Heading error weight\r\n"
            // "  status             Show all params\r\n"
            // "  help               This list\r\n");
    }
    else {
        send_ack("ERR unknown. Send 'help;'\r\n");
    }
}

void command_poll(void)
{
    while (LPUART_GetStatusFlags(LP_FLEXCOMM9_PERIPHERAL) & kLPUART_RxDataRegFullFlag) {
        uint8_t c = LPUART_ReadByte(LP_FLEXCOMM9_PERIPHERAL);

        if (c == ';') {
            if (s_cmd_idx > 0) {
                s_cmd_buf[s_cmd_idx] = '\0';
                command_execute(s_cmd_buf);
                s_cmd_idx = 0;
            }
        }
        else if (c != '\n' && c != '\r') {
            if (s_cmd_idx < CMD_BUF_SIZE - 1) {
                s_cmd_buf[s_cmd_idx] = (char)c;
                s_cmd_idx++;
            }
        }
    }
}

void esp_printf(const char *fmt, ...)
{
    char buf[128]; // adjust size if you expect longer lines

    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        // vsnprintf can report a length longer than the buffer if truncated —
        // clamp to what actually fits
        size_t actual_len = (len < (int)sizeof(buf)) ? (size_t)len : sizeof(buf) - 1;
        LPUART_WriteBlocking(LP_FLEXCOMM9_PERIPHERAL, (const uint8_t *)buf, actual_len);
    }
}
