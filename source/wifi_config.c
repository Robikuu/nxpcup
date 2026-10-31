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
#include "board.h"

#define CMD_BUF_SIZE 64

static char   s_cmd_buf[CMD_BUF_SIZE];
static size_t s_cmd_idx = 0;
volatile uint8_t g_dump_vectors = 0;
static uint8_t s_k_cte_corner_overridden = 0;
static uint8_t s_k_heading_corner_overridden = 0;

void params_load_preset(uint8_t preset_id) {
    if (preset_id == 2U) {
        g_params.active_preset        = 2U;
        g_params.speed_straight       = PRESET2_SPEED_STRAIGHT;
        g_params.speed_outside_corner = PRESET2_SPEED_OUTSIDE_CORNER;
        g_params.speed_inside_min     = PRESET2_SPEED_INSIDE_MIN;
        g_params.k_cte                = PRESET2_K_CTE;
        g_params.k_heading            = PRESET2_K_HEADING;
        g_params.k_cte_corner         = PRESET2_K_CTE_CORNER;
        g_params.k_heading_corner     = PRESET2_K_HEADING_CORNER;
        g_params.h_lo                 = PRESET2_H_LO;
        g_params.h_hi                 = PRESET2_H_HI;
        g_params.blend_attack         = PRESET2_BLEND_ATTACK;
        g_params.blend_release        = PRESET2_BLEND_RELEASE;
        g_params.front_angle_thresh   = PRESET2_FRONT_ANGLE_THRESH;
        g_params.blend_enable         = PRESET2_BLEND_ENABLE;
        g_params.steering_offset      = PRESET2_STEERING_OFFSET;
        g_params.accel_ramp           = PRESET2_ACCEL_RAMP;
        g_params.decel_ramp           = PRESET2_DECEL_RAMP;
        g_params.drive_scale          = PRESET2_DRIVE_SCALE;
        g_params.motor_enable         = PRESET2_MOTOR_ENABLE;
    } else {
        g_params.active_preset        = 1U;
        g_params.speed_straight       = PRESET1_SPEED_STRAIGHT;
        g_params.speed_outside_corner = PRESET1_SPEED_OUTSIDE_CORNER;
        g_params.speed_inside_min     = PRESET1_SPEED_INSIDE_MIN;
        g_params.k_cte                = PRESET1_K_CTE;
        g_params.k_heading            = PRESET1_K_HEADING;
        g_params.k_cte_corner         = PRESET1_K_CTE_CORNER;
        g_params.k_heading_corner     = PRESET1_K_HEADING_CORNER;
        g_params.h_lo                 = PRESET1_H_LO;
        g_params.h_hi                 = PRESET1_H_HI;
        g_params.blend_attack         = PRESET1_BLEND_ATTACK;
        g_params.blend_release        = PRESET1_BLEND_RELEASE;
        g_params.front_angle_thresh   = PRESET1_FRONT_ANGLE_THRESH;
        g_params.blend_enable         = PRESET1_BLEND_ENABLE;
        g_params.steering_offset      = PRESET1_STEERING_OFFSET;
        g_params.accel_ramp           = PRESET1_ACCEL_RAMP;
        g_params.decel_ramp           = PRESET1_DECEL_RAMP;
        g_params.drive_scale          = PRESET1_DRIVE_SCALE;
        g_params.motor_enable         = PRESET1_MOTOR_ENABLE;
    }
    s_k_cte_corner_overridden = 0;
    s_k_heading_corner_overridden = 0;
    Steer(g_params.steering_offset);
    /* Update on-board Red LED to reflect active preset (active-low: ON=Preset2, OFF=Preset1) */
    if (preset_id == 2U) {
        LED_RED_ON();
    } else {
        LED_RED_OFF();
    }
}

void params_switch_preset(void) {
    if (g_params.active_preset == 1U) {
        params_load_preset(2U);
    } else {
        params_load_preset(1U);
    }
}

void params_init(void) {
    g_params.state = STATE_IDLE;
    params_load_preset(1U);
}

static volatile uint32_t s_motor_test_ticks = 0;
static volatile int16_t  s_motor_test_left  = 0;
static volatile int16_t  s_motor_test_right = 0;

void motor_test_start(int16_t speed_l, int16_t speed_r, uint32_t ticks) {
    s_motor_test_left  = speed_l;
    s_motor_test_right = speed_r;
    s_motor_test_ticks = ticks;
    HbridgeSpeed(&g_hbridge, speed_l, speed_r);
}

void motor_test_stop(void) {
    s_motor_test_ticks = 0;
    s_motor_test_left  = 0;
    s_motor_test_right = 0;
    HbridgeBrake(&g_hbridge);
}

bool motor_test_is_active(void) {
    return (s_motor_test_ticks > 0);
}

void motor_test_tick(void) {
    if (s_motor_test_ticks > 0) {
        if (--s_motor_test_ticks == 0) {
            motor_test_stop();
            esp_printf("OK motor test complete\r\n");
        } else {
            HbridgeSpeed(&g_hbridge, s_motor_test_left, s_motor_test_right);
        }
    }
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
    int  f100   = (int)(frac * 100.0f + 0.5f);
    if (f100 >= 100) {
        f100 -= 100;
        whole += (val < 0.0f) ? -1 : 1;
    }
    const char *sign = (val < 0.0f && whole == 0 && f100 > 0) ? "-" : "";
    snprintf(buf, sizeof(buf), "OK %s%s%d.%02d\r\n", prefix, sign, whole, f100);
    send_ack(buf);
}

static int append_flt(char *buf, int off, size_t cap, const char *label, float val) {
    int whole = (int)val;
    float frac = val - (float)whole;
    if (frac < 0) frac = -frac;
    int f100 = (int)(frac * 100.0f + 0.5f);
    if (f100 >= 100) {
        f100 -= 100;
        whole += (val < 0.0f) ? -1 : 1;
    }
    const char *sign = (val < 0.0f && whole == 0 && f100 > 0) ? "-" : "";
    return off + snprintf(buf + off, cap - (size_t)off, "%s%s%d.%02d\r\n", label, sign, whole, f100);
}

static void command_execute(char *cmd) {
    char keyword[32];
    float val = 0.0f;
    int has_val = 0;

    while (*cmd != '\0' && ((unsigned char)*cmd <= ' ' || (unsigned char)*cmd >= 127U)) {
        cmd++;
    }
    if (*cmd == '\0') {
        return;
    }

    char *space = strchr(cmd, ' ');

    if (space) {
        size_t klen = (size_t)(space - cmd);
        if (klen >= sizeof(keyword)) klen = sizeof(keyword) - 1;
        memcpy(keyword, cmd, klen);
        keyword[klen] = '\0';
        while (*space == ' ') space++;
        if (parse_float(space, &val)) has_val = 1;
    } else {
        strncpy(keyword, cmd, sizeof(keyword) - 1);
        keyword[sizeof(keyword) - 1] = '\0';
    }

    for (size_t i = 0; keyword[i] != '\0'; i++) {
        if (keyword[i] >= 'A' && keyword[i] <= 'Z') {
            keyword[i] = (char)(keyword[i] - 'A' + 'a');
        }
    }

    if (!strcmp(keyword, "start")) {
        motor_test_stop();
        g_params.state = STATE_RUNNING;
        send_ack("OK state=RUNNING\r\n");
    }
    else if (!strncmp(cmd, "stop", 4) || !strncmp(keyword, "stop", 4) || !strcmp(keyword, "brake")) {
        motor_test_stop();
        g_params.state = STATE_IDLE;
        HbridgeBrake(&g_hbridge);
        Steer(g_params.steering_offset);
        send_ack("OK state=IDLE\r\n");
    }
    else if ((!strcmp(keyword, "speed_straight") || !strcmp(keyword, "spd_s") || !strcmp(keyword, "speed_s")) && has_val) {
        g_params.speed_straight = val;
        send_ack_dbl("speed_straight=", val);
    }
    else if ((!strcmp(keyword, "speed_outside") || !strcmp(keyword, "speed_out") || !strcmp(keyword, "spd_out")) && has_val) {
        g_params.speed_outside_corner = val;
        send_ack_dbl("speed_outside=", val);
    }
    else if ((!strcmp(keyword, "speed_inside") || !strcmp(keyword, "speed_in") || !strcmp(keyword, "spd_in") || !strcmp(keyword, "speed_corner")) && has_val) {
        g_params.speed_inside_min = val;
        send_ack_dbl("speed_inside=", val);
    }
    else if (!strcmp(keyword, "k_cte") && has_val) {
        g_params.k_cte = val;
        if (!s_k_cte_corner_overridden) g_params.k_cte_corner = val;
        send_ack_dbl("k_cte=", val);
    }
    else if ((!strcmp(keyword, "k_heading") || !strcmp(keyword, "k_head")) && has_val) {
        g_params.k_heading = val;
        if (!s_k_heading_corner_overridden) g_params.k_heading_corner = val;
        send_ack_dbl("k_heading=", val);
    }
    else if ((!strcmp(keyword, "k_cte_c") || !strcmp(keyword, "k_cte_corner")) && has_val) {
        g_params.k_cte_corner = val;
        s_k_cte_corner_overridden = 1;
        send_ack_dbl("k_cte_corner=", val);
    }
    else if ((!strcmp(keyword, "k_head_c") || !strcmp(keyword, "k_heading_c") || !strcmp(keyword, "k_heading_corner")) && has_val) {
        g_params.k_heading_corner = val;
        s_k_heading_corner_overridden = 1;
        send_ack_dbl("k_heading_corner=", val);
    }
    else if (!strcmp(keyword, "h_lo") && has_val) {
        if (val >= 0.0f && val < g_params.h_hi - 1.0f) {
            g_params.h_lo = val;
            send_ack_dbl("h_lo=", val);
        } else {
            send_ack("ERR h_lo must be >= 0 and < (h_hi - 1.0)\r\n");
        }
    }
    else if (!strcmp(keyword, "h_hi") && has_val) {
        if (val > g_params.h_lo + 1.0f && val <= 90.0f) {
            g_params.h_hi = val;
            send_ack_dbl("h_hi=", val);
        } else {
            send_ack("ERR h_hi must be > (h_lo + 1.0) and <= 90\r\n");
        }
    }
    else if ((!strcmp(keyword, "rel") || !strcmp(keyword, "blend_rel")) && has_val) {
        if (val >= 0.01f && val <= 0.50f) {
            g_params.blend_release = val;
            send_ack_dbl("blend_release=", val);
        } else {
            send_ack("ERR rel range: 0.01 to 0.50\r\n");
        }
    }
    else if ((!strcmp(keyword, "att") || !strcmp(keyword, "blend_att")) && has_val) {
        if (val >= 0.05f && val <= 1.00f) {
            g_params.blend_attack = val;
            send_ack_dbl("blend_attack=", val);
        } else {
            send_ack("ERR att range: 0.05 to 1.00\r\n");
        }
    }
    else if ((!strcmp(keyword, "fa_th") || !strcmp(keyword, "fa_thresh")) && has_val) {
        if (val >= 5.0f && val <= 45.0f) {
            g_params.front_angle_thresh = val;
            send_ack_dbl("front_angle_thresh=", val);
        } else {
            send_ack("ERR fa_th range: 5 to 45\r\n");
        }
    }
    else if (!strcmp(keyword, "blend") && has_val) {
        g_params.blend_enable = (uint8_t)(val > 0.5f ? 1U : 0U);
        send_ack_dbl("blend_enable=", (float)g_params.blend_enable);
    }
    else if ((!strcmp(keyword, "steering_offset") || !strcmp(keyword, "offset")) && has_val) {
        g_params.steering_offset = val;
        send_ack_dbl("steering_offset=", val);
    }
    else if ((!strcmp(keyword, "accel_ramp") || !strcmp(keyword, "ramp_a")) && has_val) {
        g_params.accel_ramp = val;
        send_ack_dbl("accel_ramp=", val);
    }
    else if ((!strcmp(keyword, "decel_ramp") || !strcmp(keyword, "ramp_d")) && has_val) {
        g_params.decel_ramp = val;
        send_ack_dbl("decel_ramp=", val);
    }
    else if ((!strcmp(keyword, "scale") || !strcmp(keyword, "drive_scale")) && has_val) {
        if (val < 0.10f) val = 0.10f;
        if (val > 1.00f) val = 1.00f;
        g_params.drive_scale = val;
        send_ack_dbl("drive_scale=", val);
    }
    else if (!strcmp(keyword, "steer") && has_val) {
        if (g_params.state == STATE_RUNNING) {
            send_ack("ERR steer only in IDLE. Send 'stop;' first.\r\n");
        } else {
            Steer(val + g_params.steering_offset);
            send_ack_dbl("steer=", val);
        }
    }
    else if (!strcmp(keyword, "motor_left")) {
        if (g_params.state == STATE_RUNNING) {
            send_ack("ERR motor only in IDLE. Send 'stop;' first.\r\n");
        } else {
            float spd = has_val ? val : 30.0f;
            if (spd > 100.0f) spd = 100.0f;
            if (spd < -100.0f) spd = -100.0f;
            motor_test_start((int16_t)spd, 0, 200);
            send_ack_dbl("OK motor_left 4s speed=", spd);
        }
    }
    else if (!strcmp(keyword, "motor_right")) {
        if (g_params.state == STATE_RUNNING) {
            send_ack("ERR motor only in IDLE. Send 'stop;' first.\r\n");
        } else {
            float spd = has_val ? val : 30.0f;
            if (spd > 100.0f) spd = 100.0f;
            if (spd < -100.0f) spd = -100.0f;
            motor_test_start(0, (int16_t)spd, 200);
            send_ack_dbl("OK motor_right 4s speed=", spd);
        }
    }
    else if ((!strcmp(keyword, "motor_enable") || !strcmp(keyword, "drive")) && has_val) {
        g_params.motor_enable = (uint8_t)val;
        if (!g_params.motor_enable) {
            HbridgeBrake(&g_hbridge);
        }
        send_ack_dbl("motor_enable=", val);
    }
    else if (!strcmp(keyword, "dump_vectors") || !strcmp(keyword, "dump")) {
        if (has_val) {
            g_dump_vectors = (uint8_t)val;
        } else {
            g_dump_vectors = (g_dump_vectors == 0) ? 2 : 0;
        }
        send_ack_dbl("dump_vectors=", (float)g_dump_vectors);
    }
    else if (!strcmp(keyword, "preset")) {
        if (has_val) {
            uint8_t p = (uint8_t)val;
            if (p == 1U || p == 2U) {
                params_load_preset(p);
                send_ack_dbl("preset=", (float)g_params.active_preset);
            } else {
                send_ack("ERR preset must be 1 or 2\r\n");
            }
        } else {
            params_switch_preset();
            send_ack_dbl("preset=", (float)g_params.active_preset);
        }
    }
    else if (!strcmp(keyword, "version")) {
        char vbuf[96];
        snprintf(vbuf, sizeof(vbuf), "BUILD: %s %s dual-cam-v2\r\n", __DATE__, __TIME__);
        send_ack(vbuf);
    }
    else if (!strcmp(keyword, "status")) {
        char buf[512];
        int n = 0;
        n = append_flt(buf, n, sizeof(buf), "spd_s=", g_params.speed_straight);
        n = append_flt(buf, n, sizeof(buf), "spd_out=", g_params.speed_outside_corner);
        n = append_flt(buf, n, sizeof(buf), "spd_in=", g_params.speed_inside_min);
        n = append_flt(buf, n, sizeof(buf), "offset=", g_params.steering_offset);
        n = append_flt(buf, n, sizeof(buf), "k_cte=", g_params.k_cte);
        n = append_flt(buf, n, sizeof(buf), "k_head=", g_params.k_heading);
        n = append_flt(buf, n, sizeof(buf), "k_cte_c=", g_params.k_cte_corner);
        n = append_flt(buf, n, sizeof(buf), "k_head_c=", g_params.k_heading_corner);
        n = append_flt(buf, n, sizeof(buf), "h_lo=", g_params.h_lo);
        n = append_flt(buf, n, sizeof(buf), "h_hi=", g_params.h_hi);
        n = append_flt(buf, n, sizeof(buf), "rel=", g_params.blend_release);
        n = append_flt(buf, n, sizeof(buf), "att=", g_params.blend_attack);
        n = append_flt(buf, n, sizeof(buf), "fa_th=", g_params.front_angle_thresh);
        n = append_flt(buf, n, sizeof(buf), "ramp_a=", g_params.accel_ramp);
        n = append_flt(buf, n, sizeof(buf), "ramp_d=", g_params.decel_ramp);
        n = append_flt(buf, n, sizeof(buf), "scale=", g_params.drive_scale);
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "blend=%u\r\n", (unsigned)g_params.blend_enable);
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "drive=%u\r\n", (unsigned)g_params.motor_enable);
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "dump=%u\r\n", (unsigned)g_dump_vectors);
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "preset=%u\r\n", (unsigned)g_params.active_preset);
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "state=%s\r\n", g_params.state ? "RUNNING" : "IDLE");
        send_ack(buf);
    }
    else if (!strcmp(keyword, "help")) {
        send_ack("Commands: start, stop, preset [1/2], motor_left, motor_right, version, status, steer, drive [0/1], blend [0/1], dump [0/1], scale, spd_s, spd_out, spd_in, k_head, k_cte, k_head_c, k_cte_c, h_lo, h_hi, rel, att, fa_th, offset, ramp_a, ramp_d\r\n");
    }
    else {
        send_ack("ERR unknown command\r\n");
    }
}

void command_poll(void) {
    uint32_t err_flags = LPUART_GetStatusFlags(LP_FLEXCOMM9_PERIPHERAL) &
                         (uint32_t)(kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag |
                                    kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag);
    if (err_flags != 0U) {
        LPUART_ClearStatusFlags(LP_FLEXCOMM9_PERIPHERAL, err_flags);
    }
    while (LPUART_GetStatusFlags(LP_FLEXCOMM9_PERIPHERAL) & kLPUART_RxDataRegFullFlag) {
        uint8_t c = LPUART_ReadByte(LP_FLEXCOMM9_PERIPHERAL);
        if (c == ';') {
            if (s_cmd_idx > 0) {
                s_cmd_buf[s_cmd_idx] = '\0';
                command_execute(s_cmd_buf);
                s_cmd_idx = 0;
            }
        } else if (c == '\n' || c == '\r') {
            if (s_cmd_idx == 0) {
                continue;
            }
        } else if (c >= 32U && c < 127U) {
            if (s_cmd_idx < CMD_BUF_SIZE - 1) {
                s_cmd_buf[s_cmd_idx] = (char)c;
                s_cmd_idx++;
            }
        }
        err_flags = LPUART_GetStatusFlags(LP_FLEXCOMM9_PERIPHERAL) &
                    (uint32_t)(kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag |
                               kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag);
        if (err_flags != 0U) {
            LPUART_ClearStatusFlags(LP_FLEXCOMM9_PERIPHERAL, err_flags);
        }
    }
}

void esp_printf(const char *fmt, ...) {
    char buf[384];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        size_t actual_len = (len < (int)sizeof(buf)) ? (size_t)len : sizeof(buf) - 1;
        LPUART_WriteBlocking(LP_FLEXCOMM9_PERIPHERAL, (const uint8_t *)buf, actual_len);
    }
}