import os

with open('source/wifi_config.c', 'r', encoding='utf-8') as f:
    content = f.read()

old_block = """    else if ((!strcmp(keyword, "kp_straight") || !strcmp(keyword, "kp_s")) && has_val) { g_params.kp_straight = val; send_ack_dbl("kp_straight=", val); }
    else if ((!strcmp(keyword, "kp_corner") || !strcmp(keyword, "kp_c")) && has_val) { g_params.kp_corner = val; send_ack_dbl("kp_corner=", val); }
    else if ((!strcmp(keyword, "kd_straight") || !strcmp(keyword, "kd_s")) && has_val) { g_params.kd_straight = val; send_ack_dbl("kd_straight=", val); }
    else if ((!strcmp(keyword, "kd_corner") || !strcmp(keyword, "kd_c")) && has_val) { g_params.kd_corner = val; send_ack_dbl("kd_corner=", val); }
    else if (!strcmp(keyword, "ki") && has_val) { g_params.ki = val; send_ack_dbl("ki=", val); }"""

new_block = """    else if ((!strcmp(keyword, "kp_straight") || !strcmp(keyword, "kp_s") ||
              !strcmp(keyword, "kp_corner")   || !strcmp(keyword, "kp_c") ||
              !strcmp(keyword, "kd_straight") || !strcmp(keyword, "kd_s") ||
              !strcmp(keyword, "kd_corner")   || !strcmp(keyword, "kd_c") ||
              !strcmp(keyword, "ki")) && has_val) {
        send_ack("ERR inactive PID param; steering uses k_head and k_cte\\r\\n");
    }"""

old_status = """        n = append_flt(buf, n, sizeof(buf), "kp_s=", g_params.kp_straight);
        n = append_flt(buf, n, sizeof(buf), "kp_c=", g_params.kp_corner);
        n = append_flt(buf, n, sizeof(buf), "ki=", g_params.ki);
        n = append_flt(buf, n, sizeof(buf), "kd_s=", g_params.kd_straight);
        n = append_flt(buf, n, sizeof(buf), "kd_c=", g_params.kd_corner);
        n = append_flt(buf, n, sizeof(buf), "spd_s=", g_params.speed_straight);"""

new_status = """        n = append_flt(buf, n, sizeof(buf), "spd_s=", g_params.speed_straight);"""

assert old_block in content, "old_block not found"
assert old_status in content, "old_status not found"

content = content.replace(old_block, new_block).replace(old_status, new_status)
with open('source/wifi_config_tmp.c', 'w', encoding='utf-8') as f:
    f.write(content)

os.replace('source/wifi_config_tmp.c', 'source/wifi_config.c')
print("SUCCESS")
