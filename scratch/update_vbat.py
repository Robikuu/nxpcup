with open(r'D:\nxpcup\source\wifi_config.c', 'r', encoding='utf-8') as f:
    c = f.read().replace('\r\n', '\n')

old_init = '''    g_params.accel_ramp          = ACCEL_RAMP_STEP;
    g_params.decel_ramp          = DECEL_RAMP_STEP;
    g_params.state               = STATE_IDLE;'''

new_init = '''    g_params.accel_ramp          = ACCEL_RAMP_STEP;
    g_params.decel_ramp          = DECEL_RAMP_STEP;
    g_params.battery_voltage      = (float)DEFAULT_BATTERY_VOLTAGE;
    g_params.vbat_scale           = (float)CALIBRATION_BATTERY_VOLTAGE / (float)DEFAULT_BATTERY_VOLTAGE;
    g_params.state               = STATE_IDLE;'''

old_cmd = '''    else if (!strcmp(keyword, "decel_ramp") && has_val) { g_params.decel_ramp = val; send_ack_dbl("decel_ramp=", val); }'''

new_cmd = '''    else if (!strcmp(keyword, "decel_ramp") && has_val) { g_params.decel_ramp = val; send_ack_dbl("decel_ramp=", val); }
    else if ((!strcmp(keyword, "vbat") || !strcmp(keyword, "battery") || !strcmp(keyword, "voltage")) && has_val) {
        if (val >= 9.0f && val <= 15.0f) {
            g_params.battery_voltage = val;
            g_params.vbat_scale      = (float)CALIBRATION_BATTERY_VOLTAGE / val;
            if (g_params.vbat_scale > 1.25f) g_params.vbat_scale = 1.25f;
            if (g_params.vbat_scale < 0.70f) g_params.vbat_scale = 0.70f;
            send_ack_dbl("battery_voltage=", val);
        } else {
            send_ack("ERR invalid battery voltage\\r\\n");
        }
    }
    else if (!strcmp(keyword, "vbat_scale") && has_val) {
        if (val >= 0.50f && val <= 1.50f) {
            g_params.vbat_scale = val;
            send_ack_dbl("vbat_scale=", val);
        }
    }'''

old_status = '''        n = append_flt(buf, n, sizeof(buf), "ramp_d=", g_params.decel_ramp);'''

new_status = '''        n = append_flt(buf, n, sizeof(buf), "ramp_d=", g_params.decel_ramp);
        n = append_flt(buf, n, sizeof(buf), "vbat=", g_params.battery_voltage);
        n = append_flt(buf, n, sizeof(buf), "vscale=", g_params.vbat_scale);'''

old_help = '''        send_ack("Commands: start, stop, kp_straight, kp_corner, kd_straight, kd_corner, speed_straight, speed_out, speed_in, k_cte, k_heading, alpha, steer, motor_left, motor_right, accel_ramp, decel_ramp, status\\r\\n");'''

new_help = '''        send_ack("Commands: start, stop, kp_straight, kp_corner, kd_straight, kd_corner, speed_straight, speed_out, speed_in, k_cte, k_heading, alpha, steer, motor_left, motor_right, accel_ramp, decel_ramp, vbat, vbat_scale, status\\r\\n");'''

assert old_init in c, 'old_init not found'
assert old_cmd in c, 'old_cmd not found'
assert old_status in c, 'old_status not found'
assert old_help in c, 'old_help not found'

c = c.replace(old_init, new_init, 1)
c = c.replace(old_cmd, new_cmd, 1)
c = c.replace(old_status, new_status, 1)
c = c.replace(old_help, new_help, 1)

with open(r'D:\nxpcup\source\wifi_config.c', 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(c)
print('wifi_config.c updated successfully')
