import os

with open('source/main.c', 'r', encoding='utf-8') as f:
    content = f.read()

old_decl = """            float steer_top    = 0.0f;
            float target_speed = g_params.speed_straight;
            float target_left  = target_speed;
            float target_right = target_speed;

            if (!g_params.blend_enable) {
                /* =========================================================================
                 * LEGACY DISCRETE MODE BYPASS (blend 0)
                 * Exact baseline behavior for instant A/B validation without reflashing
                 * ========================================================================= */
                s_corner_blend = horizon.is_sharp_corner ? 1.0f : 0.0f;
                s_blend_target = s_corner_blend;"""

new_decl = """            float front_t      = 0.0f;
            float head_t       = 0.0f;
            float steer_top    = 0.0f;
            float target_speed = g_params.speed_straight;
            float target_left  = target_speed;
            float target_right = target_speed;

            if (!g_params.blend_enable) {
                /* =========================================================================
                 * LEGACY DISCRETE MODE BYPASS (blend 0)
                 * Exact baseline behavior for instant A/B validation without reflashing
                 * ========================================================================= */
                front_t        = horizon.is_sharp_corner ? 1.0f : 0.0f;
                head_t         = 0.0f;
                s_corner_blend = front_t;
                s_blend_target = front_t;"""

old_blend_calc = """            } else {
                /* =========================================================================
                 * CONTINUOUS ASYMMETRIC CORNER BLEND GOVERNOR (blend 1)
                 * ========================================================================= */
                /* 1. Calculate Target & Diode Hold */
                float front_t = 0.0f;
                float head_t  = 0.0f;
                float hold    = 0.0f;

                if (s_intersection_ahead) {
                    /* Suppress corner blend during intersection transit with smooth release */
                    s_corner_blend += g_params.blend_release * (0.0f - s_corner_blend);
                    s_blend_target = 0.0f;
                } else {
                    /* Front Camera Preview Target */
                    if (horizon.found > 0) {
                        front_t = (fabsf(filtered_front_angle) - g_params.front_angle_thresh) / g_params.front_angle_thresh;
                        if (front_t < 0.0f) front_t = 0.0f;
                        if (front_t > 1.0f) front_t = 1.0f;
                    }"""

new_blend_calc = """            } else {
                /* =========================================================================
                 * CONTINUOUS ASYMMETRIC CORNER BLEND GOVERNOR (blend 1)
                 * ========================================================================= */
                /* 1. Calculate Target & Diode Hold */
                float hold = 0.0f;

                if (s_intersection_ahead) {
                    /* Suppress corner blend during intersection transit with smooth release */
                    s_corner_blend += g_params.blend_release * (0.0f - s_corner_blend);
                    s_blend_target = 0.0f;
                } else {
                    /* Front Camera Preview Target: decoupled start and ceiling */
                    if (horizon.found > 0) {
                        float fa_span = (float)FRONT_CORNER_ANGLE_THRESH_DEG - g_params.front_angle_thresh;
                        if (fa_span < 5.0f) fa_span = 5.0f;
                        front_t = (fabsf(filtered_front_angle) - g_params.front_angle_thresh) / fa_span;
                        if (front_t < 0.0f) front_t = 0.0f;
                        if (front_t > 1.0f) front_t = 1.0f;
                    }"""

old_telem = """            if (++s_telemetry_div >= 4) {
                s_telemetry_div = 0;
                esp_printf("L:%u/%u R:%d F:%d H:%d TS:%d FS:%d S:%d BL:%d BT:%d DOM:%c FV:%u IX:%u ST:%s VL:%d VR:%d\\r\\n",
                           (unsigned)sr.found, (unsigned)horizon.found,
                           (int)(sr.cte * 10.0f), (int)(filtered_cte * 10.0f),
                           (int)(filtered_heading * 10.0f),
                           (int)(steer_top * 10.0f),
                           (int)(filtered_front_angle * 10.0f),
                           (int)(steer_cmd * 10.0f),
                           (int)(s_corner_blend * 100.0f),
                           (int)(s_blend_target * 100.0f),
                           s_corner_blend > 0.30f ? 'F' : 'T',
                           horizon.found > 0 ? 1U : 0U,
                           (unsigned)s_intersection_ahead,
                           s_intersection_ahead ? "I" : (s_corner_blend > 0.30f ? "C" : "S"),
                           (int)applied_speed_left, (int)applied_speed_right);
            }"""

new_telem = """            if (++s_telemetry_div >= 4) {
                s_telemetry_div = 0;
                esp_printf("L:%u/%u R:%d F:%d H:%d TS:%d FS:%d S:%d BW:%d BL:%d FT:%d HT:%d DOM:%c FV:%u IX:%u ST:%s VL:%d VR:%d\\r\\n",
                           (unsigned)sr.found, (unsigned)horizon.found,
                           (int)(sr.cte * 10.0f), (int)(filtered_cte * 10.0f),
                           (int)(filtered_heading * 10.0f),
                           (int)(steer_top * 10.0f),
                           (int)(filtered_front_angle * 10.0f),
                           (int)(steer_cmd * 10.0f),
                           (int)(horizon.is_sharp_corner ? 0 : 100),
                           (int)(s_corner_blend * 100.0f),
                           (int)(front_t * 100.0f),
                           (int)(head_t * 100.0f),
                           horizon.is_sharp_corner ? 'F' : 'T',
                           horizon.found > 0 ? 1U : 0U,
                           (unsigned)s_intersection_ahead,
                           s_intersection_ahead ? "I" : (horizon.is_sharp_corner ? "C" : "S"),
                           (int)applied_speed_left, (int)applied_speed_right);
            }"""

assert old_decl in content, "old_decl not found"
assert old_blend_calc in content, "old_blend_calc not found"
assert old_telem in content, "old_telem not found"

content = content.replace(old_decl, new_decl).replace(old_blend_calc, new_blend_calc).replace(old_telem, new_telem)

with open('source/main_tmp.c', 'w', encoding='utf-8') as f:
    f.write(content)

os.replace('source/main_tmp.c', 'source/main.c')
print("MAIN SUCCESS")
