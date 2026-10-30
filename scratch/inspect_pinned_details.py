import re

with open('d:/nxpcup/latest_user_request.txt', 'r', encoding='utf-8') as f:
    text = f.read()

lines = text.splitlines()
all_frames = []
cur_params = {'k_cte': 0.45, 'k_head': 1.15, 'fa_th': 18.0, 'ramp_a': 6.5, 'spd_s': 100, 'spd_out': 65, 'spd_in': 28}
in_r = False
run_id = 0

pattern = r'L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BL:([-\d]+)\s+BT:([-\d]+)\s+DOM:(\w)\s+FV:(\d+)\s+IX:(\d+)\s+ST:(\w+)\s+VL:([-\d]+)\s+VR:([-\d]+)'

for idx, l in enumerate(lines):
    m_p = re.match(r'OK ([a-zA-Z_]+)=([0-9.]+)', l)
    if m_p:
        k, v = m_p.groups()
        if k == 'k_cte': cur_params['k_cte'] = float(v)
        elif k == 'k_heading': cur_params['k_head'] = float(v)
        elif k == 'front_angle_thresh': cur_params['fa_th'] = float(v)
        elif k == 'accel_ramp': cur_params['ramp_a'] = float(v)
        elif k == 'speed_inside': cur_params['spd_in'] = float(v)
        elif k == 'speed_outside': cur_params['spd_out'] = float(v)
    elif 'OK state=RUNNING' in l:
        in_r = True
        run_id += 1
    elif 'OK state=IDLE' in l and in_r:
        in_r = False
    elif in_r and l.startswith('L:'):
        m = re.match(pattern, l)
        if m:
            lt, lf, r, f, h, ts, fs, s, bl, bt, dom, fv, ix, st, vl, vr = m.groups()
            all_frames.append({
                'run': run_id,
                'lno': idx+1,
                'lt': int(lt), 'lf': int(lf),
                'r': float(r)/10.0, 'f': float(f)/10.0,
                'h': float(h)/10.0, 'ts': float(ts)/10.0,
                'fs': float(fs)/10.0, 's': float(s)/10.0,
                'bl': float(bl), 'bt': float(bt),
                'dom': dom, 'fv': int(fv), 'ix': int(ix),
                'st': st, 'vl': int(vl), 'vr': int(vr),
                'fa_th': cur_params['fa_th'],
                'params': dict(cur_params)
            })

print('=== PINNED FRAMES IN STATE S (STRAIGHT) ===')
s_pinned = [fr for fr in all_frames if abs(fr['f']) >= 20.0 and fr['st'] == 'S']
for fr in s_pinned[:25]:
    # Check steering direction relative to CTE
    # if f < 0, recovering means s > 0. If s < 0, it's steering further left!
    print(f"Run {fr['run']} L{fr['lno']:4d}: L:{fr['lt']}/{fr['lf']} CTE={fr['f']:+5.1f} H={fr['h']:+5.1f} TS={fr['ts']:+5.1f} FS={fr['fs']:+5.1f} S={fr['s']:+5.1f} VL={fr['vl']:3d} VR={fr['vr']:3d} BT={fr['bt']:3.0f}")

print('\n=== PINNED FRAMES IN STATE C (CORNER) ===')
c_pinned = [fr for fr in all_frames if abs(fr['f']) >= 20.0 and fr['st'] == 'C']
for fr in c_pinned[:25]:
    print(f"Run {fr['run']} L{fr['lno']:4d}: L:{fr['lt']}/{fr['lf']} CTE={fr['f']:+5.1f} H={fr['h']:+5.1f} TS={fr['ts']:+5.1f} FS={fr['fs']:+5.1f} S={fr['s']:+5.1f} VL={fr['vl']:3d} VR={fr['vr']:3d} BT={fr['bt']:3.0f}")
