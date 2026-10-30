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

pinned_frames = [fr for fr in all_frames if abs(fr['f']) >= 20.0]
print(f'Total pinned frames (|F| >= 20.0): {len(pinned_frames)} / {len(all_frames)} ({len(pinned_frames)/len(all_frames)*100:.1f}%)')

# Categorize pinned frames by state and wheel speed
st_counts = {}
vl_vr_equal = 0
lt1_count = 0
steering_towards_wall = 0
steering_neutral = 0 # |s| < 5

for fr in pinned_frames:
    st_counts[fr['st']] = st_counts.get(fr['st'], 0) + 1
    if fr['vl'] == fr['vr']:
        vl_vr_equal += 1
    if fr['lt'] <= 1:
        lt1_count += 1
    # Check if steering is towards the wall:
    # if f > 0 (car is right of center), steering towards wall means s > 0
    # if f < 0 (car is left of center), steering towards wall means s < 0
    if (fr['f'] > 0 and fr['s'] > 0) or (fr['f'] < 0 and fr['s'] < 0):
        steering_towards_wall += 1
    if abs(fr['s']) < 5.0:
        steering_neutral += 1

print(f'State distribution while pinned: {st_counts}')
print(f'Frames with VL == VR (NO differential drag): {vl_vr_equal} / {len(pinned_frames)} ({vl_vr_equal/len(pinned_frames)*100:.1f}%)')
print(f'Frames with L_top <= 1: {lt1_count} / {len(pinned_frames)} ({lt1_count/len(pinned_frames)*100:.1f}%)')
print(f'Frames steering TOWARDS the wall: {steering_towards_wall} / {len(pinned_frames)} ({steering_towards_wall/len(pinned_frames)*100:.1f}%)')
print(f'Frames steering neutral (|s| < 5 deg): {steering_neutral} / {len(pinned_frames)} ({steering_neutral/len(pinned_frames)*100:.1f}%)')
