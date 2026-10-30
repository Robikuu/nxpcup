import re
import numpy as np

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

for r_id in range(1, 6):
    r_frames = [fr for fr in all_frames if fr['run'] == r_id]
    s_frames = [fr for fr in r_frames if fr['st'] == 'S']
    c_frames = [fr for fr in r_frames if fr['st'] == 'C']
    pinned_frames = [fr for fr in r_frames if abs(fr['f']) >= 20.0]
    severe_frames = [fr for fr in r_frames if abs(fr['f']) >= 30.0]
    
    k_cte = r_frames[0]['params']['k_cte']
    k_head = r_frames[0]['params']['k_head']
    
    s_cte_std = np.std([fr['f'] for fr in s_frames]) if s_frames else 0
    s_head_std = np.std([fr['h'] for fr in s_frames]) if s_frames else 0
    max_cte = max(abs(fr['f']) for fr in r_frames)
    
    print(f"Run {r_id} (k_cte={k_cte:.2f}, k_head={k_head:.2f}):")
    print(f"  Total frames: {len(r_frames)} ({len(r_frames)*0.08:.1f}s)")
    print(f"  Straight frames: {len(s_frames)}, CTE std: {s_cte_std:.2f} px, Head std: {s_head_std:.2f} deg")
    print(f"  Pinned (|CTE| >= 20 px): {len(pinned_frames)} / {len(r_frames)} ({len(pinned_frames)/len(r_frames)*100:.1f}%)")
    print(f"  Severe Blowout (|CTE| >= 30 px): {len(severe_frames)} / {len(r_frames)} ({len(severe_frames)/len(r_frames)*100:.1f}%)")
    print(f"  Max |CTE|: {max_cte:.1f} px")
