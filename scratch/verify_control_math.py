import re
import math

with open(r'd:\nxpcup\scratch\user_run_oct31.txt', 'r') as f:
    lines = f.readlines()

frames = []
for idx, line in enumerate(lines):
    line = line.strip()
    if not line: continue
    clean_line = line
    if clean_line.startswith('LL:'):
        clean_line = 'L:' + clean_line[3:]
    m = re.search(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TS:(-?\d+)\s+FS:(-?\d+)\s+S:(-?\d+)\s+BL:(\d+)\s+BT:(\d+)\s+DOM:([TF])\s+FV:(\d+)\s+IX:(\d+)\s+ST:([SCI])\s+VL:(\d+)\s+VR:(\d+)', clean_line)
    if m:
        frames.append({
            'idx': len(frames),
            'top_l': int(m.group(1)),
            'frn_l': int(m.group(2)),
            'r_cte': float(m.group(3)) / 10.0,
            'f_cte': float(m.group(4)) / 10.0,
            'f_head': float(m.group(5)) / 10.0,
            'ts': float(m.group(6)) / 10.0,
            'fs': float(m.group(7)) / 10.0,
            's': float(m.group(8)) / 10.0,
            'bl': int(m.group(9)),
            'bt': int(m.group(10)),
            'dom': m.group(11),
            'fv': int(m.group(12)),
            'ix': int(m.group(13)),
            'st': m.group(14),
            'vl': int(m.group(15)),
            'vr': int(m.group(16))
        })

print("=== DETAILED PARAMETER VERIFICATION ===")
# k_cte = 0.55, k_head = 0.90
# TS = k_cte * f_cte + k_head * eff_heading
for f in frames[:25]:
    eff_h = f['f_head']
    # check heading deadband
    if f['bl'] == 0 and abs(eff_h) < 2.0:
        eff_h = 0.0
    calc_ts = 0.55 * f['f_cte'] + 0.90 * eff_h
    diff_ts = f['ts'] - calc_ts
    ff_part = f['s'] - f['ts']
    print(f"[{f['idx']:2d}] F_CTE={f['f_cte']:5.1f} H={f['f_head']:5.1f} | TS_log={f['ts']:5.1f} TS_calc={calc_ts:5.1f} | S_log={f['s']:5.1f} FF={ff_part:5.1f} | BL={f['bl']} ST={f['st']} VL/VR={f['vl']}/{f['vr']}")
