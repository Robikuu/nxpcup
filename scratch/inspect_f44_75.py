import re

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

print("=== DEEP DIVE: FRAMES 44 TO 75 ===")
for i in range(44, min(76, len(frames))):
    f = frames[i]
    print(f"[{f['idx']:2d}] L:{f['top_l']}/{f['frn_l']} CTE={f['f_cte']:5.1f} H={f['f_head']:5.1f} FS={f['fs']:5.1f} | S={f['s']:5.1f} (TS={f['ts']:5.1f}) | BL={f['bl']} ST={f['st']} | VL={f['vl']} VR={f['vr']}")
