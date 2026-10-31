import re

with open(r'd:\nxpcup\scratch\user_run_oct31.txt', 'r') as f:
    lines = f.readlines()

frames = []
for idx, line in enumerate(lines):
    line = line.strip()
    if not line:
        continue
    clean_line = line
    if clean_line.startswith('LL:'):
        clean_line = 'L:' + clean_line[3:]
    m = re.search(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TS:(-?\d+)\s+FS:(-?\d+)\s+S:(-?\d+)\s+BL:(\d+)\s+BT:(\d+)\s+DOM:([TF])\s+FV:(\d+)\s+IX:(\d+)\s+ST:([SCI])\s+VL:(\d+)\s+VR:(\d+)', clean_line)
    if m:
        frames.append({
            'frame_num': len(frames),
            'file_line': idx + 1,
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
    elif 'start;' in line:
        frames.append({'event': 'START_CMD', 'file_line': idx + 1})
    elif 'OK state=RUNNING' in line:
        frames.append({'event': 'RUNNING_ACK', 'file_line': idx + 1})

print(f"{'Idx':>3} | {'L':>3} | {'R_CTE':>5} | {'F_CTE':>5} | {'F_Head':>6} | {'TS':>5} | {'FS':>5} | {'S':>5} | {'BL':>3} | {'BT':>3} | {'DOM':>3} | {'ST':>2} | {'VL':>3} {'VR':>3} | {'Notes'}")
print("-" * 80)
for item in frames:
    if 'event' in item:
        print(f" >>> EVENT: {item['event']} at line {item['file_line']}")
        continue
    f = item
    notes = []
    if f['top_l'] == 0:
        notes.append("TOP_LOST")
    if f['st'] == 'C':
        notes.append("CORNER")
    if f['st'] == 'S' and f['bl'] == 0:
        pass
    if f['s'] <= -60.0 or f['s'] >= 60.0:
        notes.append("STEER_SAT")
    if f['vl'] != f['vr']:
        notes.append(f"DIFF({f['vl']}-{f['vr']})")
    
    notes_str = ", ".join(notes)
    print(f"{f['frame_num']:3d} | {f['top_l']}/{f['frn_l']} | {f['r_cte']:5.1f} | {f['f_cte']:5.1f} | {f['f_head']:6.1f} | {f['ts']:5.1f} | {f['fs']:5.1f} | {f['s']:5.1f} | {f['bl']:3d} | {f['bt']:3d} |  {f['dom']}  | {f['st']} | {f['vl']:3d} {f['vr']:3d} | {notes_str}")
