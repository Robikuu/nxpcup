import re

with open(r"d:\nxpcup\scratch\full_live_test.txt", "r", encoding="utf-8") as f:
    text = f.read()

# Split into runs by 'start;\r'
parts = text.split('OK state=RUNNING')

pattern = re.compile(
    r'L:(\d+)/(\d+)\s+'
    r'R:(-?\d+)\s+'
    r'F:(-?\d+)\s+'
    r'H:(-?\d+)\s+'
    r'TS:(-?\d+)\s+'
    r'FS:(-?\d+)\s+'
    r'S:(-?\d+)\s+'
    r'BW:(\d+)\s+'
    r'DOM:([TFB])\s+'
    r'FV:(\d+)\s+'
    r'IX:(\d+)\s+'
    r'ST:([NIL])\s+'
    r'VL:(-?\d+)\s+'
    r'VR:(-?\d+)'
)

for run_idx, part in enumerate(parts[1:], 1):
    frames = []
    for line in part.splitlines():
        m = pattern.search(line)
        if m:
            frames.append({
                'top_found': int(m.group(1)),
                'front_found': int(m.group(2)),
                'raw_cte': int(m.group(3)),
                'filt_cte': int(m.group(4)),
                'heading': int(m.group(5)),
                'ts': int(m.group(6)),
                'fs': int(m.group(7)),
                's': int(m.group(8)),
                'bw': int(m.group(9)),
                'dom': m.group(10),
                'fv': int(m.group(11)),
                'ix': int(m.group(12)),
                'st': m.group(13),
                'vl': int(m.group(14)),
                'vr': int(m.group(15))
            })
    
    print(f"\n=================== RUN {run_idx} ({len(frames)} frames) ===================")
    if not frames:
        continue
    
    top_seen = sum(1 for f in frames if f['top_found'] > 0)
    front_seen = sum(1 for f in frames if f['front_found'] > 0)
    print(f"Top Camera Seeing:   {top_seen}/{len(frames)} ({100*top_seen/len(frames):.1f}%)")
    print(f"Front Camera Seeing: {front_seen}/{len(frames)} ({100*front_seen/len(frames):.1f}%)")
    
    # State breakdown
    st_n = sum(1 for f in frames if f['st'] == 'N')
    st_l = sum(1 for f in frames if f['st'] == 'L')
    print(f"States: Normal={st_n}, LineLost={st_l}")
    
    # DOM breakdown
    dom_t = sum(1 for f in frames if f['dom'] == 'T')
    dom_f = sum(1 for f in frames if f['dom'] == 'F')
    dom_b = sum(1 for f in frames if f['dom'] == 'B')
    print(f"Dominance: T={dom_t}, F={dom_f}, B={dom_b}")
    
    # Speeds
    avg_vl = sum(f['vl'] for f in frames) / len(frames)
    avg_vr = sum(f['vr'] for f in frames) / len(frames)
    print(f"Average Speeds: VL={avg_vl:.1f}, VR={avg_vr:.1f}")

    # FS magnitude vs TS magnitude
    avg_abs_ts = sum(abs(f['ts']) for f in frames) / len(frames)
    avg_abs_fs = sum(abs(f['fs']) for f in frames) / len(frames)
    avg_abs_s = sum(abs(f['s']) for f in frames) / len(frames)
    print(f"Avg |TS|={avg_abs_ts/10:.1f} deg, Avg |FS|={avg_abs_fs/10:.1f} deg, Avg |S|={avg_abs_s/10:.1f} deg")

    # Max steer
    max_s = max(f['s'] for f in frames) / 10
    min_s = min(f['s'] for f in frames) / 10
    print(f"Steer Range: [{min_s:.1f} deg, {max_s:.1f} deg]")
