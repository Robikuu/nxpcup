import re

pattern = re.compile(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TJ:(-?\d+)\s+S:(-?\d+)\s+LA:(\d+)\s+ST:([FT])\s+VL:(\d+)\s+VR:(\d+)')

lines = []
with open('scratch/analyze_lap3.py', 'r') as f:
    for line in f:
        m = pattern.search(line)
        if m:
            top_v, front_v, r, f, h, tj, s, la, st, vl, vr = m.groups()
            lines.append({
                'idx': len(lines),
                'top_v': int(top_v),
                'front_v': int(front_v),
                'r': int(r)/10.0,
                'f': int(f)/10.0,
                'h': int(h)/10.0,
                'tj': int(tj)/10.0,
                's': int(s)/10.0,
                'la': int(la),
                'st': st,
                'vl': int(vl),
                'vr': int(vr)
            })

print(f"Parsed {len(lines)} frames.")

print("\n--- All frames where |H| <= 8.0 deg ---")
for f in lines:
    if abs(f['h']) <= 8.0:
        cond_exit = (abs(f['h']) <= 4.0 and abs(f['f']) <= 6.0 and f['la'] <= 25)
        print(f"Frame {f['idx']:3d}: H={f['h']:+5.1f}, F={f['f']:+5.1f}, TJ={f['tj']:+5.1f}, S={f['s']:+5.1f}, LA={f['la']:3d}, ST={f['st']}, ExitPossible={cond_exit}")

