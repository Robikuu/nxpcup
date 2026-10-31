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

print("=== WEAVING OSCILLATION TIMING ANALYSIS ===")
# Telemetry frame rate: 50 Hz loop / 4 ticks = 12.5 Hz = 80 ms per frame.
# Let's find zero crossings of CTE and Heading:
cte_crossings = []
for i in range(1, len(frames)):
    if (frames[i-1]['f_cte'] < 0 and frames[i]['f_cte'] >= 0) or (frames[i-1]['f_cte'] > 0 and frames[i]['f_cte'] <= 0):
        t_ms = i * 80
        cte_crossings.append((i, t_ms, frames[i]['f_cte']))

print("CTE Zero Crossings (Frame, Time_ms, CTE):")
for c in cte_crossings:
    print(f"  Frame {c[0]:2d} ({c[1]:4d} ms): CTE={c[2]:5.1f}")

head_crossings = []
for i in range(1, len(frames)):
    if (frames[i-1]['f_head'] < 0 and frames[i]['f_head'] >= 0) or (frames[i-1]['f_head'] > 0 and frames[i]['f_head'] <= 0):
        t_ms = i * 80
        head_crossings.append((i, t_ms, frames[i]['f_head']))

print("\nHeading Zero Crossings (Frame, Time_ms, Heading):")
for h in head_crossings:
    print(f"  Frame {h[0]:2d} ({h[1]:4d} ms): Heading={h[2]:5.1f}")

# Look at peak to peak
print("\nPeaks in Heading:")
for i in range(1, len(frames)-1):
    h_prev = frames[i-1]['f_head']
    h_curr = frames[i]['f_head']
    h_next = frames[i+1]['f_head']
    if (h_curr > h_prev and h_curr > h_next and h_curr > 10.0) or (h_curr < h_prev and h_curr < h_next and h_curr < -10.0):
        print(f"  Frame {i:2d} ({i*80:4d} ms): Heading Peak = {h_curr:5.1f} deg | CTE = {frames[i]['f_cte']:5.1f} px | S = {frames[i]['s']:5.1f}")
