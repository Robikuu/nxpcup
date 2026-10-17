import re
import numpy as np

with open("scratch/full_user_telemetry_latest.txt") as f:
    raw_lines = [l.strip() for l in f if l.strip().startswith("L:")]

def parse_line(l):
    m = re.match(r"L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BW:(\d+)\s+DOM:(\w+)\s+FV:(\d+)\s+IX:(\d+)\s+ST:(\w+)\s+VL:(\d+)\s+VR:(\d+)", l)
    if m:
        return {
            'r_cte': int(m.group(3)) / 10.0,
            'f_cte': int(m.group(4)) / 10.0,
            'head': int(m.group(5)) / 10.0,
            'ts': int(m.group(6)) / 10.0,
            'fs': int(m.group(7)) / 10.0,
            's': int(m.group(8)) / 10.0,
            'bw': int(m.group(9)),
            'vl': int(m.group(14)),
            'vr': int(m.group(15)),
        }
    return None

data = [parse_line(l) for l in raw_lines]
data = [d for d in data if d is not None]

# Find contiguous high-speed segments (VL >= 85)
segments = []
cur_seg = []
for idx, d in enumerate(data):
    if d['vl'] >= 85 and abs(d['head']) <= 16.0:
        cur_seg.append((idx, d))
    else:
        if len(cur_seg) >= 4:
            segments.append(cur_seg)
        cur_seg = []
if len(cur_seg) >= 4:
    segments.append(cur_seg)

print(f"Found {len(segments)} high-speed straight/near-straight segments:")
for i, seg in enumerate(segments):
    start_idx = seg[0][0]
    end_idx = seg[-1][0]
    length = len(seg)
    heads = [s[1]['head'] for s in seg]
    steers = [s[1]['s'] for s in seg]
    ctes = [s[1]['f_cte'] for s in seg]
    ts_list = [s[1]['ts'] for s in seg]
    fs_list = [s[1]['fs'] for s in seg]
    print(f"\n--- Segment {i+1} (Ticks {start_idx}..{end_idx}, length={length} ticks = {length*0.08:.2f}s) ---")
    for s_idx, d in seg:
        print(f"  [{s_idx:3d}] CTE:{d['f_cte']:5.1f} | Head:{d['head']:5.1f} | TS:{d['ts']:5.1f} FS:{d['fs']:5.1f} -> Steer:{d['s']:5.1f} | Speed:{d['vl']}")
    
    # Calculate zero-crossings / oscillations in steer
    zero_crossings = np.where(np.diff(np.sign(steers)))[0]
    print(f"  Steer range: [{min(steers):.1f}, {max(steers):.1f}] deg (Pk-Pk: {max(steers)-min(steers):.1f} deg)")
    print(f"  Heading range: [{min(heads):.1f}, {max(heads):.1f}] deg (Pk-Pk: {max(heads)-min(heads):.1f} deg)")
    print(f"  CTE range: [{min(ctes):.1f}, {max(ctes):.1f}] px (Pk-Pk: {max(ctes)-min(ctes):.1f} px)")
    print(f"  Steer zero crossings: {len(zero_crossings)}")
