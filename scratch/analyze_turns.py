import re

with open("scratch/telemetry_prebrake_run.txt") as f:
    lines = [line.strip() for line in f if line.strip().startswith("L:")]

pattern = re.compile(r"L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TJ:(-?\d+)\s+S:(-?\d+)\s+LA:(\d+)\s+ST:(\w+)\s+VL:(\d+)\s+VR:(\d+)")

records = []
for idx, line in enumerate(lines):
    m = pattern.search(line)
    if m:
        records.append({
            'frame': idx,
            'time_s': idx * 0.020,
            'l_top': int(m.group(1)),
            'l_front': int(m.group(2)),
            'r_cte': float(m.group(3)) / 10.0,
            'f_cte': float(m.group(4)) / 10.0,
            'heading': float(m.group(5)) / 10.0,
            'tj_angle': float(m.group(6)) / 10.0,
            'steer': float(m.group(7)) / 10.0,
            'la_urg': int(m.group(8)),
            'state': m.group(9),
            'vl': int(m.group(10)),
            'vr': int(m.group(11))
        })

print(f"Total frames: {len(records)} ({len(records)*0.020:.2f} seconds)")

# Let's find every turn event where steer angle exceeds 20 degrees
turns = []
in_turn = False
turn_start = 0
turn_dir = 0

for r in records:
    if abs(r['steer']) > 20:
        if not in_turn:
            in_turn = True
            turn_start = r['frame']
            turn_dir = -1 if r['steer'] < 0 else 1
    else:
        if in_turn:
            in_turn = False
            turns.append((turn_start, r['frame']-1, turn_dir))

if in_turn:
    turns.append((turn_start, len(records)-1, turn_dir))

print("\n--- Detected Maneuvers (Turn Sequence) ---")
for i, (start, end, tdir) in enumerate(turns):
    dur = (end - start + 1) * 20
    dirstr = "LEFT" if tdir == -1 else "RIGHT"
    max_steer = max(abs(records[k]['steer']) for k in range(start, end+1))
    avg_h = sum(records[k]['heading'] for k in range(start, end+1)) / (end - start + 1)
    min_v = min(min(records[k]['vl'], records[k]['vr']) for k in range(start, end+1))
    print(f"Turn #{i+1:02d}: Frames {start:03d}..{end:03d} ({dur:3d}ms) | {dirstr:5s} | Max Steer: {max_steer:5.1f} deg | Avg Heading: {avg_h:+5.1f} deg | Min Speed: {min_v}%")
