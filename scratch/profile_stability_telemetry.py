import re
import numpy as np

def parse_telemetry(filename):
    records = []
    with open(filename, 'r') as f:
        for idx, line in enumerate(f):
            line = line.strip()
            if not line.startswith("L:"):
                continue
            m = re.match(r"L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BW:(\d+)\s+DOM:(\w+)\s+FV:(\d+)\s+IX:(\d+)\s+ST:(\w+)\s+VL:(\d+)\s+VR:(\d+)", line)
            if m:
                records.append({
                    'idx': idx,
                    'line_raw': line,
                    'cam_main': int(m.group(1)),
                    'cam_front': int(m.group(2)),
                    'cte_raw': int(m.group(3)) / 10.0,
                    'cte_filt': int(m.group(4)) / 10.0,
                    'heading': int(m.group(5)) / 10.0,
                    'target_steer': int(m.group(6)) / 10.0,
                    'front_steer': int(m.group(7)) / 10.0,
                    'steer': int(m.group(8)) / 10.0,
                    'blend_w': int(m.group(9)),
                    'dom': m.group(10),
                    'fv': int(m.group(11)),
                    'ix': int(m.group(12)),
                    'st': m.group(13),
                    'vl': int(m.group(14)),
                    'vr': int(m.group(15)),
                })
    return records

records = parse_telemetry("scratch/full_user_telemetry_latest.txt")
print(f"Total parsed records: {len(records)}")

# Identify Straights vs Corners vs Corner Exits
# Straight definition: |heading| < 12 deg and speed > 85
straights = []
corners = []
corner_exits = []

for i in range(1, len(records)):
    r = records[i]
    prev_r = records[i-1]
    
    is_straight = (abs(r['heading']) <= 12.0 and abs(r['steer']) <= 15.0 and r['vl'] >= 85)
    is_corner = (abs(r['heading']) > 20.0 or abs(r['steer']) > 25.0)
    is_exit = (abs(prev_r['heading']) > 20.0 and abs(r['heading']) <= 18.0)
    
    if is_straight:
        straights.append(r)
    elif is_corner:
        corners.append(r)
    if is_exit:
        corner_exits.append((i, r))

print(f"Straight samples: {len(straights)}, Corner samples: {len(corners)}, Corner exits detected: {len(corner_exits)}")

# Weaving Analysis on Straights
if straights:
    steer_vals = [s['steer'] for s in straights]
    head_vals = [s['heading'] for s in straights]
    cte_vals = [s['cte_filt'] for s in straights]
    
    print("\n--- STRAIGHTS WEAVING METRICS ---")
    print(f"Mean Steer: {np.mean(steer_vals):.2f} deg, Std Steer: {np.std(steer_vals):.2f} deg, Max Abs Steer: {np.max(np.abs(steer_vals)):.2f} deg")
    print(f"Mean Heading: {np.mean(head_vals):.2f} deg, Std Heading: {np.std(head_vals):.2f} deg, Max Abs Heading: {np.max(np.abs(head_vals)):.2f} deg")
    print(f"Mean CTE: {np.mean(cte_vals):.2f} px, Std CTE: {np.std(cte_vals):.2f} px, Max Abs CTE: {np.max(np.abs(cte_vals)):.2f} px")

# Corner Exit Stability Analysis
print(f"\n--- CORNER EXIT METRICS (First 5 exits) ---")
for exit_idx, r in corner_exits[:8]:
    # inspect 5 ticks following exit
    window = records[exit_idx:min(len(records), exit_idx + 6)]
    steers = [w['steer'] for w in window]
    ctes = [w['cte_filt'] for w in window]
    heads = [w['heading'] for w in window]
    ts_fs_diff = [abs(w['target_steer'] - w['front_steer']) for w in window]
    bws = [w['blend_w'] for w in window]
    print(f"Exit at Tick {exit_idx}:")
    print(f"  Headings: {heads}")
    print(f"  Steers:   {steers}")
    print(f"  CTEs:     {ctes}")
    print(f"  |TS-FS|:  {ts_fs_diff}")
    print(f"  BlendW:   {bws}")
