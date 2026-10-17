import re
import math

with open("scratch/telemetry_prebrake_run.txt") as f:
    lines = [line.strip() for line in f if line.strip().startswith("L:")]

print(f"Total valid frames: {len(lines)}")

pattern = re.compile(r"L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TJ:(-?\d+)\s+S:(-?\d+)\s+LA:(\d+)\s+ST:(\w+)\s+VL:(\d+)\s+VR:(\d+)")

records = []
for idx, line in enumerate(lines):
    m = pattern.search(line)
    if m:
        records.append({
            'frame': idx,
            'l_top': int(m.group(1)),
            'l_front': int(m.group(2)),
            'raw_cte': float(m.group(3)) / 10.0,
            'filt_cte': float(m.group(4)) / 10.0,
            'heading': float(m.group(5)) / 10.0,
            'tj_angle': float(m.group(6)) / 10.0,
            'steer': float(m.group(7)) / 10.0,
            'la_urg': int(m.group(8)),
            'state': m.group(9),
            'vl': int(m.group(10)),
            'vr': int(m.group(11))
        })

print("\n--- Summary of Major Events ---")
st_l_count = sum(1 for r in records if r['state'] == 'L')
st_n_count = sum(1 for r in records if r['state'] == 'N')
print(f"State N frames: {st_n_count}, State L frames: {st_l_count}")

# Find segments where car was steering hard left vs hard right
# steer < -10 deg is LEFT, steer > +10 deg is RIGHT
left_turns = []
right_turns = []
for r in records:
    if r['steer'] < -20:
        left_turns.append(r['frame'])
    elif r['steer'] > 20:
        right_turns.append(r['frame'])

print(f"Hard LEFT steering frames: {len(left_turns)}")
print(f"Hard RIGHT steering frames: {len(right_turns)}")

# Let's print out the sequence of maneuvers
print("\n--- Key Phases in Run ---")
cur_phase = None
phase_start = 0

for r in records:
    # Classify phase based on steer & heading
    if r['state'] == 'L':
        phase = 'LOST_LINE'
    elif r['steer'] < -15:
        phase = 'STEER_LEFT'
    elif r['steer'] > 15:
        phase = 'STEER_RIGHT'
    else:
        phase = 'STRAIGHT'
        
    if phase != cur_phase:
        if cur_phase is not None:
            print(f"Frames {phase_start:03d}..{r['frame']-1:03d} ({r['frame']-phase_start:2d} frames, {(r['frame']-phase_start)*20}ms): {cur_phase:12s} | avg steer: {sum(records[k]['steer'] for k in range(phase_start, r['frame']))/(r['frame']-phase_start):+.1f} deg | avg speed: {sum((records[k]['vl']+records[k]['vr'])/2 for k in range(phase_start, r['frame']))/(r['frame']-phase_start):.0f}%")
        cur_phase = phase
        phase_start = r['frame']

if cur_phase is not None:
    print(f"Frames {phase_start:03d}..{len(records)-1:03d} ({len(records)-phase_start:2d} frames, {(len(records)-phase_start)*20}ms): {cur_phase:12s}")
