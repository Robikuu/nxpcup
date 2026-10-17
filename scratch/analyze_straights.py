# Analyze straightaway segments and why they don't reach 80%
import re
import numpy as np

# Load records from previous script
from telemetry_speed_analysis import records

# Find contiguous straight segments (where |steer| <= 15 and |heading| <= 15)
segments = []
curr_seg = []
for i, r in enumerate(records):
    is_straight = (abs(r['steer']) <= 15.0 and abs(r['heading']) <= 15.0)
    if is_straight:
        curr_seg.append((i, r))
    else:
        if len(curr_seg) > 0:
            segments.append(curr_seg)
            curr_seg = []
if len(curr_seg) > 0:
    segments.append(curr_seg)

print(f"Total straight segments: {len(segments)}")
for idx, seg in enumerate(segments):
    duration_s = len(seg) * 0.02
    speeds = [(r['vl'] + r['vr'])/2.0 for i, r in seg]
    max_spd = max(speeds)
    min_spd = min(speeds)
    mean_spd = np.mean(speeds)
    print(f"Segment {idx+1}: {len(seg)} ticks ({duration_s:.2f}s) | Min: {min_spd}%, Max: {max_spd}%, Mean: {mean_spd:.1f}%")
    if len(seg) >= 10:
        # print first few and last few speeds
        print(f"   Entry speeds: {[speeds[k] for k in range(min(5, len(speeds)))]}")
        print(f"   Exit speeds:  {[speeds[-k-1] for k in range(min(5, len(speeds)))][::-1]}")
