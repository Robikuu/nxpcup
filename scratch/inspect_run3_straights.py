# Inspect straight segments in Run 3
from parse_all_runs import parse_run, runs_raw
import numpy as np

recs = parse_run(runs_raw[3])

# Find a sustained straight section in Run 3, e.g., where |steer| < 10 for several ticks
straights = []
curr = []
for i, r in enumerate(recs):
    if abs(r['steer']) <= 8.0:
        curr.append((i, r))
    else:
        if len(curr) >= 5:
            straights.append(curr)
        curr = []
if len(curr) >= 5:
    straights.append(curr)

print(f"Number of straight segments (|S| <= 8 deg, len >= 5): {len(straights)}")
for s_idx, s in enumerate(straights[:10]):
    print(f"\n--- Straight #{s_idx+1} ({len(s)} ticks) ---")
    for idx, r in s:
        v = (r['vl']+r['vr'])/2.0
        print(f"t={idx:03d}: CTE={r['f_cte']:5.1f} H={r['heading']:5.1f} S={r['steer']:5.1f} TS={r['ts']:5.1f} FS={r['fs']:5.1f} DOM={r['dom']} Speed={v:.1f}%")
