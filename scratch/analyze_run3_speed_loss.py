import re
import numpy as np
from parse_all_runs import parse_run, runs_raw

recs = parse_run(runs_raw[3])
print(f"Run 3 record count: {len(recs)}")

# In main.c:
# GOVERNOR_HEADING_DEADBAND_DEG = 5.0
# GOVERNOR_HEADING_MAX_DEG = 30.0
# k_base = (abs_h - 5.0) / (30.0 - 5.0) if abs_h > 5.0 else 0.0
# k_front = lookahead_turn_factor * 1.25 (derived from FS / front vector curvature)
# Notice telemetry has:
# H = filtered_heading * 10
# S = steer_cmd * 10
# FS = front steering * 10
# TS = top steering * 10
# VL, VR

k_base_arr = []
speed_loss_due_to_h = 0
speed_loss_due_to_front = 0
speed_loss_due_to_blend = 0
ticks_at_max_speed = 0
ticks_sub_max = 0

for i, r in enumerate(recs):
    abs_h = abs(r['heading'])
    if abs_h > 5.0:
        kb = (abs_h - 5.0) / (30.0 - 5.0)
        if kb > 1.0: kb = 1.0
    else:
        kb = 0.0
    k_base_arr.append(kb)
    
    v = (r['vl'] + r['vr'])/2.0
    if v >= 94.0:
        ticks_at_max_speed += 1
    else:
        ticks_sub_max += 1

print(f"Ticks at max speed (>=94%): {ticks_at_max_speed} ({ticks_at_max_speed/len(recs)*100:.1f}%)")
print(f"Ticks below max speed (<94%): {ticks_sub_max} ({ticks_sub_max/len(recs)*100:.1f}%)")

# Let's inspect what happens during straightaway portions
# A straightaway is where the car SHOULD be at 95%, but might not be
# Let's see: on ticks where r['st'] == 'N', what is keeping speed down?
h_triggers = [i for i, r in enumerate(recs) if abs(r['heading']) > 5.0 and (r['vl']+r['vr'])/2.0 < 94.0]
print(f"\nTicks where Heading > 5.0 deg dragged speed down: {len(h_triggers)} ({len(h_triggers)/len(recs)*100:.1f}%)")

# What is the heading oscillation on straights?
straight_ticks = [r for r in recs if abs(r['steer']) < 15.0]
straight_headings = [abs(r['heading']) for r in straight_ticks]
straight_ctes = [abs(r['f_cte']) for r in straight_ticks]
print(f"Straightaway heading (when |steer| < 15 deg): mean = {np.mean(straight_headings):.2f} deg, max = {max(straight_headings):.2f} deg")
print(f"Straightaway CTE (when |steer| < 15 deg): mean = {np.mean(straight_ctes):.2f} px, max = {max(straight_ctes):.2f} px")

# Check distribution of speeds
speeds = [(r['vl']+r['vr'])/2.0 for r in recs]
print("\nSpeed histogram across Run 3:")
for low, high in [(90, 100), (80, 90), (70, 80), (65, 70), (60, 65), (50, 60)]:
    cnt = sum(1 for v in speeds if low <= v < high)
    print(f"  {low}% - {high}%: {cnt} ticks ({cnt/len(recs)*100:.1f}%)")
