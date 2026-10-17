import analyze_runs

run4 = analyze_runs.run4_data
print("=== RUN 4 (Shadow 1) AT TOP INTERSECTION ===")
# Let's find where Run 4 negotiated Turn 2 and the intersection
# Turn 1 was around frame 20-35
# Chicane was around frame 35-75
# Turn 2 was around frame 75-87
for d in run4[75:125]:
    print(f"F{d['frame']:3d}: L={d['lines_t']}/{d['lines_f']} CTE={d['filt_cte']:+5.1f} H={d['heading']:+5.1f} "
          f"TS={d['top_steer']:+5.1f} FS={d['front_steer']:+5.1f} S={d['steer']:+5.1f} "
          f"BW={d['blend_w']:.2f} DOM={d['dom']} IX={d['ix']} V=({d['vl']:.0f},{d['vr']:.0f})")
