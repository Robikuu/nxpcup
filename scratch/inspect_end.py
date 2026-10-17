import analyze_runs

run6 = analyze_runs.run6_data

print("=== DETAILED INSPECTION: FRAMES 85 TO 123 ===")
for d in run6[85:]:
    print(f"F{d['frame']:3d}: L={d['lines_t']}/{d['lines_f']} CTE={d['filt_cte']:+5.1f} H={d['heading']:+5.1f} "
          f"TS={d['top_steer']:+5.1f} FS={d['front_steer']:+5.1f} S={d['steer']:+5.1f} "
          f"BW={d['blend_w']:.2f} DOM={d['dom']} IX={d['ix']} V=({d['vl']:.0f},{d['vr']:.0f})")
