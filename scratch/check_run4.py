import analyze_runs

run4 = analyze_runs.run4_data
print(f"Run 4: {len(run4)} frames")
# Let's inspect Run 4 at intervals
for i in range(0, len(run4), 10):
    d = run4[i]
    print(f"F{d['frame']:3d}: L={d['lines_t']}/{d['lines_f']} CTE={d['filt_cte']:+5.1f} H={d['heading']:+5.1f} "
          f"TS={d['top_steer']:+5.1f} FS={d['front_steer']:+5.1f} S={d['steer']:+5.1f} "
          f"BW={d['blend_w']:.2f} DOM={d['dom']} IX={d['ix']} V=({d['vl']:.0f},{d['vr']:.0f})")

print("\nLast 15 frames of Run 4:")
for d in run4[-15:]:
    print(f"F{d['frame']:3d}: L={d['lines_t']}/{d['lines_f']} CTE={d['filt_cte']:+5.1f} H={d['heading']:+5.1f} "
          f"TS={d['top_steer']:+5.1f} FS={d['front_steer']:+5.1f} S={d['steer']:+5.1f} "
          f"BW={d['blend_w']:.2f} DOM={d['dom']} IX={d['ix']} V=({d['vl']:.0f},{d['vr']:.0f})")
