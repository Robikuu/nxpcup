import analyze_latest_test

data = analyze_latest_test.data
print("=== FRAMES 70 TO 115 (TOP STRAIGHT & INTERSECTION) ===")
for d in data[70:115]:
    f = d['frame']
    print(f"F{f:3d}: L={d['lines_t']}/{d['lines_f']} CTE={d['filt_cte']:+5.1f} H={d['heading']:+5.1f} "
          f"TS={d['top_steer']:+5.1f} FS={d['front_steer']:+5.1f} S={d['steer']:+5.1f} "
          f"BW={d['blend_w']:.2f} DOM={d['dom']} IX={d['ix']} ST={d['st']} V=({d['vl']:.0f},{d['vr']:.0f})")
