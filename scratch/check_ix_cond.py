import analyze_runs

run6 = analyze_runs.run6_data
print("=== FRAMES 85 TO 102 DETAILED METRICS ===")
for d in run6[85:103]:
    h_ok = abs(d['heading']) < 6.0
    s_ok = abs(d['steer']) < 8.0
    cte_ok = abs(d['filt_cte']) < 12.0
    print(f"F{d['frame']:3d}: H={d['heading']:+5.1f} ({h_ok}) | "
          f"S={d['steer']:+5.1f} ({s_ok}) | "
          f"CTE={d['filt_cte']:+5.1f} ({cte_ok}) | "
          f"TS={d['top_steer']:+5.1f} FS={d['front_steer']:+5.1f} IX={d['ix']}")
