# Performance stats of Run 3
import numpy as np
from parse_all_runs import parse_run, runs_raw

recs = parse_run(runs_raw[3])
vl = [r['vl'] for r in recs]
vr = [r['vr'] for r in recs]
v_avg = [(r['vl'] + r['vr'])/2.0 for r in recs]

print(f"Run 3 Total duration: {len(recs)*0.02:.2f}s ({len(recs)} ticks)")
print(f"Run 3 Average speed: {np.mean(v_avg):.2f}% (Previously was 60.18%!)")
print(f"Run 3 Median speed:  {np.median(v_avg):.2f}% (Previously was 56.00%!)")
print(f"Run 3 Max speed:     {max(v_avg):.2f}%")
print(f"Run 3 Min speed:     {min(v_avg):.2f}% (Previously was 46.00%!)")

print(f"Time spent >= 90% speed: {sum(1 for v in v_avg if v >= 90)*0.02:.2f}s ({sum(1 for v in v_avg if v >= 90)/len(v_avg)*100:.1f}%)")
print(f"Time spent >= 80% speed: {sum(1 for v in v_avg if v >= 80)*0.02:.2f}s ({sum(1 for v in v_avg if v >= 80)/len(v_avg)*100:.1f}%)")
print(f"Time spent < 60% speed:  {sum(1 for v in v_avg if v < 60)*0.02:.2f}s ({sum(1 for v in v_avg if v < 60)/len(v_avg)*100:.1f}% vs 40.8% previously!)")
