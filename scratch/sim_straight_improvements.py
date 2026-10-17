# Simulate modifying lookahead horizon and multiplier on Run 3 data
from parse_all_runs import parse_run, runs_raw
import numpy as np

recs = parse_run(runs_raw[3])

# Current speed distribution in Run 3:
current_speeds = [(r['vl']+r['vr'])/2.0 for r in recs]
print(f"Current Run 3 average speed: {np.mean(current_speeds):.2f}%")
print(f"Ticks at >= 90%: {sum(1 for v in current_speeds if v >= 90)} ({sum(1 for v in current_speeds if v >= 90)/len(recs)*100:.1f}%)")
print(f"Ticks at >= 80%: {sum(1 for v in current_speeds if v >= 80)} ({sum(1 for v in current_speeds if v >= 80)/len(recs)*100:.1f}%)")

# Let's inspect ticks where car was on a straight (e.g., |H| < 12 and |S| < 14) but speed was < 90%
choked_straights = [i for i, r in enumerate(recs) if abs(r['heading']) < 12.0 and abs(r['steer']) < 14.0 and (r['vl']+r['vr'])/2.0 < 85.0]
print(f"Ticks where car was roughly straight (|H|<12, |S|<14) but speed < 85%: {len(choked_straights)} ({len(choked_straights)/len(recs)*100:.1f}%)")

for i in choked_straights[:15]:
    r = recs[i]
    v = (r['vl']+r['vr'])/2.0
    print(f"  t={i:03d}: Speed={v:.1f}% | H={r['heading']:5.1f} S={r['steer']:5.1f} TS={r['ts']:5.1f} FS={r['fs']:5.1f} DOM={r['dom']}")
