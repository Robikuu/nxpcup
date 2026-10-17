# Let's inspect the starting position and corner approach for each run
import re

pattern = re.compile(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TS:(-?\d+)\s+FS:(-?\d+)\s+S:(-?\d+)\s+BW:(\d+)\s+DOM:([TFB])\s+FV:(\d+)\s+IX:(\d+)\s+ST:([NIL])\s+VL:(-?\d+)\s+VR:(-?\d+)')

def parse_lines(text):
    records = []
    for line in text.strip().split('\n'):
        m = pattern.search(line)
        if m:
            records.append({
                'top_v': int(m.group(1)),
                'front_v': int(m.group(2)),
                'r_cte': int(m.group(3)) / 10.0,
                'f_cte': int(m.group(4)) / 10.0,
                'heading': int(m.group(5)) / 10.0,
                'ts': int(m.group(6)) / 10.0,
                'fs': int(m.group(7)) / 10.0,
                'steer': int(m.group(8)) / 10.0,
                'bw': int(m.group(9)),
                'dom': m.group(10),
                'fv': int(m.group(11)),
                'ix': int(m.group(12)),
                'st': m.group(13),
                'vl': int(m.group(14)),
                'vr': int(m.group(15))
            })
    return records

import raw_runs
runs = raw_runs.raw_text.split('start;')
print(f"Number of 'start;' segments: {len(runs)}")
for idx in range(1, len(runs)):
    recs = parse_lines(runs[idx])
    print(f"\n=================== RUN {idx} ({len(recs)} ticks) ===================")
    for i, r in enumerate(recs[:15]):
        print(f"t={i:02d}: CTE={r['f_cte']:5.1f} H={r['heading']:5.1f} S={r['steer']:5.1f} TopV={r['top_v']} FrontV={r['front_v']} VL={r['vl']} VR={r['vr']}")
    if len(recs) > 15:
        print("  ... transition to corner:")
        for i, r in enumerate(recs[15:26], start=15):
            print(f"t={i:02d}: CTE={r['f_cte']:5.1f} H={r['heading']:5.1f} S={r['steer']:5.1f} TopV={r['top_v']} FrontV={r['front_v']} DOM={r['dom']} ST={r['st']} VL={r['vl']} VR={r['vr']}")
