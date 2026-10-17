import re
import numpy as np

pattern = re.compile(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TS:(-?\d+)\s+FS:(-?\d+)\s+S:(-?\d+)\s+BW:(\d+)\s+DOM:([TFB])\s+FV:(\d+)\s+IX:(\d+)\s+ST:([NIL])\s+VL:(-?\d+)\s+VR:(-?\d+)')

with open(r'd:\nxpcup\scratch\user_telemetry_raw.txt', 'r', encoding='utf-8') as f:
    text = f.read()

# Split on 'OK state=RUNNING'
runs_raw = text.split('OK state=RUNNING')
print(f"Total RUNNING sessions found: {len(runs_raw) - 1}")

def parse_run(run_text):
    recs = []
    for line in run_text.strip().split('\n'):
        line = line.strip()
        if 'OK state=IDLE' in line or '----' in line:
            break
        m = pattern.search(line)
        if m:
            recs.append({
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
    return recs

for idx in range(1, len(runs_raw)):
    recs = parse_run(runs_raw[idx])
    v_avg = [(r['vl'] + r['vr'])/2.0 for r in recs]
    print(f"\n=======================================================")
    print(f"RUN #{idx}: {len(recs)} ticks ({len(recs)*0.02:.2f} seconds)")
    print(f"=======================================================")
    if len(recs) == 0:
        continue
    print(f"Mean speed: {np.mean(v_avg):.1f}%, Max speed: {max(v_avg):.1f}%, Min speed: {min(v_avg):.1f}%")
    print("Start (first 8 ticks):")
    for t, r in enumerate(recs[:8]):
        print(f"  t={t:02d}: CTE={r['f_cte']:5.1f} H={r['heading']:5.1f} S={r['steer']:5.1f} TopV={r['top_v']} FrontV={r['front_v']} VL={r['vl']} VR={r['vr']}")
    
    print("\nApproach to Corner 1 (ticks 10 to 22):")
    for t, r in enumerate(recs[10:25], start=10):
        print(f"  t={t:02d}: CTE={r['f_cte']:5.1f} H={r['heading']:5.1f} S={r['steer']:5.1f} TopV={r['top_v']} FrontV={r['front_v']} DOM={r['dom']} ST={r['st']} VL={r['vl']} VR={r['vr']}")
        
    print("\nOutcome / Final 5 ticks:")
    for t, r in enumerate(recs[-5:], start=len(recs)-5):
        print(f"  t={t:02d}: CTE={r['f_cte']:5.1f} H={r['heading']:5.1f} S={r['steer']:5.1f} TopV={r['top_v']} FrontV={r['front_v']} DOM={r['dom']} ST={r['st']} VL={r['vl']} VR={r['vr']}")
