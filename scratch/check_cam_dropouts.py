import re

with open("scratch/telemetry_prebrake_run.txt") as f:
    lines = [line.strip() for line in f if line.strip().startswith("L:")]

pattern = re.compile(r"L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TJ:(-?\d+)\s+S:(-?\d+)\s+LA:(\d+)\s+ST:(\w+)\s+VL:(\d+)\s+VR:(\d+)")

records = []
for idx, line in enumerate(lines):
    m = pattern.search(line)
    if m:
        records.append({
            'frame': idx,
            'top_v': int(m.group(1)),
            'front_v': int(m.group(2)),
            'r_cte': float(m.group(3)) / 10.0,
            'f_cte': float(m.group(4)) / 10.0,
            'heading': float(m.group(5)) / 10.0,
            'tj': float(m.group(6)) / 10.0,
            'steer': float(m.group(7)) / 10.0,
            'la': int(m.group(8)),
            'state': m.group(9),
            'vl': int(m.group(10)),
            'vr': int(m.group(11))
        })

# Let's inspect where Top cam had 0 or 1 vector vs 2 vectors
print("--- Camera tracking state transitions ---")
for r in records:
    if r['top_v'] <= 1 or r['state'] == 'L':
        print(f"Frame {r['frame']:03d} ({r['frame']*0.020:.2f}s): Top:{r['top_v']} Frt:{r['front_v']} | CTE:{r['f_cte']:+5.1f} H:{r['heading']:+5.1f}deg | Steer:{r['steer']:+5.1f}deg | State:{r['state']}")
