import re
import numpy as np

def parse_telemetry(filename):
    records = []
    with open(filename, 'r') as f:
        for idx, line in enumerate(f):
            line = line.strip()
            if not line.startswith("L:"):
                continue
            # Pattern: L:2/4 R:100 F:86 H:114 TS:245 FS:79 S:245 BW:100 DOM:T FV:1 IX:0 ST:N VL:93 VR:74
            m = re.match(r"L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BW:(\d+)\s+DOM:(\w+)\s+FV:(\d+)\s+IX:(\d+)\s+ST:(\w+)\s+VL:(\d+)\s+VR:(\d+)", line)
            if m:
                records.append({
                    'idx': idx,
                    'line_raw': line,
                    'cam_main': int(m.group(1)),
                    'cam_front': int(m.group(2)),
                    'cte_raw': int(m.group(3)) / 10.0,
                    'cte_filt': int(m.group(4)) / 10.0,
                    'heading': int(m.group(5)) / 10.0,
                    'target_steer': int(m.group(6)) / 10.0,
                    'front_steer': int(m.group(7)) / 10.0,
                    'steer': int(m.group(8)) / 10.0,
                    'blend_w': int(m.group(9)),
                    'dom': m.group(10),
                    'fv': int(m.group(11)),
                    'ix': int(m.group(12)),
                    'st': m.group(13),
                    'vl': int(m.group(14)),
                    'vr': int(m.group(15)),
                })
    return records

records = parse_telemetry("scratch/run_crash_telemetry.txt")
print(f"Parsed {len(records)} records.")

print(f"{'Idx':<4} | {'Cam':<5} | {'R_cte':<6} | {'F_cte':<6} | {'Head':<6} | {'TS':<6} | {'FS':<6} | {'Steer':<6} | {'BW':<4} | {'DOM':<3} | {'ST':<2} | {'VL':<3} {'VR':<3}")
print("-" * 75)
for r in records:
    print(f"{r['idx']:<4} | {r['cam_main']}/{r['cam_front']:<3} | {r['cte_raw']:<6.1f} | {r['cte_filt']:<6.1f} | {r['heading']:<6.1f} | {r['target_steer']:<6.1f} | {r['front_steer']:<6.1f} | {r['steer']:<6.1f} | {r['blend_w']:<4} | {r['dom']:<3} | {r['st']:<2} | {r['vl']:<3} {r['vr']:<3}")
