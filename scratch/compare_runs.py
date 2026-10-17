import re
import numpy as np

with open('scratch/user_multi_run_telemetry.txt', 'r', encoding='utf-8') as f:
    text = f.read()

parts = text.split('OK state=RUNNING')

configs = {
    1: "Run 1 (k_head=1.70, shadow=1, spd=95)",
    3: "Run 3 (k_head=2.00, shadow=0, spd=95)",
    4: "Run 4 (k_head=2.00, shadow=1, spd=95)",
    5: "Run 5 (k_head=2.00, shadow=1, spd=95)",
    6: "Run 6 (k_head=2.00, shadow=1, spd=100)",
    7: "Run 7 (k_head=1.80, shadow=1, spd=100)",
}

for run_idx, name in configs.items():
    if run_idx >= len(parts):
        continue
    part = parts[run_idx]
    end_m = re.search(r'---- Sent utf8 encoded message:', part)
    run_text = part[:end_m.start()] if end_m else part
    lines = [l.strip() for l in run_text.splitlines() if l.strip().startswith('L:')]
    
    parsed = []
    for l in lines:
        m = re.search(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TS:(-?\d+)\s+FS:(-?\d+)\s+S:(-?\d+)\s+BW:(\d+)\s+DOM:([A-Z])\s+FV:(\d+)\s+IX:(\d+)\s+ST:([A-Z])\s+VL:(\d+)\s+VR:(\d+)', l)
        if m:
            parsed.append({
                'found': int(m.group(1)),
                'cte': int(m.group(3)) / 10.0,
                'h': int(m.group(5)) / 10.0,
                's': int(m.group(8)) / 10.0,
                'vl': int(m.group(14)),
                'vr': int(m.group(15))
            })
    if not parsed:
        continue
        
    straights = [p for p in parsed if abs(p['s']) < 12.0 and p['vl'] >= 90]
    corners = [p for p in parsed if abs(p['s']) >= 15.0 and p['found'] > 0]
    
    steer_std = np.std([p['s'] for p in straights]) if straights else 0.0
    corner_cte = np.mean([abs(p['cte']) for p in corners]) if corners else 0.0
    corner_peak_cte = max([abs(p['cte']) for p in corners]) if corners else 0.0
    max_spd = max(max(p['vl'] for p in parsed), max(p['vr'] for p in parsed))
    
    print(f"{name}:")
    print(f"  Duration: {len(parsed)*0.08:.1f}s ({len(parsed)} frames)")
    print(f"  Max Speed: {max_spd}%")
    print(f"  Straight Steer Std: {steer_std:.2f} deg  (weave damping)")
    print(f"  Corner Mean CTE: {corner_cte:.2f} px | Peak CTE: {corner_peak_cte:.2f} px")
    print(f"  Laps: ~{len(parsed)*0.08 / 13.0:.1f} laps")
