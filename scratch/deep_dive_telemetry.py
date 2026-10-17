import re
import numpy as np

with open('scratch/user_multi_run_telemetry.txt', 'r', encoding='utf-8') as f:
    text = f.read()

parts = text.split('OK state=RUNNING')

for run_idx in [3, 4, 6]:
    part = parts[run_idx]
    end_m = re.search(r'---- Sent utf8 encoded message:', part)
    run_text = part[:end_m.start()] if end_m else part
    lines = [l.strip() for l in run_text.splitlines() if l.strip().startswith('L:')]
    
    # Parse telemetry fields
    parsed = []
    for l in lines:
        m = re.search(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TS:(-?\d+)\s+FS:(-?\d+)\s+S:(-?\d+)\s+BW:(\d+)\s+DOM:([A-Z])\s+FV:(\d+)\s+IX:(\d+)\s+ST:([A-Z])\s+VL:(\d+)\s+VR:(\d+)', l)
        if m:
            parsed.append({
                'found': int(m.group(1)),
                'cte': int(m.group(3)) / 10.0,
                'f_cte': int(m.group(4)) / 10.0,
                'h': int(m.group(5)) / 10.0,
                'ts': int(m.group(6)) / 10.0,
                'fs': int(m.group(7)) / 10.0,
                's': int(m.group(8)) / 10.0,
                'bw': int(m.group(9)),
                'dom': m.group(10),
                'ix': int(m.group(12)),
                'st': m.group(13),
                'vl': int(m.group(14)),
                'vr': int(m.group(15))
            })
            
    print(f"\n==========================================")
    print(f"DEEP DIVE: Run {run_idx} ({len(parsed)} frames, {len(parsed)*0.08:.1f}s)")
    print(f"==========================================")
    
    # 1. Straightaway analysis (when |S| < 12 and VL >= 90)
    straights = [p for p in parsed if abs(p['s']) < 12.0 and p['vl'] >= 90]
    print(f"Total straightaway sprint frames (VL>=90, |S|<12): {len(straights)}")
    if straights:
        steer_vals = [p['s'] for p in straights]
        h_vals = [p['h'] for p in straights]
        cte_vals = [p['cte'] for p in straights]
        print(f"  Straight steering std dev: {np.std(steer_vals):.2f} deg (mean: {np.mean(steer_vals):.2f})")
        print(f"  Straight heading std dev: {np.std(h_vals):.2f} deg")
        print(f"  Straight CTE std dev: {np.std(cte_vals):.2f} px")
        
    # 2. Cornering analysis (when |S| >= 15)
    corners = [p for p in parsed if abs(p['s']) >= 15.0]
    print(f"Total cornering frames (|S|>=15): {len(corners)}")
    if corners:
        steer_corners = [abs(p['s']) for p in corners]
        print(f"  Cornering peak steer: {max(steer_corners):.1f} deg | mean: {np.mean(steer_corners):.1f} deg")
        
    # 3. Lap detection by finding periodic large turns (e.g. Turn 1 or hairpin)
    # Count transitions from straight (|S|<10) to sharp corner (|S|>30)
    turn_entries = 0
    for i in range(1, len(parsed)):
        if abs(parsed[i-1]['s']) < 12.0 and abs(parsed[i]['s']) >= 25.0:
            turn_entries += 1
    print(f"  Turn entry transitions detected: {turn_entries}")
    print(f"  Estimated complete track loops: ~{turn_entries // 6} to {turn_entries // 5} full laps!")
    
    # 4. Check why run ended: print last 10 frames
    print("  Last 5 frames before run stopped:")
    for p in parsed[-5:]:
        print(f"    L:{p['found']} CTE:{p['cte']} H:{p['h']} S:{p['s']} VL:{p['vl']} VR:{p['vr']}")
