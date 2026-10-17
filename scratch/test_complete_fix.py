import re
import numpy as np

pattern = re.compile(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TJ:(-?\d+)\s+S:(-?\d+)\s+LA:(\d+)\s+ST:([FT])\s+VL:(\d+)\s+VR:(\d+)')

with open('docs/debug_telemetry.md', 'r') as f:
    text = f.read()

runs = []
cur = []
for line in text.splitlines():
    if 'start;' in line or 'state=RUNNING' in line:
        if cur:
            runs.append(cur)
            cur = []
    m = pattern.search(line)
    if m:
        top_v, front_v, r, f_val, h, tj, s, la, st, vl, vr = m.groups()
        cur.append({
            'top_v': int(top_v), 'front_v': int(front_v),
            'r': int(r) / 10.0, 'f': int(f_val) / 10.0,
            'h': int(h) / 10.0, 'tj': int(tj) / 10.0,
            's': int(s) / 10.0, 'la': int(la),
            'st': st, 'vl': int(vl), 'vr': int(vr)
        })
if cur:
    runs.append(cur)

print(f"Total runs parsed: {len(runs)}")

def simulate_run(run, 
                 enter_h=8.0, 
                 exit_h=4.0, 
                 exit_cte=8.0, 
                 enter_tj=12.0, 
                 exit_tj=7.0, 
                 enter_streak=2, 
                 exit_streak=3, 
                 abort_ticks=15):
    state = 'F'
    state_streak = 0
    apex_reached = False
    abort_counter = 0
    st_history = []
    
    for f in run:
        h = abs(f['h'])
        cte = abs(f['f'])
        tj = abs(f['tj'])
        s = abs(f['s'])
        
        # Apex detection
        if state == 'T':
            if h >= enter_h or s >= 15.0 or cte >= 14.0:
                apex_reached = True
                abort_counter = 0
                
        # Corner trigger
        corner_trigger = (h >= enter_h) or (tj >= enter_tj)
        
        # Straight confirmed
        straight_confirmed = (h <= exit_h) and (cte <= exit_cte) and (tj <= exit_tj)
        
        if state == 'T':
            if apex_reached:
                desired = 'F' if straight_confirmed else 'T'
            else:
                if straight_confirmed:
                    abort_counter += 1
                    desired = 'F' if abort_counter >= abort_ticks else 'T'
                else:
                    abort_counter = 0
                    desired = 'T'
        else:
            desired = 'T' if corner_trigger else 'F'
            
        if desired != state:
            state_streak += 1
            threshold = exit_streak if desired == 'F' else enter_streak
            if state_streak >= threshold:
                state = desired
                state_streak = 0
                if state == 'T':
                    apex_reached = False
                    abort_counter = 0
        else:
            state_streak = 0
            
        st_history.append(state)
        
    return st_history

print("\n--- Testing on Run 6 (Clean Straight, 40 frames) ---")
r6 = runs[5]
st_curr_6 = [x['st'] for x in r6]
st_sim_6  = simulate_run(r6, exit_cte=8.0, abort_ticks=10)
print(f"Run 6 Original: ST:F = {st_curr_6.count('F')}, ST:T = {st_curr_6.count('T')}")
print(f"Run 6 Simulated: ST:F = {st_sim_6.count('F')}, ST:T = {st_sim_6.count('T')}")

print("\n--- Testing on Run 8 (Straight Cruise, 39 frames) ---")
r8 = runs[7]
st_curr_8 = [x['st'] for x in r8]
st_sim_8  = simulate_run(r8, exit_cte=8.0, abort_ticks=10)
print(f"Run 8 Original: ST:F = {st_curr_8.count('F')}, ST:T = {st_curr_8.count('T')}")
print(f"Run 8 Simulated: ST:F = {st_sim_8.count('F')}, ST:T = {st_sim_8.count('T')}")

print("\n--- Testing on Full Circuit Lap (Run 2, 1072 frames) ---")
r2 = runs[1]
st_curr_2 = [x['st'] for x in r2]
st_sim_2  = simulate_run(r2, exit_cte=8.0, abort_ticks=10)
print(f"Run 2 Original: ST:F = {st_curr_2.count('F')} ({st_curr_2.count('F')/len(r2)*100:.1f}%), ST:T = {st_curr_2.count('T')}")
print(f"Run 2 Simulated: ST:F = {st_sim_2.count('F')} ({st_sim_2.count('F')/len(r2)*100:.1f}%), ST:T = {st_sim_2.count('T')}")
