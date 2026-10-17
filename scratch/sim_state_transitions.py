import re
import numpy as np

pattern = re.compile(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TJ:(-?\d+)\s+S:(-?\d+)\s+LA:(\d+)\s+ST:([FT])\s+VL:(\d+)\s+VR:(\d+)')

with open('docs/debug_telemetry.md', 'r') as f:
    text = f.read()

frames = []
for line in text.split('\n'):
    m = pattern.search(line)
    if m:
        top_v, front_v, r, f_val, h, tj, s, la, st, vl, vr = m.groups()
        frames.append({
            'top_v': int(top_v),
            'front_v': int(front_v),
            'r': int(r) / 10.0,
            'f': int(f_val) / 10.0,
            'h': int(h) / 10.0,
            'tj': int(tj) / 10.0,
            's': int(s) / 10.0,
            'la': int(la),
            'st': st,
            'vl': int(vl),
            'vr': int(vr)
        })

print(f"Loaded {len(frames)} frames from debug_telemetry.md")

# Let's simulate the state machine on all frames with different logic:
# Baseline (current code in main.c):
# - straight_confirmed = (|H| <= 4.0 && |F| <= 6.0 && LA <= 15)
# - s_corner_abort_ticks >= 35

def run_simulation(enter_h=8.0, exit_h=5.0, exit_cte=8.0, enter_tj=12.0, exit_tj=7.0, exit_streak=3, abort_ticks=15):
    state = 'F'
    state_streak = 0
    apex_reached = False
    abort_counter = 0
    
    st_history = []
    
    for f in frames:
        h = abs(f['h'])
        cte = abs(f['f'])
        tj = abs(f['tj'])
        
        # Apex entry
        if state == 'T':
            if h >= enter_h or abs(f['s']) >= 15.0 or cte >= 14.0:
                apex_reached = True
                abort_counter = 0
                
        # Corner trigger
        corner_trigger = (h >= enter_h) or (tj >= enter_tj)
        
        # Straight confirmation
        # Using heading, CTE, and trajectory angle (which measures true curvature)
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
            threshold = exit_streak if desired == 'F' else 2
            if state_streak >= threshold:
                state = desired
                state_streak = 0
                if state == 'T':
                    apex_reached = False
                    abort_counter = 0
        else:
            state_streak = 0
            
        st_history.append(state)
        
    st_f_cnt = sum(1 for s in st_history if s == 'F')
    st_t_cnt = sum(1 for s in st_history if s == 'T')
    transitions = sum(1 for i in range(1, len(st_history)) if st_history[i] != st_history[i-1])
    return st_f_cnt, st_t_cnt, transitions

# Compare current vs proposed
f_curr, t_curr, tr_curr = run_simulation(enter_h=8.0, exit_h=4.0, exit_cte=6.0, enter_tj=8.0, exit_tj=4.0, exit_streak=4, abort_ticks=35)
print(f"Current Settings : ST:F = {f_curr} ({f_curr/len(frames)*100:.1f}%), ST:T = {t_curr} ({t_curr/len(frames)*100:.1f}%), Transitions = {tr_curr}")

f_opt, t_opt, tr_opt = run_simulation(enter_h=8.0, exit_h=5.0, exit_cte=8.0, enter_tj=12.0, exit_tj=7.0, exit_streak=3, abort_ticks=15)
print(f"Proposed Settings: ST:F = {f_opt} ({f_opt/len(frames)*100:.1f}%), ST:T = {t_opt} ({t_opt/len(frames)*100:.1f}%), Transitions = {tr_opt}")

f_opt2, t_opt2, tr_opt2 = run_simulation(enter_h=8.0, exit_h=5.0, exit_cte=9.0, enter_tj=14.0, exit_tj=8.0, exit_streak=3, abort_ticks=12)
print(f"Tuned Settings 2 : ST:F = {f_opt2} ({f_opt2/len(frames)*100:.1f}%), ST:T = {t_opt2} ({t_opt2/len(frames)*100:.1f}%), Transitions = {tr_opt2}")
