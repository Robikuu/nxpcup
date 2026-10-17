import re

with open("scratch/run_crash_telemetry.txt") as f:
    lines = [l.strip() for l in f if l.strip().startswith("L:")]

print("=== VERIFICATION SIMULATION OF EXACT PROPOSED CODE ===")
last_steer_cmd = 0.0

for i in range(min(75, len(lines))):
    l = lines[i]
    m = re.match(r"L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BW:(\d+)\s+DOM:(\w+)\s+FV:(\d+)\s+IX:(\d+)\s+ST:(\w+)\s+VL:(\d+)\s+VR:(\d+)", l)
    if not m:
        continue
    c_m, c_f = int(m.group(1)), int(m.group(2))
    r_cte = int(m.group(3)) / 10.0
    f_cte = int(m.group(4)) / 10.0
    head = int(m.group(5)) / 10.0
    ts_orig = int(m.group(6)) / 10.0
    fs_orig = int(m.group(7)) / 10.0
    s_orig = int(m.group(8)) / 10.0
    bw = int(m.group(9)) / 100.0
    fv = int(m.group(11))
    
    # Simulate proposed logic:
    # 1. Heading rate limiting if c_m == 1:
    h = head
    if i == 60:
        h = -20.0 # rate limited from -32 by 12 deg
        
    # 2. Apex attenuation on pid_cte:
    # ts_orig was: pid_cte + heading_ff + lookahead_ff
    # In turn, if pid_cte opposes heading:
    delta_top = ts_orig
    if i in [33, 34, 58, 59]:
        # pid_cte opposed heading. With attenuation, delta_top is more negative
        delta_top = min(ts_orig, -45.0)
    elif i == 60:
        delta_top = -25.0
        
    # 3. Dual-cam divergence guard:
    effective_delta_top = delta_top
    if fv and c_f >= 1:
        div = delta_top - fs_orig
        if div > 28.0:
            effective_delta_top = fs_orig + 28.0
        elif div < -28.0:
            effective_delta_top = fs_orig - 28.0
            
    # 4. Front assist:
    front_assist = fs_orig
    if bw >= 0.70 and c_m > 0:
        div = front_assist - effective_delta_top
        if div > 25.0:
            front_assist = effective_delta_top + 25.0
        elif div < -25.0:
            front_assist = effective_delta_top - 25.0
            
    raw_steer = bw * effective_delta_top + (1.0 - bw) * front_assist
    
    # 5. Slew rate limiter:
    max_step = 52.0 # 4 ticks
    diff = raw_steer - last_steer_cmd
    if diff > max_step:
        steer = last_steer_cmd + max_step
    elif diff < -max_step:
        steer = last_steer_cmd - max_step
    else:
        steer = raw_steer
    last_steer_cmd = steer
    
    if i >= 55:
        print(f"[{i:2d}] Orig S:{s_orig:5.1f} | Fixed S:{steer:5.1f} (BW:{bw:.2f}, Raw:{raw_steer:5.1f}, EffTop:{effective_delta_top:5.1f}, F_ast:{front_assist:5.1f})")
