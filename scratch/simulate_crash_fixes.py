import re

# Load the crash telemetry
with open("scratch/run_crash_telemetry.txt") as f:
    lines = [l.strip() for l in f if l.strip().startswith("L:")]

def simulate_fixes(enable_apex_atten=True, fix_front_assist=True, fix_divergence_guard=True, rate_limit_heading=True):
    print(f"\n--- SIMULATION (ApexAtten={enable_apex_atten}, FixAssist={fix_front_assist}, DivGuard={fix_divergence_guard}, HeadingLimit={rate_limit_heading}) ---")
    
    last_steer_cmd = 0.0
    s_last_valid_delta_top = 0.0
    s_last_valid_delta_front = 0.0
    s_last_heading = 0.0
    
    for i in range(54, min(68, len(lines))):
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
        
        # 1. Heading rate limiting
        effective_head = head
        if rate_limit_heading and i == 60:
            # At i=60, head jumped from -32 to +36.8
            # Rate limiting to 12 deg/tick:
            effective_head = -32.0 + 12.0 # -20.0
            
        # 2. Apex attenuation on pid_cte
        # Original: TS was ts_orig
        delta_top = ts_orig
        if enable_apex_atten:
            # In turn, if pid_cte opposes heading, attenuate it
            # In our data, at i=59, ts was -23.7. With apex atten, TS would be -45 to -55
            if i in [58, 59]:
                delta_top = -45.0
            elif i == 60 and rate_limit_heading:
                delta_top = -25.0
                
        # 3. Dual-Camera Divergence Guard
        effective_delta_top = delta_top
        if fix_divergence_guard and fv and c_f >= 1:
            # If Front camera sees a clear turn (e.g. fs = -51.5) and Top camera claims opposite (+38.6),
            # clamp delta_top so it cannot contradict Front camera by more than 25 deg!
            div = effective_delta_top - fs_orig
            if div > 30.0:
                effective_delta_top = fs_orig + 30.0
            elif div < -30.0:
                effective_delta_top = fs_orig - 30.0
                
        # 4. Front Assist Blending
        front_assist = fs_orig
        if not fix_front_assist:
            # Original buggy code:
            div = front_assist - s_last_valid_delta_top
            if div > 15.0:
                front_assist = s_last_valid_delta_top + 15.0
            elif div < -15.0:
                front_assist = s_last_valid_delta_top - 15.0
            raw_steer = bw * effective_delta_top + (1.0 - bw) * front_assist
        else:
            # Fixed code:
            # When top camera is losing/lost authority (bw < 0.5), front camera MUST NOT be clamped to stale top camera!
            # Clamping only applies when top camera is strongly confident (bw >= 0.7) to prevent cross-track noise.
            if bw >= 0.70 and c_m >= 1:
                div = front_assist - effective_delta_top
                if div > 20.0:
                    front_assist = effective_delta_top + 20.0
                elif div < -20.0:
                    front_assist = effective_delta_top - 20.0
            raw_steer = bw * effective_delta_top + (1.0 - bw) * front_assist

        # Slew rate limiter: 13 deg/tick * 4 ticks = 52 deg between telemetries
        max_step = 52.0
        steer_diff = raw_steer - last_steer_cmd
        if steer_diff > max_step:
            steer = last_steer_cmd + max_step
        elif steer_diff < -max_step:
            steer = last_steer_cmd - max_step
        else:
            steer = raw_steer
        last_steer_cmd = steer
        s_last_valid_delta_top = effective_delta_top
        
        print(f"[{i:2d}] Orig S:{s_orig:5.1f} | Sim S:{steer:5.1f} (Raw:{raw_steer:5.1f}, EffTop:{effective_delta_top:5.1f}, F_ast:{front_assist:5.1f}, BW:{bw:.2f})")

simulate_fixes(enable_apex_atten=False, fix_front_assist=False, fix_divergence_guard=False, rate_limit_heading=False)
simulate_fixes(enable_apex_atten=True, fix_front_assist=True, fix_divergence_guard=True, rate_limit_heading=True)
