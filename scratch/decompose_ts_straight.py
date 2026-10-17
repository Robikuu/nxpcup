import re

with open("scratch/full_user_telemetry_latest.txt") as f:
    raw_lines = [l.strip() for l in f if l.strip().startswith("L:")]

# Parameters used in that run:
k_cte = 0.85
kp_s = 0.75
kd_s = 0.14
k_head = 1.45
k_la = 0.35

print(f"{'Tick':<5} | {'CTE':<5} | {'Head':<5} | {'TS':<5} | {'Head_FF':<7} | {'pid_cte_approx':<14} | {'LA_FF_approx':<12}")
print("-" * 65)

prev_cte = None
dt = 0.08 # 4 ticks = 0.08s between telemetry samples (approx)

for idx in range(165, 175):
    l = raw_lines[idx]
    m = re.match(r"L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)", l)
    if not m:
        continue
    f_cte = int(m.group(4)) / 10.0
    head = int(m.group(5)) / 10.0
    ts = int(m.group(6)) / 10.0
    
    # 1. Heading FF
    # heading deadband is 3.5 deg!
    abs_h = abs(head)
    eff_h = (abs_h - 3.5) * (1.0 if head > 0 else -1.0) if abs_h > 3.5 else 0.0
    head_ff = k_head * eff_h
    
    # 2. pid_cte P-term:
    p_term = kp_s * k_cte * f_cte
    
    remainder = ts - head_ff
    
    print(f"[{idx:3d}] | {f_cte:5.1f} | {head:5.1f} | {ts:5.1f} | {head_ff:7.1f} | P_cte:{p_term:5.1f} (Rem:{remainder:5.1f})")
