import re
import numpy as np

with open("scratch/full_user_telemetry_latest.txt") as f:
    raw_lines = [l.strip() for l in f if l.strip().startswith("L:")]

def parse_line(l):
    m = re.match(r"L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BW:(\d+)\s+DOM:(\w+)\s+FV:(\d+)\s+IX:(\d+)\s+ST:(\w+)\s+VL:(\d+)\s+VR:(\d+)", l)
    if m:
        return {
            'r_cte': int(m.group(3)) / 10.0,
            'f_cte': int(m.group(4)) / 10.0,
            'head': int(m.group(5)) / 10.0,
            'ts': int(m.group(6)) / 10.0,
            'fs': int(m.group(7)) / 10.0,
            's': int(m.group(8)) / 10.0,
            'bw': int(m.group(9)) / 100.0,
            'vl': int(m.group(14)),
            'vr': int(m.group(15)),
        }
    return None

data = [parse_line(l) for l in raw_lines]
data = [d for d in data if d is not None]

# Compare Segment 5 (Ticks 166..174) with proposed straightaway tuning:
# Current:
# - HEADING_DEADBAND_DEG = 3.5
# - DIFFERENTIAL_DEADZONE_DEG = 8.0
# - LOOKAHEAD traj deadband = 5.0
# - k_cte = 0.65 (was 0.85)
# - kp_s = 0.75
# - kd_s = 0.14
#
# Proposed:
# - HEADING_DEADBAND_DEG = 5.0
# - DIFFERENTIAL_DEADZONE_DEG = 12.0
# - LOOKAHEAD traj deadband = 9.0 (only fires for real turns)
# - k_cte = 0.55
# - kp_s = 0.65
# - kd_s = 0.20
# - ACCEL_RAMP_STEP = 18.0 (was 25.0, smooths corner exit acceleration)

print("=== SEGMENT 5 (TICKS 166-174) STEERING COMMAND COMPARISON ===")
print(f"{'Tick':<5} | {'CTE':<5} | {'Head':<5} | {'Orig S':<7} | {'Tuned S':<8} | {'Diff Drag (Orig vs Tuned)':<25}")
print("-" * 65)

for idx in range(166, 175):
    d = data[idx]
    cte = d['f_cte']
    head = d['head']
    orig_s = d['s']
    
    # Calculate Tuned TS:
    # 1. Heading FF with 5.0 deg deadband:
    abs_h = abs(head)
    eff_h = (abs_h - 5.0) * (1.0 if head > 0 else -1.0) if abs_h > 5.0 else 0.0
    head_ff = 1.45 * eff_h
    
    # 2. PID CTE with kp_s=0.65, k_cte=0.55:
    p_cte = 0.65 * 0.55 * cte
    
    # 3. Lookahead FF with 9.0 deg deadband:
    # In this segment, front traj was ~ 7-13 deg
    # With 9.0 deg deadband, eff_traj is much smaller (0 to 4 deg)
    la_ff = 0.0 # mostly zero on straight
    
    tuned_ts = p_cte + head_ff + la_ff
    
    # Check diff drag
    diff_orig = "ACTIVE (Dragging!)" if abs(orig_s) > 8.0 else "OFF"
    diff_tuned = "ACTIVE" if abs(tuned_ts) > 12.0 else "OFF (Smooth!)"
    
    print(f"[{idx:3d}] | {cte:5.1f} | {head:5.1f} | {orig_s:7.1f} | {tuned_ts:8.1f} | {diff_orig:<14} vs {diff_tuned}")
