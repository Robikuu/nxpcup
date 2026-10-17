"""
INVESTIGATION SCRIPT: Replay the Failure Run through the main.c code logic
==========================================================================
Let's reproduce the exact failure telemetry provided by the user.
"""

# The telemetry frames provided by the user
telemetry_lines = [
    "L:2/3 R:-10 F:-13 H:24 TJ:41 S:-9 LA:0 ST:N VL:100 VR:100",
    "L:2/3 R:-10 F:-10 H:22 TJ:40 S:-9 LA:0 ST:N VL:100 VR:100",
    "L:2/3 R:-10 F:-10 H:17 TJ:30 S:-14 LA:0 ST:N VL:100 VR:100",
    "L:2/3 R:-10 F:-10 H:12 TJ:25 S:-15 LA:0 ST:N VL:100 VR:100",
    "L:2/3 R:-5 F:-6 H:0 TJ:15 S:-5 LA:0 ST:N VL:100 VR:100",
    "L:2/3 R:-5 F:-5 H:0 TJ:12 S:-6 LA:0 ST:N VL:100 VR:100",
    "L:2/3 R:-5 F:-5 H:0 TJ:17 S:-7 LA:0 ST:N VL:100 VR:100",
    "L:2/3 R:0 F:-1 H:12 TJ:20 S:1 LA:0 ST:N VL:100 VR:100",
    "L:2/3 R:10 F:5 H:25 TJ:29 S:26 LA:0 ST:N VL:99 VR:99",
    "L:0/3 R:0 F:14 H:36 TJ:30 S:61 LA:5 ST:N VL:97 VR:94",
    "L:2/3 R:45 F:37 H:53 TJ:49 S:138 LA:12 ST:N VL:94 VR:82",
    "L:2/3 R:70 F:63 H:60 TJ:74 S:197 LA:15 ST:N VL:93 VR:75",
    "L:2/4 R:85 F:81 H:51 TJ:70 S:204 LA:50 ST:N VL:78 VR:66",
    "L:2/2 R:85 F:84 H:-27 TJ:2 S:130 LA:9 ST:N VL:96 VR:85",
    "L:2/3 R:60 F:66 H:-104 TJ:-123 S:-124 LA:65 ST:N VL:67 VR:72",
    "L:2/2 R:20 F:32 H:-230 TJ:-292 S:-358 LA:96 ST:N VL:50 VR:59",
    "L:2/1 R:-10 F:1 H:-357 TJ:-600 S:-573 LA:100 ST:N VL:48 VR:58",
    "L:0/1 R:0 F:19 H:-382 TJ:-662 S:-486 LA:0 ST:I VL:48 VR:70",
    "L:0/2 R:0 F:19 H:-382 TJ:-689 S:0 LA:0 ST:L VL:50 VR:50",
    "L:0/1 R:0 F:19 H:-382 TJ:-653 S:0 LA:0 ST:L VL:50 VR:50",
    "L:0/1 R:0 F:19 H:-382 TJ:-160 S:0 LA:0 ST:L VL:50 VR:50",
    "L:0/1 R:0 F:19 H:-382 TJ:-515 S:0 LA:0 ST:L VL:50 VR:50",
    "L:0/0 R:0 F:19 H:-382 TJ:-426 S:0 LA:0 ST:L VL:0 VR:0",
    "L:0/0 R:0 F:19 H:-382 TJ:-174 S:0 LA:0 ST:L VL:0 VR:0",
    "L:0/0 R:0 F:19 H:-382 TJ:-71 S:0 LA:0 ST:L VL:0 VR:0",
]

import re
pattern = re.compile(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TJ:(-?\d+)\s+S:(-?\d+)\s+LA:(\d+)\s+ST:([NIL])\s+VL:(\d+)\s+VR:(\d+)')

parsed = []
for idx, l in enumerate(telemetry_lines):
    m = pattern.search(l)
    if m:
        top_v, front_v, r, f, h, tj, s, la, st, vl, vr = m.groups()
        parsed.append({
            'idx': idx,
            'top_v': int(top_v),
            'front_v': int(front_v),
            'r': int(r) / 10.0,
            'f': int(f) / 10.0,
            'h': int(h) / 10.0,
            'tj': int(tj) / 10.0,
            's': int(s) / 10.0,
            'la': int(la),
            'st': st,
            'vl': int(vl),
            'vr': int(vr)
        })

print(f"Total failure frames: {len(parsed)}")

# Check kappa and target speed calculation for each frame
GOVERNOR_HEADING_DEADBAND = 2.5
GOVERNOR_HEADING_MAX = 25.0
SPEED_STRAIGHT = 100.0
SPEED_CORNER = 58.0

print(f"\n{'Idx':<4} | {'H_deg':>6} | {'LA':>4} | {'k_base':>7} | {'k_front':>7} | {'kappa':>6} | {'v_target':>8} | {'VL':>4} | {'VR':>4} | {'ST':>3} | {'Steer':>6}")
print("-" * 80)
for p in parsed:
    abs_h = abs(p['h'])
    k_base = 0.0
    if abs_h > GOVERNOR_HEADING_DEADBAND:
        k_base = (abs_h - GOVERNOR_HEADING_DEADBAND) / (GOVERNOR_HEADING_MAX - GOVERNOR_HEADING_DEADBAND)
        if k_base > 1.0: k_base = 1.0
    
    k_front = p['la'] / 100.0
    if abs_h <= GOVERNOR_HEADING_DEADBAND and abs(p['f']) <= 4.0:
        k_front = 0.0
    if k_front > 1.0: k_front = 1.0
    
    kappa = max(k_base, k_front)
    target = SPEED_STRAIGHT - kappa * (SPEED_STRAIGHT - SPEED_CORNER)
    
    print(f"{p['idx']:<4} | {p['h']:>6.1f} | {p['la']:>4} | {k_base:>7.3f} | {k_front:>7.3f} | {kappa:>6.3f} | {target:>7.1f}% | {p['vl']:>4} | {p['vr']:>4} | {p['st']:>3} | {p['s']:>6.1f}")
