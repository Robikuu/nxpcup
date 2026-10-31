import re

# Simulate main.c on Run 1 and Run 3 lines
with open("scratch/run_data.txt") as f:
    raw_lines = [l.strip() for l in f if l.startswith("L:")]

def simulate_line(l):
    m = re.match(r'L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BL:([-\d]+)\s+BT:([-\d]+)\s+DOM:([TF])\s+FV:(\d+)\s+IX:(\d+)\s+ST:([SCI])\s+VL:([-\d]+)\s+VR:([-\d]+)', l)
    if not m: return None
    sr_found = int(m.group(1))
    horizon_found = int(m.group(2))
    h = float(m.group(5)) / 10.0
    ts = float(m.group(6)) / 10.0
    fs = float(m.group(7)) / 10.0
    s_actual = float(m.group(8)) / 10.0
    st_actual = m.group(14)
    vl_actual = int(m.group(15))
    vr_actual = int(m.group(16))
    
    # 1. crooked_straight_track
    front_curve_rel = fs - h
    crooked = (horizon_found > 0) and (abs(front_curve_rel) < 15.0)
    
    # 2. eff_front_angle
    eff_front = fs
    if crooked:
        eff_front = 0.0
    elif horizon_found > 0 and (fs * h > 0.0):
        if abs(fs) <= abs(h):
            eff_front = 0.0
        else:
            net = fs - 0.65 * h
            if net * fs < 0.0: net = 0.0
            eff_front = net
            
    # 3. top_turn_gate
    gate = 0.0
    if sr_found == 0:
        gate = 1.0
    elif (abs(eff_front) >= 18.0) and (h * eff_front > 0.0 or abs(h) < 3.0):
        if abs(front_curve_rel) >= 18.0:
            gate = 0.30
        if abs(h) >= 2.0:
            hn = (abs(h) - 2.0) / (12.0 - 2.0)
            if hn > 1.0: hn = 1.0
            if hn > gate: gate = hn
        if sr_found == 2 and gate > 0.75:
            gate = 0.75
            
    ff = gate * 0.50 * eff_front
    
    return {
        'h': h, 'fs': fs, 'ts': ts, 's_actual': s_actual, 'st_actual': st_actual,
        'vl': vl_actual, 'vr': vr_actual,
        'crooked': crooked, 'eff_front': eff_front, 'gate': gate, 'ff': ff
    }

print("=== RUN 1 FIRST CORNER (L18..L25) ===")
for idx in range(18, 26):
    res = simulate_line(raw_lines[idx])
    print(f"L{idx:2d} | H={res['h']:5.1f} FS={res['fs']:5.1f} | crooked={str(res['crooked']):5s} eff_FS={res['eff_front']:5.1f} gate={res['gate']:.2f} ff={res['ff']:4.1f} | TS={res['ts']:5.1f} S_act={res['s_actual']:5.1f} ST={res['st_actual']} VL={res['vl']:3d} VR={res['vr']:3d}")

print("\n=== RUN 1 SECOND CORNER (L27..L33) ===")
for idx in range(27, 34):
    res = simulate_line(raw_lines[idx])
    print(f"L{idx:2d} | H={res['h']:5.1f} FS={res['fs']:5.1f} | crooked={str(res['crooked']):5s} eff_FS={res['eff_front']:5.1f} gate={res['gate']:.2f} ff={res['ff']:4.1f} | TS={res['ts']:5.1f} S_act={res['s_actual']:5.1f} ST={res['st_actual']} VL={res['vl']:3d} VR={res['vr']:3d}")
