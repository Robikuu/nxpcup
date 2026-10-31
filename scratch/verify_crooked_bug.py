import re

# Let's inspect when crooked_straight_track is triggered across all lines in run_data.txt
with open("scratch/run_data.txt") as f:
    lines = [l.strip() for l in f if l.startswith("L:")]

print(f"Total lines: {len(lines)}")

crooked_lines = []
blocked_gate_lines = []

for i, l in enumerate(lines):
    # L:2/2 R:76 F:68 H:-225 TS:-165 FS:-235 S:-165 BL:0 BT:0 DOM:T FV:1 IX:0 ST:S VL:100 VR:100
    m = re.match(r'L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BL:([-\d]+)\s+BT:([-\d]+)\s+DOM:([TF])\s+FV:(\d+)\s+IX:(\d+)\s+ST:([SCI])\s+VL:([-\d]+)\s+VR:([-\d]+)', l)
    if not m:
        continue
    top_found = int(m.group(1))
    front_found = int(m.group(2))
    h = float(m.group(5)) / 10.0
    ts = float(m.group(6)) / 10.0
    fs = float(m.group(7)) / 10.0
    s = float(m.group(8)) / 10.0
    fv = int(m.group(12))
    st = m.group(14)
    vl = int(m.group(15))
    vr = int(m.group(16))
    
    front_curve_rel = fs - h
    
    # Check crooked_straight_track condition:
    # (horizon.found > 0) && (!front_corner_wall) && (!sr.transverse_crosses_center) && (fabsf(front_curve_rel) < 15.0f)
    if front_found > 0 and abs(front_curve_rel) < 15.0:
        crooked_lines.append((i, h, fs, front_curve_rel, st, vl, vr, l))
        
    # Check when front camera saw corner (|fs| >= 18.0) but top_turn_gate blocked it:
    # Condition: (filtered_heading * eff_front_angle > 0.0f || fabsf(filtered_heading) < 3.0f)
    if abs(fs) >= 18.0 and not (h * fs > 0.0 or abs(h) < 3.0):
        blocked_gate_lines.append((i, h, fs, ts, s, l))

print(f"\nLines where 'crooked_straight_track' was TRUE: {len(crooked_lines)}")
print("Sample of crooked_straight_track lines where |H| was actually HIGH (in a real corner!):")
for item in crooked_lines:
    if abs(item[1]) > 20.0:  # |H| > 20 deg
        print(f"  Line {item[0]:3d}: H={item[1]:5.1f}°, FS={item[2]:5.1f}°, |FS-H|={abs(item[3]):4.1f}° | ST:{item[4]} | VL:{item[5]} VR:{item[6]}")

print(f"\nLines where front camera saw corner (|FS| >= 18.0) but top_turn_gate was BLOCKED by heading sign/deadband: {len(blocked_gate_lines)}")
for item in blocked_gate_lines:
    print(f"  Line {item[0]:3d}: H={item[1]:5.1f}°, FS={item[2]:5.1f}°, TS={item[3]:5.1f}°, S={item[4]:5.1f}°")
