import re

with open("scratch/run_data.txt") as f:
    text = f.read()

# Let's split by "OK state=RUNNING"
runs = text.split("OK state=RUNNING")
print("Number of runs:", len(runs) - 1)

for r_idx in range(1, len(runs)):
    print(f"\n======================================== RUN {r_idx} ========================================")
    lines = [l.strip() for l in runs[r_idx].split("\n") if l.strip().startswith("L:")]
    for i, l in enumerate(lines):
        # Parse fields
        # L:2/1 R:150 F:152 H:10 TS:93 FS:300 S:146 BL:100 BT:37 DOM:F FV:1 IX:0 ST:C VL:84 VR:62
        # Print when corner entry happens or when FS != 0
        m = re.search(r'H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+).*?DOM:([TF])\s+FV:(\d+).*?ST:([SCI])\s+VL:([-\d]+)\s+VR:([-\d]+)', l)
        if m:
            h = float(m.group(1)) / 10.0
            ts = float(m.group(2)) / 10.0
            fs = float(m.group(3)) / 10.0
            s = float(m.group(4)) / 10.0
            dom = m.group(5)
            fv = int(m.group(6))
            st = m.group(7)
            vl = int(m.group(8))
            vr = int(m.group(9))
            
            # Print if |H| > 10 or |FS| > 15 or st == 'C' or vl < 100 or vr < 100
            flag = ""
            if st == 'C': flag += " [CORNER]"
            if abs(h) > 25: flag += " [HIGH_H]"
            if abs(fs) > 20: flag += " [FRONT_CURVE]"
            if vl < 100 or vr < 100: flag += f" [BRAKE {vl}/{vr}]"
            
            # Print around corners
            print(f"L{i:03d} | H:{h:5.1f}° FS:{fs:5.1f}° TS:{ts:5.1f}° S:{s:5.1f}° | DOM:{dom} FV:{fv} ST:{st} | VL:{vl:3d} VR:{vr:3d} | {l.split()[0]} {l.split()[1]} {l.split()[2]} {flag}")
