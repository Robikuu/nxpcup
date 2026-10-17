import re

with open("scratch/run_crash_telemetry.txt") as f:
    lines = [l.strip() for l in f if l.strip().startswith("L:")]

print("=== DETAILED TICK TRACE 55-72 ===")
for i in range(54, min(75, len(lines))):
    l = lines[i]
    m = re.match(r"L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BW:(\d+)\s+DOM:(\w+)\s+FV:(\d+)\s+IX:(\d+)\s+ST:(\w+)\s+VL:(\d+)\s+VR:(\d+)", l)
    if m:
        c_m, c_f = int(m.group(1)), int(m.group(2))
        r_cte = int(m.group(3)) / 10.0
        f_cte = int(m.group(4)) / 10.0
        head = int(m.group(5)) / 10.0
        ts = int(m.group(6)) / 10.0
        fs = int(m.group(7)) / 10.0
        s = int(m.group(8)) / 10.0
        bw = int(m.group(9))
        dom = m.group(10)
        st = m.group(13)
        vl, vr = int(m.group(14)), int(m.group(15))
        
        # Calculate what happened to heading and steering
        print(f"[{i:2d}] Cam:{c_m}/{c_f} | CTE_r:{r_cte:5.1f} CTE_f:{f_cte:5.1f} | Head:{head:5.1f} | TS:{ts:5.1f} FS:{fs:5.1f} -> Steer:{s:5.1f} | BW:{bw}% DOM:{dom} ST:{st} | VL:{vl} VR:{vr}")
