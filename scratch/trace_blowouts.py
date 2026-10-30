import re

with open('d:/nxpcup/latest_user_request.txt', 'r', encoding='utf-8') as f:
    text = f.read()

lines = text.splitlines()

def print_range(start_line, end_line, title):
    print(f"=== {title} (L{start_line} - L{end_line}) ===")
    pattern = r'L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BL:([-\d]+)\s+BT:([-\d]+)\s+DOM:(\w)\s+FV:(\d+)\s+IX:(\d+)\s+ST:(\w+)\s+VL:([-\d]+)\s+VR:([-\d]+)'
    for lno in range(start_line, end_line + 1):
        line = lines[lno - 1].strip()
        m = re.match(pattern, line)
        if m:
            lt, lf, r, f, h, ts, fs, s, bl, bt, dom, fv, ix, st, vl, vr = m.groups()
            r, f, h, ts, fs, s, bl, bt, vl, vr = [float(x) for x in (r, f, h, ts, fs, s, bl, bt, vl, vr)]
            print(f"L{lno:4d}: L:{lt}/{lf} CTE:{f/10.0:+5.1f} H:{h/10.0:+5.1f} TS:{ts/10.0:+5.1f} FS:{fs/10.0:+5.1f} S:{s/10.0:+5.1f} ST:{st} VL:{int(vl):3d} VR:{int(vr):3d} BT:{int(bt):3d}")

print_range(280, 312, "RUN 1 EPISODE 4 BLOWOUT")
print_range(1590, 1625, "RUN 5 EPISODE 21 BLOWOUT (CTE 53px)")
