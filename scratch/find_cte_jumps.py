import re

with open('d:/nxpcup/latest_user_request.txt', 'r', encoding='utf-8') as f:
    text = f.read()

lines = text.splitlines()

pattern = r'L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BL:([-\d]+)\s+BT:([-\d]+)\s+DOM:(\w)\s+FV:(\d+)\s+IX:(\d+)\s+ST:(\w+)\s+VL:([-\d]+)\s+VR:([-\d]+)'

prev_cte = None
prev_lno = None
jumps = []

for lno, line in enumerate(lines, 1):
    m = re.match(pattern, line.strip())
    if m:
        lt, lf, r, f, h, ts, fs, s, bl, bt, dom, fv, ix, st, vl, vr = m.groups()
        cte = float(f) / 10.0
        raw_cte = float(r) / 10.0
        if prev_cte is not None and abs(lno - prev_lno) <= 2:
            delta = cte - prev_cte
            if abs(delta) > 15.0:
                jumps.append((lno, prev_cte, cte, delta, raw_cte, line.strip()))
        prev_cte = cte
        prev_lno = lno
    elif 'OK state=' in line:
        prev_cte = None

print(f'Total jumps with |delta CTE| > 15 px: {len(jumps)}')
for j in jumps:
    print(f'L{j[0]}: prev_CTE={j[1]:+5.1f} -> cur_CTE={j[2]:+5.1f} (delta={j[3]:+5.1f}), raw_CTE={j[4]:+5.1f} | {j[5]}')
