import re

log = """
L:2/3 R:5 F:5 H:33 TJ:27 S:2 LA:0 ST:F VL:100 VR:100
L:2/3 R:5 F:5 H:33 TJ:25 S:2 LA:0 ST:F VL:100 VR:100
L:2/3 R:10 F:9 H:22 TJ:21 S:7 LA:0 ST:F VL:100 VR:100
L:2/3 R:15 F:13 H:11 TJ:12 S:11 LA:0 ST:F VL:100 VR:100
L:2/3 R:15 F:14 H:2 TJ:3 S:9 LA:0 ST:F VL:100 VR:100
L:2/3 R:15 F:14 H:0 TJ:-3 S:7 LA:0 ST:F VL:100 VR:100
L:2/3 R:20 F:19 H:5 TJ:-7 S:12 LA:0 ST:F VL:100 VR:100
L:2/0 R:25 F:23 H:18 TJ:-1 S:15 LA:0 ST:F VL:100 VR:100
L:2/3 R:30 F:28 H:27 TJ:1 S:19 LA:1 ST:F VL:100 VR:100
L:2/3 R:45 F:40 H:34 TJ:19 S:33 LA:12 ST:T VL:96 VR:96
L:2/4 R:60 F:55 H:42 TJ:31 S:52 LA:23 ST:T VL:93 VR:90
L:2/3 R:85 F:75 H:45 TJ:3 S:77 LA:26 ST:T VL:93 VR:86
L:2/3 R:110 F:100 H:51 TJ:135 S:109 LA:24 ST:T VL:93 VR:81
L:2/3 R:130 F:119 H:-34 TJ:109 S:100 LA:19 ST:T VL:94 VR:83
L:2/4 R:145 F:141 H:-223 TJ:-211 S:-201 LA:3 ST:T VL:58 VR:74
L:1/0 R:235 F:178 H:38 TJ:-392 S:-159 LA:43 ST:T VL:68 VR:88
"""

lines = [l.strip() for l in log.strip().split("\n") if l.strip()]
data = []

print("Frame |  T(ms) | St | CTE(px) | H(deg) | TJ(deg) | Steer(deg) | LA(%) |   VL |   VR | AvgSpd")
print("-" * 80)

for i, l in enumerate(lines):
    m = re.match(r'L:(\d+)/(\d+) R:(-?\d+) F:(-?\d+) H:(-?\d+) TJ:(-?\d+) S:(-?\d+) LA:(\d+) ST:(\w+) VL:(\d+) VR:(\d+)', l)
    if not m:
        continue
    top_l = int(m.group(1))
    front_l = int(m.group(2))
    raw_cte = int(m.group(3)) / 10.0
    f_cte = int(m.group(4)) / 10.0
    h = int(m.group(5)) / 10.0
    tj = int(m.group(6)) / 10.0
    steer = int(m.group(7)) / 10.0
    la = int(m.group(8))
    st = m.group(9)
    vl = int(m.group(10))
    vr = int(m.group(11))
    t_ms = i * 80 # telemetry div is 4 ticks = 80ms
    avg_spd = (vl + vr) / 2.0
    
    print(f"{i:5d} | {t_ms:6d} |  {st} |  {f_cte:6.1f} | {h:6.1f} |  {tj:6.1f} |     {steer:6.1f} |   {la:3d} |  {vl:3d} |  {vr:3d} |  {avg_spd:5.1f}")
