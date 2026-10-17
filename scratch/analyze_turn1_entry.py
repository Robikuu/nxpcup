import re

with open(r"d:\nxpcup\scratch\full_live_test.txt", "r", encoding="utf-8") as f:
    text = f.read()

parts = text.split('OK state=RUNNING')

pattern = re.compile(
    r'L:(\d+)/(\d+)\s+'
    r'R:(-?\d+)\s+'
    r'F:(-?\d+)\s+'
    r'H:(-?\d+)\s+'
    r'TS:(-?\d+)\s+'
    r'FS:(-?\d+)\s+'
    r'S:(-?\d+)\s+'
    r'BW:(\d+)\s+'
    r'DOM:([TFB])\s+'
    r'FV:(\d+)\s+'
    r'IX:(\d+)\s+'
    r'ST:([NIL])\s+'
    r'VL:(-?\d+)\s+'
    r'VR:(-?\d+)'
)

frames1 = []
for line in parts[1].splitlines():
    m = pattern.search(line)
    if m:
        frames1.append({k: int(v) if k not in ['dom','st'] else v for k, v in zip(
            ['top_found','front_found','raw_cte','filt_cte','heading','ts','fs','s','bw','dom','fv','ix','st','vl','vr'],
            m.groups()
        )})

print("Frame | L_top/front | CTE (px) | Heading |   TS   |   FS   | Speed L/R")
print("-" * 75)
for i in range(5, 20):
    f = frames1[i]
    print(f"{i:5d} | {f['top_found']}/{f['front_found']}         | {f['filt_cte']/10:8.1f} | {f['heading']/10:6.1f}° | {f['ts']/10:6.1f}° | {f['fs']/10:6.1f}° | {f['vl']}/{f['vr']}")
