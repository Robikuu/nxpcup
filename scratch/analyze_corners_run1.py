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

# Find all corner entries where TS magnitude increases above 20 deg
corner_entries = []
in_corner = False
for i, f in enumerate(frames1):
    if abs(f['ts']) > 200 and not in_corner:
        corner_entries.append(i)
        in_corner = True
    elif abs(f['ts']) < 100:
        in_corner = False

print(f"Detected {len(corner_entries)} corner entries in Run 1:")
for c_idx in corner_entries[:6]:
    start = max(0, c_idx - 5)
    end = min(len(frames1), c_idx + 10)
    print(f"\n--- Corner entry around frame {c_idx} ---")
    print("Frame | L_top/front | Heading |   TS   |   FS   | Speed L/R | ST | BW")
    for j in range(start, end):
        f = frames1[j]
        mark = " <-- PEAK" if j == c_idx else ""
        print(f"{j:5d} | {f['top_found']}/{f['front_found']}         | {f['heading']/10:6.1f}° | {f['ts']/10:6.1f}° | {f['fs']/10:6.1f}° | {f['vl']:3d}/{f['vr']:3d}   |  {f['st']} | {f['bw']:3d}{mark}")
