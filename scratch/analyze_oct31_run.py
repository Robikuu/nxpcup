import re

with open(r'd:\nxpcup\scratch\user_run_oct31.txt', 'r') as f:
    lines = f.readlines()

records = []
for idx, line in enumerate(lines):
    line = line.strip()
    if not line:
        continue
    if 'Sent utf8' in line:
        records.append(('CMD', line, idx))
        continue
    if 'OK state=' in line:
        records.append(('STATUS', line, idx))
        continue
    # Notice line "LL:2/2 ..." or "L:2/1 ... 34 VR:65"
    # Clean up double LL: or broken lines
    clean_line = line
    if clean_line.startswith('LL:'):
        clean_line = 'L:' + clean_line[3:]
    m = re.search(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TS:(-?\d+)\s+FS:(-?\d+)\s+S:(-?\d+)\s+BL:(\d+)\s+BT:(\d+)\s+DOM:([TF])\s+FV:(\d+)\s+IX:(\d+)\s+ST:([SCI])\s+VL:(\d+)\s+VR:(\d+)', clean_line)
    if m:
        records.append(('TELEM', {
            'file_line': idx + 1,
            'top_l': int(m.group(1)),
            'frn_l': int(m.group(2)),
            'r_cte': float(m.group(3)) / 10.0,
            'f_cte': float(m.group(4)) / 10.0,
            'f_head': float(m.group(5)) / 10.0,
            'ts': float(m.group(6)) / 10.0,
            'fs': float(m.group(7)) / 10.0,
            's': float(m.group(8)) / 10.0,
            'bl': int(m.group(9)),
            'bt': int(m.group(10)),
            'dom': m.group(11),
            'fv': int(m.group(12)),
            'ix': int(m.group(13)),
            'st': m.group(14),
            'vl': int(m.group(15)),
            'vr': int(m.group(16))
        }, idx))
    else:
        records.append(('UNPARSED', line, idx))

print(f"Total lines processed: {len(records)}")
telem_count = sum(1 for r in records if r[0] == 'TELEM')
print(f"Telem frames: {telem_count}")
for r in records:
    if r[0] != 'TELEM':
        print(f"Non-telem: {r}")
