import re, math

with open('scratch/real_intersection_dump.txt', encoding='utf-8') as f:
    text = f.read()

lines = text.splitlines()
frames = []
curr_top, curr_frn = None, None
for l in lines:
    if 'RAW_TOP' in l:
        m = re.findall(r'\((\d+),(\d+),(\d+),(\d+)\)', l)
        curr_top = [(int(x[0]), int(x[1]), int(x[2]), int(x[3])) for x in m]
    elif 'RAW_FRN' in l:
        m = re.findall(r'\((\d+),(\d+),(\d+),(\d+)\)', l)
        curr_frn = [(int(x[0]), int(x[1]), int(x[2]), int(x[3])) for x in m]
    elif l.startswith('L:'):
        frames.append({'top': curr_top, 'frn': curr_frn, 'telem': l})
        curr_top, curr_frn = None, None

for i in range(114, 135):
    print(f"=== Frame {i:3d} ===")
    print(f"  RAW_TOP: {frames[i]['top']}")
    if frames[i]['top']:
        for v in frames[i]['top']:
            x0, y0, x1, y1 = v
            dx = x1 - x0
            dy = y0 - y1 if y0 >= y1 else y1 - y0
            ang = math.atan2(dx, dy) * 180.0 / math.pi
            is_tr = (dy <= 0.27 * abs(dx) and abs(dx) >= 16)
            print(f"     v={v} dx={dx} dy={dy} ang={ang:.1f} is_trans={is_tr}")
    print(f"  RAW_FRN: {frames[i]['frn']}")
    print(f"  TELEM  : {frames[i]['telem']}")
