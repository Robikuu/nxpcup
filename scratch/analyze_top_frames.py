import re

with open('scratch/real_intersection_dump.txt', encoding='utf-8') as f:
    lines = f.read().splitlines()

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

print(f"Total frames: {len(frames)}")
for i, fr in enumerate(frames):
    top = fr['top'] if fr['top'] else []
    has_trans = False
    for v in top:
        x0, y0, x1, y1 = v
        dx = x1 - x0
        dy = abs(y1 - y0)
        if dy <= 0.27 * abs(dx) and abs(dx) >= 16:
            has_trans = True
    if has_trans or len(top) == 0 or 'L:0/' in fr['telem'] or 'L:1/' in fr['telem']:
        print(f"Frame {i:3d}: top_len={len(top)} has_trans={has_trans} top_vecs={top}")
        print(f"     telem: {fr['telem']}")
