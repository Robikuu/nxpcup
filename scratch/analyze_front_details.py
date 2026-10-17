import re, math

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

def analyze_front(vecs):
    if not vecs: return {'trans': [], 'others': [], 'center_cross': False, 'pair_info': []}
    trans = []
    others = []
    for v in vecs:
        x0, y0, x1, y1 = v
        x_near, y_near = (x0, y0) if y0 >= y1 else (x1, y1)
        x_far, y_far   = (x1, y1) if y0 >= y1 else (x0, y0)
        dx = x_far - x_near
        dy = y_near - y_far
        ang = math.atan2(dx, dy) * 180.0 / math.pi
        is_tr = (dy <= 0.27 * abs(dx) and abs(dx) >= 16.0)
        info = {'v': v, 'x_near': x_near, 'y_near': y_near, 'x_far': x_far, 'y_far': y_far, 'dx': dx, 'dy': dy, 'ang': ang, 'is_tr': is_tr}
        if is_tr: trans.append(info)
        else: others.append(info)
    
    # Check center cross
    center_cross = False
    for t in trans:
        min_x = min(t['x_near'], t['x_far'])
        max_x = max(t['x_near'], t['x_far'])
        if min_x <= 53.0 and max_x >= 25.0:
            center_cross = True
            
    # Check exit pair
    has_pair = False
    pair_info = []
    for j, o1 in enumerate(others):
        mid1 = (o1['x_near'] + o1['x_far']) * 0.5
        for k, o2 in enumerate(others):
            if j >= k: continue
            mid2 = (o2['x_near'] + o2['x_far']) * 0.5
            sep = abs(mid2 - mid1)
            pair_info.append(f"pair({o1['v']}[ang={o1['ang']:.1f},mid={mid1:.1f}], {o2['v']}[ang={o2['ang']:.1f},mid={mid2:.1f}], sep={sep:.1f})")
            
    return {
        'trans': [t['v'] for t in trans],
        'others': [(o['v'], round(o['ang'],1), round((o['x_near']+o['x_far'])*0.5,1)) for o in others],
        'center_cross': center_cross,
        'pair_info': pair_info
    }

for i in range(112, 126):
    a = analyze_front(frames[i]['frn'])
    print(f"\n--- Frame {i} --- Telem: {frames[i]['telem']}")
    print(f"Transverse: {a['trans']}, CenterCross: {a['center_cross']}")
    print(f"Others (vec, ang, x_mid): {a['others']}")
    print("Pairs:")
    for pi in a['pair_info']:
        print("  ", pi)
