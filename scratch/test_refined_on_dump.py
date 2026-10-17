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

def test_refined_detector(vecs):
    if not vecs: return False, False, False, None
    count = len(vecs)
    parsed = []
    has_trans = False
    crosses_center = False
    trans_y_far_min = 999.0
    trans_y_near_max = -1.0

    for v in vecs:
        x0, y0, x1, y1 = v
        x_near, y_near = (x0, y0) if y0 >= y1 else (x1, y1)
        x_far, y_far   = (x1, y1) if y0 >= y1 else (x0, y0)
        dx = x_far - x_near
        dy = y_near - y_far
        ang = math.atan2(dx, dy) * 180.0 / math.pi
        is_tr = (dy <= 0.27 * abs(dx) and abs(dx) >= 16.0)
        parsed.append({'v': v, 'x_near': x_near, 'y_near': y_near, 'x_far': x_far, 'y_far': y_far, 'dx': dx, 'dy': dy, 'ang': ang, 'is_tr': is_tr})

        if is_tr:
            has_trans = True
            if y_near > trans_y_near_max: trans_y_near_max = y_near
            if y_far < trans_y_far_min:   trans_y_far_min = y_far
            min_x = min(x_near, x_far)
            max_x = max(x_near, x_far)
            # A wall crosses center only if it actually straddles the center 39!
            if min_x < 36.0 and max_x > 42.0:
                crosses_center = True

    # Look for exit pair:
    # 1. Perspective angles can reach up to 45 deg converging to center
    # 2. Separation between 20 and 45 px
    # 3. Center of the pair is within the track corridor [25, 53]
    # 4. Extends beyond transverse tape
    best_pair = None
    for j in range(count):
        if parsed[j]['is_tr'] or parsed[j]['dy'] < 4.0: continue
        if abs(parsed[j]['ang']) > 45.0: continue
        mid_j = (parsed[j]['x_near'] + parsed[j]['x_far']) * 0.5
        if has_trans:
            if parsed[j]['y_far'] >= trans_y_far_min: continue
            if parsed[j]['y_near'] > (trans_y_near_max + 5.0): continue

        for k in range(j + 1, count):
            if parsed[k]['is_tr'] or parsed[k]['dy'] < 4.0: continue
            if abs(parsed[k]['ang']) > 45.0: continue
            mid_k = (parsed[k]['x_near'] + parsed[k]['x_far']) * 0.5
            if has_trans:
                if parsed[k]['y_far'] >= trans_y_far_min: continue
                if parsed[k]['y_near'] > (trans_y_near_max + 5.0): continue

            sep = abs(mid_k - mid_j)
            pair_center = (mid_j + mid_k) * 0.5
            if 20.0 <= sep <= 45.0 and 24.0 <= pair_center <= 54.0:
                best_pair = (parsed[j]['v'], parsed[k]['v'], sep, pair_center)
                break
        if best_pair: break

    is_intersection = has_trans and (not crosses_center) and (best_pair is not None)
    return is_intersection, has_trans, crosses_center, best_pair

detected_count = 0
for i in range(len(frames)):
    is_ix, has_tr, cr_cnt, pair = test_refined_detector(frames[i]['frn'])
    if is_ix:
        detected_count += 1
        print(f"Frame {i:3d}: INTERSECTION CONFIRMED! pair={pair}")
        print(f"    telem: {frames[i]['telem']}")

print(f"\nTotal frames where intersection was confirmed: {detected_count} / {len(frames)}")
