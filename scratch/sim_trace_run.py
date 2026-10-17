import re, math

with open('scratch/real_intersection_dump.txt') as f:
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

def compute_top(vecs):
    if not vecs: return 0, 0.0, 0.0, 0, 0
    has_tr, cr_cnt = 0, 0
    best_l_x, best_l_ang = -1000.0, 0.0
    best_r_x, best_r_ang = 1000.0, 0.0
    l_found, r_found = False, False
    
    for x0, y0, x1, y1 in vecs:
        x_near, y_near = (x0, y0) if y0 >= y1 else (x1, y1)
        x_far, y_far   = (x1, y1) if y0 >= y1 else (x0, y0)
        dy = y_near - y_far
        dx = x_far - x_near
        abs_dx = abs(dx)
        
        # Chassis exclusion
        if 36.0 <= x_near <= 44.0 and y_near >= 45.0 and dy <= 8.0:
            continue
        # Stateless transverse
        if dy <= 0.27 * abs_dx and abs_dx >= 16.0:
            has_tr = 1
            min_x, max_x = min(x_near, x_far), max(x_near, x_far)
            if min_x < 35.0 and max_x > 43.0:
                cr_cnt = 1
            continue
        if y_near < 16.0: continue
        if dy < 4.0: continue
        # Chord filter
        if abs_dx >= dy and dy < 10.0: continue
        ang = math.atan2(dx, dy) * 180.0 / math.pi
        if abs(ang) > 55.0 and dy < 12.0: continue
        
        if x_near < 39.0:
            if x_near > best_l_x:
                best_l_x, best_l_ang = x_near, ang
                l_found = True
        elif x_near > 39.0:
            if x_near < best_r_x:
                best_r_x, best_r_ang = x_near, ang
                r_found = True
                
    found = 0
    if l_found and r_found:
        found = 2
        cx = (best_l_x + best_r_x) * 0.5
        head = (best_l_ang + best_r_ang) * 0.5
    elif l_found:
        found = 1
        cx = best_l_x + 22.0
        head = best_l_ang
    elif r_found:
        found = 1
        cx = best_r_x - 22.0
        head = best_r_ang
    else:
        cx, head = 39.0, 0.0
    return found, cx - 39.0, head, has_tr, cr_cnt

def compute_frn(vecs):
    if not vecs: return 0, 0, 0, 0.0, 0
    count = min(len(vecs), 32)
    parsed = []
    has_tr, cr_cnt = 0, 0
    tr_y_near_max = 0.0
    tr_y_far_min = 999.0
    sum_w_ang, sum_l = 0.0, 0.0
    valid_cnt = 0
    
    for i in range(count):
        x0, y0, x1, y1 = vecs[i]
        x_near, y_near = (x0, y0) if y0 >= y1 else (x1, y1)
        x_far, y_far   = (x1, y1) if y0 >= y1 else (x0, y0)
        dx = x_far - x_near
        dy = y_near - y_far
        abs_dx = abs(dx)
        l = math.sqrt(dx*dx + dy*dy)
        ang = math.atan2(dx, dy) * 180.0 / math.pi
        is_tr = (dy <= 0.27 * abs_dx and abs_dx >= 16.0)
        
        parsed.append({'x_near': x_near, 'y_near': y_near, 'x_far': x_far, 'y_far': y_far, 'dx': dx, 'dy': dy, 'ang': ang, 'is_tr': is_tr})
        if is_tr:
            has_tr = 1
            if y_near > tr_y_near_max: tr_y_near_max = y_near
            if y_far < tr_y_far_min: tr_y_far_min = y_far
            min_x, max_x = min(x_near, x_far), max(x_near, x_far)
            if min_x < 35.0 and max_x > 43.0:
                cr_cnt = 1
        else:
            if dy >= 2.0 and l >= 5.0:
                sum_w_ang += ang * l
                sum_l += l
                valid_cnt += 1
                
    # Transverse union
    for j in range(count):
        if not parsed[j]['is_tr']: continue
        min_j, max_j = min(parsed[j]['x_near'], parsed[j]['x_far']), max(parsed[j]['x_near'], parsed[j]['x_far'])
        for k in range(j + 1, count):
            if not parsed[k]['is_tr']: continue
            if abs(parsed[j]['y_near'] - parsed[k]['y_near']) <= 6.0:
                min_k, max_k = min(parsed[k]['x_near'], parsed[k]['x_far']), max(parsed[k]['x_near'], parsed[k]['x_far'])
                if min_k <= max_j + 4.0 and min_j <= max_k + 4.0:
                    u_min = min(min_j, min_k)
                    u_max = max(max_j, max_k)
                    if u_min < 35.0 and u_max > 43.0:
                        cr_cnt = 1
                        
    # Exit pair
    has_exit = False
    if has_tr:
        for j in range(count):
            if parsed[j]['is_tr'] or parsed[j]['dy'] < 3.0 or abs(parsed[j]['ang']) > 45.0: continue
            if parsed[j]['y_far'] >= tr_y_far_min or parsed[j]['y_near'] > tr_y_near_max + 5.0: continue
            mid_j = (parsed[j]['x_near'] + parsed[j]['x_far']) * 0.5
            for k in range(j + 1, count):
                if parsed[k]['is_tr'] or parsed[k]['dy'] < 3.0 or abs(parsed[k]['ang']) > 45.0: continue
                if parsed[k]['y_far'] >= tr_y_far_min or parsed[k]['y_near'] > tr_y_near_max + 5.0: continue
                mid_k = (parsed[k]['x_near'] + parsed[k]['x_far']) * 0.5
                
                idx_l, idx_r = (j, k) if mid_j <= mid_k else (k, j)
                l_mid, r_mid = (mid_j, mid_k) if mid_j <= mid_k else (mid_k, mid_j)
                sep = r_mid - l_mid
                pair_c = (l_mid + r_mid) * 0.5
                if sep < 20.0 or sep > 45.0: continue
                if pair_c < 24.0 or pair_c > 54.0: continue
                
                s_l = parsed[idx_l]['dx'] / parsed[idx_l]['dy']
                s_r = parsed[idx_r]['dx'] / parsed[idx_r]['dy']
                x0_l = parsed[idx_l]['x_near'] + s_l * parsed[idx_l]['y_near']
                x0_r = parsed[idx_r]['x_near'] + s_r * parsed[idx_r]['y_near']
                denom = s_l - s_r
                if abs(denom) < 1e-3:
                    if abs(parsed[idx_l]['ang']) <= 15.0 and abs(parsed[idx_r]['ang']) <= 15.0:
                        y_v = -999.0
                        x_v = pair_c
                    else:
                        continue
                else:
                    y_v = (x0_l - x0_r) / denom
                    x_v = x0_l - s_l * y_v
                if y_v <= 5.0 and 15.0 <= x_v <= 60.0:
                    has_exit = True
                    break
            if has_exit: break
            
    is_ix = 0
    is_crn = 0
    if has_tr:
        if cr_cnt == 0 and has_exit:
            is_ix = 1
            is_crn = 0
        else:
            is_ix = 0
            is_crn = 1
    else:
        if sum_l > 0.1 and valid_cnt > 0:
            avg_ang = sum_w_ang / sum_l
            is_crn = 1 if abs(avg_ang) > 28.0 else 0
            
    h_ang = sum_w_ang / sum_l if sum_l > 0.1 else 0.0
    return is_ix, is_crn, has_tr, h_ang, valid_cnt

for idx in range(115, 140):
    t_found, t_cte, t_head, t_has_tr, t_cr_cnt = compute_top(frames[idx]['top'])
    f_is_ix, f_is_crn, f_has_tr, f_hang, f_cnt = compute_frn(frames[idx]['frn'])
    print(f"F{idx:3d}: TOP found={t_found} cte={t_cte:5.1f} head={t_head:5.1f} | FRN ix={f_is_ix} crn={f_is_crn} hang={f_hang:5.1f} | orig: {frames[idx]['telem'][:45]}")
