import re, math

def trace_frame(vectors):
    if not vectors:
        return {'has_trans': False, 'crosses_center': False, 'has_exit_pair': False, 'reasons': ['no vectors'], 'parsed': []}
    count = len(vectors)
    parsed = []
    has_transverse_tape = False
    trans_crosses_center = False
    trans_y_near_max = -1.0
    trans_y_far_min  = 999.0

    sum_weighted_angle = 0.0
    sum_length = 0.0
    valid_count = 0

    for v in vectors:
        x0, y0, x1, y1 = v
        x_near, y_near = (x0, y0) if y0 >= y1 else (x1, y1)
        x_far, y_far   = (x1, y1) if y0 >= y1 else (x0, y0)
        dx = x_far - x_near
        dy = y_near - y_far
        length = math.sqrt(dx*dx + dy*dy)
        angle = math.atan2(dx, dy) * 180.0 / math.pi
        is_trans = (dy <= 0.27 * abs(dx) and abs(dx) >= 16.0)

        p = {
            'v': v,
            'x_near': x_near, 'y_near': y_near,
            'x_far': x_far, 'y_far': y_far,
            'dx': dx, 'dy': dy, 'len': length,
            'angle': angle, 'is_trans': is_trans
        }
        parsed.append(p)

        if is_trans:
            has_transverse_tape = True
            if y_near > trans_y_near_max: trans_y_near_max = y_near
            if y_far < trans_y_far_min:   trans_y_far_min = y_far
            min_x = min(x_near, x_far)
            max_x = max(x_near, x_far)
            if min_x <= 53.0 and max_x >= 25.0:
                trans_crosses_center = True
        else:
            if dy >= 2.0 and length >= 5.0:
                sum_weighted_angle += angle * length
                sum_length += length
                valid_count += 1

    # Check exit pair
    has_exit_pair = False
    reasons = []
    for j in range(count):
        if parsed[j]['is_trans'] or parsed[j]['dy'] < 3.0:
            continue
        if abs(parsed[j]['angle']) > 20.0:
            reasons.append(f"j={j} {parsed[j]['v']} angle={parsed[j]['angle']:.1f}>20")
            continue
        x_mid_j = (parsed[j]['x_near'] + parsed[j]['x_far']) * 0.5
        if x_mid_j < 15.0 or x_mid_j > 36.0:
            reasons.append(f"j={j} {parsed[j]['v']} x_mid={x_mid_j:.1f} not in [15,36]")
            continue
        if has_transverse_tape:
            if parsed[j]['y_far'] >= trans_y_far_min:
                reasons.append(f"j={j} {parsed[j]['v']} y_far={parsed[j]['y_far']} >= trans_y_far_min={trans_y_far_min}")
                continue
            if parsed[j]['y_near'] > (trans_y_near_max + 3.0):
                reasons.append(f"j={j} {parsed[j]['v']} y_near={parsed[j]['y_near']} > trans_y_near_max={trans_y_near_max}+3")
                continue

        for k in range(count):
            if j == k: continue
            if parsed[k]['is_trans'] or parsed[k]['dy'] < 3.0: continue
            if abs(parsed[k]['angle']) > 20.0: continue
            x_mid_k = (parsed[k]['x_near'] + parsed[k]['x_far']) * 0.5
            if x_mid_k < 42.0 or x_mid_k > 65.0: continue
            if has_transverse_tape:
                if parsed[k]['y_far'] >= trans_y_far_min: continue
                if parsed[k]['y_near'] > (trans_y_near_max + 3.0): continue
            sep = x_mid_k - x_mid_j
            if sep >= 22.0 and sep <= 48.0:
                has_exit_pair = True
                break
        if has_exit_pair: break

    return {
        'has_trans': has_transverse_tape,
        'crosses_center': trans_crosses_center,
        'has_exit_pair': has_exit_pair,
        'trans_y_near_max': trans_y_near_max,
        'trans_y_far_min': trans_y_far_min,
        'reasons': reasons,
        'parsed': parsed
    }

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

for i in range(110, 130):
    res = trace_frame(frames[i]['frn'])
    print(f"Frame {i:3d}: trans={res['has_trans']} center_cross={res['crosses_center']} exit_pair={res['has_exit_pair']}")
    print(f"   Telem: {frames[i]['telem']}")
    print(f"   FRN Vectors:")
    for p in res['parsed']:
        t_str = "TRANS" if p['is_trans'] else f"ang={p['angle']:5.1f} dy={p['dy']:4.1f}"
        print(f"       {p['v']} -> {t_str}, x_mid={(p['x_near']+p['x_far'])*0.5:.1f}, y=[{p['y_far']},{p['y_near']}]")
    if res['reasons']:
        print(f"   Reasons left exit failed: {res['reasons']}")
