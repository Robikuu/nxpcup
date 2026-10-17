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

def solve_top_camera(vectors, filter_chords=True):
    if not vectors: return 0, 0.0, 0.0
    best_left_x = -1000.0
    best_left_angle = 0.0
    best_right_x = 1000.0
    best_right_angle = 0.0
    left_found = False
    right_found = False

    for v in vectors:
        x0, y0, x1, y1 = v
        x_near, y_near = (x0, y0) if y0 >= y1 else (x1, y1)
        x_far, y_far   = (x1, y1) if y0 >= y1 else (x0, y0)
        if y_near < 16.0: continue
        dy = y_near - y_far
        if dy < 4.0: continue
        dx = x_far - x_near
        ang = math.atan2(dx, dy) * 180.0 / math.pi

        # Chassis exclusion (tie rod artifact at x=40, y > 45)
        if 36 <= x_near <= 44 and y_near >= 45 and dy <= 8:
            continue

        if filter_chords:
            # Reject transverse tape OR diagonal crossroad chords (|dx| >= dy and dy < 10)
            if dy <= 0.27 * abs(dx) and abs(dx) >= 16.0:
                continue
            if abs(dx) >= dy and dy < 10.0:
                continue
            # Also reject lines with angle > 55 deg in top camera if short (dy < 12)
            if abs(ang) > 55.0 and dy < 12.0:
                continue

        if x_near < 39.0:
            if x_near > best_left_x:
                best_left_x = x_near
                best_left_angle = ang
                left_found = True
        elif x_near > 39.0:
            if x_near < best_right_x:
                best_right_x = x_near
                best_right_angle = ang
                right_found = True

    if left_found and right_found:
        cx = (best_left_x + best_right_x) * 0.5
        head = (best_left_angle + best_right_angle) * 0.5
        return 2, cx - 39.0, head
    elif left_found:
        cx = best_left_x + 19.5
        return 1, cx - 39.0, best_left_angle
    elif right_found:
        cx = best_right_x - 19.5
        return 1, cx - 39.0, best_right_angle
    else:
        return 0, 0.0, 0.0

print("Comparison of Top Camera heading before vs after chord filter:")
for i in range(114, 126):
    f_orig, cte_orig, h_orig = solve_top_camera(frames[i]['top'], filter_chords=False)
    f_new,  cte_new,  h_new  = solve_top_camera(frames[i]['top'], filter_chords=True)
    print(f"Frame {i:3d}:")
    print(f"   Original: found={f_orig} cte={cte_orig:5.1f} head={h_orig:5.1f}")
    print(f"   New     : found={f_new} cte={cte_new:5.1f} head={h_new:5.1f}")
    print(f"   Telem   : {frames[i]['telem']}")
