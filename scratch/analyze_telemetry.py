import re

def parse_runs(filename):
    with open(filename, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    runs = []
    current_run = {'header': [], 'lines': []}
    
    for line in lines:
        line = line.strip()
        if not line:
            continue
        if line.startswith('--- RUN') or line.startswith('=== RUN'):
            if current_run['lines']:
                runs.append(current_run)
                current_run = {'header': [], 'lines': []}
            continue
        if line.startswith('spd_') or line.startswith('OK speed') or line.startswith('OK state=RUNNING') or '=' in line and not line.startswith('L:'):
            current_run['header'].append(line)
        elif line.startswith('L:'):
            # Parse telemetry line
            # e.g.: L:2/3 R:-17 F:-16 H:-17 TS:-10 FS:10 S:0 BL:0 BT:0 DOM:T FV:1 IX:0 ST:S VL:100 VR:100
            m = re.match(r'L:(\d+)/(\d+)\s+R:([-\d]+)\s+F:([-\d]+)\s+H:([-\d]+)\s+TS:([-\d]+)\s+FS:([-\d]+)\s+S:([-\d]+)\s+BL:([-\d]+)\s+BT:([-\d]+)\s+DOM:([TF])\s+FV:(\d+)\s+IX:(\d+)\s+ST:([SCI])\s+VL:([-\d]+)\s+VR:([-\d]+)', line)
            if m:
                current_run['lines'].append({
                    'raw': line,
                    'top_found': int(m.group(1)),
                    'front_found': int(m.group(2)),
                    'R': float(m.group(3)) / 10.0,
                    'F': float(m.group(4)) / 10.0,
                    'H': float(m.group(5)) / 10.0,
                    'TS': float(m.group(6)) / 10.0,
                    'FS': float(m.group(7)) / 10.0,
                    'S': float(m.group(8)) / 10.0,
                    'BL': int(m.group(9)),
                    'BT': int(m.group(10)),
                    'DOM': m.group(11),
                    'FV': int(m.group(12)),
                    'IX': int(m.group(13)),
                    'ST': m.group(14),
                    'VL': int(m.group(15)),
                    'VR': int(m.group(16))
                })
        elif line == 'OK state=IDLE':
            if current_run['lines']:
                runs.append(current_run)
                current_run = {'header': [], 'lines': []}

    if current_run['lines']:
        runs.append(current_run)
        
    return runs

runs = parse_runs("scratch/run_data.txt")
print(f"Total runs parsed: {len(runs)}")

for idx, r in enumerate(runs):
    print(f"\n=================== RUN {idx+1} (Total lines: {len(r['lines'])}) ===================")
    print("Header config:")
    for h in r['header']:
        if any(k in h for k in ['spd_out', 'spd_in', 'fa_th', 'k_head', 'k_cte', 'speed_']):
            print("  ", h)
            
    fv_count = sum(1 for l in r['lines'] if l['FV'] > 0)
    print(f"Frames with FV > 0: {fv_count} / {len(r['lines'])} ({fv_count/len(r['lines'])*100:.1f}%)")
    
    # Let's inspect where FV drops to 0
    fv_transitions = []
    for i in range(1, len(r['lines'])):
        if r['lines'][i-1]['FV'] != r['lines'][i]['FV']:
            fv_transitions.append((i, r['lines'][i-1]['FV'], r['lines'][i]['FV']))
    print(f"FV transitions (line_idx, old -> new): {fv_transitions}")
    
    # Let's inspect corner entries: where ST becomes 'C' or |H| > 15
    print("\nCorner Events (ST:C or |H| > 15 deg):")
    in_corner = False
    corner_start = 0
    for i, l in enumerate(r['lines']):
        is_c = (l['ST'] == 'C' or abs(l['H']) >= 15.0)
        if is_c and not in_corner:
            in_corner = True
            corner_start = i
        elif not is_c and in_corner:
            in_corner = False
            # print corner summary
            c_lines = r['lines'][corner_start:i]
            max_h = max(abs(x['H']) for x in c_lines)
            max_s = max(abs(x['S']) for x in c_lines)
            min_vl = min(x['VL'] for x in c_lines)
            min_vr = min(x['VR'] for x in c_lines)
            first_c = next((j for j, x in enumerate(c_lines) if x['ST'] == 'C'), None)
            first_c_str = f"+{first_c*80}ms" if first_c is not None else "Never ST:C"
            print(f"  Event lines {corner_start:3d}..{i-1:3d} (len {len(c_lines):2d}): max|H|={max_h:5.1f} max|S|={max_s:5.1f} min(VL,VR)=({min_vl:3d},{min_vr:3d}) First ST:C: {first_c_str} (line {corner_start + (first_c or 0)})")
            # print first 3 lines of this corner
            for k in range(corner_start, min(corner_start+4, len(r['lines']))):
                x = r['lines'][k]
                print(f"    L{k:3d}: H={x['H']:5.1f} FS={x['FS']:5.1f} TS={x['TS']:5.1f} S={x['S']:5.1f} ST={x['ST']} DOM={x['DOM']} FV={x['FV']} VL={x['VL']} VR={x['VR']}")
