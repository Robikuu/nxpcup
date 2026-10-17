import re
import matplotlib.pyplot as plt

def parse_telemetry(lines):
    pattern = re.compile(
        r"L:(?P<lines_t>\d+)/(?P<lines_f>\d+)\s+"
        r"R:(?P<raw_cte>[-\d]+)\s+"
        r"F:(?P<filt_cte>[-\d]+)\s+"
        r"H:(?P<heading>[-\d]+)\s+"
        r"TS:(?P<top_steer>[-\d]+)\s+"
        r"FS:(?P<front_steer>[-\d]+)\s+"
        r"S:(?P<steer>[-\d]+)\s+"
        r"BW:(?P<blend_w>[-\d]+)\s+"
        r"DOM:(?P<dom>\w+)\s+"
        r"FV:(?P<front_val>\d+)\s+"
        r"IX:(?P<ix>\d+)\s+"
        r"ST:(?P<state>\w+)\s+"
        r"VL:(?P<vl>[-\d]+)\s+"
        r"VR:(?P<vr>[-\d]+)"
    )
    data = []
    for idx, l in enumerate(lines):
        m = pattern.search(l)
        if m:
            d = m.groupdict()
            row = {
                'frame': idx,
                'lines_t': int(d['lines_t']),
                'lines_f': int(d['lines_f']),
                'raw_cte': float(d['raw_cte']) / 10.0,
                'filt_cte': float(d['filt_cte']) / 10.0,
                'heading': float(d['heading']) / 10.0,
                'top_steer': float(d['top_steer']) / 10.0,
                'front_steer': float(d['front_steer']) / 10.0,
                'steer': float(d['steer']) / 10.0,
                'blend_w': float(d['blend_w']) / 100.0,
                'dom': d['dom'],
                'front_val': int(d['front_val']),
                'ix': int(d['ix']),
                'st': d['state'],
                'vl': float(d['vl']),
                'vr': float(d['vr']),
            }
            data.append(row)
    return data

with open(r'd:\nxpcup\scratch\new_run_telem.txt', 'r') as f:
    lines = [l for l in f if l.startswith('L:')]

data = parse_telemetry(lines)
print(f"Parsed {len(data)} frames")

# Print breakdown of key segments
print("\n=== DETAILED BREAKDOWN OF NEW RUN ===")
for d in data:
    f = d['frame']
    if f % 5 == 0 or abs(d['steer']) > 30 or d['ix'] == 1 or d['dom'] != 'T':
        print(f"F{f:3d}: L={d['lines_t']}/{d['lines_f']} CTE={d['filt_cte']:+5.1f} H={d['heading']:+5.1f} "
              f"TS={d['top_steer']:+5.1f} FS={d['front_steer']:+5.1f} S={d['steer']:+5.1f} "
              f"BW={d['blend_w']:.2f} DOM={d['dom']} IX={d['ix']} V=({d['vl']:.0f},{d['vr']:.0f})")
