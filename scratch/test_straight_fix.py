import re
import numpy as np

pattern = re.compile(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TJ:(-?\d+)\s+S:(-?\d+)\s+LA:(\d+)\s+ST:([FT])\s+VL:(\d+)\s+VR:(\d+)')

with open('docs/debug_telemetry.md', 'r') as f:
    text = f.read()

# Let's inspect Run 6 (the clean straight from debug_telemetry.md)
# and the user's latest full lap from the prompt
lines_lap = []
for line in text.split('\n'):
    m = pattern.search(line)
    if m:
        top_v, front_v, r, f_val, h, tj, s, la, st, vl, vr = m.groups()
        lines_lap.append({
            'top_v': int(top_v),
            'front_v': int(front_v),
            'r': int(r) / 10.0,
            'f': int(f_val) / 10.0,
            'h': int(h) / 10.0,
            'tj': int(tj) / 10.0,
            's': int(s) / 10.0,
            'la': int(la),
            'st': st,
            'vl': int(vl),
            'vr': int(vr)
        })

print(f"Total lines in debug_telemetry.md: {len(lines_lap)}")

# Let's see the correlation between F (filtered CTE) and LA (lookahead) on straight segments
straights = [x for x in lines_lap if abs(x['h']) <= 4.0]
print(f"Frames with |H| <= 4.0: {len(straights)}")
f_straight = np.array([x['f'] for x in straights])
la_straight = np.array([x['la'] for x in straights])
tj_straight = np.array([x['tj'] for x in straights])

print(f"Straight frames CTE stats: mean={f_straight.mean():.2f}, std={f_straight.std():.2f}, median={np.median(np.abs(f_straight)):.2f}")
print(f"Straight frames |CTE| <= 4.0px: {np.mean(np.abs(f_straight) <= 4.0)*100:.1f}%")
print(f"Straight frames |CTE| <= 6.0px: {np.mean(np.abs(f_straight) <= 6.0)*100:.1f}%")
print(f"Straight frames |CTE| <= 8.0px: {np.mean(np.abs(f_straight) <= 8.0)*100:.1f}%")
print(f"Straight frames LA stats: mean={la_straight.mean():.2f}, median={np.median(la_straight):.2f}")
print(f"Straight frames TJ (trajectory angle) stats: mean={tj_straight.mean():.2f}, median={np.median(np.abs(tj_straight)):.2f}")
