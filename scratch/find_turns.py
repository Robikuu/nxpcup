import analyze_latest_test

data = analyze_latest_test.data
print("Total frames:", len(data))

# Identify major turns where |steer| > 25 for at least 3 frames
in_turn = False
turn_start = 0
turns = []

for d in data:
    f = d['frame']
    s = d['steer']
    if abs(s) >= 20.0:
        if not in_turn:
            in_turn = True
            turn_start = f
    else:
        if in_turn:
            if f - turn_start >= 3:
                # Calculate mean steer in turn
                mean_s = sum(data[i]['steer'] for i in range(turn_start, f)) / (f - turn_start)
                turns.append((turn_start, f-1, mean_s))
            in_turn = False
if in_turn:
    turns.append((turn_start, len(data)-1, sum(data[i]['steer'] for i in range(turn_start, len(data))) / (len(data) - turn_start)))

print("Detected turns:")
for start, end, mean_s in turns:
    direction = "LEFT" if mean_s < 0 else "RIGHT"
    print(f"Frames {start:3d} to {end:3d} ({end-start+1:2d} frames): Mean Steer = {mean_s:+5.1f} deg ({direction})")
