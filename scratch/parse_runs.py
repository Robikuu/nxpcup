import json
import re

with open(r'd:\nxpcup\scratch\full_telemetry_2890.txt', 'r', encoding='utf-8') as f:
    lines = f.readlines()

runs = []
cur_run = []
cur_meta = []
in_run = False
last_shadow = 1 # default in status was 1

for l in lines:
    l_str = l.strip()
    if 'shadow 0;' in l_str:
        last_shadow = 0
    elif 'shadow 1;' in l_str:
        last_shadow = 1
    
    if 'start;' in l_str and 'Sent utf8' in l_str:
        in_run = True
        cur_run = []
    elif ('stop;' in l_str and 'Sent utf8' in l_str) or 'Opened the serial port' in l_str:
        if in_run:
            runs.append({'shadow': last_shadow, 'meta': cur_meta, 'frames': cur_run})
            in_run = False
            cur_run = []
    elif in_run:
        if l_str.startswith('L:'):
            cur_run.append(l_str)
    else:
        if l_str:
            cur_meta.append(l_str)
        if len(cur_meta) > 20:
            cur_meta = cur_meta[-20:]

if in_run and cur_run:
    runs.append({'shadow': last_shadow, 'meta': cur_meta, 'frames': cur_run})

print(f"Total runs found: {len(runs)}")
for idx, r in enumerate(runs):
    frames = r['frames']
    print(f"\n--- Run {idx+1} (Shadow mode = {r['shadow']}, Frame count = {len(frames)}) ---")
    if frames:
        print("First frame:", frames[0])
        print("Frame 10:   ", frames[min(9, len(frames)-1)])
        print("Last frame: ", frames[-1])
