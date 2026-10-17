import re

with open("scratch/telemetry_prebrake_run.txt") as f:
    lines = [line.strip() for line in f if line.strip().startswith("L:")]

pattern = re.compile(r"L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TJ:(-?\d+)\s+S:(-?\d+)\s+LA:(\d+)\s+ST:(\w+)\s+VL:(\d+)\s+VR:(\d+)")

top_counts = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0}
front_counts = {0: 0, 1: 0, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0}

for line in lines:
    m = pattern.search(line)
    if m:
        t = int(m.group(1))
        f = int(m.group(2))
        top_counts[t] = top_counts.get(t, 0) + 1
        front_counts[f] = front_counts.get(f, 0) + 1

total = len(lines)
print(f"Total frames: {total}")
print("\nTop Camera Vector Count Distribution:")
for k in sorted(top_counts.keys()):
    print(f"  {k} vectors: {top_counts[k]:3d} frames ({top_counts[k]/total*100:5.1f}%)")

print("\nFront Camera Vector Count Distribution:")
for k in sorted(front_counts.keys()):
    print(f"  {k} vectors: {front_counts[k]:3d} frames ({front_counts[k]/total*100:5.1f}%)")

