with open("scratch/run_data.txt") as f:
    lines = [l.strip() for l in f if l.startswith("L:")]

# Find all transitions where FV changes from 1 to 0
for i in range(1, len(lines)):
    prev = lines[i-1]
    curr = lines[i]
    if "FV:1" in prev and "FV:0" in curr:
        print(f"\n=== FV DROP AT LINE {i} ===")
        start = max(0, i-4)
        end = min(len(lines), i+6)
        for k in range(start, end):
            prefix = ">>> " if k == i else "    "
            print(f"{prefix}L{k:3d}: {lines[k]}")
