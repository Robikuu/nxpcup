import re

with open(r"d:\nxpcup\scratch\full_live_test.txt", "r", encoding="utf-8") as f:
    text = f.read()

parts = text.split('OK state=RUNNING')

print("Number of runs:", len(parts) - 1)

for r_idx in range(1, len(parts)):
    lines = [l for l in parts[r_idx].splitlines() if l.startswith('L:')]
    print(f"\nRun {r_idx} total L: lines = {len(lines)}")
    if lines:
        print("  First 5 lines:")
        for l in lines[:5]:
            print("   ", l)
        print("  Last 5 lines:")
        for l in lines[-5:]:
            print("   ", l)
