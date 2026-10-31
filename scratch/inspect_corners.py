import re

with open("scratch/run_data.txt", "r", encoding="utf-8") as f:
    lines = f.readlines()

print("=== DETAILED CORNER ANALYSIS WHERE FV:1 WAS ACTIVE ===")

# In Run 1: Corner around L19..L25
# In Run 1: Corner around L37..L45
# In Run 3: Corner around L19..L25
# In Run 3: Corner around L38..L45

def inspect_lines(start, end, label):
    print(f"\n--- {label} (Lines {start}..{end}) ---")
    for i in range(start, end + 1):
        if i < len(lines):
            print(f"{i:3d}: {lines[i].strip()}")

inspect_lines(15, 27, "Run 1: First Corner (FV=1)")
inspect_lines(34, 46, "Run 1: Second Corner (FV=1)")
inspect_lines(115, 127, "Run 2: First Corner (FV=1)")
inspect_lines(155, 168, "Run 3: First Corner (FV=1)")
inspect_lines(174, 186, "Run 3: Second Corner (FV=1)")
