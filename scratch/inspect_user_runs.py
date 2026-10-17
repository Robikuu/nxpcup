# Let's inspect the telemetry leading up to the freeze in Run 1, Run 2, Run 3
from parse_all_user_runs import raw_text

runs = raw_text.split("OK state=RUNNING")
print(f"Number of runs: {len(runs)-1}")

for r_idx, run in enumerate(runs[1:], 1):
    lines = [l.strip() for l in run.split("\n") if l.strip().startswith("L:")]
    print(f"\n--- RUN {r_idx} (Total lines: {len(lines)}) ---")
    if not lines:
        continue
    print("Start (first 5 lines):")
    for l in lines[:5]:
        print("  ", l)
    print("End (last 10 lines):")
    for l in lines[-10:]:
        print("  ", l)
