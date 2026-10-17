with open('docs/debug_telemetry.md') as f:
    text = f.read()

import re
pattern = re.compile(r'VL:(\d+)\s+VR:(\d+)')
speeds = [max(int(m.group(1)), int(m.group(2))) for m in pattern.finditer(text)]
print(f"Total speed entries in debug_telemetry.md: {len(speeds)}")
print(f"Max speed ever recorded in debug_telemetry.md: {max(speeds)}%")
print(f"Min speed ever recorded: {min(speeds)}%")
print(f"Distribution:")
for threshold in [100, 90, 80, 70, 65, 60, 50, 40]:
    cnt = sum(1 for s in speeds if s >= threshold)
    print(f"  Speed >= {threshold}%: {cnt} ({cnt/len(speeds)*100:.1f}%)")
