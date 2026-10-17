with open(r'd:\nxpcup\scratch\latest_telemetry.txt', 'r', encoding='utf-16', errors='ignore') as f:
    text = f.read()
lines = [l.strip() for l in text.splitlines() if l.strip().startswith('L:')]
print(f"Total lines: {len(lines)}")
for i, l in enumerate(lines):
    print(f"{i:02d}: {l}")
