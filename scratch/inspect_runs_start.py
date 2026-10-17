with open('docs/debug_telemetry.md') as f:
    text = f.read()

runs = text.split('OK state=RUNNING')[1:]
for r_i, r in enumerate(runs):
    lines = [l for l in r.split('\n') if l.startswith('L:')]
    print(f"Run {r_i+1}: {len(lines)} frames")
    if lines:
        print("  First 5 frames:")
        for l in lines[:5]:
            print("   ", l)
