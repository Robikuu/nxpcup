with open('scratch/full_user_msg.txt') as f:
    text = f.read()

runs = text.split("OK state=RUNNING")

for r_idx, run in enumerate(runs[1:], 1):
    lines = [l.strip() for l in run.split('\n') if l.strip().startswith('L:')]
    print(f"\n=================== FLIGHT RECORDER ANALYSIS: RUN {r_idx} ===================")
    # Find when it started losing control and when it froze
    for idx, line in enumerate(lines):
        # Extract R, F, H, TS, S, VL, VR
        # Format: L:2/3 R:40 F:40 H:-67 TS:-12 FS:1 S:0 BW:100 DOM:T FV:1 IX:0 ST:N VL:99 VR:99
        parts = line.split()
        d = {}
        for p in parts:
            if ":" in p:
                k, v = p.split(":", 1)
                d[k] = v
        r = float(d.get('R', 0)) / 10.0
        f = float(d.get('F', 0)) / 10.0
        h = float(d.get('H', 0)) / 10.0
        s = float(d.get('S', 0)) / 10.0
        vl = int(d.get('VL', 0))
        vr = int(d.get('VR', 0))
        
        # Look for the corner entry or abnormal behavior
        if abs(h) > 20.0 or abs(s) > 30.0:
            print(f"Frame {idx:3d}: CTE={r:+5.1f}px (filt {f:+5.1f}px) | Heading={h:+5.1f} deg | Steer={s:+5.1f} deg | Motors L={vl} R={vr}")
