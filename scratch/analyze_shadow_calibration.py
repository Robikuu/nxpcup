import re
import sys

def analyze_calibration(log_filename):
    print(f"--- Analyzing Shadow Mode Calibration: {log_filename} ---")
    with open(log_filename, "r", encoding="utf-8", errors="ignore") as f:
        lines = [l.strip() for l in f if "TS:" in l and "FS:" in l]

    if not lines:
        print("No valid telemetry lines found with TS: and FS: keys!")
        return

    pattern = re.compile(
        r"L:(?P<top_v>\d+)/(?P<front_v>\d+)\s+"
        r"R:(?P<r_cte>-?\d+)\s+"
        r"F:(?P<f_cte>-?\d+)\s+"
        r"H:(?P<heading>-?\d+)\s+"
        r"TS:(?P<top_steer>-?\d+)\s+"
        r"FS:(?P<front_steer>-?\d+)\s+"
        r"S:(?P<servo_steer>-?\d+)\s+"
        r"BW:(?P<blend_weight>\d+)\s+"
        r"DOM:(?P<dom>[TFB])\s+"
        r"FV:(?P<front_valid>[01])\s+"
        r"IX:(?P<intersection>[01])\s+"
        r"ST:(?P<state>[NIL])"
    )

    records = []
    for l in lines:
        m = pattern.search(l)
        if m:
            records.append({
                'top_v': int(m.group('top_v')),
                'front_v': int(m.group('front_v')),
                'ts': float(m.group('top_steer')) / 10.0,
                'fs': float(m.group('front_steer')) / 10.0,
                's': float(m.group('servo_steer')) / 10.0,
                'fv': int(m.group('front_valid')),
                'ix': int(m.group('intersection')),
                'st': m.group('state')
            })

    total = len(records)
    print(f"Total parsed frames: {total}")
    if total == 0:
        return

    # Dual-valid frames: top >= 1, front >= 1 and fv == 1
    dual_valid = [r for r in records if r['top_v'] >= 1 and r['front_v'] >= 1 and r['fv'] == 1]
    print(f"Dual-valid frames: {len(dual_valid)} ({len(dual_valid)/total*100:.1f}%)")

    if not dual_valid:
        print("No dual-valid frames available for comparison.")
        return

    diffs = [abs(r['ts'] - r['fs']) for r in dual_valid]
    signed_diffs = [(r['fs'] - r['ts']) for r in dual_valid]
    mae = sum(diffs) / len(diffs)
    mean_bias = sum(signed_diffs) / len(signed_diffs)
    max_diff = max(diffs)

    # Direction agreement: do TS and FS steer in the same direction?
    agreed_dir = sum(1 for r in dual_valid if (r['ts'] * r['fs'] >= 0.0) or (abs(r['ts']) < 5.0 and abs(r['fs']) < 5.0))
    dir_pct = (agreed_dir / len(dual_valid)) * 100.0

    print("\n=== CALIBRATION METRICS ===")
    print(f"Mean Absolute Error (|FS - TS|): {mae:.2f} deg")
    print(f"Mean Signed Bias (FS - TS):      {mean_bias:+.2f} deg")
    print(f"Max Discrepancy:                 {max_diff:.2f} deg")
    print(f"Direction Agreement Rate:        {dir_pct:.1f}%")

    ix_triggers = sum(1 for r in records if r['ix'] == 1)
    print(f"Intersection triggers (IX=1):    {ix_triggers} frames")

    if mae < 15.0 and dir_pct >= 85.0:
        print("\n--> CALIBRATION RESULT: EXCELLENT! Front camera closely tracks Top camera.")
        print("--> Safe to proceed with Phase 1B (enabling live active blending).")
    else:
        print("\n--> CALIBRATION RESULT: GAIN TUNING RECOMMENDED before live blend.")

if __name__ == "__main__":
    fn = sys.argv[1] if len(sys.argv) > 1 else "scratch/telemetry_prebrake_run.txt"
    analyze_calibration(fn)
