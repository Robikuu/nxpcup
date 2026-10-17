import sys
sys.path.append(r'd:\nxpcup\scratch')
import analyze_latest_test
data = analyze_latest_test.data
print("=== TELEMETRY F115 TO F155 ===")
for d in data[115:155]:
    print(f"F{d['frame']:3d}: L={d['lines_t']}/{d['lines_f']} CTE={d['filt_cte']:+5.1f} H={d['heading']:+5.1f} TS={d['top_steer']:+5.1f} FS={d['front_steer']:+5.1f} S={d['steer']:+5.1f} BW={d['blend_w']:.2f} DOM={d['dom']}")
