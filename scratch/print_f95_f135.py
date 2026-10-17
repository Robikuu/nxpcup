import sys
sys.path.append(r'd:\nxpcup\scratch')
import analyze_latest_test

data = analyze_latest_test.data
print('Frame | Lines |   CTE |     H |    TS |    FS |     S |   BW | DOM | IX | V_L, V_R')
print('-'*75)
for d in data[95:135]:
    print(f"F{d['frame']:3d}   | {d['lines_t']}/{d['lines_f']}   | {d['filt_cte']:+5.1f} | {d['heading']:+5.1f} | {d['top_steer']:+5.1f} | {d['front_steer']:+5.1f} | {d['steer']:+5.1f} | {d['blend_w']:.2f} | {d['dom']}   |  {d['ix']} | {d['vl']:.0f},{d['vr']:.0f}")
