import sys
sys.path.insert(0, 'scratch')
from analyze_both_runs import d2

print("Frames 10 to 40 of Run 2:")
for i in range(10, 40):
    r = d2[i]
    st = r['st']
    cte = r['f_cte']
    h = r['h']
    tj = r['tj']
    la = r['la']
    s = r['s']
    vl = r['vl']
    vr = r['vr']
    print(f"R2[{i:2d}]: ST={st} CTE={cte:5.1f} H={h:5.1f} TJ={tj:5.1f} LA={la:3d}% Steer={s:5.1f} VL={vl} VR={vr}")
