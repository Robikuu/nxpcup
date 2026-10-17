import re

with open('scratch/user_multi_run_telemetry.txt', 'r', encoding='utf-8') as f:
    text = f.read()

# Split by 'OK state=RUNNING'
parts = text.split('OK state=RUNNING')
print(f"Total runs started: {len(parts) - 1}")

configs = [
    "Run 1: k_head=1.70, shadow=1, spd_s=95",
    "Run 2: k_head=2.00, shadow=1, spd_s=95",
    "Run 3: k_head=2.00, shadow=0, spd_s=95",
    "Run 4: k_head=2.00, shadow=1, spd_s=95",
    "Run 5: k_head=2.00, shadow=1, spd_s=95 (restart)",
    "Run 6: k_head=2.00, shadow=1, spd_s=100",
    "Run 7: k_head=1.80, shadow=1, spd_s=100",
]

for idx in range(1, len(parts)):
    part = parts[idx]
    end_m = re.search(r'---- Sent utf8 encoded message:', part)
    if end_m:
        run_text = part[:end_m.start()]
    else:
        run_text = part
    
    lines = [l.strip() for l in run_text.splitlines() if l.strip().startswith('L:')]
    cfg_name = configs[idx-1] if idx-1 < len(configs) else f"Run {idx}"
    print(f"\n=======================================================")
    print(f"=== {cfg_name} ===")
    print(f"=======================================================")
    print(f"Frames: {len(lines)} | Duration: {len(lines)*0.08:.2f} seconds ({len(lines)*4} ticks)")
    
    vl_list = []
    vr_list = []
    s_list = []
    h_list = []
    found_top = []
    st_list = []
    
    for l in lines:
        m = re.search(r'L:(\d+)/(\d+).*?H:(-?\d+).*?S:(-?\d+).*?ST:([A-Z]).*?VL:(\d+)\s+VR:(\d+)', l)
        if m:
            found_top.append(int(m.group(1)))
            h_list.append(int(m.group(3)) / 10.0)
            s_list.append(int(m.group(4)) / 10.0)
            st_list.append(m.group(5))
            vl_list.append(int(m.group(6)))
            vr_list.append(int(m.group(7)))
    
    if vl_list:
        max_v = max(max(vl_list), max(vr_list))
        min_v = min(min(vl_list), min(vr_list))
        avg_v = sum(vl_list + vr_list) / (2 * len(vl_list))
        sprint_frames = sum(1 for vl, vr in zip(vl_list, vr_list) if vl >= 90 and vr >= 90)
        diff_frames = sum(1 for vl, vr in zip(vl_list, vr_list) if abs(vl - vr) > 5)
        blind_frames = sum(1 for f in found_top if f == 0)
        
        # Straightaway weaving analysis (when |S| < 12)
        straight_steer = [s for s in s_list if abs(s) < 12.0]
        max_straight_steer = max([abs(s) for s in straight_steer]) if straight_steer else 0
        
        print(f"  Max Speed: {max_v}% | Min Speed: {min_v}% | Avg Speed: {avg_v:.1f}%")
        print(f"  Sprints (>=90% both wheels): {sprint_frames} frames ({sprint_frames/len(vl_list)*100:.1f}%)")
        print(f"  Differential Drag Active: {diff_frames} frames ({diff_frames/len(vl_list)*100:.1f}%)")
        print(f"  Top Camera Line Blind: {blind_frames} frames ({blind_frames/len(found_top)*100:.1f}%)")
        
        # State breakdown
        normal_cnt = sum(1 for st in st_list if st == 'N')
        ix_cnt = sum(1 for st in st_list if st == 'I')
        lost_cnt = sum(1 for st in st_list if st == 'L')
        print(f"  Macro States: Normal={normal_cnt}, Intersection={ix_cnt}, LineLost={lost_cnt}")
