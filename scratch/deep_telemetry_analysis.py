import re
import numpy as np

pattern = re.compile(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TJ:(-?\d+)\s+S:(-?\d+)\s+LA:(\d+)\s+ST:([FT])\s+VL:(\d+)\s+VR:(\d+)')

runs = []
current_run = []

with open('docs/debug_telemetry.md', 'r') as f:
    for line in f:
        if 'start;' in line or 'state=RUNNING' in line:
            if current_run:
                runs.append(current_run)
                current_run = []
        m = pattern.search(line)
        if m:
            top_v, front_v, r, f_val, h, tj, s, la, st, vl, vr = m.groups()
            current_run.append({
                'top_v': int(top_v),
                'front_v': int(front_v),
                'r': int(r) / 10.0,
                'f': int(f_val) / 10.0,
                'h': int(h) / 10.0,
                'tj': int(tj) / 10.0,
                's': int(s) / 10.0,
                'la': int(la),
                'st': st,
                'vl': int(vl),
                'vr': int(vr)
            })

if current_run:
    runs.append(current_run)

print(f"Total separate runs found: {len(runs)}")
for idx, r in enumerate(runs):
    print(f"Run {idx+1}: {len(r)} frames")

# Let's analyze Run 1 (the longest run, 120+ frames) and Run 2 (if any)
for idx, r in enumerate(runs):
    print(f"\n==========================================")
    print(f"  DETAILED ANALYSIS OF RUN {idx+1} ({len(r)} frames)")
    print(f"==========================================")
    
    st_t = sum(1 for x in r if x['st'] == 'T')
    st_f = sum(1 for x in r if x['st'] == 'F')
    print(f"ST:T = {st_t} ({st_t/len(r)*100:.1f}%), ST:F = {st_f} ({st_f/len(r)*100:.1f}%)")
    
    f_vals = np.array([x['f'] for x in r])
    r_vals = np.array([x['r'] for x in r])
    h_vals = np.array([x['h'] for x in r])
    la_vals = np.array([x['la'] for x in r])
    
    print("\n--- CTE Distribution (F = filtered_cte, R = raw_cte) ---")
    print(f"Raw CTE (R): mean={r_vals.mean():.2f}, std={r_vals.std():.2f}, min={r_vals.min():.1f}, max={r_vals.max():.1f}")
    print(f"Filtered CTE (F): mean={f_vals.mean():.2f}, std={f_vals.std():.2f}, min={f_vals.min():.1f}, max={f_vals.max():.1f}")
    print(f"|Filtered CTE| quantiles:")
    for q in [10, 25, 50, 75, 90]:
        print(f"  {q}%: {np.percentile(np.abs(f_vals), q):.2f} px")
    print(f"|Filtered CTE| <= 4.0 px: {np.mean(np.abs(f_vals) <= 4.0)*100:.1f}%")
    print(f"|Filtered CTE| <= 6.0 px: {np.mean(np.abs(f_vals) <= 6.0)*100:.1f}%")
    
    print("\n--- Heading Distribution (H = filtered_heading) ---")
    print(f"|Filtered Heading| <= 3.0 deg: {np.mean(np.abs(h_vals) <= 3.0)*100:.1f}%")
    print(f"|Filtered Heading| <= 4.0 deg: {np.mean(np.abs(h_vals) <= 4.0)*100:.1f}%")
    print(f"|Filtered Heading| <= 8.0 deg: {np.mean(np.abs(h_vals) <= 8.0)*100:.1f}%")
    
    print("\n--- Lookahead Urgency Distribution (LA = lookahead_turn_factor * 100) ---")
    print(f"LA == 60 (clamped): {np.mean(la_vals == 60)*100:.1f}%")
    print(f"LA <= 15: {np.mean(la_vals <= 15)*100:.1f}%")
    print(f"LA <= 25: {np.mean(la_vals <= 25)*100:.1f}%")
    print(f"LA >= 60: {np.mean(la_vals >= 60)*100:.1f}%")
    
    # Check Joint Conditions for Straight Confirmation:
    # Condition in main.c:
    # straight_confirmed = (|H| <= 4.0 && |CTE| <= 6.0 && la_offset <= 3.0px [approx LA <= 15])
    # User's mentioned condition from screenshot:
    # |H| <= 3.0 && |CTE| <= 4.0
    c_user = (np.abs(h_vals) <= 3.0) & (np.abs(f_vals) <= 4.0)
    c_main = (np.abs(h_vals) <= 4.0) & (np.abs(f_vals) <= 6.0)
    c_main_full = c_main & (la_vals <= 15)
    
    print("\n--- Joint Condition Match Rates ---")
    print(f"User screenshot sanity veto condition (|H|<=3.0 AND |CTE|<=4.0): {np.mean(c_user)*100:.1f}% ({np.sum(c_user)} frames)")
    print(f"main.c straight base check (|H|<=4.0 AND |CTE|<=6.0): {np.mean(c_main)*100:.1f}% ({np.sum(c_main)} frames)")
    print(f"Full main.c straight check (|H|<=4.0 AND |CTE|<=6.0 AND LA<=15): {np.mean(c_main_full)*100:.1f}% ({np.sum(c_main_full)} frames)")
