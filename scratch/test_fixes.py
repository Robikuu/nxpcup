from analyze_new_run import records

print('Frame | Orig Steer | New Steer (GainSched + ApexProt + LookaheadFF)')
print('-'*70)

for r in records[10:20]:
    cte = float(r['R']) / 10.0
    fcte = float(r['F']) / 10.0
    h = float(r['H']) / 10.0
    tj = float(r['TJ']) / 10.0
    s_orig = float(r['S']) / 10.0
    la = int(r['LA']) / 100.0
    
    # 1. Curvature kappa
    abs_h = abs(h)
    k_base = (abs_h - 2.5) / (25.0 - 2.5) if abs_h > 2.5 else 0.0
    k_base = min(1.0, max(0.0, k_base))
    kappa = max(k_base, la)
    
    # 2. Dynamic Gain Scheduling:
    kp = 0.75 + kappa * (2.60 - 0.75)
    
    # 3. Base Heading FF
    eff_h = (abs_h - 2.0) * (1.0 if h > 0 else -1.0) if abs_h > 2.0 else 0.0
    heading_ff = 1.70 * eff_h
    
    # 4. Front Lookahead FF (k_la = 0.35, deadband = 3.0 deg)
    abs_tj = abs(tj)
    eff_tj = (abs_tj - 3.0) * (1.0 if tj > 0 else -1.0) if abs_tj > 3.0 else 0.0
    lookahead_ff = 0.35 * eff_tj
    
    # 5. PID CTE with dynamic Kp
    p_cte = kp * 0.60 * fcte
    
    # 6. Turn direction
    turn_dir = 0.0
    if abs_h > 3.0:
        turn_dir = h
    elif abs_tj > 5.0:
        turn_dir = tj
        
    # Apex attenuation if p_cte opposes turn_dir
    if turn_dir != 0.0 and (p_cte * turn_dir < 0.0):
        atten = 1.0 - 0.75 * kappa
        p_cte *= atten
        
    new_steer = p_cte + heading_ff + lookahead_ff
    print(f"Frame {r['idx']:02d}: Orig={s_orig:+5.1f} -> New={new_steer:+5.1f} | kp={kp:.2f} P_cte={p_cte:+4.1f} H_ff={heading_ff:+5.1f} LA_ff={lookahead_ff:+5.1f}")
