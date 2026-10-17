raw_telemetry = """
L:2/3 R:-5 F:-6 H:29 TJ:36 S:5 LA:1 ST:N VL:99 VR:99
L:2/3 R:-5 F:-5 H:28 TJ:34 S:6 LA:1 ST:N VL:99 VR:99
L:2/3 R:-5 F:-5 H:22 TJ:24 S:-2 LA:0 ST:N VL:100 VR:100
L:2/3 R:0 F:-1 H:8 TJ:22 S:2 LA:0 ST:N VL:100 VR:100
L:2/3 R:0 F:0 H:2 TJ:7 S:1 LA:0 ST:N VL:100 VR:100
L:2/0 R:0 F:0 H:-3 TJ:6 S:0 LA:0 ST:N VL:100 VR:100
L:2/3 R:0 F:0 H:-10 TJ:3 S:0 LA:0 ST:N VL:100 VR:100
L:0/3 R:0 F:-3 H:-2 TJ:5 S:-8 LA:0 ST:N VL:100 VR:100
L:2/3 R:-5 F:-4 H:8 TJ:9 S:-9 LA:0 ST:N VL:100 VR:100
L:2/3 R:-5 F:-4 H:24 TJ:10 S:0 LA:0 ST:N VL:100 VR:100
L:2/4 R:5 F:2 H:31 TJ:25 S:30 LA:3 ST:N VL:98 VR:98
L:2/3 R:15 F:11 H:46 TJ:7 S:74 LA:9 ST:N VL:96 VR:91
L:2/2 R:35 F:28 H:50 TJ:-38 S:114 LA:11 ST:N VL:95 VR:86
L:2/3 R:50 F:44 H:54 TJ:-16 S:149 LA:65 ST:N VL:72 VR:65
L:2/3 R:60 F:57 H:-1 TJ:-67 S:106 LA:98 ST:N VL:58 VR:56
L:2/2 R:60 F:61 H:-102 TJ:-290 S:-48 LA:99 ST:N VL:57 VR:58
L:2/1 R:50 F:56 H:-192 TJ:-567 S:-265 LA:99 ST:N VL:52 VR:58
L:1/0 R:85 F:78 H:-353 TJ:-531 S:-567 LA:100 ST:N VL:48 VR:58
L:0/1 R:0 F:69 H:-336 TJ:-682 S:-537 LA:100 ST:N VL:48 VR:58
L:0/1 R:0 F:69 H:-336 TJ:-670 S:-537 LA:0 ST:L VL:48 VR:48
L:0/1 R:0 F:35 H:-252 TJ:-352 S:-537 LA:0 ST:L VL:48 VR:48
L:0/1 R:0 F:35 H:-252 TJ:-537 S:-537 LA:0 ST:L VL:48 VR:48
L:0/1 R:0 F:35 H:-252 TJ:-440 S:-537 LA:0 ST:L VL:48 VR:48
L:0/1 R:0 F:35 H:-252 TJ:-322 S:-435 LA:0 ST:L VL:0 VR:0
L:1/1 R:145 F:117 H:-344 TJ:-219 S:-285 LA:0 ST:L VL:0 VR:0
L:1/1 R:165 F:174 H:-281 TJ:-139 S:-443 LA:100 ST:N VL:48 VR:58
L:1/1 R:175 F:178 H:-200 TJ:-70 S:-226 LA:77 ST:N VL:58 VR:67
L:1/1 R:205 F:191 H:-138 TJ:-41 S:3 LA:50 ST:N VL:78 VR:78
L:1/3 R:215 F:211 H:-54 TJ:-9 S:291 LA:15 ST:N VL:93 VR:65
L:1/4 R:215 F:214 H:20 TJ:0 S:344 LA:8 ST:N VL:96 VR:60
L:1/2 R:165 F:187 H:41 TJ:66 S:300 LA:12 ST:N VL:94 VR:64
L:1/1 R:115 F:139 H:-11 TJ:73 S:160 LA:21 ST:N VL:90 VR:77
L:1/1 R:45 F:72 H:-151 TJ:7 S:-204 LA:56 ST:N VL:64 VR:76
L:1/0 R:-35 F:-12 H:-207 TJ:14 S:-437 LA:81 ST:N VL:48 VR:65
L:1/0 R:-85 F:-64 H:-244 TJ:5 S:-565 LA:97 ST:N VL:48 VR:58
L:1/0 R:-105 F:-96 H:-297 TJ:2 S:-677 LA:100 ST:N VL:48 VR:58
L:0/0 R:0 F:-103 H:-278 TJ:0 S:-630 LA:0 ST:L VL:48 VR:48
L:0/1 R:0 F:-103 H:-278 TJ:113 S:-630 LA:0 ST:L VL:48 VR:48
L:1/1 R:-135 F:-135 H:199 TJ:188 S:-500 LA:77 ST:N VL:48 VR:66
L:1/1 R:-135 F:-135 H:178 TJ:251 S:19 LA:99 ST:N VL:58 VR:58
L:1/2 R:-125 F:-131 H:220 TJ:280 S:305 LA:99 ST:N VL:58 VR:51
L:1/3 R:-115 F:-119 H:258 TJ:261 S:405 LA:100 ST:N VL:58 VR:49
L:0/4 R:0 F:-97 H:256 TJ:260 S:401 LA:100 ST:N VL:58 VR:49
L:1/2 R:5 F:-17 H:147 TJ:334 S:246 LA:100 ST:N VL:58 VR:52
L:1/3 R:-5 F:-1 H:89 TJ:340 S:147 LA:99 ST:N VL:58 VR:55
L:1/1 R:-55 F:-37 H:91 TJ:303 S:42 LA:99 ST:N VL:58 VR:57
L:1/1 R:-105 F:-87 H:96 TJ:265 S:-42 LA:99 ST:N VL:57 VR:58
L:1/1 R:-125 F:-116 H:101 TJ:219 S:-58 LA:99 ST:N VL:57 VR:58
L:1/1 R:-125 F:-123 H:99 TJ:192 S:-52 LA:92 ST:N VL:60 VR:61
L:1/1 R:-115 F:-118 H:95 TJ:178 S:-38 LA:81 ST:N VL:65 VR:65
L:1/1 R:-95 F:-100 H:68 TJ:145 S:-58 LA:69 ST:N VL:69 VR:70
L:1/1 R:-85 F:-92 H:53 TJ:117 S:-73 LA:55 ST:N VL:74 VR:76
L:1/1 R:-75 F:-81 H:22 TJ:66 S:-108 LA:30 ST:N VL:80 VR:87
L:1/1 R:-65 F:-71 H:-37 TJ:6 S:-127 LA:16 ST:N VL:83 VR:93
L:1/1 R:-55 F:-59 H:-85 TJ:-35 S:-187 LA:26 ST:N VL:73 VR:88
L:1/1 R:-55 F:-55 H:-107 TJ:-27 S:-227 LA:36 ST:N VL:67 VR:84
L:1/1 R:-55 F:-55 H:-115 TJ:-5 S:-244 LA:40 ST:N VL:65 VR:83
L:1/1 R:-45 F:-47 H:-127 TJ:-19 S:-247 LA:45 ST:N VL:64 VR:80
L:1/1 R:-45 F:-45 H:-120 TJ:-43 S:-237 LA:42 ST:N VL:65 VR:82
L:1/1 R:-35 F:-37 H:-119 TJ:-52 S:-218 LA:42 ST:N VL:67 VR:82
L:1/1 R:-25 F:-29 H:-95 TJ:-10 S:-162 LA:31 ST:N VL:74 VR:86
L:1/1 R:-15 F:-16 H:-63 TJ:41 S:-86 LA:16 ST:N VL:87 VR:92
L:1/1 R:-5 F:-4 H:-28 TJ:108 S:-7 LA:53 ST:N VL:77 VR:77
L:1/2 R:5 F:3 H:4 TJ:143 S:16 LA:78 ST:N VL:67 VR:67
L:1/1 R:-5 F:-2 H:39 TJ:172 S:24 LA:98 ST:N VL:58 VR:58
L:1/2 R:-25 F:-16 H:55 TJ:184 S:18 LA:99 ST:N VL:58 VR:58
L:1/1 R:-35 F:-30 H:58 TJ:194 S:0 LA:99 ST:N VL:58 VR:58
L:1/1 R:-45 F:-42 H:56 TJ:225 S:-20 LA:99 ST:N VL:58 VR:58
L:1/1 R:-65 F:-59 H:54 TJ:207 S:-54 LA:99 ST:N VL:57 VR:58
L:1/1 R:-85 F:-75 H:55 TJ:180 S:-80 LA:95 ST:N VL:58 VR:59
L:1/2 R:-95 F:-90 H:66 TJ:180 S:-83 LA:92 ST:N VL:59 VR:61
L:1/1 R:-95 F:-94 H:76 TJ:128 S:-60 LA:79 ST:N VL:65 VR:66
L:1/1 R:-95 F:-94 H:88 TJ:140 S:-27 LA:88 ST:N VL:63 VR:63
L:1/1 R:-95 F:-94 H:99 TJ:96 S:3 LA:78 ST:N VL:66 VR:66
L:1/1 R:-75 F:-82 H:82 TJ:83 S:-7 LA:66 ST:N VL:72 VR:72
L:1/2 R:-65 F:-69 H:62 TJ:25 S:-20 LA:41 ST:N VL:82 VR:82
L:1/1 R:-35 F:-46 H:11 TJ:29 S:-45 LA:21 ST:N VL:89 VR:90
L:0/2 R:0 F:-31 H:-35 TJ:-5 S:-53 LA:11 ST:N VL:93 VR:95
L:1/2 R:-15 F:-17 H:-134 TJ:-98 S:-203 LA:48 ST:N VL:66 VR:79
L:1/2 R:-25 F:-23 H:-158 TJ:-28 S:-273 LA:59 ST:N VL:59 VR:75
L:1/1 R:-55 F:-45 H:-156 TJ:-1 S:-322 LA:58 ST:N VL:56 VR:75
L:1/1 R:-65 F:-57 H:-151 TJ:0 S:-332 LA:56 ST:N VL:56 VR:76
L:1/0 R:-65 F:-64 H:-141 TJ:0 S:-316 LA:51 ST:N VL:57 VR:78
L:1/1 R:-65 F:-64 H:-112 TJ:0 S:-261 LA:38 ST:N VL:64 VR:83
L:1/3 R:-55 F:-58 H:-32 TJ:75 S:-107 LA:13 ST:N VL:86 VR:94
L:1/3 R:-35 F:-45 H:23 TJ:107 S:-47 LA:15 ST:N VL:92 VR:93
L:1/5 R:-25 F:-33 H:37 TJ:471 S:-5 LA:80 ST:N VL:66 VR:66
L:1/0 R:-25 F:-26 H:57 TJ:617 S:34 LA:84 ST:N VL:64 VR:64
L:1/5 R:-35 F:-29 H:64 TJ:500 S:28 LA:99 ST:N VL:58 VR:58
L:1/5 R:-15 F:-25 H:243 TJ:228 S:263 LA:97 ST:N VL:59 VR:53
L:1/5 R:15 F:-9 H:220 TJ:194 S:379 LA:0 ST:L VL:58 VR:49
L:2/4 R:-60 F:-41 H:106 TJ:552 S:67 LA:99 ST:N VL:58 VR:57
L:2/3 R:-115 F:-99 H:20 TJ:173 S:-218 LA:84 ST:N VL:57 VR:64
L:1/3 R:265 F:119 H:10 TJ:-195 S:189 LA:99 ST:N VL:58 VR:54
L:2/3 R:225 F:208 H:-209 TJ:-55 S:-74 LA:82 ST:N VL:63 VR:65
L:2/2 R:210 F:155 H:-215 TJ:-3 S:-269 LA:84 ST:N VL:55 VR:64
L:2/2 R:200 F:196 H:-242 TJ:-79 S:-362 LA:96 ST:N VL:50 VR:59
L:2/2 R:185 F:185 H:-186 TJ:-232 S:-172 LA:74 ST:N VL:61 VR:68
L:2/3 R:200 F:193 H:-107 TJ:-26 S:113 LA:43 ST:N VL:81 VR:75
L:2/3 R:-180 F:-130 H:73 TJ:82 S:-358 LA:26 ST:N VL:56 VR:88
L:2/3 R:-155 F:-155 H:119 TJ:51 S:-88 LA:42 ST:N VL:77 VR:82
L:2/3 R:-140 F:-142 H:118 TJ:0 S:-17 LA:41 ST:N VL:82 VR:82
L:2/3 R:-110 F:-119 H:136 TJ:44 S:81 LA:49 ST:N VL:79 VR:75
L:2/3 R:-75 F:-87 H:113 TJ:-4 S:76 LA:39 ST:N VL:83 VR:79
L:2/2 R:-35 F:-49 H:27 TJ:37 S:-15 LA:13 ST:N VL:94 VR:94
L:2/2 R:-10 F:-19 H:-53 TJ:-544 S:-44 LA:97 ST:N VL:58 VR:58
L:1/2 R:105 F:73 H:-135 TJ:-709 S:-48 LA:99 ST:N VL:57 VR:58
L:1/2 R:125 F:61 H:-187 TJ:-350 S:-221 LA:84 ST:N VL:56 VR:64
L:1/2 R:105 F:101 H:310 TJ:-526 S:298 LA:100 ST:N VL:58 VR:51
L:1/2 R:135 F:120 H:239 TJ:-478 S:525 LA:98 ST:N VL:58 VR:48
L:1/2 R:215 F:171 H:317 TJ:-520 S:525 LA:100 ST:N VL:58 VR:48
"""

lines = raw_telemetry.strip().split('\n')
records = []
for i, l in enumerate(lines):
    parts = l.split()
    d = {'idx': i}
    for p in parts:
        if ':' in p:
            k, v = p.split(':', 1)
            d[k] = v
    records.append(d)

print("Total frames:", len(records))
print("Idx | R(CTE) | F(filtCTE) | H(deg) | TJ(deg) | SteerCmd | LA(%) | VL  VR | ST")
print("-" * 75)
for r in records[:30]:
    cte = float(r.get('R', 0)) / 10.0
    fcte = float(r.get('F', 0)) / 10.0
    h = float(r.get('H', 0)) / 10.0
    tj = float(r.get('TJ', 0)) / 10.0
    s = float(r.get('S', 0)) / 10.0
    la = int(r.get('LA', 0))
    vl = int(r.get('VL', 0))
    vr = int(r.get('VR', 0))
    st = r.get('ST', '')
    print(f"{r['idx']:02d}  | {cte:+6.1f} | {fcte:+10.1f} | {h:+6.1f} | {tj:+7.1f} | {s:+8.1f} | {la:4d}% | {vl:3d} {vr:3d} | {st}")

print("\n--- DETAILED CORNER 1 ANALYSIS (Frames 10 - 20) ---")
for r in records[10:20]:
    cte = float(r.get('R', 0)) / 10.0
    fcte = float(r.get('F', 0)) / 10.0
    h = float(r.get('H', 0)) / 10.0
    tj = float(r.get('TJ', 0)) / 10.0
    s = float(r.get('S', 0)) / 10.0
    la = int(r.get('LA', 0))
    vl = int(r.get('VL', 0))
    vr = int(r.get('VR', 0))
    
    # Calculate steering components assuming K_cte=0.6, K_heading=1.7, Kp=2.6
    # effective_h = (|h| - 2.0) * sgn(h) if |h| > 2.0 else 0
    eff_h = (abs(h) - 2.0) * (1.0 if h > 0 else -1.0) if abs(h) > 2.0 else 0.0
    heading_ff = 1.7 * eff_h
    # PID CTE approx: - Kp * (0.6 * fcte) = - 2.6 * 0.6 * fcte = - 1.56 * fcte
    # Wait, in the firmware:
    # float pid_cte = -pid_update(&steer_pid, 0.0f, g_params.k_cte * filtered_cte, g_params.dt);
    # pid_update(0, k_cte * fcte): error = 0 - k_cte * fcte = - k_cte * fcte
    # P_term = Kp * error = - Kp * k_cte * fcte
    # pid_cte = - P_term = + Kp * k_cte * fcte!
    p_cte = 2.6 * 0.6 * fcte
    print(f"Frame {r['idx']:02d}: fCTE={fcte:+4.1f} -> P_cte={p_cte:+5.1f} | H={h:+5.1f} -> H_ff={heading_ff:+5.1f} | Sum={p_cte+heading_ff:+5.1f} | Actual S={s:+5.1f} | Speed=({vl},{vr})")

