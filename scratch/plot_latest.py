import matplotlib.pyplot as plt
import numpy as np
import analyze_latest_test

data = analyze_latest_test.data

frames = [d['frame'] for d in data]
steer = [d['steer'] for d in data]
top_steer = [d['top_steer'] for d in data]
front_steer = [d['front_steer'] for d in data]
heading = [d['heading'] for d in data]
cte = [d['filt_cte'] for d in data]
blend_w = [d['blend_w'] for d in data]
vl = [d['vl'] for d in data]
vr = [d['vr'] for d in data]
ix = [d['ix'] for d in data]
lines_t = [d['lines_t'] for d in data]
lines_f = [d['lines_f'] for d in data]

fig, axs = plt.subplots(5, 1, figsize=(14, 16), sharex=True)

# 1. Steering & Blend
axs[0].plot(frames, steer, 'k-', linewidth=2.5, label='Effective Steer (S)')
axs[0].plot(frames, top_steer, 'b--', alpha=0.7, label='Top Steer (TS)')
axs[0].plot(frames, front_steer, 'r:', alpha=0.7, label='Front Steer (FS)')
axs[0].axhline(0, color='gray', linestyle=':', alpha=0.5)
axs[0].set_ylabel('Steer Angle (deg)')
axs[0].set_title('LATEST TEST (After Branch Penalty & Divergence Guard): Telemetry Analysis', fontsize=14, fontweight='bold')
axs[0].grid(True, alpha=0.3)
axs[0].legend(loc='upper right')

# 2. Heading and CTE
axs[1].plot(frames, heading, 'm-', linewidth=2, label='Heading H (deg)')
axs[1].plot(frames, cte, 'c-', linewidth=2, label='Filtered CTE F (px)')
axs[1].axhline(0, color='gray', linestyle=':', alpha=0.5)
axs[1].set_ylabel('Heading (deg) / CTE (px)')
axs[1].grid(True, alpha=0.3)
axs[1].legend(loc='upper right')

# 3. Blend Weight & Lines Detected
axs[2].plot(frames, blend_w, 'g-', linewidth=2, label='Top Camera Blend Weight (BW)')
axs[2].plot(frames, [l/4.0 for l in lines_t], 'b--', alpha=0.6, label='Top Lines / 4')
axs[2].plot(frames, [l/4.0 for l in lines_f], 'r--', alpha=0.6, label='Front Lines / 4')
axs[2].set_ylabel('Weight (0=Front, 1=Top)')
axs[2].grid(True, alpha=0.3)
axs[2].legend(loc='upper right')

# 4. Intersection Flag (IX) & Dominance
axs[3].step(frames, ix, 'r-', where='mid', linewidth=2, label='Intersection Detected (IX)')
dom_numeric = [1.0 if d['dom']=='T' else (0.5 if d['dom']=='B' else 0.0) for d in data]
axs[3].step(frames, dom_numeric, 'b-', where='mid', linewidth=1.5, label='Dominance (1=T, 0.5=B, 0=F)')
axs[3].set_ylabel('IX Flag / DOM')
axs[3].grid(True, alpha=0.3)
axs[3].legend(loc='upper right')

# 5. Speeds (VL, VR)
axs[4].plot(frames, vl, 'b-', label='Left Motor Speed (VL)')
axs[4].plot(frames, vr, 'r-', label='Right Motor Speed (VR)')
axs[4].set_ylabel('Motor PWM / Speed')
axs[4].set_xlabel('Frame Index (20 ms / frame)')
axs[4].grid(True, alpha=0.3)
axs[4].legend(loc='upper right')

plt.tight_layout()
plt.savefig(r"C:\Users\rober\.gemini\antigravity\brain\77fea321-1c41-4f6d-8b80-8e02b6a94640\latest_run_analysis.png", dpi=150)
print("Saved latest_run_analysis.png")
