import numpy as np
import matplotlib.pyplot as plt

# Simulate straightaway vehicle dynamics under current vs tuned parameters
# State: [y (m), theta (rad), steer_cmd (rad), applied_steer (rad)]

def sim_straightaway(k_cte=0.65, kp=0.75, kd=0.14, k_h=1.45, h_deadband_deg=3.5, 
                     k_la=0.35, diff_deadzone_deg=8.0, speed=1.8, steps=100, dt=0.02):
    L = 0.16 # wheelbase (m)
    px_to_m = 0.005 # 1px = 5mm
    
    y = 0.03 # initial 3cm disturbance (6px offset)
    theta = np.radians(6.0) # initial 6 deg heading disturbance
    steer = 0.0
    
    history_y = []
    history_theta = []
    history_steer = []
    history_diff = []
    
    d_filtered = 0.0
    alpha = 0.25
    prev_y = y
    
    for t in range(steps):
        # 1. Sensors:
        e_px = y / px_to_m
        h_deg = np.degrees(theta)
        
        # 2. Heading feedforward with deadband:
        abs_h = abs(h_deg)
        eff_h = (abs_h - h_deadband_deg) * np.sign(h_deg) if abs_h > h_deadband_deg else 0.0
        h_ff_deg = -k_h * eff_h # restoring
        
        # 3. PID CTE:
        d_raw = (y - prev_y) / dt
        prev_y = y
        d_filtered = alpha * d_raw + (1.0 - alpha) * d_filtered
        d_term_deg = -kd * (d_filtered / px_to_m)
        p_term_deg = -kp * k_cte * e_px
        pid_cte_deg = p_term_deg + d_term_deg
        
        # 4. Lookahead feedforward (front cam 1.0m ahead):
        # y_ahead = y + 1.0 * theta
        offset_ahead_px = (y + 1.0 * theta) / px_to_m
        abs_off = abs(offset_ahead_px)
        # on straight, lookahead offset threshold:
        eff_off = (abs_off - 5.0) * np.sign(offset_ahead_px) if abs_off > 5.0 else 0.0
        la_ff_deg = -k_la * eff_off
        
        # Total steer
        target_steer_deg = pid_cte_deg + h_ff_deg + la_ff_deg
        
        # Slew rate: 13 deg/tick
        max_step = 13.0
        diff = target_steer_deg - steer
        steer += np.clip(diff, -max_step, max_step)
        
        # Differential drag yaw rate assist:
        abs_s = abs(steer)
        diff_yaw_assist = 0.0
        if abs_s > diff_deadzone_deg:
            diff_norm = (abs_s - diff_deadzone_deg) / (45.0 - diff_deadzone_deg)
            diff_norm = min(1.0, max(0.0, diff_norm))
            # differential drag creates direct yaw acceleration / yaw rate
            diff_yaw_assist = np.sign(steer) * diff_norm * 1.5 # rad/s
            
        # Kinematics:
        steer_rad = np.radians(steer)
        yaw_rate = (speed / L) * np.tan(steer_rad) + diff_yaw_assist
        theta += yaw_rate * dt
        y += speed * np.sin(theta) * dt
        
        history_y.append(y * 100) # cm
        history_theta.append(np.degrees(theta))
        history_steer.append(steer)
        history_diff.append(abs_s > diff_deadzone_deg)
        
    return history_y, history_theta, history_steer, history_diff

# 1. Current parameters:
y1, th1, st1, df1 = sim_straightaway(k_cte=0.65, kp=0.75, kd=0.14, k_h=1.45, h_deadband_deg=3.5, 
                                     k_la=0.35, diff_deadzone_deg=8.0)

# 2. Tuned parameters:
# - Heading deadband on straights: 5.5 deg (stops micro-jitter from firing k_heading)
# - Differential drag deadzone: 14.0 deg (only fires in real corners > 14 deg, NEVER on straights!)
# - Lookahead feedforward: gated on straight (or deadband 10px ahead so straight yaw doesn't excite it)
# - Damping kd_s: increased from 0.14 to 0.22 (suppresses velocity oscillations)
# - k_cte: 0.55 (smooth, non-oscillatory centering)
y2, th2, st2, df2 = sim_straightaway(k_cte=0.55, kp=0.70, kd=0.22, k_h=1.45, h_deadband_deg=5.0, 
                                     k_la=0.0, diff_deadzone_deg=14.0)

print(f"Current Config: Max Steer Pk-Pk = {max(st1)-min(st1):.1f} deg, Diff Active ticks = {sum(df1)} / 100")
print(f"Tuned Config:   Max Steer Pk-Pk = {max(st2)-min(st2):.1f} deg, Diff Active ticks = {sum(df2)} / 100")

print(f"\nSettling time to |y| < 1cm:")
t1 = next((i*0.02 for i in range(len(y1)-5) if all(abs(y1[i+j]) < 1.0 for j in range(5))), None)
t2 = next((i*0.02 for i in range(len(y2)-5) if all(abs(y2[i+j]) < 1.0 for j in range(5))), None)
print(f"Current Config Settling Time: {t1} s")
print(f"Tuned Config Settling Time:   {t2} s")
