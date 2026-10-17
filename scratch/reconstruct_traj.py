import numpy as np
import matplotlib.pyplot as plt
import analyze_runs

def reconstruct_trajectory(data, dt=0.02):
    # Differential drive kinematics or bicycle model
    # speed v = (vl + vr) / 2
    # yaw rate omega = (vr - vl) / wheelbase or from steering angle
    # In nxpcup, wheelbase / track width: let's estimate or use yaw from steering
    # But we also have front wheel steering: delta = steer (in degrees)
    # v_car = speed * scale_factor (e.g. at 60% PWM ~ 1.5 - 2.0 m/s)
    # Let's integrate (x, y, theta)
    x = [0.0]
    y = [0.0]
    theta = [0.0] # start facing +X (east)
    
    wheelbase = 0.20 # 20 cm
    # Let's assume 60 PWM is ~1.5 m/s, or pwm/100 * 2.5 m/s
    speed_scale = 0.025 # m/s per PWM unit
    
    for d in data:
        v = 0.5 * (d['vl'] + d['vr']) * speed_scale
        steer_rad = np.radians(d['steer'])
        
        # dtheta = (v / wheelbase) * tan(delta)
        # Note: In nxpcup firmware, positive steer turns which way?
        # Let's check sign: S > 0 turns right or left?
        dtheta = (v / wheelbase) * np.sin(steer_rad) * dt
        
        new_theta = theta[-1] + dtheta
        new_x = x[-1] + v * np.cos(new_theta) * dt
        new_y = y[-1] + v * np.sin(new_theta) * dt
        
        x.append(new_x)
        y.append(new_y)
        theta.append(new_theta)
        
    return np.array(x), np.array(y), np.array(theta)

run6 = analyze_runs.run6_data
print(f"Run 6 frames: {len(run6)}")
# Check signs:
# Bottom straight: starts at frame 0 to ~15.
# At frame 16-30: what is steering?
for i in range(15, 35):
    d = run6[i]
    print(f"F{i:2d}: S={d['steer']:+5.1f} TS={d['top_steer']:+5.1f} FS={d['front_steer']:+5.1f} H={d['heading']:+5.1f} CTE={d['filt_cte']:+5.1f}")
