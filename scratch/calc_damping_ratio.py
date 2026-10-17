import numpy as np

# Vehicle parameters
# Mass m ~ 1.0 kg
# Wheelbase L ~ 0.16 m
# Speed v ~ 1.8 m/s (at 95% duty)
# Steer ratio: delta = K_steer * (Kp * e + Kd * de/dt + Kh * h)
# Since dy/dt = v * theta, and dtheta/dt = (v / L) * tan(delta) ~ (v / L) * delta
#
# Let's inspect the closed loop state space:
# x = [y (lateral offset), theta (heading angle)]
# y_dot = v * theta
# theta_dot = (v / L) * delta
#
# delta = - [ K_cte_eff * y + K_h_eff * theta + Kd_eff * y_dot ]
#
# theta_dot = - (v/L) * [ K_cte_eff * y + (K_h_eff + Kd_eff * v) * theta ]
#
# Characteristic equation:
# s^2 + (v/L)*(K_h_eff + Kd_eff * v)*s + (v^2 / L)*K_cte_eff = 0

v = 1.8  # m/s
L = 0.16 # m

# Convert pixel to meters: 1 pixel ~ 0.005 m (track width 40cm is ~80px)
px_to_m = 0.005
deg_to_rad = np.pi / 180.0

# Current gains:
# Steer delta in deg = kp_s * k_cte * (e_px) + k_head * (h_deg) + kd_s * (de/dt)
# Convert all to rad / m / s:
# kp_s = 0.75, k_cte = 0.65 -> K_p = 0.75 * 0.65 = 0.4875 deg/px = 0.4875 * (1/0.005) * deg_to_rad = 1.70 rad/m
K_cte_eff = 0.75 * 0.65 / px_to_m * deg_to_rad

# k_head = 1.45 deg/deg = 1.45 rad/rad
K_h_eff = 1.45

# kd_s = 0.14 deg/(px/s)
Kd_eff = 0.14 / px_to_m * deg_to_rad

# 2*zeta*omega_n = (v / L) * (K_h_eff + Kd_eff * v)
# omega_n^2 = (v^2 / L) * K_cte_eff

omega_n = np.sqrt((v**2 / L) * K_cte_eff)
two_zeta_omega_n = (v / L) * (K_h_eff + Kd_eff * v)
zeta = two_zeta_omega_n / (2 * omega_n)

print(f"Speed: {v} m/s")
print(f"K_cte_eff: {K_cte_eff:.2f} rad/m")
print(f"K_h_eff: {K_h_eff:.2f} rad/rad")
print(f"Kd_eff: {Kd_eff:.4f} rad/(m/s)")
print(f"Natural Frequency omega_n: {omega_n:.2f} rad/s ({omega_n / (2*np.pi):.2f} Hz)")
print(f"Damping Ratio zeta: {zeta:.3f}")

# Look at damping ratio:
# If zeta < 1.0, the system is underdamped and WILL oscillate/weave!
# If zeta < 0.7, it exhibits substantial overshoot and ringing!
# If differential drag or lookahead adds positive feedback, effective zeta drops even further!
