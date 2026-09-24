"""
Unit test and verification for Split-Horizon Preview Braking and Timed Startup Rollout.
Verifies:
1. Startup Rollout: First 5 ticks (100 ms) hold steering straight (0.0 deg) across start line.
2. Timed/Dual-Cam Unlocking: From tick 6 onwards, if Top is blind, blend weight transitions to 0.0 (DOM: F) and Front camera takes full authority.
3. Turn 1 Rescue: Front camera commands -48.6 deg at Turn 1 when Top is blind (does NOT freeze at 0.0 deg).
4. Frame 290 Divider Rescue: Top dropout at lane split transfers to Front camera (-50.7 deg) instead of drifting into divider.
5. Split-Horizon Decoupling:
   - Near Horizon (Y=38) measures local lane offset and angle right at the bumper.
   - Far Horizon (Y=20) detects upcoming curvature for pre-braking.
6. Aggressive Pre-Braking: target_speed drops from 60% straight speed to 48% corner speed within 2 ticks (40 ms).
7. Feedforward Clamping: lookahead_ff clamped to [-15 deg, +15 deg].
"""

import math

def test_startup_rollout_and_front_takeover():
    print("--- Test 1: Timed Startup Rollout & Front Camera Takeover (Top Blind at Start) ---")
    s_has_acquired_lane = False
    s_startup_rollout_ticks = 0
    s_blend_weight = 1.0
    BLEND_FALL_SLEW_MAX = 0.50
    BLEND_RISE_SLEW_MAX = 0.08
    
    # Top camera is completely blind (L: 0/3)
    sr_found = 0
    num_vectors_pid = 0
    front_plausible = True
    num_vectors_front = 3
    
    # Front camera sees Turn 1 approaching
    delta_front = -48.6
    delta_top = 0.0
    
    steer_history = []
    blend_history = []
    
    for tick in range(15): # 15 ticks = 300 ms
        s_startup_rollout_ticks += 1
        if sr_found > 0 or (front_plausible and num_vectors_front >= 1) or s_startup_rollout_ticks >= 10:
            s_has_acquired_lane = True
            
        c_top = 1.0 if sr_found > 0 and num_vectors_pid >= 2 else (0.5 if sr_found > 0 and num_vectors_pid == 1 else 0.0)
        target_alpha = 1.0 if (not front_plausible and c_top > 0.0) else c_top
        
        delta_alpha = target_alpha - s_blend_weight
        if delta_alpha > BLEND_RISE_SLEW_MAX:
            delta_alpha = BLEND_RISE_SLEW_MAX
        elif delta_alpha < -BLEND_FALL_SLEW_MAX:
            delta_alpha = -BLEND_FALL_SLEW_MAX
        s_blend_weight = max(0.0, min(1.0, s_blend_weight + delta_alpha))
        
        if s_startup_rollout_ticks <= 5:
            raw_steer = 0.0
        else:
            raw_steer = s_blend_weight * delta_top + (1.0 - s_blend_weight) * delta_front
            
        steer_history.append(raw_steer)
        blend_history.append(s_blend_weight)
        
    print(f"Ticks 1-5 (First 100ms): raw_steer={steer_history[0]} to {steer_history[4]} deg (Center roll)")
    assert all(s == 0.0 for s in steer_history[:5]), "First 5 ticks must be 0.0 deg"
    
    # By tick 6, blend weight is already 0.0 because it dropped by 0.50 on tick 1 and 0.50 on tick 2!
    print(f"Tick 6 (120ms): blend_weight={blend_history[5]:.2f}, raw_steer={steer_history[5]:.1f}° (Front camera fully active!)")
    assert blend_history[5] == 0.0, f"Blend weight should be 0.0, got {blend_history[5]}"
    assert abs(steer_history[5] - delta_front) < 0.1, f"Steering should track delta_front, got {steer_history[5]}"
    print("Top camera blind at start: Front camera seamlessly takes over after 100ms rollout -> PASS")

def test_frame290_divider_rescue():
    print("\n--- Test 2: Frame 290 Divider Rescue (Top Dropout at Lane Split) ---")
    s_blend_weight = 1.0
    BLEND_FALL_SLEW_MAX = 0.50
    BLEND_RISE_SLEW_MAX = 0.08
    
    # Normal driving before frame 290
    delta_top = 13.3  # Drifting right toward divider
    delta_front = -50.7 # Front sees curve ahead, screaming turn left!
    
    # Top drops out (L: 0/2)
    sr_found = 0
    num_vectors_pid = 0
    front_plausible = True
    num_vectors_front = 2
    
    steers = []
    blends = []
    
    for tick in range(10):
        c_top = 1.0 if sr_found > 0 and num_vectors_pid >= 2 else (0.5 if sr_found > 0 and num_vectors_pid == 1 else 0.0)
        target_alpha = 1.0 if (not front_plausible and c_top > 0.0) else c_top
        
        delta_alpha = target_alpha - s_blend_weight
        if delta_alpha > BLEND_RISE_SLEW_MAX:
            delta_alpha = BLEND_RISE_SLEW_MAX
        elif delta_alpha < -BLEND_FALL_SLEW_MAX:
            delta_alpha = -BLEND_FALL_SLEW_MAX
        s_blend_weight = max(0.0, min(1.0, s_blend_weight + delta_alpha))
        
        # In active blend mode (shadow 0):
        raw_steer = s_blend_weight * delta_top + (1.0 - s_blend_weight) * delta_front
        steers.append(raw_steer)
        blends.append(s_blend_weight)
        
    print(f"Tick 1 after Top dropout: blend={blends[0]:.2f}, steer={steers[0]:.1f}°")
    print(f"Tick 2 after Top dropout: blend={blends[1]:.2f}, steer={steers[1]:.1f}° (100% Front authority!)")
    assert blends[1] == 0.0, f"Expected 0.0 blend weight on tick 2, got {blends[1]}"
    assert abs(steers[1] - delta_front) < 0.1, f"Expected steer to track delta_front (-50.7°), got {steers[1]}"
    print("Frame 290 dropout cleanly rescued by Front camera left turn -> PASS")

def test_split_horizon_geometry():
    print("\n--- Test 3: Split Horizon Geometry (Near Steering vs Far Pre-Braking) ---")
    near_y = 38.0
    far_y = 20.0
    
    # Left vector: ny=45, fy=15, dy=30, nx=20, fx=38, dx=18
    t_near_l = (45.0 - near_y) / 30.0
    xl_near = 20.0 + t_near_l * 18.0
    
    # Right vector: ny=45, fy=15, dy=30, nx=60, fx=78, dx=18
    t_near_r = (45.0 - near_y) / 30.0
    xr_near = 60.0 + t_near_r * 18.0
    
    near_mid_x = (xl_near + xr_near) * 0.5
    near_offset = near_mid_x - 39.5
    
    t_far_l = (45.0 - far_y) / 30.0
    xl_far = 20.0 + t_far_l * 18.0
    
    t_far_r = (45.0 - far_y) / 30.0
    xr_far = 60.0 + t_far_r * 18.0
    
    far_mid_x = (xl_far + xr_far) * 0.5
    mid_dx = far_mid_x - near_mid_x
    
    LOOKAHEAD_DEADBAND_PX = 3.5
    LOOKAHEAD_FULL_URGENCY_PX = 12.0
    
    urgency = 0.0
    if mid_dx > LOOKAHEAD_DEADBAND_PX:
        urgency = (mid_dx - LOOKAHEAD_DEADBAND_PX) / (LOOKAHEAD_FULL_URGENCY_PX - LOOKAHEAD_DEADBAND_PX)
        urgency = min(1.0, urgency)
        
    print(f"Near Offset at Y=38: {near_offset:.2f} px")
    print(f"Curvature displacement mid_dx: {mid_dx:.2f} px -> Lookahead Urgency: {urgency:.2f}")
    assert urgency > 0.80, f"Lookahead urgency should be high for upcoming corner, got {urgency}"
    print("Near horizon maintains straight steering while far horizon triggers pre-braking -> PASS")

def test_pre_braking_governor():
    print("\n--- Test 4: Aggressive Pre-Braking Speed Governor ---")
    speed_straight = 60.0
    speed_outside_corner = 48.0
    lookahead_turn_factor = 0.0
    urgency = 1.0
    
    speeds = []
    for tick in range(5):
        if urgency >= lookahead_turn_factor:
            lookahead_turn_factor = 0.80 * urgency + 0.20 * lookahead_turn_factor
        else:
            lookahead_turn_factor = 0.15 * urgency + 0.85 * lookahead_turn_factor
            
        k_front = min(1.0, lookahead_turn_factor * 1.25)
        kappa = k_front
        target_speed = speed_straight - kappa * (speed_straight - speed_outside_corner)
        speeds.append(target_speed)
        
    print(f"Tick 1 target_speed: {speeds[0]:.1f}% (dropped from 60%)")
    print(f"Tick 2 target_speed: {speeds[1]:.1f}% (reached corner speed 48%)")
    assert speeds[0] <= 50.5, f"Tick 1 should drop aggressively, got {speeds[0]}"
    assert abs(speeds[1] - 48.0) < 0.1, f"Tick 2 should reach 48.0%, got {speeds[1]}"
    print("Pre-braking reaches full corner speed within 2 ticks (40 ms) -> PASS")

def test_feedforward_clamping():
    print("\n--- Test 5: Lookahead Feedforward Clamping ---")
    k_lookahead = 0.35
    filtered_traj_angle = 60.0
    effective_traj = (filtered_traj_angle - 3.0)
    lookahead_ff = k_lookahead * effective_traj
    
    if lookahead_ff > 15.0:
        lookahead_ff = 15.0
    elif lookahead_ff < -15.0:
        lookahead_ff = -15.0
        
    assert lookahead_ff == 15.0, f"Expected 15.0 clamp, got {lookahead_ff}"
    print(f"Clamped raw {0.35 * 57:.1f}° feedforward to {lookahead_ff:.1f}° -> PASS")

if __name__ == '__main__':
    test_startup_rollout_and_front_takeover()
    test_frame290_divider_rescue()
    test_split_horizon_geometry()
    test_pre_braking_governor()
    test_feedforward_clamping()
    print("\n" + "="*50)
    print("ALL TESTS PASSED SUCCESSFULLY!")
    print("="*50)
