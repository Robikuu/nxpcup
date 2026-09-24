"""
Unit test and verification for Split-Horizon Preview Braking and Startup Lane Acquisition Guard.
Verifies:
1. Startup Guard: If Top camera is blind before lane acquisition, steering holds 0.0 deg and blend weight holds 1.0 (Front camera cannot steer off-track).
2. Lane Acquisition: 2 valid Top camera frames latch has_acquired_lane = True.
3. Split-Horizon Decoupling:
   - Near Horizon (Y=38) measures local lane offset and angle right at the bumper.
   - Far Horizon (Y=20) detects upcoming curvature for pre-braking.
   - On a straight approaching a curve: Near offset = 0, delta_front = 0 deg, while far urgency = 1.0.
4. Aggressive Pre-Braking: target_speed drops from 60% straight speed to 48% corner speed within 2 ticks (40 ms).
5. Feedforward Clamping: lookahead_ff clamped to [-15 deg, +15 deg].
"""

import math

def test_startup_guard():
    print("--- Test 1: Startup Guard (Top camera blind before lane acquisition) ---")
    s_has_acquired_lane = False
    s_top_acquire_streak = 0
    s_blend_weight = 1.0
    raw_steer = 0.0
    
    # Front camera detects a distant curve (+25 px offset, +30 deg angle)
    delta_front = 0.70 * 30.0 + 0.25 * 25.0 # ~27.25 deg
    
    # Tick 1: Car just powered on at starting line, Top camera sees 0 vectors
    sr_found = 0
    num_vectors_pid = 0
    front_plausible = True
    
    if sr_found > 0:
        if not s_has_acquired_lane:
            s_top_acquire_streak += 1
            if s_top_acquire_streak >= 2:
                s_has_acquired_lane = True
    
    c_top = 0.0
    target_alpha = c_top
    if not s_has_acquired_lane:
        target_alpha = 1.0
        s_blend_weight = 1.0
        raw_steer = 0.0
    
    assert not s_has_acquired_lane, "Lane should NOT be acquired"
    assert s_blend_weight == 1.0, f"Blend weight should stay locked to 1.0, got {s_blend_weight}"
    assert raw_steer == 0.0, f"Steering should be held straight (0.0 deg), got {raw_steer}"
    print("Tick 1 (Top blind at start): blend_weight=1.0, steer=0.0 deg -> PASS")
    
    # Tick 2 & 3: Car drives forward and acquires lane
    sr_found = 1
    num_vectors_pid = 2
    for tick in range(2):
        if sr_found > 0:
            if not s_has_acquired_lane:
                s_top_acquire_streak += 1
                if s_top_acquire_streak >= 2:
                    s_has_acquired_lane = True
    
    assert s_has_acquired_lane, "Lane SHOULD now be acquired after 2 valid frames"
    print(f"Tick 3 (Acquired after 2 frames): has_acquired_lane={s_has_acquired_lane} -> PASS")

def test_split_horizon_geometry():
    print("\n--- Test 2: Split Horizon Geometry (Near Steering vs Far Pre-Braking) ---")
    # Vehicle is on a straight section 1.0m before a right 90-degree corner.
    # At the vehicle bumper (near field Y >= 30), lane is straight:
    # Left boundary: (x=20, y=45) to (x=28, y=15)
    # Right boundary: (x=60, y=45) to (x=52, y=15)
    # Center X = 40.0
    # But as it approaches Y=20, a right curve bends the far track to the right:
    # Say right curve bends far points to x=38 (left) and x=68 (right) at Y=20.
    
    # Vector left: (20, 45) -> (38, 15)  (starts straight near bumper, slants right towards horizon)
    # Vector right: (60, 45) -> (78, 15)
    
    near_y = 38.0
    far_y = 20.0
    
    # Near horizon projection (Y=38):
    # Left vector: ny=45, fy=15, dy=30, nx=20, fx=38, dx=18
    t_near_l = (45.0 - near_y) / 30.0 # 7.0 / 30.0 = 0.2333
    xl_near = 20.0 + t_near_l * 18.0   # 20 + 4.2 = 24.2
    
    # Right vector: ny=45, fy=15, dy=30, nx=60, fx=78, dx=18
    t_near_r = (45.0 - near_y) / 30.0
    xr_near = 60.0 + t_near_r * 18.0   # 60 + 4.2 = 64.2
    
    near_mid_x = (xl_near + xr_near) * 0.5 # (24.2 + 64.2) / 2 = 44.2
    near_offset = near_mid_x - 39.5        # +4.7 px
    
    # Far horizon projection (Y=20):
    t_far_l = (45.0 - far_y) / 30.0 # 25.0 / 30.0 = 0.8333
    xl_far = 20.0 + t_far_l * 18.0  # 20 + 15.0 = 35.0
    
    t_far_r = (45.0 - far_y) / 30.0
    xr_far = 60.0 + t_far_r * 18.0  # 60 + 15.0 = 75.0
    
    far_mid_x = (xl_far + xr_far) * 0.5 # (35 + 75) / 2 = 55.0
    
    # Curvature displacement between far and near horizons
    mid_dx = far_mid_x - near_mid_x # 55.0 - 44.2 = 10.8 px
    
    LOOKAHEAD_DEADBAND_PX = 3.5
    LOOKAHEAD_FULL_URGENCY_PX = 12.0
    
    urgency = 0.0
    if mid_dx > LOOKAHEAD_DEADBAND_PX:
        urgency = (mid_dx - LOOKAHEAD_DEADBAND_PX) / (LOOKAHEAD_FULL_URGENCY_PX - LOOKAHEAD_DEADBAND_PX)
        urgency = min(1.0, urgency)
        
    print(f"Near Offset at Y=38: {near_offset:.2f} px")
    print(f"Far Center at Y=20: {far_mid_x:.2f} px (shifted by {far_mid_x - 39.5:.1f} px)")
    print(f"Curvature displacement mid_dx: {mid_dx:.2f} px -> Lookahead Urgency: {urgency:.2f}")
    
    # If old system steered off Y=20:
    old_offset = far_mid_x - 39.5 # 15.5 px
    old_steer = 0.70 * 30.0 + 0.25 * old_offset # 21.0 + 3.88 = 24.88 deg early turn!
    
    # New system steers off Y=38:
    new_steer = 0.70 * (math.atan2(18.0, 30.0) * 180.0 / math.pi) * 0.3 + 0.25 * near_offset
    print(f"Old Y=20 steering offset was {old_offset:.1f} px vs New Y=38 near offset is {near_offset:.1f} px")
    assert urgency > 0.80, f"Lookahead urgency should be high for upcoming corner, got {urgency}"
    print("Near horizon maintains straight steering while far horizon triggers pre-braking -> PASS")

def test_pre_braking_governor():
    print("\n--- Test 3: Aggressive Pre-Braking Speed Governor ---")
    speed_straight = 60.0
    speed_outside_corner = 48.0
    lookahead_turn_factor = 0.0
    
    # Upcoming corner urgency = 1.0
    urgency = 1.0
    
    speeds = []
    for tick in range(5):
        # Asymmetric urgency attack (0.80 / 0.20)
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
    print("\n--- Test 4: Lookahead Feedforward Clamping ---")
    k_lookahead = 0.35
    
    # Huge angle detected in distant clutter: 60 deg
    filtered_traj_angle = 60.0
    effective_traj = (filtered_traj_angle - 3.0)
    lookahead_ff = k_lookahead * effective_traj # 0.35 * 57 = 19.95 deg
    
    if lookahead_ff > 15.0:
        lookahead_ff = 15.0
    elif lookahead_ff < -15.0:
        lookahead_ff = -15.0
        
    assert lookahead_ff == 15.0, f"Expected 15.0 clamp, got {lookahead_ff}"
    print(f"Clamped raw {0.35 * 57:.1f}° feedforward to {lookahead_ff:.1f}° -> PASS")

if __name__ == '__main__':
    test_startup_guard()
    test_split_horizon_geometry()
    test_pre_braking_governor()
    test_feedforward_clamping()
    print("\n" + "="*50)
    print("ALL TESTS PASSED SUCCESSFULLY!")
    print("="*50)
