import numpy as np

def simulate_governor(lookahead_urgency_profile):
    speed_straight = 100.0
    speed_corner = 58.0
    decel_ramp = 25.0
    accel_ramp = 18.0
    
    applied_speed = 100.0
    lookahead_turn_factor = 0.0
    
    speeds = []
    kappas = []
    
    for urgency in lookahead_urgency_profile:
        # Asymmetric filter: fast attack (0.70), slow decay (0.15)
        if urgency >= lookahead_turn_factor:
            lookahead_turn_factor = 0.70 * urgency + 0.30 * lookahead_turn_factor
        else:
            lookahead_turn_factor = 0.15 * urgency + 0.85 * lookahead_turn_factor
            
        # Map urgency to kappa with 1.25x scaling so urgency >= 0.80 achieves full corner speed (58%)
        kappa = min(1.0, lookahead_turn_factor * 1.25)
        kappas.append(kappa)
        
        target_speed = speed_straight - kappa * (speed_straight - speed_corner)
        
        # Rate-limited speed update
        if target_speed < applied_speed:
            applied_speed -= decel_ramp
            if applied_speed < target_speed:
                applied_speed = target_speed
        else:
            applied_speed += accel_ramp
            if applied_speed > target_speed:
                applied_speed = target_speed
                
        speeds.append(applied_speed)
        
    return speeds, kappas

# Test scenario:
# Car on straight (ticks 0-10): urgency = 0.0
# Front camera detects curve at 1.0m (ticks 11-20): urgency = 0.90
# Bumper enters curve at tick 25: base heading swings
profile = [0.0]*11 + [0.90]*15

speeds, kappas = simulate_governor(profile)

print("Tick | Preview Urgency | Filtered Kappa | Target/Applied Speed")
print("-" * 55)
for tick in range(len(profile)):
    print(f"{tick:02d}   | {profile[tick]:15.2f} | {kappas[tick]:14.2f} | {speeds[tick]:18.1f}%")

# Assertions for pre-braking timing:
# 1. Curve detected at tick 11.
# 2. Applied speed should drop below 65% by tick 13 (within 40ms of preview detection).
# 3. Applied speed must be at minimum corner speed (58.0%) well before tick 20.
assert speeds[13] <= 65.0, f"Speed at tick 13 is {speeds[13]}, expected <= 65.0"
assert speeds[15] == 58.0, f"Speed at tick 15 is {speeds[15]}, expected == 58.0"

print("\nPRE-BRAKING TIMING TEST PASSED: Speed drops from 100% to 58% in 40ms, 200ms before bumper entry!")
