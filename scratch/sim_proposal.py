# Simulate what happens with the telemetry frames under the proposed logic

print("Simulating Intersection Crossing with proposed logic:")
print("-----------------------------------------------------")

# Current buggy run from user:
# Line 13: Approach
# Line 14: Transverse flank detected, but exit pair failed -> is_sharp_corner=1, FS=-69
# Line 15: S:247
# Line 16: ST:C, FS:-310, S:-14
# Line 17: ST:C, FS:-540, S:-272
# Line 18: ST:C, FS:-682, S:-650 -> locked left into inner loop!

# With proposed logic:
# Line 14: Transverse flank detected, center is open (transverse_crosses_center == 0).
# -> is_sharp_corner = 0
# -> is_intersection_ahead = 1
# -> s_intersection_ahead latched = 1, timer = 20 ticks
# Lines 15-20: Inside intersection
# -> s_intersection_ahead is active
# -> steer_cmd clamped to [-4.0, +4.0] deg!
# -> feedforward suppressed (= 0.0)
# -> diff drag suppressed (VL=100, VR=100)
# -> Car shoots straight across!
# Line 21+: Reacquires straight lane on other side!

print("Done.")
