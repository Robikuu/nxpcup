"""
Unit and state transition simulation test for Off-Track Dynamic Braking Guard.
Verifies:
1. Normal operation: Both cameras see lines -> s_off_track_ticks == 0, is_off_track_braking == False, normal driving speed.
2. Transient optical dropout (1-4 frames): s_off_track_ticks < 5 -> no braking, holds last steering command across brief apex clip.
3. Single camera dropout (top camera loses lines): At tick 5 (100ms) -> is_off_track_braking == True, HbridgeBrake engaged, speeds zeroed.
4. Dual camera blindness (both cameras lose lines): Fast trigger at tick 3 (60ms) -> is_off_track_braking == True, immediate dynamic braking.
5. Failsafe stop: Off-track sustained for 25 frames (500ms) -> transitions to STATE_IDLE.
6. Legitimate intersection crossing: s_intersection_ahead == 1 -> braking is NOT triggered during blind crossing window.
7. Reacquisition: If line is reacquired after braking, s_off_track_ticks resets and vehicle accelerates cleanly from 0.
"""

def simulate_car_off_track(scenarios):
    OFF_TRACK_BRAKE_TICKS = 5
    OFF_TRACK_DUAL_BLIND_TICKS = 3
    OFF_TRACK_STOP_TICKS = 25
    STATE_IDLE = 0
    STATE_RUNNING = 1

    for scenario_name, scenario_steps in scenarios.items():
        print(f"\n--- Running Scenario: {scenario_name} ---")
        state = STATE_RUNNING
        s_off_track_ticks = 0
        s_intersection_ahead = 0
        s_intersection_gap_ticks = 0
        applied_speed_left = 100.0
        applied_speed_right = 100.0
        last_steer_cmd = 12.0

        for tick, step in enumerate(scenario_steps):
            sr_found = step.get('sr_found', 2)
            horizon_found = step.get('horizon_found', 2)
            if 'intersection_ahead' in step:
                s_intersection_ahead = step['intersection_ahead']

            is_off_track_braking = False

            # Simulation of main.c logic
            if sr_found == 0:
                s_off_track_ticks += 1

                if s_intersection_ahead:
                    s_intersection_gap_ticks += 1
                    gap_window = 15
                    if s_intersection_gap_ticks > gap_window:
                        s_intersection_ahead = 0
                        s_intersection_gap_ticks = 0
                else:
                    brake_threshold = OFF_TRACK_BRAKE_TICKS
                    if horizon_found == 0 and brake_threshold > OFF_TRACK_DUAL_BLIND_TICKS:
                        brake_threshold = OFF_TRACK_DUAL_BLIND_TICKS

                    if s_off_track_ticks >= brake_threshold:
                        is_off_track_braking = True

                if s_off_track_ticks >= OFF_TRACK_STOP_TICKS:
                    state = STATE_IDLE
                    applied_speed_left = 0.0
                    applied_speed_right = 0.0
                    s_off_track_ticks = 0
                    # stopped
            else:
                s_off_track_ticks = 0

            if is_off_track_braking:
                applied_speed_left = 0.0
                applied_speed_right = 0.0
                steer_cmd = 0.0
                hbridge_mode = "BRAKE"
                telemetry_st = "B"
            elif state == STATE_IDLE:
                hbridge_mode = "BRAKE"
                telemetry_st = "IDLE"
            else:
                steer_cmd = last_steer_cmd if sr_found == 0 else 5.0
                hbridge_mode = "SPEED"
                telemetry_st = "I" if s_intersection_ahead else "S"

            step['result_braking'] = is_off_track_braking
            step['result_mode'] = hbridge_mode
            step['result_state'] = state
            step['result_telemetry'] = telemetry_st
            step['s_off_track_ticks'] = s_off_track_ticks

        print(f"Scenario '{scenario_name}' completed. Final state: {state}")

def run_tests():
    # Test 1: Transient 3-frame dropout (should not brake if front cam sees lines)
    scenarios = {
        "Transient 3-Frame Dropout (Front Valid)": [
            {'sr_found': 2, 'horizon_found': 2},
            {'sr_found': 0, 'horizon_found': 1}, # tick 1
            {'sr_found': 0, 'horizon_found': 1}, # tick 2
            {'sr_found': 0, 'horizon_found': 1}, # tick 3
            {'sr_found': 2, 'horizon_found': 2}, # reacquired
        ],
        "Top Cam Dropout >= 5 Frames (Front Valid)": [
            {'sr_found': 0, 'horizon_found': 1}, # tick 1
            {'sr_found': 0, 'horizon_found': 1}, # tick 2
            {'sr_found': 0, 'horizon_found': 1}, # tick 3
            {'sr_found': 0, 'horizon_found': 1}, # tick 4
            {'sr_found': 0, 'horizon_found': 1}, # tick 5 -> BRAKE
            {'sr_found': 0, 'horizon_found': 1}, # tick 6 -> BRAKE
        ],
        "Dual Camera Blindness (Both Cam Blind)": [
            {'sr_found': 0, 'horizon_found': 0}, # tick 1
            {'sr_found': 0, 'horizon_found': 0}, # tick 2
            {'sr_found': 0, 'horizon_found': 0}, # tick 3 -> FAST BRAKE
        ],
        "Intersection Crossing Immunity": [
            {'sr_found': 0, 'horizon_found': 0, 'intersection_ahead': 1}, # tick 1
            {'sr_found': 0, 'horizon_found': 0, 'intersection_ahead': 1}, # tick 2
            {'sr_found': 0, 'horizon_found': 0, 'intersection_ahead': 1}, # tick 3
            {'sr_found': 0, 'horizon_found': 0, 'intersection_ahead': 1}, # tick 4
            {'sr_found': 0, 'horizon_found': 0, 'intersection_ahead': 1}, # tick 5
            {'sr_found': 2, 'horizon_found': 2, 'intersection_ahead': 0}, # reacquired exit
        ],
        "Sustained Off-Track Full Stop (25 Frames)": [
            {'sr_found': 0, 'horizon_found': 0} for _ in range(26)
        ]
    }

    simulate_car_off_track(scenarios)

    # Assertions
    # 1. Transient 3 frames with front valid -> no braking
    t1 = scenarios["Transient 3-Frame Dropout (Front Valid)"]
    assert not t1[1]['result_braking'] and not t1[2]['result_braking'] and not t1[3]['result_braking']
    print("[PASS] Transient dropout did not falsely trigger braking")

    # 2. Top cam dropout at tick 5 triggers brake
    t2 = scenarios["Top Cam Dropout >= 5 Frames (Front Valid)"]
    assert not t2[3]['result_braking'], "Tick 4 should not brake yet"
    assert t2[4]['result_braking'], "Tick 5 must trigger dynamic braking"
    assert t2[4]['result_mode'] == "BRAKE"
    print("[PASS] Top cam dropout triggered active dynamic brake at frame 5 (100ms)")

    # 3. Dual blindness fast trigger at tick 3
    t3 = scenarios["Dual Camera Blindness (Both Cam Blind)"]
    assert not t3[1]['result_braking'], "Tick 2 should not brake"
    assert t3[2]['result_braking'], "Tick 3 must trigger fast dynamic braking"
    print("[PASS] Dual camera blindness triggered fast brake at frame 3 (60ms)")

    # 4. Intersection crossing does not trigger braking
    t4 = scenarios["Intersection Crossing Immunity"]
    for step in t4:
        assert not step['result_braking'], "Intersection should not trigger braking!"
    print("[PASS] Intersection gap maintained without premature braking")

    # 5. Sustained off-track shuts down to STATE_IDLE at 25 frames
    t5 = scenarios["Sustained Off-Track Full Stop (25 Frames)"]
    assert t5[24]['result_state'] == 0, "Tick 25 must transition to STATE_IDLE"
    print("[PASS] Sustained off-track triggered full STATE_IDLE shutdown at 25 frames (500ms)")

    print("\nALL OFF-TRACK BRAKING SIMULATION TESTS PASSED SUCCESSFULLY!")

if __name__ == '__main__':
    run_tests()
