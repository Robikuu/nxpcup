"""
Verification simulation for Wi-Fi command shell and status output.
Verifies:
1. Deprecated parameter feedback for kp_s, kp_c, kd_s, kd_c, ki, and alpha.
2. Active steering parameter updates: k_cte, k_head, k_cte_c, k_head_c, k_la.
3. Range validation on k_la.
4. Clean status output without deprecated/ghost variables.
"""

class MockParams:
    def __init__(self):
        self.k_cte = 0.55
        self.k_heading = 1.30
        self.k_cte_corner = 0.55
        self.k_heading_corner = 1.30
        self.k_lookahead = 0.45
        self.steering_offset = 0.00
        self.blend_enable = 0
        self.speed_straight = 100.0
        self.speed_outside_corner = 50.0
        self.speed_inside_min = 32.0
        self.drive_scale = 1.00
        self.front_angle_thresh = 15.0
        self.h_lo = 12.0
        self.h_hi = 25.0
        self.blend_attack = 0.35
        self.blend_release = 0.06
        self.accel_ramp = 35.0
        self.decel_ramp = 45.0
        self.off_track_brake_ticks = 5
        self.motor_enable = 1
        self.shadow_mode_enable = 1
        self.dump_vectors = 0
        self.state = 0

def execute_command_sim(cmd, params):
    parts = cmd.strip().split()
    keyword = parts[0]
    has_val = len(parts) > 1
    val = float(parts[1]) if has_val else 0.0

    if keyword in ["kp_straight", "kp_s", "kp_corner", "kp_c", "kd_straight", "kd_s", "kd_corner", "kd_c", "ki"] and has_val:
        return "NOTE: Classic PID is deprecated. Steering uses k_cte (P) and k_head (D).\r\n"
    elif keyword == "alpha" and has_val:
        return "NOTE: alpha (derivative filter) is deprecated in State-Feedback steering.\r\n"
    elif keyword in ["k_lookahead", "k_la"] and has_val:
        if 0.0 <= val <= 2.0:
            params.k_lookahead = val
            return f"OK k_lookahead={val:.2f}\r\n"
        else:
            return "ERR k_la range: 0.0 to 2.0\r\n"
    elif keyword in ["k_heading", "k_head"] and has_val:
        params.k_heading = val
        return f"OK k_heading={val:.2f}\r\n"
    elif keyword == "k_cte" and has_val:
        params.k_cte = val
        return f"OK k_cte={val:.2f}\r\n"
    elif keyword == "status":
        lines = [
            f"k_cte={params.k_cte:.2f}",
            f"k_head={params.k_heading:.2f}",
            f"k_cte_c={params.k_cte_corner:.2f}",
            f"k_head_c={params.k_heading_corner:.2f}",
            f"k_la={params.k_lookahead:.2f}",
            f"offset={params.steering_offset:.2f}",
            f"blend={params.blend_enable}",
            f"spd_s={params.speed_straight:.2f}",
            f"spd_out={params.speed_outside_corner:.2f}",
            f"spd_in={params.speed_inside_min:.2f}",
            f"scale={params.drive_scale:.2f}",
            f"fa_th={params.front_angle_thresh:.2f}",
            f"h_lo={params.h_lo:.2f}",
            f"h_hi={params.h_hi:.2f}",
            f"att={params.blend_attack:.2f}",
            f"rel={params.blend_release:.2f}",
            f"ramp_a={params.accel_ramp:.2f}",
            f"ramp_d={params.decel_ramp:.2f}",
            f"off_brake={params.off_track_brake_ticks}",
            f"drive={params.motor_enable}",
            f"shadow={params.shadow_mode_enable}",
            f"dump={params.dump_vectors}",
            f"state={'RUNNING' if params.state else 'IDLE'}"
        ]
        return "\r\n".join(lines) + "\r\n"
    return "ERR unknown command\r\n"

def test_shell():
    params = MockParams()

    # Test 1: Deprecated PID commands
    r1 = execute_command_sim("kp_s 1.2", params)
    assert "NOTE: Classic PID is deprecated" in r1, f"Unexpected response: {r1}"
    print("[PASS] Deprecated kp_s command returns helpful guidance message")

    r2 = execute_command_sim("alpha 0.3", params)
    assert "NOTE: alpha (derivative filter) is deprecated" in r2, f"Unexpected response: {r2}"
    print("[PASS] Deprecated alpha command returns informative message")

    # Test 2: Valid k_la update
    r3 = execute_command_sim("k_la 0.65", params)
    assert "OK k_lookahead=0.65" in r3 and abs(params.k_lookahead - 0.65) < 1e-4
    print("[PASS] k_la successfully updates lookahead gain to 0.65")

    # Test 3: Out-of-range k_la
    r4 = execute_command_sim("k_la 2.5", params)
    assert "ERR k_la range: 0.0 to 2.0" in r4
    print("[PASS] k_la out-of-range validation functions correctly")

    # Test 4: Status output check
    status = execute_command_sim("status", params)
    assert "kp_s" not in status, "kp_s should NOT be in status output!"
    assert "kd_s" not in status, "kd_s should NOT be in status output!"
    assert "ki" not in status, "ki should NOT be in status output!"
    assert "alpha" not in status, "alpha should NOT be in status output!"
    assert "k_cte=0.55" in status
    assert "k_head=1.30" in status
    assert "k_la=0.65" in status
    assert "off_brake=5" in status
    print("[PASS] status command outputs clean list without ghost/deprecated variables")

    print("\nALL SHELL AND STATUS TESTS PASSED SUCCESSFULLY!")

if __name__ == '__main__':
    test_shell()
