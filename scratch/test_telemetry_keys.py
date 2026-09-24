import re
import sys

# New telemetry line format:
sample_line = "L:2/3 R:-10 F:-11 H:6 TS:-15 FS:-12 S:-14 BW:85 DOM:T FV:1 IX:0 ST:N VL:88 VR:88"

# Old telemetry line format (for comparison/rejection):
old_line = "L:2/3 R:-10 F:-11 H:6 TJ:27 S:-5 LA:0 ST:N VL:100 VR:100"

pattern = re.compile(
    r"^L:(?P<top_v>\d+)/(?P<front_v>\d+)\s+"
    r"R:(?P<r_cte>-?\d+)\s+"
    r"F:(?P<f_cte>-?\d+)\s+"
    r"H:(?P<heading>-?\d+)\s+"
    r"TS:(?P<top_steer>-?\d+)\s+"
    r"FS:(?P<front_steer>-?\d+)\s+"
    r"S:(?P<servo_steer>-?\d+)\s+"
    r"BW:(?P<blend_weight>\d+)\s+"
    r"DOM:(?P<dom>[TFB])\s+"
    r"FV:(?P<front_valid>[01])\s+"
    r"IX:(?P<intersection>[01])\s+"
    r"ST:(?P<state>[NIL])\s+"
    r"VL:(?P<vl>\d+)\s+"
    r"VR:(?P<vr>\d+)$"
)

def test_new_format():
    m = pattern.match(sample_line.strip())
    assert m is not None, f"Failed to match valid line: {sample_line}"
    d = m.groupdict()
    assert d['top_steer'] == "-15", f"Expected TS=-15, got {d['top_steer']}"
    assert d['front_steer'] == "-12", f"Expected FS=-12, got {d['front_steer']}"
    assert d['servo_steer'] == "-14", f"Expected S=-14, got {d['servo_steer']}"
    assert d['blend_weight'] == "85", f"Expected BW=85, got {d['blend_weight']}"
    assert d['dom'] == "T", f"Expected DOM=T, got {d['dom']}"
    assert d['front_valid'] == "1", f"Expected FV=1, got {d['front_valid']}"
    assert d['intersection'] == "0", f"Expected IX=0, got {d['intersection']}"
    assert d['state'] == "N", f"Expected ST=N, got {d['state']}"
    print("NEW TELEMETRY FORMAT TEST PASSED!")

def test_old_format_rejected():
    m = pattern.match(old_line.strip())
    assert m is None, "Old format should NOT match new pattern"
    print("OLD FORMAT REJECTION TEST PASSED!")

if __name__ == "__main__":
    try:
        test_new_format()
        test_old_format_rejected()
        print("ALL TELEMETRY TESTS PASSED!")
    except AssertionError as e:
        print(f"TEST FAILED: {e}")
        sys.exit(1)
