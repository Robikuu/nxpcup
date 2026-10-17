# Script to analyze the 3 runs in the latest prompt
import re

pattern = re.compile(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TS:(-?\d+)\s+FS:(-?\d+)\s+S:(-?\d+)\s+BW:(\d+)\s+DOM:([TFB])\s+FV:(\d+)\s+IX:(\d+)\s+ST:([NIL])\s+VL:(-?\d+)\s+VR:(-?\d+)')

with open(r"d:\nxpcup\scratch\telemetry_speed_analysis.py", "r", encoding="utf-8") as f:
    full_user_script = f.read()

# But the user's latest prompt has 3 runs separated by stop/start!
# Let's inspect the user's latest prompt directly:
# Run 1:
# L:2/3 R:-25 F:-14 H:63 TS:29 FS:28 S:0 BW:100 DOM:T FV:1 IX:0 ST:N VL:94 VR:94
# ...
# L:0/0 R:0 F:25 H:368 TS:174 FS:-46 S:7 BW:0 DOM:F FV:0 IX:0 ST:L VL:0 VR:0
#
# Run 2:
# L:2/3 R:50 F:50 H:46 TS:36 FS:-37 S:0 BW:100 DOM:T FV:0 IX:0 ST:N VL:95 VR:95
# ...
# L:1/0 R:-85 F:-67 H:-192 TS:-333 FS:-44 S:-333 BW:100 DOM:T FV:0 IX:0 ST:N VL:62 VR:86
#
# Run 3:
# L:2/3 R:-25 F:-25 H:11 TS:-9 FS:-9 S:0 BW:100 DOM:T FV:1 IX:0 ST:N VL:95 VR:95
# ...
# (Hundreds of lines, completed multiple laps!)

print("Script template ready")
