import re
import sys

with open("scratch/save_latest_run.py") as f:
    text = f.read()

# Let's extract the actual raw text from user prompt
# Let's save all three runs into a dedicated file
raw_runs = """
=== RUN 1 ===
spd_s=100.00 spd_out=62.00 spd_in=28.00 fa_th=18.00
=== RUN 2 ===
fa_th=23.00 spd_out=62.00 spd_in=32.00 k_cte=0.55 k_head=0.90
=== RUN 3 ===
speed_inside=25.00 speed_outside=60.00 fa_th=23.00
"""
