import re
import numpy as np
import json

pattern = re.compile(r'L:(\d+)/(\d+)\s+R:(-?\d+)\s+F:(-?\d+)\s+H:(-?\d+)\s+TJ:(-?\d+)\s+S:(-?\d+)\s+LA:(\d+)\s+ST:([FT])\s+VL:(\d+)\s+VR:(\d+)')

def parse_telemetry(file_path):
    with open(file_path, 'r') as f:
        text = f.read()

    run_blocks = text.split('OK state=RUNNING')[1:]
    runs = []
    for r_idx, block in enumerate(run_blocks):
        frames = []
        for line in block.split('\n'):
            m = pattern.search(line)
            if m:
                top_v, front_v, r, f_val, h, tj, s, la, st, vl, vr = m.groups()
                frames.append({
                    'top_v': int(top_v),
                    'front_v': int(front_v),
                    'r': int(r) / 10.0,
                    'f': int(f_val) / 10.0,
                    'h': int(h) / 10.0,
                    'tj': int(tj) / 10.0,
                    's': int(s) / 10.0,
                    'la': int(la),
                    'st_base': st,
                    'vl_base': int(vl),
                    'vr_base': int(vr)
                })
        if frames:
            runs.append(frames)
    return runs

def simulate_governor(frames):
    """
    Simulates the new Continuous Curvature Governor (source/main.c)
    on the recorded telemetry frame stream.
    Each telemetry frame represents 4 ticks = 80ms of control execution.
    """
    # Parameters from Config.h
    GOVERNOR_HEADING_DEADBAND = 2.5   # deg
    GOVERNOR_HEADING_MAX = 25.0       # deg
    LOOKAHEAD_DEADBAND = 3.5          # px/urgency margin
    LOOKAHEAD_FULL = 12.0
    SPEED_STRAIGHT = 100.0
    SPEED_CORNER = 58.0
    SPEED_INSIDE_MIN = 35.0
    DIFF_DEADZONE = 3.5
    STEER_FULL_BRAKE = 30.0
    
    # 4 ticks per frame * 18% / tick = 72% max accel per frame
    # 4 ticks per frame * 25% / tick = 100% max decel per frame
    MAX_ACCEL_FRAME = 18.0 * 4.0
    MAX_DECEL_FRAME = 25.0 * 4.0

    sim_results = []
    applied_speed = SPEED_STRAIGHT
    applied_left = SPEED_STRAIGHT
    applied_right = SPEED_STRAIGHT

    for i, frame in enumerate(frames):
        h = frame['h']
        abs_h = abs(h)
        
        # 1. Base Camera Curvature with Deadband
        k_base = 0.0
        if abs_h > GOVERNOR_HEADING_DEADBAND:
            k_base = (abs_h - GOVERNOR_HEADING_DEADBAND) / (GOVERNOR_HEADING_MAX - GOVERNOR_HEADING_DEADBAND)
            if k_base > 1.0: k_base = 1.0

        # 2. Front Camera Curvature
        # In telemetry, frame['la'] was computed from lookahead_turn_factor * 100
        # When |h| <= 3.0 on straights, the new vectors.c deadband clamps lookahead curvature to 0.0
        # If |h| <= 2.5 deg (confirmed straight), curvature is 0.0
        k_front = frame['la'] / 100.0
        if abs_h <= GOVERNOR_HEADING_DEADBAND and abs(frame['f']) <= 4.0:
            k_front = 0.0  # Straightaway noise suppression confirmed by deadbands

        if k_front > 1.0: k_front = 1.0

        # Combined Kappa
        kappa = max(k_base, k_front)
        if kappa > 1.0: kappa = 1.0

        # Target Longitudinal Speed
        target_speed = SPEED_STRAIGHT - kappa * (SPEED_STRAIGHT - SPEED_CORNER)

        # Global Speed Slew Rate Limiter
        speed_diff = target_speed - applied_speed
        if speed_diff > MAX_ACCEL_FRAME:
            applied_speed += MAX_ACCEL_FRAME
        elif speed_diff < -MAX_DECEL_FRAME:
            applied_speed -= MAX_DECEL_FRAME
        else:
            applied_speed = target_speed

        # Steering & Dynamic Differential Drag
        steer_cmd = frame['s']
        abs_steer = abs(steer_cmd)
        steer_norm = 0.0
        if abs_steer > DIFF_DEADZONE:
            steer_norm = (abs_steer - DIFF_DEADZONE) / (STEER_FULL_BRAKE - DIFF_DEADZONE)
            if steer_norm > 1.0: steer_norm = 1.0

        diff_delta = steer_norm * (applied_speed - SPEED_INSIDE_MIN)
        if diff_delta < 0.0: diff_delta = 0.0

        if steer_cmd > DIFF_DEADZONE:
            # Turning right
            target_left = applied_speed
            target_right = applied_speed - diff_delta
        elif steer_cmd < -DIFF_DEADZONE:
            # Turning left
            target_left = applied_speed - diff_delta
            target_right = applied_speed
        else:
            target_left = applied_speed
            target_right = applied_speed

        # Slew applied left and right
        diff_l = target_left - applied_left
        if diff_l > MAX_ACCEL_FRAME: applied_left += MAX_ACCEL_FRAME
        elif diff_l < -MAX_DECEL_FRAME: applied_left -= MAX_DECEL_FRAME
        else: applied_left = target_left

        diff_r = target_right - applied_right
        if diff_r > MAX_ACCEL_FRAME: applied_right += MAX_ACCEL_FRAME
        elif diff_r < -MAX_DECEL_FRAME: applied_right -= MAX_DECEL_FRAME
        else: applied_right = target_right

        sim_results.append({
            'frame_idx': i,
            'kappa': kappa,
            'target_speed': target_speed,
            'applied_speed': applied_speed,
            'applied_left': applied_left,
            'applied_right': applied_right,
            'avg_speed_sim': (applied_left + applied_right) / 2.0,
            'avg_speed_base': (frame['vl_base'] + frame['vr_base']) / 2.0,
            'st_base': frame['st_base'],
            'h': h,
            'f': frame['f'],
            'la': frame['la']
        })

    return sim_results

def segment_laps_run2(run2_frames):
    """
    Segments the 1072-frame Run 2 into individual laps using Turn 1 landmarks.
    """
    turn1_indices = [14, 110, 216, 319, 412, 528, 624, 726, 822, 929, 1040]
    laps = []
    for i in range(len(turn1_indices) - 1):
        start = turn1_indices[i]
        end = turn1_indices[i+1]
        laps.append((i + 1, start, end, run2_frames[start:end]))
    return laps

def classify_track_sector(frame):
    """
    Classifies frame into track sectors based on track geometry:
    - Start/Finish Straight: |H| <= 4.0, before Turn 1
    - Turn 1 (90° Left): H < -15.0
    - Top Wavy Section: -15.0 <= H <= 15.0, between Turn 1 and Turn 2
    - Turn 2 (90° Left): H < -15.0
    - Inner Hairpin / Turn 4-5: H > 25.0
    - Sweeper 180°: Large heading swing
    """
    h = frame['h']
    if abs(h) <= 2.5 and abs(frame['f']) <= 4.0:
        return 'STRAIGHT'
    elif h < -18.0:
        return 'SHARP_LEFT'
    elif h > 22.0:
        return 'SHARP_RIGHT'
    else:
        return 'CHICANE_TRANSITION'

def analyze_lap(lap_num, frames, sim_res):
    n = len(frames)
    base_speeds = [x['avg_speed_base'] for x in sim_res]
    sim_speeds = [x['avg_speed_sim'] for x in sim_res]
    kappas = [x['kappa'] for x in sim_res]

    base_mean = np.mean(base_speeds)
    sim_mean = np.mean(sim_speeds)
    speed_lift = ((sim_mean - base_mean) / base_mean) * 100.0
    # Sprint %: fraction of time speed >= 90% (near-max throttle)
    base_sprint_pct = (sum(1 for s in base_speeds if s >= 90.0) / n) * 100.0
    sim_sprint_pct = (sum(1 for s in sim_speeds if s >= 90.0) / n) * 100.0

    # True 100% full sprint %
    base_full_100_pct = (sum(1 for s in base_speeds if s >= 99.0) / n) * 100.0
    sim_full_100_pct = (sum(1 for s in sim_speeds if s >= 99.0) / n) * 100.0

    # Corner apex speed (minimum speed negotiated during turns)
    base_min = np.min(base_speeds)
    sim_min = np.min(sim_speeds)
    base_max = np.max(base_speeds)
    sim_max = np.max(sim_speeds)

    # State thrashing: count number of times base flipped ST:F <-> ST:T
    flips = 0
    for i in range(1, n):
        if frames[i]['st_base'] != frames[i-1]['st_base']:
            flips += 1

    # In governor: zero discrete state flips (pure continuous)
    # Estimate lap time (assuming distance is fixed, lap time is inversely proportional to mean speed)
    base_time_sec = n * 0.080
    sim_time_sec = base_time_sec * (base_mean / sim_mean)
    time_saved_sec = base_time_sec - sim_time_sec
    time_saved_pct = (time_saved_sec / base_time_sec) * 100.0

    return {
        'lap': lap_num,
        'frames': n,
        'base_mean_speed': base_mean,
        'sim_mean_speed': sim_mean,
        'speed_lift_pct': speed_lift,
        'base_sprint_pct': base_sprint_pct,
        'sim_sprint_pct': sim_sprint_pct,
        'base_full_100_pct': base_full_100_pct,
        'sim_full_100_pct': sim_full_100_pct,
        'base_min_speed': base_min,
        'sim_min_speed': sim_min,
        'base_max_speed': base_max,
        'sim_max_speed': sim_max,
        'base_state_flips': flips,
        'base_time_sec': base_time_sec,
        'sim_time_sec': sim_time_sec,
        'time_saved_sec': time_saved_sec,
        'time_saved_pct': time_saved_pct
    }

def main():
    runs = parse_telemetry('docs/debug_telemetry.md')
    print("================================================================================")
    print("      CONTINUOUS CURVATURE GOVERNOR: FULL-FLEET SIMULATION BENCHMARK            ")
    print("================================================================================")
    print(f"Total Telemetry Runs Parsed: {len(runs)}")
    print(f"Total Frames Analyzed: {sum(len(r) for r in runs):,} (equivalent to {sum(len(r) for r in runs)*0.080:.1f} seconds of driving)\n")

    # Analyze Run 2 (Multi-Lap Race Simulation, 1072 frames)
    run2_frames = runs[1]
    run2_sim = simulate_governor(run2_frames)
    laps_run2 = segment_laps_run2(run2_frames)

    lap_reports = []
    print(f"--- RUN 2: 10 FULL CONSECUTIVE LAPS (Frames {laps_run2[0][1]} to {laps_run2[-1][2]}) ---")
    print(f"{'Lap':<5} | {'Frames':<7} | {'Base Mean':<10} | {'Gov Mean':<10} | {'Lift %':<8} | {'Base Sprint':<12} | {'Gov Sprint':<12} | {'Base Time':<10} | {'Gov Time':<10} | {'Saved':<8}")
    print("-" * 105)

    for lap_num, start, end, l_frames in laps_run2:
        l_sim = run2_sim[start:end]
        rep = analyze_lap(lap_num, l_frames, l_sim)
        lap_reports.append(rep)
        print(f"Lap {rep['lap']:<2} | {rep['frames']:<7} | {rep['base_mean_speed']:>8.1f}%  | {rep['sim_mean_speed']:>8.1f}%  | {rep['speed_lift_pct']:>+6.1f}% | {rep['base_sprint_pct']:>9.1f}%   | {rep['sim_sprint_pct']:>9.1f}%   | {rep['base_time_sec']:>7.2f}s   | {rep['sim_time_sec']:>7.2f}s  | -{rep['time_saved_sec']:>4.2f}s ({rep['time_saved_pct']:.1f}%)")

    # Aggregate Run 2 Stats
    tot_base_time = sum(r['base_time_sec'] for r in lap_reports)
    tot_sim_time = sum(r['sim_time_sec'] for r in lap_reports)
    tot_saved = tot_base_time - tot_sim_time
    avg_base_spd = np.mean([r['base_mean_speed'] for r in lap_reports])
    avg_sim_spd = np.mean([r['sim_mean_speed'] for r in lap_reports])
    avg_base_sprint = np.mean([r['base_sprint_pct'] for r in lap_reports])
    avg_sim_sprint = np.mean([r['sim_sprint_pct'] for r in lap_reports])
    tot_flips = sum(r['base_state_flips'] for r in lap_reports)

    print("-" * 105)
    print(f"{'TOTAL':<5} | {sum(r['frames'] for r in lap_reports):<7} | {avg_base_spd:>8.1f}%  | {avg_sim_spd:>8.1f}%  | {((avg_sim_spd-avg_base_spd)/avg_base_spd)*100:>+6.1f}% | {avg_base_sprint:>9.1f}%   | {avg_sim_sprint:>9.1f}%   | {tot_base_time:>7.2f}s   | {tot_sim_time:>7.2f}s  | -{tot_saved:>4.2f}s ({tot_saved/tot_base_time*100:.1f}%)")
    print(f"\n* Discrete State Thrashing (Jumping ST:F <-> ST:T): Baseline = {tot_flips} flips -> Governor = 0 flips (100% eliminated)!\n")

    # Analyze All Other Runs
    print("================================================================================")
    print("      ALL RECORDED RUNS SUMMARY BENCHMARK                                       ")
    print("================================================================================")
    print(f"{'Run':<6} | {'Frames':<7} | {'Duration':<9} | {'Base Speed':<11} | {'Gov Speed':<11} | {'Speed Lift':<11} | {'Base Sprint':<12} | {'Gov Sprint':<12}")
    print("-" * 95)
    all_runs_summary = []
    for r_idx, r_frames in enumerate(runs):
        r_sim = simulate_governor(r_frames)
        b_spd = np.mean([x['avg_speed_base'] for x in r_sim])
        s_spd = np.mean([x['avg_speed_sim'] for x in r_sim])
        lift = ((s_spd - b_spd) / b_spd) * 100.0
        b_spr = (sum(1 for x in r_sim if x['avg_speed_base'] >= 90.0) / len(r_sim)) * 100.0
        s_spr = (sum(1 for x in r_sim if x['avg_speed_sim'] >= 90.0) / len(r_sim)) * 100.0
        dur = len(r_frames) * 0.080
        all_runs_summary.append({
            'run': r_idx + 1, 'frames': len(r_frames), 'dur': dur,
            'base_spd': b_spd, 'sim_spd': s_spd, 'lift': lift,
            'base_spr': b_spr, 'sim_spr': s_spr
        })
        print(f"Run {r_idx+1:<2} | {len(r_frames):<7} | {dur:>6.2f}s   | {b_spd:>9.1f}%  | {s_spd:>9.1f}%  | {lift:>+8.1f}%   | {b_spr:>9.1f}%   | {s_spr:>9.1f}%")

    # Sector-by-Sector Analysis on Track Layout
    print("\n================================================================================")
    print("      SECTOR-BY-SECTOR TRACK PERFORMANCE (Run 2 Aggregate)                      ")
    print("================================================================================")
    sector_data = {'STRAIGHT': [], 'SHARP_LEFT': [], 'SHARP_RIGHT': [], 'CHICANE_TRANSITION': []}
    for f, s in zip(run2_frames, run2_sim):
        sec = classify_track_sector(f)
        sector_data[sec].append((s['avg_speed_base'], s['avg_speed_sim'], s['kappa']))

    print(f"{'Track Sector':<20} | {'Frames':<7} | {'Base Speed':<11} | {'Gov Speed':<11} | {'Speed Lift':<11} | {'Mean Kappa':<10}")
    print("-" * 80)
    for sec, vals in sector_data.items():
        if vals:
            b_s = np.mean([x[0] for x in vals])
            g_s = np.mean([x[1] for x in vals])
            kap = np.mean([x[2] for x in vals])
            lift = ((g_s - b_s) / b_s) * 100.0
            print(f"{sec:<20} | {len(vals):<7} | {b_s:>9.1f}%  | {g_s:>9.1f}%  | {lift:>+8.1f}%   | {kap:>8.2f}")

    # Export full JSON report for inspection
    report_data = {
        'lap_reports': lap_reports,
        'runs_summary': all_runs_summary,
        'totals': {
            'tot_base_time': tot_base_time,
            'tot_sim_time': tot_sim_time,
            'tot_saved_sec': tot_saved,
            'tot_saved_pct': (tot_saved / tot_base_time) * 100.0,
            'avg_base_spd': avg_base_spd,
            'avg_sim_spd': avg_sim_spd,
            'tot_flips_eliminated': tot_flips
        }
    }
    with open('scratch/simulation_results.json', 'w') as f_out:
        json.dump(report_data, f_out, indent=2)

if __name__ == '__main__':
    main()
