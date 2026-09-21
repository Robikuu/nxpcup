# Design Spec: Lookahead Continuous Urgency Blending (Approach 1)

## Overview
Enable the vehicle to sprint at 100% duty cycle on straights while using the 2nd (front lookahead) Pixy2 camera to proactively decelerate and pivot into corners on time, resolving the late-turn-in failure mode caused by dual-motor forward thrust.

## Background & Problem
With both rear motors physically synchronized to drive forward:
1. **Kinetic Energy Doubled:** On straights at `SPEED_STRAIGHT = 100%`, the car reaches corners much faster.
2. **Artificial Throttle on Lookahead Preview:** In `source/main.c`, `traj_blend` was choked to `0.35f` and differential drag (`active_steer_norm`) was choked to `0.15f` whenever `drive_state == DRIVE_FORWARD`.
3. **Discrete Late Transition:** `drive_state` only transitioned into `DRIVE_TURNING` when the bumper camera (`cam_pid`) detected `filtered_heading >= 18.0f`. By that point, at 100% speed, the car has already missed the turn-in point.
4. **Insufficient Corner Deceleration:** `SPEED_OUTSIDE_CORNER` was set to `88%` and `SPEED_INSIDE_MIN` to `62%`, leaving almost zero trail-braking capability.

## Proposed Architecture

### 1. Continuous Urgency Scaling (Replacing Binary State Gates)
Eliminate rigid binary branching based on `drive_state == DRIVE_FORWARD` for steering preview blend and differential drag. Instead, scale these parameters continuously using `trusted_corner_urgency` ($u \in [0.0, 1.0]$), which is already driven by `lookahead_turn_factor` from the front camera.

#### A. Trajectory Steering Preview Blend (`traj_blend`)
$$ \text{traj\_blend} = 0.35f + u \cdot (0.75f - 0.35f) $$
* On straights ($u \approx 0.0$): `traj_blend = 0.35f` (smooth, jitter-free horizon tracking).
* On corner approach ($u \to 1.0$): `traj_blend` smoothly rises to `0.75f`, applying decisive steering angle into the curve before the car physically reaches it.

#### B. Differential Drag Authority (`active_steer_norm`)
$$ \text{diff\_scale} = 0.15f + u \cdot 0.85f $$
$$ \text{active\_steer\_norm} = \text{steer\_norm} \cdot \text{diff\_scale} $$
* On straights ($u \approx 0.0$): `diff_scale = 0.15f` (avoids yaw shock during minor line-centering trim).
* On corner approach ($u \to 1.0$): `diff_scale` opens up to `1.0f` (100%), allowing the inside rear wheel to drag and pivot the car into the turn.

### 2. Longitudinal Speed Modulation
Tune corner speeds in `include/Config.h`:
* `SPEED_STRAIGHT`: `100.0f` (unchanged, full straightaway sprint).
* `SPEED_OUTSIDE_CORNER`: Reduced from `88.0f` to `70.0f` (carries solid momentum while permitting sharp radius capture).
* `SPEED_INSIDE_MIN`: Reduced from `62.0f` to `35.0f` (restores effective pivot anchor braking on the inside wheel).
* As $u \to 1.0$, symmetric longitudinal base speed drops from 100% to 70%, followed by inside wheel deceleration down towards 35% proportional to steering demand.

### 3. Predictive State Machine Transition
Update the state transition condition in `source/main.c`:
* If either `fabsf(filtered_heading) >= CORNER_HEADING_THRESHOLD_DEG` OR `lookahead_turn_factor >= 0.45f`, `desired_state = DRIVE_TURNING`.
* Ensures telemetry, state flags, and PID reset behavior remain coherent when approaching curves at high speed.

## Files to Modify
1. `D:/nxpcup/include/Config.h`: Update `SPEED_OUTSIDE_CORNER` to 70.0f, `SPEED_INSIDE_MIN` to 35.0f.
2. `D:/nxpcup/source/main.c`: Implement continuous urgency scaling for `traj_blend`, `active_steer_norm`, and update the state machine transition condition.
3. Also mirror updates to `d:/robotica/workspace/nxpcup-ciorba_curata` to maintain parity between repositories.

## Verification
1. Clean build using ARM GNU Toolchain for MCXN947 in `D:\nxpcup\Debug`.
2. Verify binary compilation with zero errors.
3. Track test: Verify car maintains 100% on straights and initiates progressive deceleration and inside wheel pivot as soon as the front camera detects an approaching curve.
