# Lookahead Continuous Urgency Blending Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Enable the car to sprint at 100% on straights while using the front lookahead camera to smoothly pre-brake and pivot-steer into corners on time.

**Architecture:** Replace binary `drive_state == DRIVE_FORWARD` throttles on steering trajectory blend and differential drag with continuous scaling based on `trusted_corner_urgency`. Lower corner baseline and inside pivot speeds to allow effective deceleration.

**Tech Stack:** C (ARM Cortex-M33 / NXP MCXN947), MCUXpresso SDK, Pixy2 dual-camera pipeline.

---

### Task 1: Update Speed Parameters in Config.h

**Files:**
- Modify: `D:/nxpcup/include/Config.h:22-26`
- Mirror: `d:/robotica/workspace/nxpcup-ciorba_curata/include/Config.h:22-26`

- [ ] **Step 1: Update `SPEED_OUTSIDE_CORNER` and `SPEED_INSIDE_MIN`**

In `Config.h`:
```c
#define SPEED_STRAIGHT               100.0f    // Straight line sprint speed (%) (full 100% push)
#define SPEED_OUTSIDE_CORNER         70.0f     // Fast apex outside wheel drive (%) (carries momentum through turns)
#define SPEED_INSIDE_MIN             35.0f     // Inside wheel pivot anchor duty (%) (effective pivot drag)
#define SPEED_CORNER_AVG_MIN         ((SPEED_OUTSIDE_CORNER + SPEED_INSIDE_MIN) * 0.5f) // 52.5%
```

- [ ] **Step 2: Commit Config.h change in D:/nxpcup**

```bash
git -C D:/nxpcup add include/Config.h
git -C D:/nxpcup commit -m "tune(config): adjust corner outside and inside pivot speeds for lookahead braking"
```

---

### Task 2: Implement Continuous Urgency Scaling in main.c

**Files:**
- Modify: `D:/nxpcup/source/main.c:205-212,289-295,333-336`
- Mirror: `d:/robotica/workspace/nxpcup-ciorba_curata/source/main.c:205-212,289-295,333-336`

- [ ] **Step 1: Update state machine condition to include lookahead preview**

In `source/main.c`:
```c
            /* State machine transitions with hysteresis */
            drive_state_t desired_state;
            if (sr.found == 0) {
                desired_state = DRIVE_BRAKING;
            } else {
                desired_state = (fabsf(filtered_heading) >= CORNER_HEADING_THRESHOLD_DEG || lookahead_turn_factor >= 0.45f) ? DRIVE_TURNING : DRIVE_FORWARD;
            }
```

- [ ] **Step 2: Update trajectory steering preview blend to scale continuously**

In `source/main.c`:
```c
                float trajectory_steer = 0.0f;
                if (g_traj.found > 0) {
                    /* On straights, blend lightly (0.35x) to maintain stable tracking without twitch;
                     * As corner urgency increases, blend up to 0.75x for early turn-in preview. */
                    float traj_blend = 0.35f + trusted_corner_urgency * (0.75f - 0.35f);
                    trajectory_steer = traj_blend * (0.45f * filtered_traj_angle + 0.20f * filtered_traj_offset);
                }
```

- [ ] **Step 3: Update differential drag authority to scale continuously**

In `source/main.c`:
```c
                /* Continuous differential authority:
                 * On straights (urgency ~ 0), differential drag is suppressed (0.15x) for stable high-speed push;
                 * As corner urgency rises (urgency -> 1.0), full differential authority opens up (up to 1.0x). */
                float diff_scale = 0.15f + trusted_corner_urgency * 0.85f;
                float active_steer_norm = steer_norm * diff_scale;
```

- [ ] **Step 4: Commit main.c change in D:/nxpcup**

```bash
git -C D:/nxpcup add source/main.c
git -C D:/nxpcup commit -m "feat(main): implement continuous urgency scaling for trajectory blend and differential drag"
```

---

### Task 3: Build & Verification

- [ ] **Step 1: Clean build with ARM GNU Toolchain**

```powershell
$env:PATH = "C:\Users\rober\.mcuxpressotools\arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi\bin;" + $env:PATH
make -C D:\nxpcup\Debug all
```

- [ ] **Step 2: Verify zero compiler warnings/errors and check binary size**
